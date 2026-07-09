# AGENTS.md — VANT Flight Controller (FlightControl)

## Projeto

Firmware de controlador de voo para **VANT convencional (Tractor) com cauda de lança dupla (Twin-Boom)**, operando em **ESP32 Dual-Core** com **FreeRTOS** e framework **Arduino/PlatformIO**.

---

## Regras Absolutas (Nunca Violar)

### Aeronave e Atuação
- A aeronave é um **avião convencional**. **NÃO** implementar mixagem elevon, V-tail, ou qualquer mixagem cruzada.
- O mapeamento é **direto (Direct Pass-Through)**:
  - 2× Ailerons nas asas → controlam **ESTRITAMENTE** Roll.
  - 1× Profundor na cauda → controla **ESTRITAMENTE** Pitch.
  - Estabilizadores verticais são **FIXOS**. **Não existe leme (rudder).**
- Hardware Alvo:
  - Motor: A2212 1000KV
  - ESC: 30A com BEC
  - Hélice: 1045
  - Bateria: Li-ion 2S (Pre-Arm 7.0V, Failsafe crítico 6.0V)
- Cauda fixada por duas hastes de fibra de vidro.
- Filosofia de controle: **"Bank and Yank"** — guinada é coordenada via roll.

### Arquitetura de Núcleos
- **Core 1**: Malha de controle atitudinal (250Hz, Prioridade 5). **NADA pode bloquear esta task.** Jitter alvo: < 100µs.
- **Core 0**: Navegação (50Hz), GPS (async), LoRa (10Hz), System Monitor (1Hz).
- **IPC**: Toda comunicação inter-tasks via `FlightState globalState` protegida por `stateMutex` (Mutex FreeRTOS).
- Padrão de acesso ao estado global:
  ```cpp
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      // Ler/escrever globalState
      xSemaphoreGive(stateMutex);
  }
  ```

### Thread Safety
- Todo acesso a `globalState` **DEVE** ser protegido por `stateMutex`.
- Nunca usar `delay()` no Core 1. Usar `vTaskDelayUntil()` para timing preciso.
- Nunca fazer I/O bloqueante (Serial.print, Flash write) no Core 1.

---

## Pinagem ESP32 (Não Alterar)

| Periférico | Pinos | Barramento |
|------------|-------|------------|
| MPU6050 (IMU) | SDA=21, SCL=22 | I2C Fast 400kHz (Core 1) |
| BMP280 (Baro) | SDA=32, SCL=33 | I2C Slow 400kHz (Core 0) |
| GPS NEO-6M | RX=16, TX=17 | UART2, 9600 baud |
| LoRa SX1278 | SCK=18, MISO=19, MOSI=23, CS=5, RST=14, DIO0=26 | SPI, 433MHz |
| Aileron Esq | Pino 13 | LEDC CH0, 50Hz, 14-bit |
| Aileron Dir | Pino 12 | LEDC CH1 |
| Profundor | Pino 15 | LEDC CH2 |
| ESC Motor | Pino 27 | LEDC CH3 |
| Bateria ADC | Pino 34 | ADC, atenuação 11dB |

---

## Convenções de Código

### Linguagem
- **C++ (Arduino/PlatformIO)**. Framework `espressif32`.
- Comentários de engenharia aeroespacial em **Português**.
- Nomes de variáveis, funções e classes em **inglês**.

### Estilo
- Prefixo `_` para membros privados de classe (ex: `_kp`, `_integral`).
- `static constexpr` para constantes de hardware e limites físicos.
- Structs de pacotes LoRa: `__attribute__((packed))` obrigatório (sem padding).
- Enums tipados (`enum class`) para modos, estados e IDs de parâmetros.

### Unidades Físicas
- Ângulos: **graus** na interface, **radianos** internamente no AHRS.
- Taxas angulares: **graus/segundo (dps)**.
- Acelerações: **g** no sensor, **m/s²** no estado global.
- Altitude: **metros** (relativa ao boot).
- Velocidade: **m/s**.
- PWM servos: **microsegundos (µs)**, range 1000-2000, centro 1500.
- Comandos normalizados: **-1.0 a +1.0** (roll/pitch), **0.0 a 1.0** (throttle).

### Build
- Compilar com `-Wall -Wextra`. Objetivo: **zero warnings** no código do projeto.
- Dependências: `TinyGPSPlus`, `LoRa` (Sandeep Mistry), `Preferences`, `Wire`.
- Build command: `pio run` (PlatformIO CLI em `C:\Users\souza\.platformio\penv\Scripts\platformio.exe`).

---

## Arquitetura GNC

### PID
- Cascata **Angle → Rate** com **FeedForward (FF)** aerodinâmico como termo primário.
- Filtro **PT1** no termo derivativo (cutoff 25Hz).
- **Anti-Windup Dinâmico**: integrador congela quando mixer reporta saturação.
- **TPA** (Throttle PID Attenuation): atenua Kp em alta potência do motor.
- **Bumpless Transfer**: reset obrigatório de todos os PIDs em transição de modo.

### AHRS
- Filtro de **Mahony** (quaternion) com correção de **força centrífuga** em curvas.
- **COG Yaw Fusion**: filtro complementar suave (α=0.02) com GPS Course Over Ground quando velocidade > 5 m/s. Sem magnetômetro.
- **Notch Filter Dinâmico**: Filtro biquad IIR com frequência central adaptativa. Rastreia o throttle para estimar a RPM do motor e rejeitar a frequência exata de vibração (50Hz a 170Hz para o combo A2212 1000KV/1045 em 2S).
- **Auto-calibração Robusta**: Ocorre no boot com 500 amostras. Utiliza análise de desvio padrão (stddev) para detecção de movimento, rejeitando amostras corrompidas e executando _retry_ caso a aeronave seja sacudida pelo piloto.

### Navegação
- **L1 Guidance**: aceleração lateral `a_lat = 2·V²/L1·sin(η)`, roll clampado a **±35°** (proteção tip stall).
- **TECS Híbrido**: detecção de estol por **confirmação dupla** (GPS ground speed + velocidade inercial integrada). Histerese em 9.5 m/s.
- **Kalman 1D**: fusão barômetro + acelerômetro para altitude.

### FSM e Segurança
- Modos: MANUAL, ANGLE, HOLD (com override por stick), AUTO, RTH.
- **Pre-Arm Checks**: GPS fix, Vbat ≥ 7.0V (bateria Li-ion 2S), inclinação ≤ 15°.
- **Failsafe**: timeout LoRa (default 1500ms, parametrizável NVS) → RTH forçado. Se Vbat cair para 6.0V em voo, entra em failsafe crítico.

---

## Protocolo LoRa

| Header | Direção | Struct | Conteúdo |
|--------|---------|--------|----------|
| `0xBB` | Uplink | `PacketUplinkLoRa_t` | RC: roll±1000, pitch±1000, thr 0-1000, mode, arm |
| `0xCC` | Uplink | `PacketWaypointLoRa_t` | Waypoint: index, lat×1e7, lon×1e7, alt(dm), speed(cm/s) |
| `0xDD` | Uplink | `PacketTuningLoRa_t` | Tuning: ParamID + float value |
| `0xAA` | Downlink | `PacketTelemetryLoRa_t` | Telemetria completa a 10Hz |

---

## Parâmetros NVS

- Namespace: `"VANT_PARAM"` (via `Preferences.h`).
- Tuning LoRa aplica valor em **RAM imediatamente**, agenda gravação Flash para `Task_System_Mon` (1Hz).
- Tabela completa em `PARAM_TABLE[]` no `main.cpp`.
- Nunca gravar Flash no Core 1 (tempo de escrita ~10ms é inaceitável na malha de controle).

/**
 * ============================================================
 *  main.cpp — Orquestrador FreeRTOS do Controlador de Voo
 * ============================================================
 *  VANT Flight Controller - ESP32 Dual-Core
 *  Avião Convencional (Tractor) com Cauda Twin-Boom
 * 
 *  ARQUITETURA DE NÚCLEOS:
 *  ┌─────────────────────────────────────────────────────┐
 *  │ CORE 1 — Controle Atitudinal (250Hz, Prio 5)       │
 *  │   MPU6050 → Notch → Mahony AHRS → FSM → PID → PWM │
 *  ├─────────────────────────────────────────────────────┤
 *  │ CORE 0 — Navegação e Sistema                       │
 *  │   Task_Navigation (50Hz, Prio 4)                    │
 *  │   Task_GPS (Async, Prio 3)                          │
 *  │   Task_LoRa (10Hz, Prio 3)                          │
 *  │   Task_System_Mon (1Hz, Prio 1)                     │
 *  └─────────────────────────────────────────────────────┘
 * 
 *  IPC: FlightState globalState protegida por stateMutex
 * 
 *  CALIBRAÇÃO: Auto-calibração do MPU6050 no boot
 *  (média de 500 amostras com aeronave estática).
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>

#include "SharedTypes.h"
#include "PID.h"
#include "FlightModeManager.h"
#include "StandardMixer.h"
#include "OutputManager.h"
#include "TECS.h"
#include "L1_Guidance.h"
#include "LoRaManager.h"

// ============================================================
//  VARIÁVEIS GLOBAIS (Declaradas extern em SharedTypes.h)
// ============================================================

FlightState globalState;
SemaphoreHandle_t stateMutex = NULL;

// ── Tabela de Parâmetros NVS ──
const ParamMeta PARAM_TABLE[] = {
    {"roll_kp",      4.5f },  // 0  - ROLL_KP
    {"roll_ki",      0.8f },  // 1  - ROLL_KI
    {"roll_kd",      0.03f},  // 2  - ROLL_KD
    {"roll_ff",      0.5f },  // 3  - ROLL_FF
    {"pitch_kp",     5.0f },  // 4  - PITCH_KP
    {"pitch_ki",     1.0f },  // 5  - PITCH_KI
    {"pitch_kd",     0.04f},  // 6  - PITCH_KD
    {"pitch_ff",     0.6f },  // 7  - PITCH_FF
    {"ang_kp",       6.0f },  // 8  - ANGLE_KP
    {"thr_cruise",   0.55f},  // 9  - THR_CRUISE
    {"l1_period",   17.0f },  // 10 - L1_PERIOD
    {"lora_timeout", 1500.0f},// 11 - LORA_TIMEOUT (ms)
    {"tpa_brkpt",    0.65f},  // 12 - TPA_BREAKPOINT
};

float paramValues[(uint8_t)ParamID::PARAM_COUNT];
volatile bool nvsFlushPending = false;

// ── NVS / Preferences ──
Preferences preferences;

// ── Objetos de Subsistemas ──
static FlightModeManager fsmManager;
static StandardMixer     mixer;
static OutputManager     outputMgr;
static TECSController    tecs;
static L1Guidance        l1;
static LoRaManager       loraMgr;
static TinyGPSPlus       gps;

// ── PIDs ──
// Cascata: Angle Loop (externo, calcula rate target) → Rate Loop (interno, comanda servo)
static PIDController rollRatePID(4.5f, 0.8f, 0.03f, 0.5f, -1.0f, 1.0f, 0.4f, 25.0f);
static PIDController pitchRatePID(5.0f, 1.0f, 0.04f, 0.6f, -1.0f, 1.0f, 0.4f, 25.0f);
static PIDController rollAnglePID(6.0f, 0.0f, 0.0f, 0.0f, -150.0f, 150.0f, 30.0f);
static PIDController pitchAnglePID(6.0f, 0.0f, 0.0f, 0.0f, -150.0f, 150.0f, 30.0f);

// ── I2C para BMP280 (Core 0) ──
static TwoWire I2C_Slow = TwoWire(1);  // Segundo barramento I2C

// ── UART GPS ──
static HardwareSerial SerialGPS(2);     // UART2

// ── Offsets de Calibração MPU6050 ──
static float gyroOffsetX = 0.0f, gyroOffsetY = 0.0f, gyroOffsetZ = 0.0f;
static float accelOffsetX = 0.0f, accelOffsetY = 0.0f, accelOffsetZ = 0.0f;

// ── Estado do BMP280 ──
static float bmp280_altRef_m = 0.0f;       // Altitude de referência (nível do solo)
static bool  bmp280_refSet = false;

// ── Kalman 1D para altitude (fusão baro + acelerômetro) ──
static float kalman_alt = 0.0f;
static float kalman_vz  = 0.0f;
static float kalman_P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};

// (FSM requests são comunicados via globalState, escritos pela Task_LoRa)

// ============================================================
//  MPU6050 — Driver Bare-Metal I2C
// ============================================================

/**
 * Escreve um byte em um registrador do MPU6050.
 */
static void mpu6050WriteReg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

/**
 * Lê N bytes a partir de um registrador do MPU6050.
 */
static void mpu6050ReadRegs(uint8_t reg, uint8_t* buffer, uint8_t len)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buffer[i] = Wire.read();
    }
}

/**
 * Inicializa o MPU6050 com configuração otimizada para voo.
 * - Saída a 1kHz (DLPF 42Hz pré-filtra antes do Notch)
 * - Giroscópio: ±500 dps (range adequado para avião, boa resolução)
 * - Acelerômetro: ±4g (cobre manobras normais + rajadas)
 */
static void mpu6050Init()
{
    // Acordar o MPU6050 (sai do sleep mode)
    mpu6050WriteReg(0x6B, 0x00);  // PWR_MGMT_1: Clock interno
    delay(50);
    
    // Configurar clock source para PLL do giroscópio X (mais estável)
    mpu6050WriteReg(0x6B, 0x01);  // PWR_MGMT_1: PLL with X axis gyro
    delay(10);
    
    // DLPF (Digital Low Pass Filter) — Bandwidth 42Hz
    // Reduz aliasing antes da nossa amostragem de 250Hz.
    // O Notch filter posterior remove a vibração residual em 150Hz.
    mpu6050WriteReg(0x1A, 0x03);  // CONFIG: DLPF_CFG = 3 (42Hz)
    
    // Sample Rate Divider: 1kHz / (1+3) = 250Hz
    // Coincide exatamente com nossa taxa de controle.
    mpu6050WriteReg(0x19, 0x03);  // SMPLRT_DIV = 3
    
    // Giroscópio: ±500 dps
    // Resolução: 500/32768 = 0.0153 dps/LSB (excelente para avião)
    // Range cobre loops suaves até ~300 dps
    mpu6050WriteReg(0x1B, 0x08);  // GYRO_CONFIG: FS_SEL = 1 (±500 dps)
    
    // Acelerômetro: ±4g
    // Resolução: 4/32768 = 0.000122 g/LSB
    // Range cobre manobras de ±4g (inclinações de 75° + rajadas)
    mpu6050WriteReg(0x1C, 0x08);  // ACCEL_CONFIG: AFS_SEL = 1 (±4g)

    Serial.println(F("[MPU] Inicializado: ±500dps, ±4g, DLPF 42Hz, 250Hz"));
}

/**
 * Auto-calibração ROBUSTA do MPU6050 no boot.
 * 
 * PROBLEMA ORIGINAL:
 * Se o piloto plugar a bateria balançando o avião na mão,
 * os offsets do giroscópio ficam corrompidos, e o AHRS
 * estimará ângulos errados → voo torto ou instável.
 * 
 * SOLUÇÃO — Detecção de Movimento:
 * 1. Coleta NUM_SAMPLES amostras do giroscópio e acelerômetro
 * 2. Calcula média E desvio padrão (stddev) do giroscópio
 * 3. Se stddev > MOTION_THRESHOLD_DPS → REJEITA calibração
 *    (aeronave está sendo movida durante a amostragem)
 * 4. Também verifica se aceleração total ≈ 1g (está nivelada)
 * 5. Se rejeitada, espera 2 segundos e tenta novamente
 * 6. Máximo MAX_RETRIES tentativas antes de usar offsets zero
 * 
 * O piloto receberá feedback serial claro do estado de calibração.
 */
static void mpu6050Calibrate()
{
    constexpr int   NUM_SAMPLES          = 500;
    constexpr int   MAX_RETRIES          = 5;
    constexpr float MOTION_THRESHOLD_DPS = 2.0f;   // Desvio padrão máximo aceitável
    constexpr float GRAVITY_TOL_G        = 0.15f;   // Tolerância: |accel_total - 1g|
    constexpr int   RETRY_DELAY_MS       = 2000;    // Espera entre tentativas

    Serial.println(F("[MPU] ═══════════════════════════════════════════"));
    Serial.println(F("[MPU] CALIBRAÇÃO — Aeronave ESTÁTICA e NIVELADA"));
    Serial.println(F("[MPU] ═══════════════════════════════════════════"));

    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        Serial.print(F("[MPU] ⏳ Tentativa "));
        Serial.print(attempt);
        Serial.print(F("/"));
        Serial.print(MAX_RETRIES);
        Serial.println(F(" — NÃO MOVER..."));

        // ── Fase 1: Coleta de amostras ──
        float sumGx = 0, sumGy = 0, sumGz = 0;
        float sumAx = 0, sumAy = 0, sumAz = 0;

        // Arrays para cálculo de desvio padrão (apenas giroscópio)
        float samplesGx[NUM_SAMPLES];
        float samplesGy[NUM_SAMPLES];
        float samplesGz[NUM_SAMPLES];

        for (int i = 0; i < NUM_SAMPLES; i++) {
            uint8_t raw[14];
            mpu6050ReadRegs(0x3B, raw, 14);

            int16_t ax_raw = (raw[0] << 8) | raw[1];
            int16_t ay_raw = (raw[2] << 8) | raw[3];
            int16_t az_raw = (raw[4] << 8) | raw[5];
            int16_t gx_raw = (raw[8] << 8) | raw[9];
            int16_t gy_raw = (raw[10] << 8) | raw[11];
            int16_t gz_raw = (raw[12] << 8) | raw[13];

            // Converter para unidades físicas
            float gx = (float)gx_raw / 65.5f;  // ±500 dps → 65.5 LSB/dps
            float gy = (float)gy_raw / 65.5f;
            float gz = (float)gz_raw / 65.5f;
            float ax = (float)ax_raw / 8192.0f; // ±4g → 8192 LSB/g
            float ay = (float)ay_raw / 8192.0f;
            float az = (float)az_raw / 8192.0f;

            sumGx += gx; sumGy += gy; sumGz += gz;
            sumAx += ax; sumAy += ay; sumAz += az;

            samplesGx[i] = gx;
            samplesGy[i] = gy;
            samplesGz[i] = gz;

            delay(2);  // ~500Hz amostragem

            // Progresso visual a cada 100 amostras
            if ((i + 1) % 100 == 0) {
                Serial.print(F("[MPU]   Progresso: "));
                Serial.print((i + 1) * 100 / NUM_SAMPLES);
                Serial.println(F("%"));
            }
        }

        // ── Fase 2: Calcular médias ──
        float meanGx = sumGx / NUM_SAMPLES;
        float meanGy = sumGy / NUM_SAMPLES;
        float meanGz = sumGz / NUM_SAMPLES;
        float meanAx = sumAx / NUM_SAMPLES;
        float meanAy = sumAy / NUM_SAMPLES;
        float meanAz = sumAz / NUM_SAMPLES;

        // ── Fase 3: Calcular desvio padrão do giroscópio ──
        // O desvio padrão indica QUANTO os valores variaram.
        // Em repouso, o gyro tem ruído de ~0.05 dps (stddev).
        // Se alguém está movendo o avião, stddev sobe para >5 dps.
        float varGx = 0, varGy = 0, varGz = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            float dx = samplesGx[i] - meanGx;
            float dy = samplesGy[i] - meanGy;
            float dz = samplesGz[i] - meanGz;
            varGx += dx * dx;
            varGy += dy * dy;
            varGz += dz * dz;
        }
        float stdGx = sqrtf(varGx / NUM_SAMPLES);
        float stdGy = sqrtf(varGy / NUM_SAMPLES);
        float stdGz = sqrtf(varGz / NUM_SAMPLES);
        float maxStd = fmaxf(stdGx, fmaxf(stdGy, stdGz));

        // ── Fase 4: Verificar aceleração total ≈ 1g (nivelada) ──
        float accelMag = sqrtf(meanAx * meanAx + meanAy * meanAy + meanAz * meanAz);
        float accelError = fabsf(accelMag - 1.0f);

        // ── Fase 5: Decisão — aceitar ou rejeitar ──
        Serial.print(F("[MPU]   Gyro stddev (dps): X="));
        Serial.print(stdGx, 3); Serial.print(F(" Y="));
        Serial.print(stdGy, 3); Serial.print(F(" Z="));
        Serial.println(stdGz, 3);
        Serial.print(F("[MPU]   Accel magnitude: "));
        Serial.print(accelMag, 3);
        Serial.println(F("g"));

        bool motionDetected = (maxStd > MOTION_THRESHOLD_DPS);
        bool notLevel       = (accelError > GRAVITY_TOL_G);

        if (motionDetected) {
            Serial.print(F("[MPU] ✗ MOVIMENTO DETECTADO (stddev="));
            Serial.print(maxStd, 2);
            Serial.print(F(" > "));
            Serial.print(MOTION_THRESHOLD_DPS, 1);
            Serial.println(F(" dps)"));
            Serial.println(F("[MPU]   → Coloque a aeronave no chão e não toque!"));
        }
        if (notLevel) {
            Serial.print(F("[MPU] ✗ AERONAVE NÃO NIVELADA (|accel|="));
            Serial.print(accelMag, 3);
            Serial.println(F("g, esperado ~1.0g)"));
            Serial.println(F("[MPU]   → Nivele a aeronave antes de energizar."));
        }

        if (!motionDetected && !notLevel) {
            // ── CALIBRAÇÃO ACEITA ──
            gyroOffsetX = meanGx;
            gyroOffsetY = meanGy;
            gyroOffsetZ = meanGz;
            accelOffsetX = meanAx;
            accelOffsetY = meanAy;
            accelOffsetZ = meanAz - 1.0f;  // Compensar gravidade

            Serial.println(F("[MPU] ✓ CALIBRAÇÃO ACEITA"));
            Serial.print(F("  Gyro offsets (dps): "));
            Serial.print(gyroOffsetX, 3); Serial.print(F(", "));
            Serial.print(gyroOffsetY, 3); Serial.print(F(", "));
            Serial.println(gyroOffsetZ, 3);
            Serial.print(F("  Accel offsets (g):  "));
            Serial.print(accelOffsetX, 4); Serial.print(F(", "));
            Serial.print(accelOffsetY, 4); Serial.print(F(", "));
            Serial.println(accelOffsetZ, 4);
            return;  // Sucesso — sair da função
        }

        // ── Tentativa falhou — esperar e tentar novamente ──
        if (attempt < MAX_RETRIES) {
            Serial.print(F("[MPU] Aguardando "));
            Serial.print(RETRY_DELAY_MS / 1000);
            Serial.println(F("s para nova tentativa..."));
            delay(RETRY_DELAY_MS);
        }
    }

    // ── TODAS as tentativas falharam ──
    // Usar offsets zero como fallback (melhor que offsets corrompidos)
    Serial.println(F("[MPU] ⚠ TODAS AS TENTATIVAS FALHARAM"));
    Serial.println(F("[MPU]   Usando offsets ZERO — Voo NÃO recomendado!"));
    Serial.println(F("[MPU]   Re-energize com a aeronave parada no chão."));
    gyroOffsetX = 0; gyroOffsetY = 0; gyroOffsetZ = 0;
    accelOffsetX = 0; accelOffsetY = 0; accelOffsetZ = 0;
}

/**
 * Lê os 6 eixos do MPU6050 e aplica calibração.
 * Retorna valores em unidades físicas (dps e g).
 */
static void mpu6050Read(float& gx, float& gy, float& gz,
                        float& ax, float& ay, float& az)
{
    uint8_t raw[14];
    mpu6050ReadRegs(0x3B, raw, 14);

    int16_t rawAx = (raw[0] << 8) | raw[1];
    int16_t rawAy = (raw[2] << 8) | raw[3];
    int16_t rawAz = (raw[4] << 8) | raw[5];
    int16_t rawGx = (raw[8] << 8) | raw[9];
    int16_t rawGy = (raw[10] << 8) | raw[11];
    int16_t rawGz = (raw[12] << 8) | raw[13];

    // Converter e remover offsets de calibração
    gx = (float)rawGx / 65.5f - gyroOffsetX;
    gy = (float)rawGy / 65.5f - gyroOffsetY;
    gz = (float)rawGz / 65.5f - gyroOffsetZ;

    ax = (float)rawAx / 8192.0f - accelOffsetX;
    ay = (float)rawAy / 8192.0f - accelOffsetY;
    az = (float)rawAz / 8192.0f - accelOffsetZ;
}

// ============================================================
//  BMP280 — Driver Bare-Metal I2C (Simplificado)
// ============================================================

// Coeficientes de calibração do BMP280 (lidos do chip)
static uint16_t bmp280_dig_T1;
static int16_t  bmp280_dig_T2, bmp280_dig_T3;
static uint16_t bmp280_dig_P1;
static int16_t  bmp280_dig_P2, bmp280_dig_P3, bmp280_dig_P4, bmp280_dig_P5;
static int16_t  bmp280_dig_P6, bmp280_dig_P7, bmp280_dig_P8, bmp280_dig_P9;
static int32_t  bmp280_t_fine;

static void bmp280WriteReg(uint8_t reg, uint8_t value)
{
    I2C_Slow.beginTransmission(BMP280_ADDR);
    I2C_Slow.write(reg);
    I2C_Slow.write(value);
    I2C_Slow.endTransmission();
}

static uint8_t bmp280ReadReg(uint8_t reg)
{
    I2C_Slow.beginTransmission(BMP280_ADDR);
    I2C_Slow.write(reg);
    I2C_Slow.endTransmission(false);
    I2C_Slow.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
    return I2C_Slow.read();
}

static void bmp280ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    I2C_Slow.beginTransmission(BMP280_ADDR);
    I2C_Slow.write(reg);
    I2C_Slow.endTransmission(false);
    I2C_Slow.requestFrom((uint8_t)BMP280_ADDR, len);
    for (uint8_t i = 0; i < len && I2C_Slow.available(); i++) {
        buf[i] = I2C_Slow.read();
    }
}

static void bmp280Init()
{
    // Ler chip ID para verificar comunicação
    uint8_t chipId = bmp280ReadReg(0xD0);
    if (chipId != 0x58) {
        Serial.print(F("[BMP] ✗ Chip ID incorreto: 0x"));
        Serial.println(chipId, HEX);
        return;
    }

    // Ler coeficientes de calibração (registros 0x88-0x9F)
    uint8_t calib[26];
    bmp280ReadRegs(0x88, calib, 26);
    
    bmp280_dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    bmp280_dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    bmp280_dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
    bmp280_dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    bmp280_dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    bmp280_dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    bmp280_dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    bmp280_dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    bmp280_dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    bmp280_dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    bmp280_dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    bmp280_dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

    // Configurar: oversampling ×4 temp, ×8 press, modo normal
    bmp280WriteReg(0xF4, 0x2F);  // ctrl_meas: osrs_t=×2, osrs_p=×4, mode=normal
    bmp280WriteReg(0xF5, 0x0C);  // config: t_sb=0.5ms, filter=×4, spi3w_en=0

    Serial.println(F("[BMP] ✓ BMP280 inicializado"));
}

/**
 * Lê temperatura e pressão compensadas do BMP280.
 * Usa as fórmulas de compensação do datasheet Bosch.
 */
static void bmp280Read(float& temperature, float& pressure)
{
    uint8_t data[6];
    bmp280ReadRegs(0xF7, data, 6);

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

    // Compensação de temperatura (datasheet BMP280)
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp280_dig_T1 << 1))) * ((int32_t)bmp280_dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp280_dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_dig_T1))) >> 12) * ((int32_t)bmp280_dig_T3)) >> 14;
    bmp280_t_fine = var1 + var2;
    temperature = (float)((bmp280_t_fine * 5 + 128) >> 8) / 100.0f;

    // Compensação de pressão (datasheet BMP280)
    int64_t v1 = ((int64_t)bmp280_t_fine) - 128000;
    int64_t v2 = v1 * v1 * (int64_t)bmp280_dig_P6;
    v2 = v2 + ((v1 * (int64_t)bmp280_dig_P5) << 17);
    v2 = v2 + (((int64_t)bmp280_dig_P4) << 35);
    v1 = ((v1 * v1 * (int64_t)bmp280_dig_P3) >> 8) + ((v1 * (int64_t)bmp280_dig_P2) << 12);
    v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)bmp280_dig_P1) >> 33;
    if (v1 == 0) { pressure = 0; return; }
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = (((int64_t)bmp280_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    v2 = (((int64_t)bmp280_dig_P8) * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (((int64_t)bmp280_dig_P7) << 4);
    pressure = (float)p / 256.0f;
}

/**
 * Converte pressão barométrica em altitude (fórmula barométrica).
 * Altitude relativa ao nível de referência (set no boot).
 */
static float pressureToAltitude(float pressure_Pa)
{
    // Fórmula barométrica internacional:
    // h = 44330 × (1 - (P/P0)^0.19029)
    constexpr float P0 = 101325.0f;  // Pressão ao nível do mar padrão
    return 44330.0f * (1.0f - powf(pressure_Pa / P0, 0.19029f));
}

// ============================================================
//  FILTRO NOTCH DINÂMICO — Rastreia RPM do motor via throttle
// ============================================================

/**
 * Filtro Notch DINÂMICO (rejeita-banda) de 2ª ordem.
 * 
 * PROBLEMA ORIGINAL:
 * O filtro Notch estático centrado em 150Hz só funcionava para
 * uma RPM específica (~9000 RPM). Motores brushless como o A2212
 * mudam drasticamente de RPM conforme o acelerador:
 *   - Idle (~10% throttle):  ~2000 RPM → vibração em ~67Hz  (2 pás)
 *   - Cruzeiro (~55%):       ~5000 RPM → vibração em ~167Hz
 *   - Full throttle (100%):  ~7500 RPM → vibração em ~250Hz
 * 
 * Um notch estático em 150Hz deixa passar vibrações em idle e
 * em potência alta, contaminando o giroscópio e causando
 * oscilações parasitas no PID.
 * 
 * SOLUÇÃO — Notch Dinâmico (inspirado no Betaflight RPM Filter):
 * 1. Estima a frequência de vibração a partir do comando de throttle
 *    usando modelo linear: freq = IDLE_FREQ + throttle × (MAX_FREQ - IDLE_FREQ)
 * 2. Recalcula os coeficientes IIR quando a frequência estimada muda
 *    significativamente (>5Hz de variação)
 * 3. Atualização limitada a 10Hz (a cada 25 ciclos de 250Hz) para
 *    evitar custo computacional excessivo (sin/cos no recálculo)
 * 4. Os estados do filtro (x1,x2,y1,y2) são preservados no recálculo
 *    para evitar transientes
 * 
 * MODELO A2212 COM HÉLICE 2 PÁS:
 *   Vibração predominante = (RPM / 60) × nº_pás
 *   KV × V_bateria = RPM_max (sem carga), mas com carga ~70%
 *   A2212 1000KV, 2S (7.4V): RPM_max ≈ 7400 → freq ≈ 247Hz
 *   Idle: ~2000 RPM → freq ≈ 67Hz
 */
struct DynamicNotchFilter {
    // Coeficientes do biquad IIR
    float b0, b1, b2, a1, a2;
    // Estados anteriores (preservados durante recálculo)
    float x1, x2, y1, y2;
    // Frequência atual do filtro (para detectar mudança significativa)
    float _currentFreq;
    float _sampleRate;
    float _bandwidth;

    void init(float centerFreq, float bandwidth, float sampleRate) {
        _currentFreq = centerFreq;
        _sampleRate  = sampleRate;
        _bandwidth   = bandwidth;
        x1 = x2 = y1 = y2 = 0.0f;
        recalcCoefficients(centerFreq);
    }

    /**
     * Recalcula coeficientes do biquad para nova frequência central.
     * CUSTO: 1× sinf() + 1× cosf() — chamado no máximo a 10Hz.
     * PRESERVA estados (x1,x2,y1,y2) para transição suave.
     */
    void recalcCoefficients(float centerFreq) {
        // Proteger contra frequências fora do range Nyquist
        if (centerFreq < 20.0f) centerFreq = 20.0f;
        if (centerFreq > _sampleRate * 0.45f) centerFreq = _sampleRate * 0.45f;
        
        _currentFreq = centerFreq;
        
        float w0 = 2.0f * PI * centerFreq / _sampleRate;
        float Q  = centerFreq / _bandwidth;
        if (Q < 1.0f) Q = 1.0f;  // Q mínimo para estabilidade
        float alpha = sinf(w0) / (2.0f * Q);

        b0 = 1.0f;
        b1 = -2.0f * cosf(w0);
        b2 = 1.0f;
        float a0 = 1.0f + alpha;
        a1 = -2.0f * cosf(w0);
        a2 = 1.0f - alpha;

        // Normalizar por a0
        b0 /= a0; b1 /= a0; b2 /= a0;
        a1 /= a0; a2 /= a0;
        // NÃO zerar x1,x2,y1,y2 — manter continuidade do sinal
    }

    /**
     * Atualiza a frequência central se a mudança for significativa.
     * Retorna true se houve recálculo.
     */
    bool updateFrequency(float newFreq) {
        if (fabsf(newFreq - _currentFreq) > 5.0f) {
            recalcCoefficients(newFreq);
            return true;
        }
        return false;
    }

    float apply(float input) {
        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = input;
        y2 = y1; y1 = output;
        return output;
    }
};

// 6 filtros notch dinâmicos: 1 por eixo (3 gyro + 3 accel)
static DynamicNotchFilter notchGx, notchGy, notchGz;
static DynamicNotchFilter notchAx, notchAy, notchAz;

// Constantes do modelo de vibração A2212 (1000KV) + hélice 1045 + Bateria Li-Ion 2S
// freq_vibração = (RPM / 60) × 2 (hélice de 2 pás)
// KV=1000 em 2S (7.4V) = 7400 RPM sem carga.
// Com hélice grande 1045 a RPM cai bastante (provavelmente ~5000 RPM máximo).
// Max vib freq ≈ 5000 / 60 * 2 = 166 Hz.
static constexpr float NOTCH_IDLE_FREQ_HZ   = 50.0f;   // ~1500 RPM em marcha lenta
static constexpr float NOTCH_MAX_FREQ_HZ    = 170.0f;  // ~5100 RPM em 100% acelerador
static constexpr float NOTCH_BANDWIDTH_HZ   = 25.0f;   // Largura de banda
static constexpr int   NOTCH_UPDATE_DIVIDER = 25;       // Atualizar a cada 25 ciclos (10Hz)
static uint32_t notchUpdateCounter = 0;

// ============================================================
//  AHRS — Mahony com Correção de Força Centrífuga
// ============================================================

/**
 * Filtro AHRS de Mahony — estimação de atitude.
 * 
 * O filtro de Mahony é preferido sobre o Madgwick para sistemas
 * embarcados porque:
 * 1. Menor custo computacional (sem iterações do gradiente)
 * 2. Ganho PI ajustável (Kp controla convergência, Ki compensa drift)
 * 3. Estável com IMU de baixo custo (MPU6050)
 * 
 * Correção de Força Centrífuga:
 * Em curvas, o acelerômetro mede a resultante de gravidade +
 * força centrífuga. Sem correção, o AHRS interpreta a centrífuga
 * como inclinação, causando erro de roll em curvas sustentadas.
 * 
 * A correção subtrai a componente centrífuga estimada:
 *   a_centripetal = ω × v (produto vetorial de taxa angular × velocidade)
 */

// Quaternion de atitude
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Ganhos do Mahony
static constexpr float MAHONY_KP = 2.0f;   // Proporcional (convergência)
static constexpr float MAHONY_KI = 0.005f;  // Integral (compensa gyro drift)

// Integral de erro do Mahony
static float mahonyIntX = 0.0f, mahonyIntY = 0.0f, mahonyIntZ = 0.0f;

/**
 * Atualiza o quaternion de atitude usando o filtro de Mahony.
 * 
 * @param gx,gy,gz  Taxas angulares (rad/s)
 * @param ax,ay,az  Acelerações (g, normalizadas)
 * @param dt        Intervalo de tempo (s)
 */
static void mahonyUpdate(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    // Normalizar acelerômetro
    float recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az + 1e-10f);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Direção estimada da gravidade (do quaternion atual)
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Erro = produto vetorial (acelerômetro medido × gravidade estimada)
    // Este erro indica QUANTO a estimativa está errada
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // Integrador PI (compensa drift do giroscópio a longo prazo)
    mahonyIntX += MAHONY_KI * ex * dt;
    mahonyIntY += MAHONY_KI * ey * dt;
    mahonyIntZ += MAHONY_KI * ez * dt;

    // Aplicar correção PI ao giroscópio
    gx += MAHONY_KP * ex + mahonyIntX;
    gy += MAHONY_KP * ey + mahonyIntY;
    gz += MAHONY_KP * ez + mahonyIntZ;

    // Integrar quaternion (equação diferencial do quaternion)
    float halfDt = 0.5f * dt;
    float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * halfDt;
    float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * halfDt;
    float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * halfDt;
    float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * halfDt;

    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    // Renormalizar quaternion (previne drift numérico)
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3 + 1e-10f);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

/**
 * Extrai ângulos de Euler do quaternion.
 * Convenção: Roll (X), Pitch (Y), Yaw (Z), rotações intrínsecas.
 */
static void quaternionToEuler(float& roll, float& pitch, float& yaw)
{
    // Roll (eixo X)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (eixo Y) — com proteção gimbal lock
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf(90.0f, sinp);  // Gimbal lock: ±90°
    } else {
        pitch = asinf(sinp) * RAD_TO_DEG;
    }

    // Yaw (eixo Z)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    if (yaw < 0.0f) yaw += 360.0f;  // Normalizar para [0, 360)
}

// ============================================================
//  KALMAN 1D — Fusão Altitude (Barômetro + Acelerômetro)
// ============================================================

/**
 * Filtro de Kalman 1D para estimação de altitude.
 * Estado: [altitude, velocidade_vertical]
 * Observação: altitude barométrica
 * Entrada de controle: aceleração vertical do IMU
 * 
 * O barômetro fornece altitude absoluta (lenta mas sem drift).
 * O acelerômetro fornece aceleração instantânea (rápida mas com drift).
 * O Kalman funde ambos, resultando em altitude suave e responsiva.
 */
static void kalmanAltUpdate(float baroAlt, float accelZ_world, float dt)
{
    // ── Variâncias do processo e observação ──
    constexpr float Q_alt = 0.01f;   // Incerteza do processo (altitude)
    constexpr float Q_vel = 0.1f;    // Incerteza do processo (velocidade)
    constexpr float R_baro = 0.5f;   // Incerteza da medição barométrica

    // ── Predição (modelo: integração cinemática) ──
    // x_pred = F × x + B × u
    // altitude += vz × dt + 0.5 × az × dt²
    // vz += az × dt
    kalman_alt += kalman_vz * dt + 0.5f * accelZ_world * dt * dt;
    kalman_vz  += accelZ_world * dt;

    // P_pred = F × P × F' + Q
    kalman_P[0][0] += dt * (kalman_P[1][0] + kalman_P[0][1] + dt * kalman_P[1][1]) + Q_alt;
    kalman_P[0][1] += dt * kalman_P[1][1];
    kalman_P[1][0] += dt * kalman_P[1][1];
    kalman_P[1][1] += Q_vel;

    // ── Atualização (com medição barométrica) ──
    // Inovação
    float y = baroAlt - kalman_alt;

    // S = H × P × H' + R
    float S = kalman_P[0][0] + R_baro;

    // Ganho de Kalman: K = P × H' / S
    float K0 = kalman_P[0][0] / S;
    float K1 = kalman_P[1][0] / S;

    // Atualizar estado
    kalman_alt += K0 * y;
    kalman_vz  += K1 * y;

    // Atualizar covariância: P = (I - K×H) × P
    float P00_new = (1.0f - K0) * kalman_P[0][0];
    float P01_new = (1.0f - K0) * kalman_P[0][1];
    float P10_new = kalman_P[1][0] - K1 * kalman_P[0][0];
    float P11_new = kalman_P[1][1] - K1 * kalman_P[0][1];
    kalman_P[0][0] = P00_new; kalman_P[0][1] = P01_new;
    kalman_P[1][0] = P10_new; kalman_P[1][1] = P11_new;
}

// ============================================================
//  NVS — Carregar/Salvar Parâmetros
// ============================================================

static void loadParamsFromNVS()
{
    preferences.begin("VANT_PARAM", true);  // true = read-only
    for (uint8_t i = 0; i < (uint8_t)ParamID::PARAM_COUNT; i++) {
        paramValues[i] = preferences.getFloat(PARAM_TABLE[i].nvsKey,
                                              PARAM_TABLE[i].defaultValue);
    }
    preferences.end();
    Serial.println(F("[NVS] Parâmetros carregados da Flash"));
}

static void saveParamsToNVS()
{
    preferences.begin("VANT_PARAM", false);  // false = read-write
    for (uint8_t i = 0; i < (uint8_t)ParamID::PARAM_COUNT; i++) {
        preferences.putFloat(PARAM_TABLE[i].nvsKey, paramValues[i]);
    }
    preferences.end();
    nvsFlushPending = false;
    Serial.println(F("[NVS] Parâmetros salvos na Flash"));
}

/**
 * Aplica os valores de paramValues[] aos controladores PID.
 * Chamado após carregar NVS e após receber tuning via LoRa.
 */
static void applyParamsToPIDs()
{
    rollRatePID.setGains(
        paramValues[(uint8_t)ParamID::ROLL_KP],
        paramValues[(uint8_t)ParamID::ROLL_KI],
        paramValues[(uint8_t)ParamID::ROLL_KD],
        paramValues[(uint8_t)ParamID::ROLL_FF]
    );
    pitchRatePID.setGains(
        paramValues[(uint8_t)ParamID::PITCH_KP],
        paramValues[(uint8_t)ParamID::PITCH_KI],
        paramValues[(uint8_t)ParamID::PITCH_KD],
        paramValues[(uint8_t)ParamID::PITCH_FF]
    );
    rollAnglePID.setKp(paramValues[(uint8_t)ParamID::ANGLE_KP]);
    pitchAnglePID.setKp(paramValues[(uint8_t)ParamID::ANGLE_KP]);
    
    tecs.setThrottleCruise(paramValues[(uint8_t)ParamID::THR_CRUISE]);
    l1.setL1Period(paramValues[(uint8_t)ParamID::L1_PERIOD]);
}

// ============================================================
//  ADC — Leitura de Tensão da Bateria
// ============================================================

/**
 * Lê a tensão da bateria via ADC com divisor de tensão.
 * Divisor de tensão típico: R1=10k, R2=3.3k → fator ≈ 4.03
 * Atenuação ADC: 11dB (range 0-3.3V no pino)
 * 
 * Vbat = ADC_voltage × divisor_factor
 */
static float readBatteryVoltage()
{
    // Média de 8 leituras para filtrar ruído ADC
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += analogRead(PIN_VBAT_ADC);
    }
    float adcAvg = (float)sum / 8.0f;
    
    // ESP32 ADC: 12-bit (0-4095), atenuação 11dB → range 0-3.3V
    float adcVoltage = adcAvg / 4095.0f * 3.3f;
    
    // Fator do divisor de tensão (R1=10k, R2=3.3k)
    // Vbat = Vadc × (R1+R2)/R2 = Vadc × 4.03
    constexpr float DIVIDER_FACTOR = 4.03f;
    return adcVoltage * DIVIDER_FACTOR;
}

// ============================================================
//  TASKS FreeRTOS
// ============================================================

/**
 * ═══════════════════════════════════════════════════════════
 *  Task_FlightControl — CORE 1 (250Hz, Prioridade 5)
 * ═══════════════════════════════════════════════════════════
 *  Malha de controle atitudinal crítica.
 *  NADA pode bloquear esta task. Jitter alvo: < 100µs.
 * 
 *  Pipeline por ciclo:
 *  1. Ler MPU6050 (I2C Fast, ~400µs)
 *  2. Filtrar Notch 150Hz (remove vibração motor)
 *  3. Mahony AHRS (estima atitude, ~50µs)
 *  4. Fusão COG Yaw (se GPS > 5m/s)
 *  5. FSM (verificar modo e Pre-Arm)
 *  6. Cascata PID Angle→Rate (calcular atuação)
 *  7. Mixer (comandos → µs)
 *  8. Output LEDC (escrever servos)
 */
static void Task_FlightControl(void* pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / FLIGHT_CTRL_HZ);  // 4ms

    // Variável para integração de velocidade inercial (TECS)
    float inertialSpeedInteg = 0.0f;

    while (true) {
        // ── 0) ATUALIZAÇÃO DO NOTCH DINÂMICO (10Hz) ──
        if (++notchUpdateCounter >= NOTCH_UPDATE_DIVIDER) {
            notchUpdateCounter = 0;
            float currentThrottle = 0.0f;
            if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
                // Se em modo automático, usar nav_throttle, senão rc_throttle
                currentThrottle = globalState.mode >= FlightMode::MODE_AUTO ? 
                                  globalState.nav_throttle : globalState.rc_throttle;
                xSemaphoreGive(stateMutex);
            }
            float targetFreq = NOTCH_IDLE_FREQ_HZ + currentThrottle * (NOTCH_MAX_FREQ_HZ - NOTCH_IDLE_FREQ_HZ);
            notchGx.updateFrequency(targetFreq);
            notchGy.updateFrequency(targetFreq);
            notchGz.updateFrequency(targetFreq);
            notchAx.updateFrequency(targetFreq);
            notchAy.updateFrequency(targetFreq);
            notchAz.updateFrequency(targetFreq);
        }

        // ── 1) LEITURA MPU6050 ──
        float gx_dps, gy_dps, gz_dps;  // Giroscópio (°/s)
        float ax_g, ay_g, az_g;        // Acelerômetro (g)
        mpu6050Read(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);

        // ── 2) FILTRO NOTCH DINÂMICO ──
        // Remove a frequência dominante de vibração do motor A2212.
        // Frequência é ajustada em tempo real baseada no throttle.
        gx_dps = notchGx.apply(gx_dps);
        gy_dps = notchGy.apply(gy_dps);
        gz_dps = notchGz.apply(gz_dps);
        ax_g = notchAx.apply(ax_g);
        ay_g = notchAy.apply(ay_g);
        az_g = notchAz.apply(az_g);

        // ── 3) MAHONY AHRS ──
        // Converter giroscópio de dps para rad/s (Mahony espera rad/s)
        float gx_rad = gx_dps * DEG_TO_RAD;
        float gy_rad = gy_dps * DEG_TO_RAD;
        float gz_rad = gz_dps * DEG_TO_RAD;

        // Correção de força centrífuga em curvas:
        // Em curva sustentada, o acelerômetro mede g + centrífuga.
        // a_centripetal = v × ω_yaw (simplificado para body frame)
        // Subtrair a componente centrífuga estimada do accel Y.
        float groundSpeedLocal = 0.0f;
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
            groundSpeedLocal = globalState.groundSpeed_ms;
            xSemaphoreGive(stateMutex);
        }
        // Correção centrífuga no eixo Y (lateral)
        // a_centripetal = V × ωz (taxa de yaw em rad/s × velocidade)
        float centripetal_g = groundSpeedLocal * gz_rad / GRAVITY;
        float ay_corrected = ay_g - centripetal_g;

        mahonyUpdate(gx_rad, gy_rad, gz_rad,
                     ax_g, ay_corrected, az_g,
                     FLIGHT_CTRL_DT);

        float roll_deg, pitch_deg, yaw_deg;
        quaternionToEuler(roll_deg, pitch_deg, yaw_deg);

        // ── 4) FUSÃO COG YAW (Bússola Cega) ──
        // Sem magnetômetro, o yaw do giroscópio sofre drift.
        // Quando voando acima de 5 m/s, o Course Over Ground (COG)
        // do GPS fornece a direção real de deslocamento.
        // Filtro complementar suave: mistura yaw inercial com COG.
        float cogDegLocal = 0.0f;
        float gsLocal = 0.0f;
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
            cogDegLocal = globalState.cogDeg;
            gsLocal = globalState.groundSpeed_ms;
            xSemaphoreGive(stateMutex);
        }

        if (gsLocal > MIN_COG_FUSION_SPEED) {
            // Filtro complementar: τ ≈ 2s (suave para não oscilar)
            // O COG é ruidoso em voo turbulento, então usamos peso baixo.
            constexpr float COG_ALPHA = 0.02f;  // ~2% por ciclo a 250Hz
            float yawError = wrapAngle180(cogDegLocal - yaw_deg);
            yaw_deg += COG_ALPHA * yawError;
            yaw_deg = wrapAngle360(yaw_deg);
        }

        // ── Integração de velocidade inercial (para TECS anti-estol) ──
        // Integrar aceleração longitudinal (eixo X body) para estimar
        // velocidade do avião sem depender do GPS.
        // Decay exponencial para limitar drift (~5s time constant)
        float accelLongitudinal = ax_g * GRAVITY;  // m/s²
        inertialSpeedInteg += accelLongitudinal * FLIGHT_CTRL_DT;
        inertialSpeedInteg *= 0.998f;  // Decay (~2s time constant a 250Hz)

        // ── Atualizar FlightState com dados do AHRS ──
        FlightMode currentMode = FlightMode::MODE_MANUAL;
        ArmState   currentArm  = ArmState::DISARMED;
        float rcRoll = 0.0f, rcPitch = 0.0f, rcThrottle = 0.0f;
        float navRollDeg = 0.0f, navPitchDeg = 0.0f, navThrottle = 0.0f;
        bool  mixerWasSaturated = false;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            globalState.roll_deg      = roll_deg;
            globalState.pitch_deg     = pitch_deg;
            globalState.yaw_deg       = yaw_deg;
            globalState.gyro_roll_dps = gx_dps;
            globalState.gyro_pitch_dps = gy_dps;
            globalState.gyro_yaw_dps  = gz_dps;
            globalState.accel_x       = ax_g * GRAVITY;
            globalState.accel_y       = ay_g * GRAVITY;
            globalState.accel_z       = az_g * GRAVITY;
            globalState.inertialSpeed_ms = fabsf(inertialSpeedInteg);
            
            // Ler RC e modo (escritos pela Task_LoRa)
            currentMode  = globalState.mode;
            currentArm   = globalState.armState;
            rcRoll       = globalState.rc_roll;
            rcPitch      = globalState.rc_pitch;
            rcThrottle   = globalState.rc_throttle;
            navRollDeg   = globalState.nav_roll_deg;
            navPitchDeg  = globalState.nav_pitch_deg;
            navThrottle  = globalState.nav_throttle;
            mixerWasSaturated = globalState.mixerSaturated;
            
            xSemaphoreGive(stateMutex);
        }

        // ── 5) FSM — Gerenciamento de Modo ──
        bool armRequest = (currentArm == ArmState::ARMED);

        // Copiar estado local para verificação da FSM
        FlightState localState;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            localState = globalState;
            xSemaphoreGive(stateMutex);
        }

        fsmManager.update(currentMode, armRequest, localState,
                          rollRatePID, pitchRatePID,
                          rollAnglePID, pitchAnglePID);

        FlightMode activeMode = fsmManager.getCurrentMode();
        ArmState   activeArm  = fsmManager.getArmState();

        // ── 6) CASCATA PID / COMANDO ──
        float rollCmd = 0.0f, pitchCmd = 0.0f, thrCmd = 0.0f;

        // TPA (Throttle PID Attenuation)
        float tpaBrkpt = paramValues[(uint8_t)ParamID::TPA_BREAKPOINT];
        float tpaFactor = 1.0f;
        if (rcThrottle > tpaBrkpt) {
            // Atenuar Kp linearmente de 1.0 até 0.5 entre breakpoint e 100%
            tpaFactor = 1.0f - 0.5f * (rcThrottle - tpaBrkpt) / (1.0f - tpaBrkpt);
            if (tpaFactor < 0.5f) tpaFactor = 0.5f;
        }

        switch (activeMode) {
            case FlightMode::MODE_MANUAL: {
                // ── Bypass total: RC → Servo direto, sem PID ──
                rollCmd  = rcRoll;
                pitchCmd = rcPitch;
                thrCmd   = rcThrottle;
                break;
            }

            case FlightMode::MODE_ANGLE: {
                // ── Estabilizado: Piloto comanda ângulo, PID estabiliza ──
                // RC normalizado [-1,+1] → ângulo alvo [±45°] (roll), [±30°] (pitch)
                float rollTarget  = rcRoll * 45.0f;   // ±45° max roll
                float pitchTarget = rcPitch * 30.0f;  // ±30° max pitch

                // Angle loop (externo): erro de ângulo → taxa desejada (dps)
                float rollRateTarget = rollAnglePID.update(
                    rollTarget, roll_deg, FLIGHT_CTRL_DT);
                float pitchRateTarget = pitchAnglePID.update(
                    pitchTarget, pitch_deg, FLIGHT_CTRL_DT);

                // Rate loop (interno): erro de taxa → comando servo
                rollCmd = rollRatePID.update(
                    rollRateTarget, gx_dps, FLIGHT_CTRL_DT,
                    tpaFactor, mixerWasSaturated);
                pitchCmd = pitchRatePID.update(
                    pitchRateTarget, gy_dps, FLIGHT_CTRL_DT,
                    tpaFactor, mixerWasSaturated);

                thrCmd = rcThrottle;
                break;
            }

            case FlightMode::MODE_HOLD: {
                // ── Hold com override de piloto ──
                // Se piloto move stick → comportamento ANGLE
                // Se piloto não toca → L1/TECS controlam
                if (FlightModeManager::isStickOverride(rcRoll, rcPitch)) {
                    // Override: piloto tem controle (mesma lógica que ANGLE)
                    float rollTarget  = rcRoll * 45.0f;
                    float pitchTarget = rcPitch * 30.0f;

                    float rollRateTarget = rollAnglePID.update(
                        rollTarget, roll_deg, FLIGHT_CTRL_DT);
                    float pitchRateTarget = pitchAnglePID.update(
                        pitchTarget, pitch_deg, FLIGHT_CTRL_DT);

                    rollCmd = rollRatePID.update(
                        rollRateTarget, gx_dps, FLIGHT_CTRL_DT,
                        tpaFactor, mixerWasSaturated);
                    pitchCmd = pitchRatePID.update(
                        pitchRateTarget, gy_dps, FLIGHT_CTRL_DT,
                        tpaFactor, mixerWasSaturated);

                    thrCmd = rcThrottle;
                } else {
                    // Navegação: L1/TECS controlam
                    float rollRateTarget = rollAnglePID.update(
                        navRollDeg, roll_deg, FLIGHT_CTRL_DT);
                    float pitchRateTarget = pitchAnglePID.update(
                        navPitchDeg, pitch_deg, FLIGHT_CTRL_DT);

                    rollCmd = rollRatePID.update(
                        rollRateTarget, gx_dps, FLIGHT_CTRL_DT,
                        tpaFactor, mixerWasSaturated);
                    pitchCmd = pitchRatePID.update(
                        pitchRateTarget, gy_dps, FLIGHT_CTRL_DT,
                        tpaFactor, mixerWasSaturated);

                    thrCmd = navThrottle;
                }
                break;
            }

            case FlightMode::MODE_AUTO:
            case FlightMode::MODE_RTH: {
                // ── Navegação autônoma: manche ignorado ──
                float rollRateTarget = rollAnglePID.update(
                    navRollDeg, roll_deg, FLIGHT_CTRL_DT);
                float pitchRateTarget = pitchAnglePID.update(
                    navPitchDeg, pitch_deg, FLIGHT_CTRL_DT);

                rollCmd = rollRatePID.update(
                    rollRateTarget, gx_dps, FLIGHT_CTRL_DT,
                    tpaFactor, mixerWasSaturated);
                pitchCmd = pitchRatePID.update(
                    pitchRateTarget, gy_dps, FLIGHT_CTRL_DT,
                    tpaFactor, mixerWasSaturated);

                thrCmd = navThrottle;
                break;
            }
        }

        // ── 7) MIXER ──
        MixerOutput mOut;
        if (activeMode == FlightMode::MODE_MANUAL) {
            mOut = mixer.mixManual(rollCmd, pitchCmd, thrCmd);
        } else {
            mOut = mixer.mix(rollCmd, pitchCmd, thrCmd);
        }

        // ── 8) OUTPUT LEDC ──
        bool isArmed = (activeArm == ArmState::ARMED);
        outputMgr.write(mOut.ailLeftUs, mOut.ailRightUs,
                        mOut.elevatorUs, mOut.motorUs,
                        isArmed);

        // ── Atualizar estado global com saídas e saturação ──
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
            globalState.cmd_roll     = rollCmd;
            globalState.cmd_pitch    = pitchCmd;
            globalState.cmd_throttle = thrCmd;
            globalState.mode         = activeMode;
            globalState.armState     = activeArm;
            globalState.failsafe     = fsmManager.getFailsafeState();
            globalState.mixerSaturated = mOut.saturated;
            xSemaphoreGive(stateMutex);
        }

        // ── Dormir até o próximo ciclo (250Hz timing preciso) ──
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
 * ═══════════════════════════════════════════════════════════
 *  Task_Navigation — CORE 0 (50Hz, Prioridade 4)
 * ═══════════════════════════════════════════════════════════
 *  Navegação: Kalman altitude, L1 Guidance, TECS.
 *  Lê barômetro e calcula comandos de navegação.
 */
static void Task_Navigation(void* pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / NAV_HZ);  // 20ms

    while (true) {
        // ── Ler BMP280 ──
        float temp, press;
        bmp280Read(temp, press);
        float baroAlt = pressureToAltitude(press);

        // Referência de altitude (first run)
        if (!bmp280_refSet) {
            bmp280_altRef_m = baroAlt;
            bmp280_refSet = true;
            kalman_alt = 0.0f;
        }
        float relAlt = baroAlt - bmp280_altRef_m;

        // ── Ler aceleração vertical (world frame) do estado global ──
        float accelZ_world = 0.0f;
        float gsMs = 0.0f, inertialSpdMs = 0.0f;
        double curLat = 0, curLon = 0;
        double homeLat = 0, homeLon = 0;
        float homeAlt = 0.0f;
        bool  homeSet = false;
        FlightMode curMode = FlightMode::MODE_MANUAL;
        uint8_t wpCurrent = 0;
        FlightState::Waypoint wpTarget, wpPrev;
        wpTarget.lat = 0; wpTarget.lon = 0; wpTarget.alt_m = 0; wpTarget.speed_ms = 12; wpTarget.valid = false;
        wpPrev = wpTarget;
        float cogDeg = 0.0f;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            // Aceleração Z no body frame → world frame (simplificado)
            // Para ângulos pequenos (~< 30°): az_world ≈ az_body - g
            accelZ_world = globalState.accel_z - GRAVITY;
            gsMs = globalState.groundSpeed_ms;
            inertialSpdMs = globalState.inertialSpeed_ms;
            curLat = globalState.gps_lat;
            curLon = globalState.gps_lon;
            homeLat = globalState.home_lat;
            homeLon = globalState.home_lon;
            homeAlt = globalState.home_alt_m;
            homeSet = globalState.home_set;
            curMode = globalState.mode;
            wpCurrent = globalState.wp_current;
            cogDeg = globalState.cogDeg;
            if (wpCurrent < FlightState::MAX_WAYPOINTS) {
                wpTarget = globalState.waypoints[wpCurrent];
            }
            if (wpCurrent > 0 && (wpCurrent - 1) < FlightState::MAX_WAYPOINTS) {
                wpPrev = globalState.waypoints[wpCurrent - 1];
            }
            xSemaphoreGive(stateMutex);
        }

        // ── Kalman 1D — Fusão Baro + Accel ──
        kalmanAltUpdate(relAlt, accelZ_world, NAV_DT);

        // ── Calcular comandos de navegação (se modo requer) ──
        float navRoll = 0.0f, navPitch = 0.0f, navThr = 0.5f;

        if (curMode == FlightMode::MODE_HOLD ||
            curMode == FlightMode::MODE_AUTO ||
            curMode == FlightMode::MODE_RTH) {

            float targetAlt = kalman_alt;  // Default: manter altitude atual
            float targetSpd = 12.0f;       // Default: 12 m/s

            if (curMode == FlightMode::MODE_RTH && homeSet) {
                // ── RTH: orbitar sobre Home ──
                navRoll = l1.updateLoiter(curLat, curLon,
                                          homeLat, homeLon,
                                          50.0f,  // raio 50m
                                          gsMs, cogDeg);
                targetAlt = homeAlt + 30.0f;  // 30m acima de Home
            } else if (curMode == FlightMode::MODE_AUTO && wpTarget.valid) {
                // ── AUTO: seguir waypoints ──
                double prevLatUse = wpPrev.valid ? wpPrev.lat : curLat;
                double prevLonUse = wpPrev.valid ? wpPrev.lon : curLon;

                navRoll = l1.update(curLat, curLon,
                                    wpTarget.lat, wpTarget.lon,
                                    prevLatUse, prevLonUse,
                                    gsMs, cogDeg);

                targetAlt = wpTarget.alt_m;
                targetSpd = wpTarget.speed_ms;

                // Verificar se waypoint alcançado
                if (l1.waypointReached(curLat, curLon,
                                       wpTarget.lat, wpTarget.lon, 30.0f)) {
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                        if (globalState.wp_current < globalState.wp_count - 1) {
                            globalState.wp_current++;
                        }
                        // Se último WP, manter loiter no ponto
                        xSemaphoreGive(stateMutex);
                    }
                }
            } else if (curMode == FlightMode::MODE_HOLD) {
                // ── HOLD: manter rumo atual e altitude ──
                navRoll = 0.0f;  // Wings level
            }

            // ── TECS — Controle de energia ──
            tecs.update(targetAlt, targetSpd,
                        kalman_alt, kalman_vz,
                        gsMs, inertialSpdMs,
                        NAV_DT,
                        navPitch, navThr);
        }

        // ── Atualizar estado global com resultados de navegação ──
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            globalState.altitude_m     = kalman_alt;
            globalState.vario_ms       = kalman_vz;
            globalState.nav_roll_deg   = navRoll;
            globalState.nav_pitch_deg  = navPitch;
            globalState.nav_throttle   = navThr;
            xSemaphoreGive(stateMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
 * ═══════════════════════════════════════════════════════════
 *  Task_GPS — CORE 0 (Assíncrono, Prioridade 3)
 * ═══════════════════════════════════════════════════════════
 *  Lê sentenças NMEA da serial do NEO-6M e parseia via TinyGPS++.
 *  Assíncrono: roda tão rápido quanto os dados chegam (~1-5Hz do GPS).
 */
static void Task_GPS(void* pvParameters)
{
    (void)pvParameters;

    while (true) {
        // ── Ler todos os bytes disponíveis na serial GPS ──
        while (SerialGPS.available() > 0) {
            char c = SerialGPS.read();
            gps.encode(c);
        }

        // ── Atualizar estado global se fix válido ──
        if (gps.location.isUpdated()) {
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                globalState.gps_lat       = gps.location.lat();
                globalState.gps_lon       = gps.location.lng();
                globalState.gps_alt_m     = gps.altitude.meters();
                globalState.gps_sats      = gps.satellites.value();
                globalState.gps_fix       = gps.location.isValid() && (gps.satellites.value() >= 4);
                globalState.groundSpeed_ms = gps.speed.mps();
                globalState.cogDeg        = gps.course.deg();

                // ── Gravar Home na primeira fixação ──
                if (!globalState.home_set && globalState.gps_fix) {
                    globalState.home_lat  = globalState.gps_lat;
                    globalState.home_lon  = globalState.gps_lon;
                    globalState.home_alt_m = globalState.gps_alt_m;
                    globalState.home_set  = true;
                    Serial.println(F("[GPS] ✓ Home gravado"));
                }

                xSemaphoreGive(stateMutex);
            }
        }

        // Yield para outras tasks (GPS é assíncrono, não precisa
        // de timing fixo — roda quando há dados na serial)
        vTaskDelay(pdMS_TO_TICKS(10));  // ~100Hz polling
    }
}

/**
 * ═══════════════════════════════════════════════════════════
 *  Task_LoRa — CORE 0 (10Hz, Prioridade 3)
 * ═══════════════════════════════════════════════════════════
 *  Comunicação LoRa bidirecional:
 *  - Recebe: RC (0xBB), Waypoints (0xCC), Tuning (0xDD)
 *  - Transmite: Telemetria (0xAA) a cada ciclo
 */
static void Task_LoRa(void* pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / LORA_HZ);  // 100ms

    while (true) {
        // ── Processar pacotes recebidos ──
        loraMgr.processIncoming();

        // ── Transmitir telemetria ──
        loraMgr.sendTelemetry();

        // ── Aplicar parâmetros atualizados aos PIDs ──
        // (Se tuning LoRa recebido neste ciclo)
        if (nvsFlushPending) {
            applyParamsToPIDs();
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
 * ═══════════════════════════════════════════════════════════
 *  Task_System_Mon — CORE 0 (1Hz, Prioridade 1)
 * ═══════════════════════════════════════════════════════════
 *  Monitoramento de sistema:
 *  - Watchdog de failsafe (timeout LoRa)
 *  - Leitura de tensão da bateria
 *  - Persistência NVS (flush de parâmetros para Flash)
 */
static void Task_System_Mon(void* pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / SYSTEM_MON_HZ);  // 1000ms

    while (true) {
        // ── 1) Leitura de tensão da bateria ──
        float vbat = readBatteryVoltage();
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            globalState.vbat_V = vbat;
            xSemaphoreGive(stateMutex);
        }

        // ── 2) Failsafe Watchdog ──
        FlightState localState;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            localState = globalState;
            xSemaphoreGive(stateMutex);
        }
        fsmManager.checkFailsafe(localState,
                                  rollRatePID, pitchRatePID,
                                  rollAnglePID, pitchAnglePID);

        // Atualizar modo/failsafe no estado global
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            globalState.mode     = fsmManager.getCurrentMode();
            globalState.failsafe = fsmManager.getFailsafeState();
            xSemaphoreGive(stateMutex);
        }

        // ── 3) Persistência NVS ──
        // Gravar parâmetros na Flash apenas quando há pendência.
        // A gravação é feita aqui (1Hz, baixa prioridade) para
        // não bloquear a malha de controle. A Flash tem tempo de
        // escrita de ~10ms, inaceitável no Core 1.
        if (nvsFlushPending) {
            saveParamsToNVS();
        }

        // ── 4) Heartbeat serial (debug) ──
        Serial.print(F("[MON] Vbat="));
        Serial.print(vbat, 1);
        Serial.print(F("V Mode="));
        Serial.print((uint8_t)fsmManager.getCurrentMode());
        Serial.print(F(" Arm="));
        Serial.print((uint8_t)fsmManager.getArmState());
        Serial.print(F(" FS="));
        Serial.print((uint8_t)fsmManager.getFailsafeState());
        Serial.print(F(" GPS="));
        Serial.print(localState.gps_sats);
        Serial.print(F("sat Alt="));
        Serial.print(localState.altitude_m, 1);
        Serial.println(F("m"));

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================
//  SETUP & LOOP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println(F(""));
    Serial.println(F("╔═══════════════════════════════════════════════╗"));
    Serial.println(F("║  VANT Flight Controller v1.0                 ║"));
    Serial.println(F("║  Twin-Boom Tractor — ESP32 Dual-Core         ║"));
    Serial.println(F("║  GNC: Mahony AHRS + L1 + TECS Híbrido       ║"));
    Serial.println(F("╚═══════════════════════════════════════════════╝"));
    Serial.println(F(""));

    // ── 1) Inicializar ADC ──
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(PIN_VBAT_ADC, INPUT);

    // ── 2) Inicializar I2C Fast (Core 1 — MPU6050) ──
    Wire.begin(PIN_I2C_FAST_SDA, PIN_I2C_FAST_SCL);
    Wire.setClock(I2C_FAST_FREQ);
    mpu6050Init();
    mpu6050Calibrate();  // Auto-calibração (500 amostras estáticas)

    // ── 3) Inicializar I2C Slow (Core 0 — BMP280) ──
    I2C_Slow.begin(PIN_I2C_SLOW_SDA, PIN_I2C_SLOW_SCL);
    I2C_Slow.setClock(I2C_SLOW_FREQ);
    bmp280Init();

    // ── 4) Inicializar UART GPS ──
    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial.println(F("[GPS] UART2 inicializado (9600 baud)"));

    // ── 5) Inicializar LoRa ──
    if (!loraMgr.init()) {
        Serial.println(F("[FATAL] LoRa falhou! Sistema continuará sem link."));
    }

    // ── 6) Inicializar Saídas PWM (LEDC) ──
    outputMgr.init();

    // ── 7) Inicializar NVS — Carregar parâmetros persistidos ──
    loadParamsFromNVS();
    applyParamsToPIDs();

    // ── 8) Inicializar Filtros Notch Dinâmicos (BW=25Hz, Fs=250Hz) ──
    // Inicializa na frequência de idle (67Hz)
    notchGx.init(NOTCH_IDLE_FREQ_HZ, NOTCH_BANDWIDTH_HZ, (float)FLIGHT_CTRL_HZ);
    notchGy.init(NOTCH_IDLE_FREQ_HZ, NOTCH_BANDWIDTH_HZ, (float)FLIGHT_CTRL_HZ);
    notchGz.init(NOTCH_IDLE_FREQ_HZ, NOTCH_BANDWIDTH_HZ, (float)FLIGHT_CTRL_HZ);
    notchAx.init(NOTCH_IDLE_FREQ_HZ, NOTCH_BANDWIDTH_HZ, (float)FLIGHT_CTRL_HZ);
    notchAy.init(NOTCH_IDLE_FREQ_HZ, NOTCH_BANDWIDTH_HZ, (float)FLIGHT_CTRL_HZ);
    notchAz.init(NOTCH_IDLE_FREQ_HZ, NOTCH_BANDWIDTH_HZ, (float)FLIGHT_CTRL_HZ);
    Serial.println(F("[NOTCH] Filtros dinâmicos inicializados"));

    // ── 9) Inicializar Subsistemas ──
    fsmManager.init();
    tecs.init();
    l1.init();

    // ── 10) Inicializar FlightState ──
    memset(&globalState, 0, sizeof(FlightState));
    globalState.mode     = FlightMode::MODE_MANUAL;
    globalState.armState = ArmState::DISARMED;
    globalState.failsafe = FailsafeState::NOMINAL;

    // ── 11) Criar Mutex ──
    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL) {
        Serial.println(F("[FATAL] Falha ao criar Mutex!"));
        while (true) { delay(1000); }
    }

    // ── 12) Criar Tasks FreeRTOS ──
    Serial.println(F("[RTOS] Criando tasks..."));

    // CORE 1: Controle Atitudinal (250Hz, Prio 5, Stack 8KB)
    xTaskCreatePinnedToCore(
        Task_FlightControl,
        "FlightCtrl",
        8192,             // Stack size (bytes)
        NULL,             // Parâmetro
        5,                // Prioridade (máxima)
        NULL,             // Handle (não necessário)
        1                 // Core 1
    );

    // CORE 0: Navegação (50Hz, Prio 4, Stack 4KB)
    xTaskCreatePinnedToCore(
        Task_Navigation,
        "Navigation",
        4096,
        NULL,
        4,
        NULL,
        0
    );

    // CORE 0: GPS (Async, Prio 3, Stack 2KB)
    xTaskCreatePinnedToCore(
        Task_GPS,
        "GPS",
        2048,
        NULL,
        3,
        NULL,
        0
    );

    // CORE 0: LoRa (10Hz, Prio 3, Stack 4KB)
    xTaskCreatePinnedToCore(
        Task_LoRa,
        "LoRa",
        4096,
        NULL,
        3,
        NULL,
        0
    );

    // CORE 0: System Monitor (1Hz, Prio 1, Stack 2KB)
    xTaskCreatePinnedToCore(
        Task_System_Mon,
        "SysMon",
        2048,
        NULL,
        1,
        NULL,
        0
    );

    Serial.println(F("[RTOS] ✓ Todas as tasks criadas"));
    Serial.println(F("[INIT] ✓ Sistema inicializado — Aguardando ARM"));
    Serial.println(F(""));
}

void loop()
{
    // loop() do Arduino é uma task de baixa prioridade no Core 1.
    // Todas as operações são feitas nas tasks FreeRTOS.
    // Deletar esta task para liberar recursos.
    vTaskDelete(NULL);
}

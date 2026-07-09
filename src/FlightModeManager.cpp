/**
 * ============================================================
 *  FlightModeManager.cpp — Implementação da FSM do VANT
 * ============================================================
 *  Máquina de Estados com Pre-Arm Checks rigorosos,
 *  Failsafe Watchdog e Bumpless Transfer.
 * 
 *  FILOSOFIA DE SEGURANÇA (ArduPilot-Style):
 *  - NUNCA armar sem GPS fix (impossível RTH de emergência)
 *  - NUNCA armar com bateria fraca (risco de brownout em voo)
 *  - NUNCA armar com aeronave inclinada (IMU possivelmente
 *    descalibrada ou avião em posição instável)
 *  - Timeout LoRa = RTH automático (perda de link = emergência)
 * 
 *  BUMPLESS TRANSFER:
 *  Ao transitar entre modos, os integradores PID são zerados.
 *  Sem isso, o integrador acumulado em um modo causa um "coice"
 *  (transient spike) ao entrar no próximo modo. Exemplo:
 *  Em MODE_ANGLE com vento lateral, o integrador compensa o trim.
 *  Ao ir para MANUAL e voltar para ANGLE, o integrador antigo
 *  aplicaria um trim incorreto para a condição atual.
 * ============================================================
 */

#include "FlightModeManager.h"

FlightModeManager::FlightModeManager()
    : _currentMode(FlightMode::MODE_MANUAL),
      _previousMode(FlightMode::MODE_MANUAL),
      _armState(ArmState::DISARMED),
      _failsafeState(FailsafeState::NOMINAL),
      _preArmFailReason("Não verificado")
{
}

void FlightModeManager::init()
{
    _currentMode    = FlightMode::MODE_MANUAL;
    _previousMode   = FlightMode::MODE_MANUAL;
    _armState       = ArmState::DISARMED;
    _failsafeState  = FailsafeState::NOMINAL;
    _preArmFailReason = "Aguardando ARM";
    Serial.println(F("[FSM] Inicializado: DISARMED / MANUAL"));
}

void FlightModeManager::update(FlightMode requestedMode, bool requestArm,
                               const FlightState& state,
                               PIDController& rollRatePID, PIDController& pitchRatePID,
                               PIDController& rollAnglePID, PIDController& pitchAnglePID)
{
    // ════════════════════════════════════════════════════════
    //  1) GERENCIAMENTO DE ARMAMENTO
    // ════════════════════════════════════════════════════════
    if (requestArm && _armState == ArmState::DISARMED) {
        // ── Tentativa de ARM: executar Pre-Arm Checks ──
        if (runPreArmChecks(state)) {
            _armState = ArmState::ARMED;
            _failsafeState = FailsafeState::NOMINAL;
            Serial.println(F("[FSM] ✓ ARMED — Pre-Arm Checks OK"));
        } else {
            Serial.print(F("[FSM] ✗ ARM REJEITADO: "));
            Serial.println(_preArmFailReason);
            // Permanece DISARMED
        }
    } else if (!requestArm && _armState == ArmState::ARMED) {
        // ── Pedido de DISARM ──
        _armState = ArmState::DISARMED;
        _currentMode = FlightMode::MODE_MANUAL;
        _failsafeState = FailsafeState::NOMINAL;
        bumplessTransfer(rollRatePID, pitchRatePID, rollAnglePID, pitchAnglePID);
        Serial.println(F("[FSM] DISARMED"));
    }

    // ════════════════════════════════════════════════════════
    //  2) TRANSIÇÃO DE MODO (só se armado)
    // ════════════════════════════════════════════════════════
    if (_armState == ArmState::ARMED) {
        // Não permitir mudança de modo durante failsafe crítico
        // (o failsafe já forçou RTH, piloto precisa desarmar)
        if (_failsafeState == FailsafeState::CRITICAL &&
            requestedMode != FlightMode::MODE_RTH) {
            // Ignorar — manter RTH de emergência
            return;
        }

        if (requestedMode != _currentMode) {
            _previousMode = _currentMode;
            _currentMode  = requestedMode;

            // ── Bumpless Transfer em toda transição ──
            bumplessTransfer(rollRatePID, pitchRatePID, rollAnglePID, pitchAnglePID);

            Serial.print(F("[FSM] Modo: "));
            Serial.print((uint8_t)_previousMode);
            Serial.print(F(" → "));
            Serial.println((uint8_t)_currentMode);
        }
    }
}

void FlightModeManager::checkFailsafe(const FlightState& state,
                                      PIDController& rollRatePID, PIDController& pitchRatePID,
                                      PIDController& rollAnglePID, PIDController& pitchAnglePID)
{
    // ════════════════════════════════════════════════════════
    //  FAILSAFE: TIMEOUT DO LORA (Perda de Link)
    // ════════════════════════════════════════════════════════
    // Se não recebemos nenhum pacote LoRa dentro do timeout,
    // significa que perdemos o link de rádio. O VANT deve
    // retornar autonomamente ao ponto de Home.
    //
    // O timeout é parametrizável via NVS (default 1500ms).
    // Em áreas com interferência RF, pode ser aumentado.
    
    if (_armState != ArmState::ARMED) {
        _failsafeState = FailsafeState::NOMINAL;
        return;
    }

    uint32_t loraTimeout = (uint32_t)paramValues[(uint8_t)ParamID::LORA_TIMEOUT];
    uint32_t elapsed = millis() - state.lastLoRaRxMs;

    if (elapsed > loraTimeout * 2) {
        // ── CRÍTICO: Perda total de link — RTH forçado ──
        if (_failsafeState != FailsafeState::CRITICAL) {
            _failsafeState = FailsafeState::CRITICAL;
            _previousMode = _currentMode;
            _currentMode = FlightMode::MODE_RTH;
            bumplessTransfer(rollRatePID, pitchRatePID, rollAnglePID, pitchAnglePID);
            Serial.println(F("[FAILSAFE] ⚠ CRÍTICO — Perda de link LoRa → RTH"));
        }
    } else if (elapsed > loraTimeout) {
        // ── AVISO: Link degradado ──
        if (_failsafeState == FailsafeState::NOMINAL) {
            _failsafeState = FailsafeState::WARN;
            Serial.println(F("[FAILSAFE] Link LoRa degradado"));
        }
    } else {
        // ── Link OK ──
        if (_failsafeState == FailsafeState::WARN) {
            _failsafeState = FailsafeState::NOMINAL;
            Serial.println(F("[FAILSAFE] Link LoRa restaurado"));
        }
        // Se estava CRITICAL e voltou, manter RTH até piloto
        // explicitamente mudar de modo (decisão de segurança)
    }

    // ════════════════════════════════════════════════════════
    //  FAILSAFE: BATERIA CRÍTICA EM VOO
    // ════════════════════════════════════════════════════════
    // Se Vbat cai abaixo de 6.0V em voo (Li-ion 2S, 3.0V por célula sob carga),
    // a aeronave não terá energia para voar por muito mais tempo.
    constexpr float VBAT_CRITICAL_INFLIGHT = 6.0f;
    if (state.vbat_V > 0.5f && state.vbat_V < VBAT_CRITICAL_INFLIGHT) {
        if (_failsafeState != FailsafeState::CRITICAL) {
            _failsafeState = FailsafeState::CRITICAL;
            _previousMode = _currentMode;
            _currentMode = FlightMode::MODE_RTH;
            bumplessTransfer(rollRatePID, pitchRatePID, rollAnglePID, pitchAnglePID);
            Serial.print(F("[FAILSAFE] ⚠ Vbat CRÍTICO: "));
            Serial.print(state.vbat_V, 1);
            Serial.println(F("V → RTH"));
        }
    }
}

bool FlightModeManager::runPreArmChecks(const FlightState& state)
{
    // ════════════════════════════════════════════════════════
    //  PRE-ARM CHECK 1: GPS FIX
    // ════════════════════════════════════════════════════════
    // Sem GPS fix, o VANT não pode executar RTH de emergência.
    // Armar sem GPS é permitido APENAS em bancada (não implementado).
    if (!state.gps_fix) {
        _preArmFailReason = "GPS sem fix";
        return false;
    }

    // ════════════════════════════════════════════════════════
    //  PRE-ARM CHECK 2: TENSÃO DA BATERIA
    // ════════════════════════════════════════════════════════
    // Bateria LiPo 2S: 7.4V nominal, 7.0V é ~10% restante.
    // Decolar com bateria fraca = risco de brownout em voo.
    if (state.vbat_V < PREARM_MIN_VBAT) {
        _preArmFailReason = "Bateria fraca (<7.0V)";
        return false;
    }

    // ════════════════════════════════════════════════════════
    //  PRE-ARM CHECK 3: INCLINAÇÃO DA AERONAVE
    // ════════════════════════════════════════════════════════
    // Se a aeronave está inclinada > 15° no chão, pode indicar:
    // 1. IMU descalibrada (offsets incorretos)
    // 2. Aeronave em posição instável (pode cair ao armar motor)
    // 3. Sensor montado incorretamente
    float tilt = sqrtf(state.roll_deg * state.roll_deg +
                       state.pitch_deg * state.pitch_deg);
    if (tilt > PREARM_MAX_TILT_DEG) {
        _preArmFailReason = "Inclinacao >15 graus";
        return false;
    }

    // ── Todos os checks passaram ──
    _preArmFailReason = "OK";
    return true;
}

void FlightModeManager::bumplessTransfer(PIDController& rollRatePID,
                                         PIDController& pitchRatePID,
                                         PIDController& rollAnglePID,
                                         PIDController& pitchAnglePID)
{
    // ── Reset completo de todos os PIDs ──
    // Cada PID zera: integrador, erro anterior, filtro D.
    // Isto garante transição suave sem transients.
    rollRatePID.reset();
    pitchRatePID.reset();
    rollAnglePID.reset();
    pitchAnglePID.reset();
}

bool FlightModeManager::isStickOverride(float rcRoll, float rcPitch, float deadzone)
{
    // ── Override de piloto no MODE_HOLD ──
    // Se o piloto move o stick além da deadzone, ele quer controle.
    // O sistema retorna controle imediato ao piloto (HOLD → ANGLE behavior).
    return (fabsf(rcRoll) > deadzone || fabsf(rcPitch) > deadzone);
}

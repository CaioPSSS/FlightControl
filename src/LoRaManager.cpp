/**
 * ============================================================
 *  LoRaManager.cpp — Implementação do Link LoRa SX1278
 * ============================================================
 *  Protocolo binário bidirecional sobre LoRa 433MHz.
 * 
 *  CONFIGURAÇÃO RF:
 *    Frequência:       433 MHz (ISM band)
 *    Spreading Factor: SF7 (melhor throughput para 10Hz)
 *    Bandwidth:        125 kHz (padrão)
 *    Coding Rate:      4/5 (mínimo FEC, maximiza throughput)
 *    TX Power:         17 dBm (~50mW)
 *    Header:           Implícito (economia de bytes)
 * 
 *  THROUGHPUT ESTIMADO:
 *    SF7, BW125, CR4/5 → ~5.47 kbps → ~683 bytes/s
 *    Pacote telemetria (0xAA) = ~24 bytes → overhead ok para 10Hz
 * 
 *  ESCALA RC (Decisão de Projeto):
 *    Roll/Pitch: int16_t -1000..+1000 (resolução 0.1%)
 *    Throttle:   uint16_t 0..1000 (resolução 0.1%)
 *    Vantagens: Compacto (2 bytes), resolução > PWM (1000 steps
 *    vs 1000µs range), compatível com SBUS/CRSF industry standard.
 *    Normalizado para float [-1,+1] no receptor.
 * ============================================================
 */

#include "LoRaManager.h"
#include <LoRa.h>

LoRaManager::LoRaManager()
    : _lastRSSI(0),
      _lastRxMs(0)
{
}

bool LoRaManager::init()
{
    // ── Configurar pinos SPI do SX1278 ──
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

    // ── Inicializar módulo na frequência 433MHz ──
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println(F("[LORA] ✗ Falha na inicialização do SX1278!"));
        return false;
    }

    // ── Configuração RF otimizada para 10Hz bidirecional ──
    LoRa.setSpreadingFactor(7);     // SF7: menor latência
    LoRa.setSignalBandwidth(250E3); // 250kHz
    LoRa.setCodingRate4(5);         // CR 4/5: mínimo FEC
    LoRa.setTxPower(17);           // 17 dBm (~50mW)
    LoRa.enableCrc();              // CRC habilitado (integridade)

    // Header implícito não é suportado na lib Sandeep Mistry
    // diretamente, mas CRC protege contra corrupção.

    _lastRxMs = millis();  // Inicializar timestamp

    Serial.println(F("[LORA] ✓ SX1278 inicializado — 433MHz, SF7, BW250"));
    return true;
}

void LoRaManager::processIncoming()
{
    // ── Non-blocking: verifica se há pacote disponível ──
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return;

    // Ler RSSI do pacote recebido
    _lastRSSI = (int8_t)LoRa.packetRssi();
    _lastRxMs = millis();

    // Ler todos os bytes do pacote
    uint8_t buffer[64];
    int idx = 0;
    while (LoRa.available() && idx < 64) {
        buffer[idx++] = (uint8_t)LoRa.read();
    }

    if (idx < 2) return;  // Pacote muito curto (deve conter pelo menos header + systemId)

    // Rejeitar pacotes com systemId diferente de 0x42 (para 0xBB, 0xCC, 0xDD)
    if (buffer[1] != 0x42) {
        return;
    }

    // ── Discriminar por header byte ──
    switch (buffer[0]) {
        case 0xBB:
            handleUplink(buffer, idx);
            break;
        case 0xCC:
            handleWaypoint(buffer, idx);
            break;
        case 0xDD:
            handleTuning(buffer, idx);
            break;
        default:
            // Header desconhecido — descartar (pode ser ruído RF)
            Serial.print(F("[LORA] Pacote desconhecido: 0x"));
            Serial.println(buffer[0], HEX);
            break;
    }

    // ── Atualizar timestamp e RSSI no globalState ──
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        globalState.lastLoRaRxMs = _lastRxMs;
        globalState.lora_rssi    = _lastRSSI;
        xSemaphoreGive(stateMutex);
    }
}

void LoRaManager::sendTelemetry()
{
    PacketTelemetryLoRa_t pkt;
    pkt.header = 0xAA;
    pkt.systemId = 0x42;

    // ── Ler estado global com mutex ──
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        pkt.roll        = (int16_t)(globalState.roll_deg * 100.0f);
        pkt.pitch       = (int16_t)(globalState.pitch_deg * 100.0f);
        pkt.yaw         = (int16_t)(globalState.yaw_deg * 100.0f);
        pkt.altitude    = (int16_t)(globalState.altitude_m * 10.0f);
        pkt.vbat        = (uint16_t)(globalState.vbat_V * 100.0f);
        pkt.lat         = (int32_t)(globalState.gps_lat * 1e7);
        pkt.lon         = (int32_t)(globalState.gps_lon * 1e7);
        pkt.gps_sats    = globalState.gps_sats;
        pkt.mode        = (uint8_t)globalState.mode;
        pkt.armState    = (uint8_t)globalState.armState;
        pkt.failsafe    = (uint8_t)globalState.failsafe;
        pkt.rssi        = _lastRSSI;
        pkt.groundSpeed = (uint16_t)(globalState.groundSpeed_ms * 100.0f);
        xSemaphoreGive(stateMutex);
    } else {
        // Não conseguiu o mutex — enviar telemetria com zeros
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xAA;
    }

    // ── Transmitir pacote de telemetria ──
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&pkt, sizeof(pkt));
    LoRa.endPacket(true);  // true = async (non-blocking TX)
}

void LoRaManager::handleUplink(const uint8_t* data, int len)
{
    // ════════════════════════════════════════════════════════
    //  0xBB — COMANDO RC DO PILOTO
    // ════════════════════════════════════════════════════════
    if (len < (int)sizeof(PacketUplinkLoRa_t)) {
        Serial.println(F("[LORA] Pacote RC truncado"));
        return;
    }

    const PacketUplinkLoRa_t* pkt = (const PacketUplinkLoRa_t*)data;

    // ── Normalizar int16_t para float ──
    // Roll/Pitch: -1000..+1000 → -1.0..+1.0
    // Throttle:   0..1000 → 0.0..1.0
    float rcRoll     = clampValue((float)pkt->roll / 1000.0f, -1.0f, 1.0f);
    float rcPitch    = clampValue((float)pkt->pitch / 1000.0f, -1.0f, 1.0f);
    float rcThrottle = clampValue((float)pkt->throttle / 1000.0f, 0.0f, 1.0f);

    FlightMode reqMode = (FlightMode)pkt->mode;
    if (pkt->mode > 4) {
        Serial.println(F("[LORA] Modo inválido, ignorando pacote"));
        return;
    }
    bool reqArm = (pkt->arm != 0);

    // ── Atualizar globalState com mutex ──
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        globalState.rc_roll     = rcRoll;
        globalState.rc_pitch    = rcPitch;
        globalState.rc_throttle = rcThrottle;
        globalState.requested_mode = (uint8_t)reqMode;
        globalState.requested_arm  = reqArm;
        xSemaphoreGive(stateMutex);
    }
}

void LoRaManager::handleWaypoint(const uint8_t* data, int len)
{
    // ════════════════════════════════════════════════════════
    //  0xCC — WAYPOINT DE MISSÃO
    // ════════════════════════════════════════════════════════
    if (len < (int)sizeof(PacketWaypointLoRa_t)) {
        Serial.println(F("[LORA] Pacote WP truncado"));
        return;
    }

    const PacketWaypointLoRa_t* pkt = (const PacketWaypointLoRa_t*)data;

    // Validar índice do waypoint
    if (pkt->index >= FlightState::MAX_WAYPOINTS) {
        Serial.print(F("[LORA] WP index inválido: "));
        Serial.println(pkt->index);
        return;
    }

    // ── Converter e armazenar com mutex ──
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        FlightState::Waypoint& wp = globalState.waypoints[pkt->index];
        wp.lat      = (double)pkt->lat / 1e7;         // int32 × 1e7 → graus
        wp.lon      = (double)pkt->lon / 1e7;
        wp.alt_m    = (float)pkt->alt / 10.0f;        // decímetros → metros
        wp.speed_ms = (float)pkt->speed / 100.0f;     // cm/s → m/s
        wp.valid    = true;

        // Atualizar contador de waypoints
        if (pkt->index >= globalState.wp_count) {
            globalState.wp_count = pkt->index + 1;
        }
        xSemaphoreGive(stateMutex);
    }

    Serial.print(F("[LORA] WP "));
    Serial.print(pkt->index);
    Serial.println(F(" recebido"));
}

void LoRaManager::handleTuning(const uint8_t* data, int len)
{
    // ════════════════════════════════════════════════════════
    //  0xDD — TUNING REMOTO DE PARÂMETROS
    // ════════════════════════════════════════════════════════
    // O valor é aplicado IMEDIATAMENTE em RAM para efeito
    // instantâneo nos PIDs, e AGENDADO para gravação NVS
    // pela Task_System_Mon (evita wear excessivo da Flash).
    
    if (len < (int)sizeof(PacketTuningLoRa_t)) {
        Serial.println(F("[LORA] Pacote Tuning truncado"));
        return;
    }

    const PacketTuningLoRa_t* pkt = (const PacketTuningLoRa_t*)data;

    // Validar ParamID
    if (pkt->paramId >= (uint8_t)ParamID::PARAM_COUNT) {
        Serial.print(F("[LORA] ParamID inválido: "));
        Serial.println(pkt->paramId);
        return;
    }

    // ── Validar valor recebido ──
    if (!isfinite(pkt->value)) {
        Serial.println(F("[LORA] Tuning: valor NaN/Inf rejeitado"));
        return;
    }

    // Limites de segurança por parâmetro
    static constexpr float PARAM_MAX[] = {
        50.0f,   // ROLL_KP
        20.0f,   // ROLL_KI
        5.0f,    // ROLL_KD
        5.0f,    // ROLL_FF
        50.0f,   // PITCH_KP
        20.0f,   // PITCH_KI
        5.0f,    // PITCH_KD
        5.0f,    // PITCH_FF
        50.0f,   // ANGLE_KP
        1.0f,    // THR_CRUISE
        50.0f,   // L1_PERIOD
        10000.0f,// LORA_TIMEOUT
        0.99f,   // TPA_BREAKPOINT
    };

    float clampedValue = pkt->value;
    if (clampedValue < 0.0f) clampedValue = 0.0f;
    if (clampedValue > PARAM_MAX[pkt->paramId]) clampedValue = PARAM_MAX[pkt->paramId];

    // ── Aplicar em RAM (efeito imediato) ──
    paramValues[pkt->paramId] = clampedValue;

    // ── Agendar gravação NVS (será feita pela Task_System_Mon) ──
    nvsFlushPending = true;

    // ── Sinalizar novos ganhos para o Core 1 ──
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        globalState.new_pid_gains = true;
        xSemaphoreGive(stateMutex);
    }

    Serial.print(F("[LORA] Tuning: "));
    Serial.print(PARAM_TABLE[pkt->paramId].nvsKey);
    Serial.print(F(" = "));
    Serial.println(pkt->value, 4);
}

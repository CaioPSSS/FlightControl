/**
 * ============================================================
 *  LoRaManager.h — Gerenciador do Link LoRa SX1278
 * ============================================================
 *  Comunicação bidirecional via SX1278 (433MHz, Header Implícito).
 *  Protocolo binário denso com header byte para discriminação.
 * 
 *  UPLINK (Ground → VANT):
 *    0xBB - PacketUplinkLoRa_t:   Comandos RC (Roll, Pitch, Thr, Mode, Arm)
 *    0xCC - PacketWaypointLoRa_t: Upload de Waypoints de Missão
 *    0xDD - PacketTuningLoRa_t:   Tuning Remoto de Parâmetros PID
 * 
 *  DOWNLINK (VANT → Ground):
 *    0xAA - PacketTelemetryLoRa_t: Telemetria Completa (10Hz)
 * 
 *  DECISÃO: Biblioteca Sandeep Mistry "LoRa"
 *    - API simples e bem testada com Arduino/ESP32
 *    - Suporte nativo a SPI com CS/RST/DIO0 configuráveis
 *    - Suficiente para protocolo de 10Hz com pacotes < 64 bytes
 *    - Overhead mínimo vs RadioLib
 * ============================================================
 */

#pragma once

#include <Arduino.h>
#include "SharedTypes.h"

class LoRaManager {
public:
    LoRaManager();

    /**
     * Inicializa o módulo SX1278 com a configuração LoRa.
     * Configura frequência, spreading factor, bandwidth e potência.
     * 
     * @return true se inicialização bem-sucedida
     */
    bool init();

    /**
     * Processa pacotes recebidos (non-blocking).
     * Lê o buffer do SX1278, identifica o header byte e
     * decodifica o pacote para a struct apropriada.
     * 
     * Deve ser chamado na Task_LoRa a cada ciclo (10Hz).
     * Atualiza globalState com mutex.
     */
    void processIncoming();

    /**
     * Transmite pacote de telemetria (downlink 0xAA).
     * Lê o estado atual do globalState (com mutex) e
     * empacota em PacketTelemetryLoRa_t.
     * 
     * Deve ser chamado na Task_LoRa alternando com processIncoming.
     */
    void sendTelemetry();

    /**
     * Retorna RSSI do último pacote recebido (dBm).
     */
    int8_t getLastRSSI() const { return _lastRSSI; }

    /**
     * Retorna timestamp do último pacote recebido (millis).
     */
    uint32_t getLastRxTimestamp() const { return _lastRxMs; }

private:
    int8_t   _lastRSSI;
    uint32_t _lastRxMs;

    /**
     * Processa pacote de comando RC (0xBB).
     * Normaliza valores int16_t para float [-1,+1] e [0,1].
     */
    void handleUplink(const uint8_t* data, int len);

    /**
     * Processa pacote de waypoint (0xCC).
     * Converte de int32_t (×1e7) para double (graus decimais).
     */
    void handleWaypoint(const uint8_t* data, int len);

    /**
     * Processa pacote de tuning (0xDD).
     * Aplica novo valor ao parâmetro em RAM e seta flag NVS.
     */
    void handleTuning(const uint8_t* data, int len);
};

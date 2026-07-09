/**
 * ============================================================
 *  L1_Guidance.h — Navegação Lateral L1 (Cross-Track)
 * ============================================================
 *  O algoritmo L1 é o controlador de navegação lateral padrão
 *  em controladores de voo comerciais (ArduPilot, PX4).
 * 
 *  PRINCÍPIO:
 *  O L1 calcula a aceleração lateral necessária para o avião
 *  convergir para a linha entre dois waypoints. O "L1 distance"
 *  é o ponto de referência à frente do avião na rota desejada.
 * 
 *  EQUAÇÕES:
 *    η = ângulo entre o vetor velocidade e a direção ao L1 point
 *    a_lat = 2·V²/L1 · sin(η)    [aceleração lateral, m/s²]
 *    φ = atan(a_lat / g)          [roll comandado, rad]
 *    L1 = V × L1_period           [distância L1, m]
 * 
 *  PROTEÇÃO:
 *    Roll limitado a ±35° (tip stall protection em curvas).
 *    Em curvas apertadas a baixa velocidade, exceder 35° pode
 *    causar estol da asa interna → autorotação → crash.
 * 
 *  Referência: S. Park, J. Deyst, J.P. How, "A New Nonlinear
 *  Guidance Logic for Trajectory Tracking", AIAA 2004.
 * ============================================================
 */

#pragma once

#include <Arduino.h>
#include "SharedTypes.h"

class L1Guidance {
public:
    L1Guidance();

    /**
     * Inicializa o L1 Guidance.
     */
    void init();

    /**
     * Atualiza o L1 e calcula o roll comandado para seguir a rota.
     * 
     * @param currentLat     Latitude atual (graus decimais)
     * @param currentLon     Longitude atual (graus decimais)
     * @param targetLat      Latitude do waypoint alvo (graus decimais)
     * @param targetLon      Longitude do waypoint alvo (graus decimais)
     * @param prevLat        Latitude do waypoint anterior (graus decimais)
     * @param prevLon        Longitude do waypoint anterior (graus decimais)
     * @param groundSpeed_ms Velocidade sobre o solo (m/s)
     * @param cogDeg         Course Over Ground (graus, 0=Norte)
     * @return               Comando de roll (graus, clampado a ±35°)
     */
    float update(double currentLat, double currentLon,
                 double targetLat, double targetLon,
                 double prevLat, double prevLon,
                 float groundSpeed_ms, float cogDeg);

    /**
     * Calcula o roll para orbitar um ponto (loiter/RTH).
     * 
     * @param currentLat     Latitude atual
     * @param currentLon     Longitude atual
     * @param centerLat      Latitude do centro do loiter
     * @param centerLon      Longitude do centro do loiter
     * @param radius_m       Raio da órbita (metros)
     * @param groundSpeed_ms Velocidade sobre o solo (m/s)
     * @param cogDeg         COG atual (graus)
     * @return               Comando de roll (graus)
     */
    float updateLoiter(double currentLat, double currentLon,
                       double centerLat, double centerLon,
                       float radius_m,
                       float groundSpeed_ms, float cogDeg);

    /**
     * Verifica se o waypoint alvo foi alcançado.
     * 
     * @param currentLat, currentLon  Posição atual
     * @param targetLat, targetLon    Posição do waypoint
     * @param acceptRadius_m          Raio de aceitação (metros)
     * @return                        True se dentro do raio
     */
    bool waypointReached(double currentLat, double currentLon,
                         double targetLat, double targetLon,
                         float acceptRadius_m = 30.0f);

    // --- Parâmetros ---
    void setL1Period(float period) { _l1Period = period; }
    float getL1Period() const { return _l1Period; }
    
    // --- Debug ---
    float getLastCrossTrackError() const { return _lastXTrackErr; }
    float getLastEta() const { return _lastEta; }

private:
    float _l1Period;        // Período L1 (segundos, default 17)
    float _lastXTrackErr;   // Último cross-track error (metros)
    float _lastEta;         // Último ângulo η (radianos)
};

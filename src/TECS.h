/**
 * ============================================================
 *  TECS.h — Total Energy Control System (Híbrido)
 * ============================================================
 *  O TECS controla simultaneamente a velocidade e a altitude
 *  da aeronave, distribuindo a energia total entre as formas
 *  cinética (velocidade) e potencial (altitude).
 * 
 *  EQUAÇÃO FUNDAMENTAL:
 *    E_total = E_cinética + E_potencial
 *    E_total = ½mV² + mgh
 * 
 *  PRINCÍPIO:
 *  - Throttle controla a ENERGIA TOTAL (sobe/desce ambas)
 *  - Pitch controla a DISTRIBUIÇÃO de energia (trade-off V↔h)
 * 
 *  GROUND SPEED ILLUSION (Problema Crítico):
 *  O GPS mede Ground Speed (GS), não Airspeed (AS).
 *  Com vento contrário: AS = GS + Vento → GS < AS
 *  O TECS vê GS baixo e pensa que está em estol, mas AS é ok.
 *  Mergulhar para ganhar velocidade é SUICÍDIO contra o vento.
 * 
 *  MITIGAÇÃO: Exigir confirmação dupla para detecção de estol:
 *  1. GS do GPS < limite
 *  2. Velocidade inercial integrada (aceleração × tempo) < limite
 *  Se apenas o GPS diz "estol" mas o acelerômetro não confirma,
 *  provavelmente é vento contrário → NÃO mergulhar.
 * ============================================================
 */

#pragma once

#include <Arduino.h>
#include "SharedTypes.h"

class TECSController {
public:
    TECSController();

    /**
     * Inicializa o TECS com parâmetros default.
     */
    void init();

    /**
     * Atualiza o TECS e calcula comandos de pitch e throttle.
     * 
     * @param targetAlt_m      Altitude alvo (metros)
     * @param targetSpeed_ms   Velocidade alvo (m/s)
     * @param currentAlt_m     Altitude atual filtrada (Kalman, metros)
     * @param currentVario_ms  Taxa de subida/descida (m/s)
     * @param groundSpeed_ms   Velocidade sobre o solo (GPS, m/s)
     * @param inertialSpeed_ms Velocidade longitudinal integrada (IMU, m/s)
     * @param dt               Intervalo de tempo (segundos)
     * @param pitchOut         [OUT] Comando de pitch (graus, +nariz cima)
     * @param throttleOut      [OUT] Comando de throttle (0.0 - 1.0)
     */
    void update(float targetAlt_m, float targetSpeed_ms,
                float currentAlt_m, float currentVario_ms,
                float groundSpeed_ms, float inertialSpeed_ms,
                float dt,
                float& pitchOut, float& throttleOut);

    /**
     * Reset do TECS (para Bumpless Transfer).
     */
    void reset();

    // --- Parâmetros ajustáveis ---
    void setThrottleCruise(float thr) { _thrCruise = thr; }

private:
    // Ganhos do controlador de energia
    float _kAltP;         // Proporcional altitude
    float _kAltI;         // Integral altitude (lento)
    float _kSpdP;         // Proporcional velocidade
    float _kClimbRate;    // Ganho de taxa de subida
    
    // Parâmetros operacionais
    float _thrCruise;     // Throttle de cruzeiro (0-1)
    float _thrMin;        // Throttle mínimo
    float _thrMax;        // Throttle máximo
    float _pitchMin;      // Pitch mínimo (graus, mergulho)
    float _pitchMax;      // Pitch máximo (graus, subida)
    
    // Estado interno
    float _altIntegral;   // Integrador de erro de altitude
    float _altIntMax;     // Limite do integrador
    
    // Detecção de estol híbrida
    bool    _stallDetected; // Flag de estol confirmado
    uint8_t _stallCounter;  // Contador temporal para histerese
    
    /**
     * Detecção de estol com confirmação dupla (GPS + IMU).
     * Retorna true APENAS se ambas as fontes concordam em baixa energia.
     */
    bool detectStallHybrid(float groundSpeed_ms, float inertialSpeed_ms);
};

/**
 * ============================================================
 *  TECS.cpp — Total Energy Control System (Híbrido)
 * ============================================================
 *  Implementação do TECS adaptativo a vento com detecção de
 *  estol por confirmação dupla (Ground Speed + Inercial).
 * 
 *  PRINCÍPIO FÍSICO:
 *  A energia total de uma aeronave em regime subsônico é:
 *    E = ½mV² + mgh
 *    dE/dt = mV·(dV/dt) + mg·(dh/dt)
 *    dE/dt = mV·a + mg·Vz
 * 
 *  O motor (throttle) controla dE/dt (variação de energia total).
 *  O pitch controla a PARTIÇÃO entre V e h:
 *    - Pitch up   → troca V por h (subir desacelerando)
 *    - Pitch down → troca h por V (descer acelerando)
 * 
 *  CONTROLADOR:
 *  1. Erro de energia total → Throttle
 *     (throttle_cmd = thrCruise + kSpdP × speedErr + kAltP × altErr)
 *  2. Balanço energético → Pitch
 *     (pitch_cmd = kClimb × climbRateError)
 *  3. Detecção de estol → Override pitch mínimo
 * ============================================================
 */

#include "TECS.h"
#include <math.h>

TECSController::TECSController()
    : _kAltP(0.25f),
      _kAltI(0.05f),
      _kSpdP(0.8f),
      _kClimbRate(3.0f),
      _thrCruise(0.55f),
      _thrMin(0.0f),
      _thrMax(1.0f),
      _pitchMin(-20.0f),
      _pitchMax(25.0f),
      _altIntegral(0.0f),
      _altIntMax(10.0f),
      _stallDetected(false),
      _stallCounter(0)
{
}

void TECSController::init()
{
    reset();
    Serial.println(F("[TECS] Inicializado — Modo Híbrido (GPS+IMU)"));
}

void TECSController::update(float targetAlt_m, float targetSpeed_ms,
                            float currentAlt_m, float currentVario_ms,
                            float groundSpeed_ms, float inertialSpeed_ms,
                            float dt,
                            float& pitchOut, float& throttleOut)
{
    if (dt <= 0.0f || dt > 1.0f) {
        pitchOut = 0.0f;
        throttleOut = _thrCruise;
        return;
    }

    // ════════════════════════════════════════════════════════
    //  1) ERROS DE ALTITUDE E VELOCIDADE
    // ════════════════════════════════════════════════════════
    float altError = targetAlt_m - currentAlt_m;
    
    // Usar a maior velocidade entre GPS e inercial para
    // estimativa conservadora (evitar falsos estóis)
    float speedEstimate = fmaxf(groundSpeed_ms, inertialSpeed_ms * 0.7f);
    float speedError = targetSpeed_ms - speedEstimate;

    // ════════════════════════════════════════════════════════
    //  2) CONTROLADOR DE THROTTLE (Energia Total)
    // ════════════════════════════════════════════════════════
    // O throttle controla a taxa de variação da energia total.
    // Em cruzeiro nivelado, throttle = thrCruise compensa o arrasto.
    // Erro positivo de altitude OU velocidade → mais throttle.
    
    // Taxa de subida desejada (limitada para evitar manobras bruscas)
    float desiredClimbRate = altError * _kAltP;
    desiredClimbRate = clampValue(desiredClimbRate, -3.0f, 3.0f);  // ±3 m/s max
    
    // Integrador de altitude (lento, compensa bias de trim)
    _altIntegral += altError * _kAltI * dt;
    _altIntegral = clampValue(_altIntegral, -_altIntMax, _altIntMax);
    
    // Comando de throttle
    throttleOut = _thrCruise
                + speedError * _kSpdP * 0.01f  // Correção de velocidade
                + _altIntegral * 0.01f;         // Correção integral altitude
    
    // Clamp throttle aos limites do ESC
    throttleOut = clampValue(throttleOut, _thrMin, _thrMax);

    // ════════════════════════════════════════════════════════
    //  3) CONTROLADOR DE PITCH (Distribuição de Energia)
    // ════════════════════════════════════════════════════════
    // O pitch redistribui energia entre V e h.
    // Erro de taxa de subida → comando de pitch.
    float climbRateError = desiredClimbRate - currentVario_ms;
    pitchOut = climbRateError * _kClimbRate;

    // ════════════════════════════════════════════════════════
    //  4) DETECÇÃO DE ESTOL HÍBRIDA (Ground Speed Illusion)
    // ════════════════════════════════════════════════════════
    // Este é o sistema CRÍTICO que previne o "mergulho suicida".
    // Ver documentação completa em detectStallHybrid().
    _stallDetected = detectStallHybrid(groundSpeed_ms, inertialSpeed_ms);

    if (_stallDetected) {
        // ── Estol confirmado por AMBAS as fontes ──
        // Ação: NÃO mergulhar. Manter pitch moderado e
        // aumentar throttle para ganhar energia total.
        // Em estol real, pitch down é perigoso sem altitude.
        
        // Limitar pitch a no máximo -5° (leve descida controlada)
        if (pitchOut < -5.0f) {
            pitchOut = -5.0f;
        }
        // Forçar throttle máximo para recuperação
        throttleOut = _thrMax;
        
        static uint32_t lastStallPrint = 0;
        if (millis() - lastStallPrint > 2000) {
            Serial.println(F("[TECS] ⚠ ESTOL CONFIRMADO — Recuperação ativa"));
            lastStallPrint = millis();
        }
    }

    // Clamp pitch aos limites estruturais
    pitchOut = clampValue(pitchOut, _pitchMin, _pitchMax);
}

void TECSController::reset()
{
    _altIntegral   = 0.0f;
    _stallDetected = false;
    _stallCounter  = 0;
}

bool TECSController::detectStallHybrid(float groundSpeed_ms, float inertialSpeed_ms)
{
    // ════════════════════════════════════════════════════════
    //  DETECÇÃO DE ESTOL COM CONFIRMAÇÃO DUPLA
    // ════════════════════════════════════════════════════════
    //
    //  PROBLEMA: O GPS mede Ground Speed (GS), não Airspeed (AS).
    //  A relação é: AS = GS + componente_de_vento
    //
    //  Cenário 1 — Vento contrário forte:
    //    AS = 15 m/s (seguro), Vento = -8 m/s → GS = 7 m/s
    //    GPS diz "abaixo do estol" mas o avião está voando bem!
    //    TECS ingênuo: "Mergulha para ganhar velocidade!"
    //    Resultado: Avião mergulha CONTRA o vento → crash
    //
    //  Cenário 2 — Estol real:
    //    AS = 8 m/s (estol), sem vento → GS = 8 m/s
    //    AMBOS GPS e acelerômetro confirmam perda de energia.
    //    TECS deve agir: throttle max + pitch moderado
    //
    //  SOLUÇÃO: Histerese com confirmação dupla.
    //  Estol = verdadeiro APENAS se:
    //    1. GS do GPS < STALL_SPEED_MS (9.5 m/s), E
    //    2. Velocidade inercial integrada < STALL_SPEED_MS
    //
    //  A velocidade inercial é obtida integrando a aceleração
    //  longitudinal do MPU6050 (body frame, eixo X). Ela não
    //  é afetada pelo vento (mede aceleração real do avião).
    //  Sofre drift, mas para detecção de estol (curto prazo),
    //  é confiável o suficiente.

    bool gpsLowSpeed      = (groundSpeed_ms < STALL_SPEED_MS);
    bool inertialLowSpeed = (inertialSpeed_ms < STALL_SPEED_MS);

    if (gpsLowSpeed && inertialLowSpeed) {
        if (_stallCounter < 250) _stallCounter++;
    } else {
        _stallCounter = 0;
    }

    // Estol confirmado APENAS se ambas as fontes concordam por 10 ciclos (200ms a 50Hz)
    return (_stallCounter >= 10);
}

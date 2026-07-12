/**
 * ============================================================
 *  PID.h — Controlador PID de Grau Aeroespacial
 * ============================================================
 *  Implementa cascata PID (Angle → Rate) com:
 *  - FeedForward (FF) aerodinâmico
 *  - Filtro PT1 no termo Derivativo (cutoff 25Hz)
 *  - Anti-Windup Dinâmico (congela integrador na saturação)
 *  - TPA (Throttle PID Attenuation)
 *  - Reset para Bumpless Transfer em transições de modo
 * 
 *  Referência: ArduPilot AP_PIDInfo, PX4 rate_control
 * ============================================================
 */

#pragma once

#include <Arduino.h>

class PIDController {
public:
    /**
     * Construtor.
     * @param kp         Ganho Proporcional
     * @param ki         Ganho Integral
     * @param kd         Ganho Derivativo
     * @param kff        Ganho FeedForward aerodinâmico
     * @param outputMin  Limite inferior de saída (-1.0 para servos)
     * @param outputMax  Limite superior de saída (+1.0 para servos)
     * @param iMax       Limite absoluto do integrador (anti-windup estático)
     * @param dCutoffHz  Frequência de corte do filtro PT1 no D (default 25Hz)
     */
    PIDController(float kp, float ki, float kd, float kff,
                  float outputMin, float outputMax,
                  float iMax, float dCutoffHz = 25.0f);

    /**
     * Atualiza o PID com novo erro.
     * 
     * @param setpoint    Valor desejado (ex: taxa angular alvo em dps)
     * @param measurement Valor medido (ex: taxa angular do giroscópio)
     * @param dt          Intervalo de tempo desde a última chamada (segundos)
     * @param tpaFactor   Fator TPA (0.0 a 1.0) — 1.0 = sem atenuação
     * @param saturated   Flag de saturação do mixer (anti-windup externo)
     * @return            Saída do controlador (clampada a [outputMin, outputMax])
     */
    float update(float setpoint, float measurement, float dt,
                 float tpaFactor = 1.0f, bool saturated = false);

    /**
     * Reset completo — Bumpless Transfer.
     * Zera integrador, derivativo e estado do filtro PT1.
     * OBRIGATÓRIO chamar em toda transição de modo de voo.
     */
    void reset();

    // --- Setters para tuning remoto via LoRa ---
    void setKp(float kp)   { _kp = kp; }
    void setKi(float ki)   { _ki = ki; }
    void setKd(float kd)   { _kd = kd; }
    void setKff(float kff) { _kff = kff; }
    
    void setGains(float kp, float ki, float kd, float kff) {
        _kp = kp; _ki = ki; _kd = kd; _kff = kff;
    }

    // --- Getters para telemetria/debug ---
    float getP()   const { return _lastP; }
    float getI()   const { return _lastI; }
    float getD()   const { return _lastD; }
    float getFF()  const { return _lastFF; }
    float getOutput() const { return _lastOutput; }

private:
    // Ganhos do controlador
    float _kp, _ki, _kd, _kff;
    
    // Limites de saída
    float _outputMin, _outputMax;
    
    // Limite estático do integrador
    float _iMax;
    
    // Coeficiente do filtro PT1 para o termo Derivativo
    // alpha = dt / (RC + dt), onde RC = 1 / (2π × cutoff)
    float _dCutoffHz;
    
    // Estado interno
    float _integral;        // Acumulador integral
    float _prevMeasurement; // Medição anterior (S-02: derivar medição evita kick)
    float _dFiltered;       // Saída filtrada do termo D
    
    // Últimos termos calculados (para telemetria)
    float _lastP, _lastI, _lastD, _lastFF, _lastOutput;
    
    bool _firstRun;  // Flag de primeira execução (evita spike no D)
};

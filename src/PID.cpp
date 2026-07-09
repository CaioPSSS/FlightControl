/**
 * ============================================================
 *  PID.cpp — Implementação do Controlador PID Aeroespacial
 * ============================================================
 *  Cascata PID com FeedForward, Filtro D, Anti-Windup e TPA.
 * 
 *  FILOSOFIA DE CONTROLE (ArduPilot-Style):
 *  Para aeronaves de asa fixa, o FeedForward é o termo PRIMÁRIO.
 *  A taxa de rotação desejada deve comandar o servo diretamente
 *  via FF, e o PID corrige apenas os distúrbios de vento e
 *  imperfeições aerodinâmicas. Isto difere de multirotores onde
 *  o PID é o controlador principal.
 * 
 *  Cmd_total = FF + P + I + D
 *  FF = TargetRate × Kff  (comando direto, sem erro)
 *  P  = Error × Kp × TPA  (proporcional ao erro, atenuado)
 *  I  = Σ(Error × dt × Ki) (integral, com anti-windup)
 *  D  = d(Error)/dt × Kd, filtrado por PT1
 * ============================================================
 */

#include "PID.h"

PIDController::PIDController(float kp, float ki, float kd, float kff,
                             float outputMin, float outputMax,
                             float iMax, float dCutoffHz)
    : _kp(kp), _ki(ki), _kd(kd), _kff(kff),
      _outputMin(outputMin), _outputMax(outputMax),
      _iMax(iMax), _dCutoffHz(dCutoffHz),
      _integral(0.0f), _prevError(0.0f), _dFiltered(0.0f),
      _lastP(0.0f), _lastI(0.0f), _lastD(0.0f),
      _lastFF(0.0f), _lastOutput(0.0f),
      _firstRun(true)
{
}

float PIDController::update(float setpoint, float measurement, float dt,
                            float tpaFactor, bool saturated)
{
    // ── Proteção contra dt degenerado ──
    if (dt <= 0.0f || dt > 0.5f) {
        return _lastOutput;
    }

    float error = setpoint - measurement;

    // ════════════════════════════════════════════════════════
    //  1) FEEDFORWARD AERODINÂMICO
    // ════════════════════════════════════════════════════════
    // O FF comanda o servo proporcional à taxa DESEJADA, não ao erro.
    // Para asa fixa, este é o termo dominante (~50-70% do comando).
    // A deflexão de aileron/profundor é quase linear com a taxa de
    // rotação desejada em regime subsônico.
    _lastFF = setpoint * _kff;

    // ════════════════════════════════════════════════════════
    //  2) TERMO PROPORCIONAL com TPA
    // ════════════════════════════════════════════════════════
    // TPA (Throttle PID Attenuation): Em altas potências do motor,
    // o fluxo de ar sobre as superfícies aumenta a efetividade
    // dos servos. Sem TPA, o avião fica "nervoso" em alta velocidade.
    // tpaFactor varia de 1.0 (baixa potência) a ~0.5 (potência max).
    _lastP = error * _kp * tpaFactor;

    // ════════════════════════════════════════════════════════
    //  3) TERMO INTEGRAL com Anti-Windup Dinâmico
    // ════════════════════════════════════════════════════════
    // Anti-Windup: Se o mixer reporta saturação mecânica do servo
    // (pulso atingiu 2000µs ou 1000µs), o integrador CONGELA.
    // Sem isto, o integrador acumula erro irrecuperável ("wind-up")
    // e causa overshoots violentos quando o servo dessatura.
    if (!saturated) {
        _integral += error * _ki * dt;
        
        // Clamp estático do integrador (proteção adicional)
        if (_integral > _iMax)       _integral = _iMax;
        else if (_integral < -_iMax) _integral = -_iMax;
    }
    // Se saturated == true: _integral não muda (congelado)
    
    _lastI = _integral;

    // ════════════════════════════════════════════════════════
    //  4) TERMO DERIVATIVO com Filtro PT1 (Low-Pass)
    // ════════════════════════════════════════════════════════
    // O termo D puro amplifica ruído de alta frequência do giroscópio.
    // Filtro PT1 (passa-baixa de 1ª ordem) com cutoff em 25Hz atenua
    // o ruído mantendo resposta a perturbações reais.
    //
    // Filtro PT1: y[n] = y[n-1] + α × (x[n] - y[n-1])
    // onde α = dt / (RC + dt), RC = 1/(2π×fc)
    if (_firstRun) {
        // Na primeira execução, inicializa sem spike
        _prevError = error;
        _dFiltered = 0.0f;
        _firstRun = false;
        _lastD = 0.0f;
    } else {
        float dRaw = (error - _prevError) / dt;
        
        // Coeficiente do filtro PT1
        float RC = 1.0f / (2.0f * PI * _dCutoffHz);
        float alpha = dt / (RC + dt);
        
        // Aplicar filtro PT1
        _dFiltered = _dFiltered + alpha * (dRaw - _dFiltered);
        
        _lastD = _dFiltered * _kd;
    }
    _prevError = error;

    // ════════════════════════════════════════════════════════
    //  5) SOMATÓRIO E CLAMP DE SAÍDA
    // ════════════════════════════════════════════════════════
    float output = _lastFF + _lastP + _lastI + _lastD;

    // Clamp final — proteção contra comandos que excedem
    // a deflexão mecânica máxima dos servos
    if (output > _outputMax)      output = _outputMax;
    else if (output < _outputMin) output = _outputMin;

    _lastOutput = output;
    return output;
}

void PIDController::reset()
{
    // ── Bumpless Transfer ──
    // Zera TODOS os estados internos para evitar "coice" na
    // transição entre modos de voo. Se não resetar, o integrador
    // acumulado em MODE_ANGLE pode causar um pico de comando
    // ao transitar para MODE_MANUAL e voltar.
    _integral   = 0.0f;
    _prevError  = 0.0f;
    _dFiltered  = 0.0f;
    _lastP      = 0.0f;
    _lastI      = 0.0f;
    _lastD      = 0.0f;
    _lastFF     = 0.0f;
    _lastOutput = 0.0f;
    _firstRun   = true;
}

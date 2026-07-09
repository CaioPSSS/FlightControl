/**
 * ============================================================
 *  StandardMixer.cpp — Mixagem Direta Avião Convencional
 * ============================================================
 *  Implementação do mapeamento direto (sem elevon/V-tail).
 * 
 *  GEOMETRIA DA AERONAVE:
 *  - Motor tractor (A2212) na frente, puxa a aeronave
 *  - 2 hastes de fibra de vidro (twin-boom) levam à cauda
 *  - Aileron esquerdo e direito nas asas (roll)
 *  - Profundor na cauda entre os booms (pitch)
 *  - Estabilizadores verticais FIXOS nos booms (sem leme)
 * 
 *  CONVENÇÃO DE SINAIS:
 *  - rollCmd  > 0 → rolar para a DIREITA → aileron esq SOBE,
 *                                           aileron dir DESCE
 *  - pitchCmd > 0 → nariz para CIMA → profundor SOBE (trailing
 *                                      edge para cima)
 * 
 *  SATURAÇÃO:
 *  Quando qualquer servo atinge o limite mecânico (1000 ou 2000µs),
 *  o flag 'saturated' é setado para informar o anti-windup do PID.
 * ============================================================
 */

#include "StandardMixer.h"

StandardMixer::StandardMixer()
{
}

MixerOutput StandardMixer::mix(float rollCmd, float pitchCmd, float throttleCmd)
{
    MixerOutput out;
    out.saturated = false;
    bool sat = false;

    // ════════════════════════════════════════════════════════
    //  AILERONS — Atuação Diferencial Simétrica
    // ════════════════════════════════════════════════════════
    // Aileron esquerdo: INVERTIDO em relação ao comando de roll.
    // Quando rollCmd > 0 (rolar direita):
    //   - Aileron esq SOBE (diminui sustentação asa esq) → PWM diminui
    //   - Aileron dir DESCE (aumenta sustentação asa dir) → PWM aumenta
    out.ailLeftUs  = cmdToServoUs(-rollCmd, sat);  // Invertido
    out.saturated |= sat;
    
    out.ailRightUs = cmdToServoUs(rollCmd, sat);   // Direto
    out.saturated |= sat;

    // ════════════════════════════════════════════════════════
    //  PROFUNDOR — Atuação Direta
    // ════════════════════════════════════════════════════════
    // pitchCmd > 0 → nariz sobe → profundor deflete para cima
    out.elevatorUs = cmdToServoUs(pitchCmd, sat);
    out.saturated |= sat;

    // ════════════════════════════════════════════════════════
    //  MOTOR — Conversão linear throttle → PWM ESC
    // ════════════════════════════════════════════════════════
    out.motorUs = throttleToMotorUs(throttleCmd);

    return out;
}

MixerOutput StandardMixer::mixManual(float rcRoll, float rcPitch, float rcThrottle)
{
    // Modo manual: RC → Servo diretamente, sem PID
    // Mesma lógica de inversão dos ailerons
    return mix(rcRoll, rcPitch, rcThrottle);
}

uint16_t StandardMixer::cmdToServoUs(float cmd, bool& saturated)
{
    // Mapear -1.0..+1.0 para 1000..2000 µs
    // Centro em 1500µs, range de ±500µs
    float us = PWM_CENTER_US + cmd * 500.0f;

    // Verificar saturação mecânica ANTES do clamp
    if (us <= (float)PWM_MIN_US || us >= (float)PWM_MAX_US) {
        saturated = true;
    }

    // Clamp para limites mecânicos do servo
    if (us < (float)PWM_MIN_US) us = (float)PWM_MIN_US;
    if (us > (float)PWM_MAX_US) us = (float)PWM_MAX_US;

    return (uint16_t)(us + 0.5f);  // Arredondamento
}

uint16_t StandardMixer::throttleToMotorUs(float thr)
{
    // Clamp throttle para [0, 1]
    if (thr < 0.0f) thr = 0.0f;
    if (thr > 1.0f) thr = 1.0f;

    // Mapear 0.0..1.0 para 1000..2000 µs (padrão ESC)
    return (uint16_t)(PWM_MIN_US + thr * (PWM_MAX_US - PWM_MIN_US) + 0.5f);
}

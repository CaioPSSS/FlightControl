/**
 * ============================================================
 *  StandardMixer.h — Mixagem Direta para Avião Convencional
 * ============================================================
 *  MAPEAMENTO DIRETO (Direct Pass-Through):
 *  Este NÃO é um mixer elevon nem V-tail.
 *  O avião tem superfícies de controle convencionais:
 * 
 *    Roll  → 2x Ailerons (esq invertido, dir direto)
 *    Pitch → 1x Profundor (elevator)
 *    Yaw   → NÃO EXISTE (estabilizadores verticais fixos)
 * 
 *  Filosofia "Bank and Yank": A guinada é controlada
 *  coordenando roll + pitch (curva coordenada via L1).
 *  Sem leme = sem adverse yaw correction, mas o FeedForward
 *  aerodinâmico dos ailerons compensa parcialmente.
 * 
 *  O mixer retorna flag de saturação para o anti-windup do PID.
 * ============================================================
 */

#pragma once

#include <Arduino.h>
#include "SharedTypes.h"

/**
 * Saída do mixer — valores em microsegundos prontos para o LEDC.
 */
struct MixerOutput {
    uint16_t ailLeftUs;    // Servo aileron esquerdo (µs)
    uint16_t ailRightUs;   // Servo aileron direito (µs)
    uint16_t elevatorUs;   // Servo profundor (µs)
    uint16_t motorUs;      // ESC motor frontal (µs)
    bool     saturated;    // True se algum servo atingiu limite mecânico
};

class StandardMixer {
public:
    StandardMixer();

    /**
     * Calcula os pulsos PWM a partir dos comandos normalizados.
     * 
     * MAPEAMENTO DIRETO:
     *   Aileron Esq = Center - (rollCmd × range)  ← INVERTIDO
     *   Aileron Dir = Center + (rollCmd × range)   ← DIRETO
     *   Profundor   = Center + (pitchCmd × range)
     *   Motor       = 1000 + (throttle × 1000)
     * 
     * @param rollCmd     Comando de roll (-1.0 a +1.0, +dir = rolar direita)
     * @param pitchCmd    Comando de pitch (-1.0 a +1.0, +up = nariz sobe)
     * @param throttleCmd Comando de throttle (0.0 a 1.0)
     * @return            MixerOutput com pulsos em µs e flag de saturação
     */
    MixerOutput mix(float rollCmd, float pitchCmd, float throttleCmd);

    /**
     * Modo manual — converte RC normalizado diretamente em PWM.
     * Bypass completo dos PIDs.
     */
    MixerOutput mixManual(float rcRoll, float rcPitch, float rcThrottle);

private:
    /**
     * Converte um comando normalizado (-1..+1) em pulso servo (µs).
     * Detecta saturação (clamp nos limites mecânicos).
     */
    uint16_t cmdToServoUs(float cmd, bool& saturated);

    /**
     * Converte throttle normalizado (0..1) em pulso ESC (µs).
     */
    uint16_t throttleToMotorUs(float thr);
};

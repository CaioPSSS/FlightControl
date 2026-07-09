/**
 * ============================================================
 *  OutputManager.h — Gerenciador de Saídas PWM (LEDC)
 * ============================================================
 *  Controla os 4 canais de saída do ESP32 usando o periférico
 *  LEDC (LED Controller) configurado como gerador PWM de 50Hz
 *  com resolução de 14 bits (16384 steps).
 * 
 *  Canais:
 *    CH0: Servo Aileron Esquerdo (Pino 13)
 *    CH1: Servo Aileron Direito  (Pino 12)
 *    CH2: Servo Profundor        (Pino 15)
 *    CH3: ESC Motor Frontal      (Pino 27)
 * 
 *  Segurança: Quando DISARMED, o ESC é forçado a 1000µs.
 *  Os servos permanecem em posição neutra (1500µs).
 * ============================================================
 */

#pragma once

#include <Arduino.h>
#include "SharedTypes.h"

class OutputManager {
public:
    OutputManager();

    /**
     * Inicializa os 4 canais LEDC com 50Hz e 14-bit.
     * Deve ser chamado uma vez no setup().
     * Após init, todos os canais ficam em posição segura.
     */
    void init();

    /**
     * Escreve os pulsos PWM nos 4 canais simultaneamente.
     * 
     * @param ailLeftUs   Pulso do aileron esquerdo (µs)
     * @param ailRightUs  Pulso do aileron direito (µs)
     * @param elevatorUs  Pulso do profundor (µs)
     * @param motorUs     Pulso do ESC (µs)
     * @param armed       Se false, força ESC em 1000µs (safety)
     */
    void write(uint16_t ailLeftUs, uint16_t ailRightUs,
               uint16_t elevatorUs, uint16_t motorUs,
               bool armed);

    /**
     * Força todos os canais para posição segura.
     * Servos em 1500µs (neutro), ESC em 1000µs (idle).
     * Chamado no disarm e em failsafe crítico.
     */
    void safeState();

private:
    /**
     * Escreve um pulso em microsegundos em um canal LEDC específico.
     * Faz o clamp automático para [1000, 2000]µs.
     */
    void writeChannel(uint8_t channel, uint16_t us);
};

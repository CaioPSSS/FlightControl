/**
 * ============================================================
 *  OutputManager.cpp — Implementação do Gerenciador PWM LEDC
 * ============================================================
 *  O periférico LEDC do ESP32 foi originalmente projetado para
 *  controle de LEDs, mas com 14 bits de resolução a 50Hz
 *  oferece resolução de ~1.2µs por step — mais que suficiente
 *  para servos de aeromodelismo (resolução típica: ~4µs).
 * 
 *  Conversão µs → Duty:
 *    Período = 20000µs (50Hz)
 *    Duty = (pulso_us / 20000) × 16383
 *    1000µs → duty 819   (servo mín)
 *    1500µs → duty 1229  (servo centro)
 *    2000µs → duty 1638  (servo máx)
 * ============================================================
 */

#include "OutputManager.h"

OutputManager::OutputManager()
{
}

void OutputManager::init()
{
    // ── Configuração dos 4 canais LEDC ──
    // Cada canal usa um timer independente para evitar glitches
    // quando os duties são atualizados em momentos diferentes.
    // Na prática, com atualização síncrona no mesmo ciclo de
    // controle (250Hz), o mesmo timer poderia ser compartilhado.
    
    ledcSetup(LEDC_CH_AIL_ESQ,  LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_AIL_DIR,  LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_ELEVATOR, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_MOTOR,    LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);

    ledcAttachPin(PIN_SERVO_AIL_ESQ,  LEDC_CH_AIL_ESQ);
    ledcAttachPin(PIN_SERVO_AIL_DIR,  LEDC_CH_AIL_DIR);
    ledcAttachPin(PIN_SERVO_ELEVATOR, LEDC_CH_ELEVATOR);
    ledcAttachPin(PIN_ESC_MOTOR,      LEDC_CH_MOTOR);

    // Inicializa em estado seguro
    safeState();

    Serial.println(F("[OUTPUT] LEDC 4ch inicializado (50Hz, 14-bit)"));
}

void OutputManager::write(uint16_t ailLeftUs, uint16_t ailRightUs,
                          uint16_t elevatorUs, uint16_t motorUs,
                          bool armed)
{
    writeChannel(LEDC_CH_AIL_ESQ,  ailLeftUs);
    writeChannel(LEDC_CH_AIL_DIR,  ailRightUs);
    writeChannel(LEDC_CH_ELEVATOR, elevatorUs);

    // ── Segurança: ESC só recebe comando se ARMED ──
    // Quando desarmado, o ESC SEMPRE recebe throttle mínimo.
    // Isto previne partida acidental do motor durante manuseio.
    if (armed) {
        writeChannel(LEDC_CH_MOTOR, motorUs);
    } else {
        writeChannel(LEDC_CH_MOTOR, PWM_ESC_ARM_US);
    }
}

void OutputManager::safeState()
{
    // Servos em posição neutra (centro mecânico)
    writeChannel(LEDC_CH_AIL_ESQ,  PWM_CENTER_US);
    writeChannel(LEDC_CH_AIL_DIR,  PWM_CENTER_US);
    writeChannel(LEDC_CH_ELEVATOR, PWM_CENTER_US);
    
    // ESC em throttle mínimo (motor parado)
    writeChannel(LEDC_CH_MOTOR, PWM_ESC_ARM_US);
}

void OutputManager::writeChannel(uint8_t channel, uint16_t us)
{
    // Clamp para a faixa válida de servos (proteção mecânica)
    if (us < PWM_MIN_US) us = PWM_MIN_US;
    if (us > PWM_MAX_US) us = PWM_MAX_US;
    
    // Converter µs para duty LEDC
    uint32_t duty = pwmUsToDuty(us);
    ledcWrite(channel, duty);
}

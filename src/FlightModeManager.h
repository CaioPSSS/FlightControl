/**
 * ============================================================
 *  FlightModeManager.h — FSM, Pre-Arm Checks e Bumpless Transfer
 * ============================================================
 *  Gerencia a Máquina de Estados Finita (FSM) do VANT.
 * 
 *  MODOS DE VOO:
 *    MANUAL → Bypass total, sem PID, RC direto
 *    ANGLE  → Estabilizado, PIDs ativos, piloto comanda ângulo
 *    HOLD   → L1/TECS seguram rumo/alt, override por stick
 *    AUTO   → Navegação por waypoints, manche ignorado
 *    RTH    → Return-To-Home, manche ignorado
 * 
 *  PRE-ARM CHECKS:
 *    - GPS fix válido
 *    - Vbat ≥ 7.0V
 *    - Inclinação ≤ 15° (IMU calibrada, aeronave nivelada)
 * 
 *  FAILSAFE:
 *    - Timeout LoRa (default 1500ms) → RTH forçado
 *    - Vbat crítico em voo → RTH forçado
 * 
 *  BUMPLESS TRANSFER:
 *    Em toda transição de modo, os integradores e derivativos
 *    dos PIDs são resetados para evitar "coices" (transients).
 * ============================================================
 */

#pragma once

#include <Arduino.h>
#include "SharedTypes.h"
#include "PID.h"

class FlightModeManager {
public:
    FlightModeManager();

    /**
     * Inicializa a FSM no estado DISARMED/MANUAL.
     */
    void init();

    /**
     * Processa a requisição de modo e armamento vinda do LoRa.
     * Aplica Pre-Arm Checks antes de permitir armamento.
     * Gerencia transições com Bumpless Transfer.
     * 
     * @param requestedMode  Modo requisitado pelo piloto
     * @param requestArm     True se piloto quer armar
     * @param state          Referência ao FlightState (para checks)
     * @param rollRatePID    PID de taxa de roll (para reset)
     * @param pitchRatePID   PID de taxa de pitch (para reset)
     * @param rollAnglePID   PID de ângulo de roll (para reset)
     * @param pitchAnglePID  PID de ângulo de pitch (para reset)
     */
    void update(FlightMode requestedMode, bool requestArm,
                const FlightState& state,
                PIDController& rollRatePID, PIDController& pitchRatePID,
                PIDController& rollAnglePID, PIDController& pitchAnglePID);

    /**
     * Verifica timeout do LoRa e dispara failsafe se necessário.
     * Chamado pela Task_System_Mon a cada 1Hz.
     * 
     * @param state  Referência ao FlightState
     * @param rollRatePID, pitchRatePID, rollAnglePID, pitchAnglePID - PIDs para reset
     */
    void checkFailsafe(const FlightState& state,
                       PIDController& rollRatePID, PIDController& pitchRatePID,
                       PIDController& rollAnglePID, PIDController& pitchAnglePID);

    /**
     * Retorna o modo de voo atual.
     */
    FlightMode getCurrentMode() const { return _currentMode; }

    /**
     * Retorna o estado de armamento atual.
     */
    ArmState getArmState() const { return _armState; }

    /**
     * Retorna o estado de failsafe atual.
     */
    FailsafeState getFailsafeState() const { return _failsafeState; }

    /**
     * Retorna string com motivo da falha do Pre-Arm Check.
     * Útil para telemetria/debug.
     */
    const char* getPreArmFailReason() const { return _preArmFailReason; }

    /**
     * Verifica se o piloto está movendo o stick (para override no HOLD).
     * Retorna true se |rcRoll| > deadzone ou |rcPitch| > deadzone.
     */
    static bool isStickOverride(float rcRoll, float rcPitch, float deadzone = 0.1f);

private:
    FlightMode    _currentMode;
    FlightMode    _previousMode;
    ArmState      _armState;
    FailsafeState _failsafeState;
    const char*   _preArmFailReason;

    /**
     * Executa TODOS os Pre-Arm Checks.
     * Retorna true se TODOS passam. Caso contrário, seta _preArmFailReason.
     */
    bool runPreArmChecks(const FlightState& state);

    /**
     * Aplica Bumpless Transfer: reseta todos os PIDs.
     */
    void bumplessTransfer(PIDController& rollRatePID, PIDController& pitchRatePID,
                          PIDController& rollAnglePID, PIDController& pitchAnglePID);
};

/**
 * ============================================================
 *  L1_Guidance.cpp — Implementação do Controlador L1
 * ============================================================
 *  Navegação lateral por cross-track error com convergência
 *  garantida para a rota desejada.
 * 
 *  O ALGORITMO L1:
 *  1. Projeta a posição atual na linha entre WP_prev e WP_target
 *  2. Calcula o "L1 point" na rota, a uma distância L1 à frente
 *  3. Calcula o ângulo η entre a velocidade e a direção ao L1 point
 *  4. Aceleração lateral a_lat = 2·V²/L1 · sin(η)
 *  5. Roll comandado φ = atan(a_lat / g)
 * 
 *  O período L1 controla a agressividade:
 *  - L1_period baixo (~10s) → convergência rápida, oscilações
 *  - L1_period alto (~25s) → convergência suave, erros maiores
 *  - Default: 17s (bom compromisso para aviões pequenos)
 * 
 *  A distância L1 é: L1 = V × L1_period / π
 *  Quanto mais rápido o avião, mais longe o ponto de referência.
 * ============================================================
 */

#include "L1_Guidance.h"
#include <math.h>

L1Guidance::L1Guidance()
    : _l1Period(17.0f),
      _lastXTrackErr(0.0f),
      _lastEta(0.0f)
{
}

void L1Guidance::init()
{
    _lastXTrackErr = 0.0f;
    _lastEta       = 0.0f;
    Serial.println(F("[L1] Guidance inicializado"));
}

float L1Guidance::update(double currentLat, double currentLon,
                         double targetLat, double targetLon,
                         double prevLat, double prevLon,
                         float groundSpeed_ms, float cogDeg)
{
    // ════════════════════════════════════════════════════════
    //  PROTEÇÃO: Velocidade mínima para navegação
    // ════════════════════════════════════════════════════════
    // Abaixo de 3 m/s, o COG do GPS é muito ruidoso e o L1
    // oscila violentamente. Melhor manter roll = 0 (wings level).
    if (groundSpeed_ms < 3.0f) {
        return 0.0f;
    }

    // ════════════════════════════════════════════════════════
    //  1) GEOMETRIA: Bearing da rota e bearing ao target
    // ════════════════════════════════════════════════════════
    // Bearing da rota planejada (prev → target)
    float trackBearing = bearingDeg(prevLat, prevLon, targetLat, targetLon);
    
    // Bearing do avião até o target
    float bearingToTarget = bearingDeg(currentLat, currentLon, targetLat, targetLon);
    
    // Distância até o waypoint target
    float distToTarget = haversineMeters(currentLat, currentLon, targetLat, targetLon);

    // ════════════════════════════════════════════════════════
    //  2) CROSS-TRACK ERROR
    // ════════════════════════════════════════════════════════
    // Distância perpendicular do avião à linha da rota.
    // Positivo = avião à direita da rota.
    // Usando projeção trigonométrica simples (válida para distâncias curtas).
    float trackError = wrapAngle180(bearingToTarget - trackBearing);
    _lastXTrackErr = distToTarget * sinf(degToRad(trackError));

    // ════════════════════════════════════════════════════════
    //  3) DISTÂNCIA L1 E PONTO DE REFERÊNCIA
    // ════════════════════════════════════════════════════════
    // L1 = V × T_l1 / π
    // Este é o ponto "à frente" na rota que o avião persegue.
    float L1dist = groundSpeed_ms * _l1Period / PI;
    
    // Limitar L1 mínimo para evitar divisão por zero e oscilações
    if (L1dist < 10.0f) L1dist = 10.0f;

    // ════════════════════════════════════════════════════════
    //  4) ÂNGULO η (eta) — Erro angular de curso
    // ════════════════════════════════════════════════════════
    // η = ângulo entre o vetor velocidade (COG) e a direção
    // ao ponto L1 na rota. É a "correção angular" necessária.
    //
    // Abordagem simplificada: usar o bearing ao target como
    // proxy do bearing ao L1 point (válido quando distToTarget > L1dist).
    // Para distâncias curtas, adicionar correção de cross-track.
    
    float navBearing;
    if (distToTarget > L1dist) {
        // Avião longe do WP — navegar pela rota (cross-track correction)
        // Adicionar correção proporcional ao cross-track error
        float correction = atanf(_lastXTrackErr / L1dist);
        navBearing = trackBearing + radToDeg(correction);
    } else {
        // Avião perto do WP — apontar direto ao waypoint
        navBearing = bearingToTarget;
    }

    // η = diferença entre o COG atual e o bearing desejado
    float eta = degToRad(wrapAngle180(navBearing - cogDeg));
    _lastEta = eta;

    // ════════════════════════════════════════════════════════
    //  5) ACELERAÇÃO LATERAL E COMANDO DE ROLL
    // ════════════════════════════════════════════════════════
    // a_lat = 2·V²/L1 · sin(η)  [m/s²]
    // φ = atan(a_lat / g)        [rad]
    float aLat = 2.0f * groundSpeed_ms * groundSpeed_ms / L1dist * sinf(eta);
    float rollRad = atanf(aLat / GRAVITY);
    float rollDeg = radToDeg(rollRad);

    // ════════════════════════════════════════════════════════
    //  6) PROTEÇÃO DE TIP STALL — Clamp absoluto ±35°
    // ════════════════════════════════════════════════════════
    // Em curvas apertadas a baixa velocidade, um ângulo de bank
    // excessivo aumenta drasticamente a carga alar (load factor):
    //   n = 1 / cos(φ)
    //   φ = 35° → n = 1.22g (aceitável para VANT leve)
    //   φ = 60° → n = 2.0g  (risco de estol de ponta de asa)
    // 
    // Estol de ponta de asa (tip stall) em curva causa autorotação
    // (spin entry), que é irrecuperável a baixa altitude.
    rollDeg = clampValue(rollDeg, -MAX_BANK_ANGLE_DEG, MAX_BANK_ANGLE_DEG);

    return rollDeg;
}

float L1Guidance::updateLoiter(double currentLat, double currentLon,
                               double centerLat, double centerLon,
                               float radius_m,
                               float groundSpeed_ms, float cogDeg)
{
    // ════════════════════════════════════════════════════════
    //  LOITER (Órbita circular) — Usado em RTH e HOLD
    // ════════════════════════════════════════════════════════
    // O avião orbita um ponto a um raio fixo. A lógica é similar
    // ao track-following, mas o "waypoint alvo" é um ponto
    // tangente ao círculo na direção do voo.

    if (groundSpeed_ms < 3.0f) {
        return 0.0f;
    }

    float distToCenter = haversineMeters(currentLat, currentLon, centerLat, centerLon);
    float bearingToCenter = bearingDeg(currentLat, currentLon, centerLat, centerLon);

    // Erro radial: positivo = fora do círculo
    float radialError = distToCenter - radius_m;

    // L1 distance
    float L1dist = groundSpeed_ms * _l1Period / PI;
    if (L1dist < 10.0f) L1dist = 10.0f;

    // Ângulo de correção para manter o raio
    // Se estamos fora do círculo, virar mais para dentro
    // Se estamos dentro, virar menos (ou para fora)
    float correctionAngle = atanf(radialError / L1dist);

    // Bearing tangente ao círculo (orbitar no sentido horário)
    // Tangente = bearing ao centro + 90° (perpendicular)
    float tangentBearing = wrapAngle360(bearingToCenter + 90.0f);

    // Adicionar correção radial ao bearing tangente
    float navBearing = tangentBearing - radToDeg(correctionAngle);

    // Calcular η e aceleração lateral
    float eta = degToRad(wrapAngle180(navBearing - cogDeg));
    float aLat = 2.0f * groundSpeed_ms * groundSpeed_ms / L1dist * sinf(eta);
    float rollRad = atanf(aLat / GRAVITY);
    float rollDeg = radToDeg(rollRad);

    // Tip stall protection
    rollDeg = clampValue(rollDeg, -MAX_BANK_ANGLE_DEG, MAX_BANK_ANGLE_DEG);

    return rollDeg;
}

bool L1Guidance::waypointReached(double currentLat, double currentLon,
                                 double targetLat, double targetLon,
                                 float acceptRadius_m)
{
    float dist = haversineMeters(currentLat, currentLon, targetLat, targetLon);
    return (dist < acceptRadius_m);
}

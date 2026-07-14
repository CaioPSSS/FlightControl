/**
 * ============================================================
 *  SharedTypes.h — Tipos Compartilhados IPC, Enums e Constantes
 * ============================================================
 *  VANT Flight Controller - ESP32 Dual-Core FreeRTOS
 *  Avião Convencional (Tractor) com Cauda Twin-Boom (Lança Dupla)
 * 
 *  Este header define TODAS as estruturas de dados compartilhadas
 *  entre as tasks FreeRTOS. A comunicação inter-núcleos é feita
 *  exclusivamente via a struct FlightState protegida por Mutex.
 * 
 *  Filosofia: "Mission Critical" — Jitter zero na malha primária.
 * ============================================================
 */

#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ============================================================
//  CONSTANTES DE HARDWARE — Pinagem ESP32
// ============================================================

// --- I2C Fast (Core 1) — MPU6050 ---
static constexpr uint8_t PIN_I2C_FAST_SDA     = 21;
static constexpr uint8_t PIN_I2C_FAST_SCL     = 22;
static constexpr uint32_t I2C_FAST_FREQ        = 400000;  // 400kHz
static constexpr uint8_t MPU6050_ADDR          = 0x68;

// --- I2C Slow (Core 0) — BMP280 ---
static constexpr uint8_t PIN_I2C_SLOW_SDA     = 32;
static constexpr uint8_t PIN_I2C_SLOW_SCL     = 33;
static constexpr uint32_t I2C_SLOW_FREQ        = 400000;  // 400kHz
static constexpr uint8_t BMP280_ADDR           = 0x76;

// --- UART GPS (NEO-6M) ---
static constexpr uint8_t PIN_GPS_RX            = 16;
static constexpr uint8_t PIN_GPS_TX            = 17;
static constexpr uint32_t GPS_BAUD             = 9600;

// --- SPI LoRa (SX1278) ---
static constexpr uint8_t PIN_LORA_SCK          = 18;
static constexpr uint8_t PIN_LORA_MISO         = 19;
static constexpr uint8_t PIN_LORA_MOSI         = 23;
static constexpr uint8_t PIN_LORA_CS           = 5;
static constexpr uint8_t PIN_LORA_RST          = 14;
static constexpr uint8_t PIN_LORA_DIO0         = 26;
static constexpr long    LORA_FREQUENCY        = 433E6;  // 433MHz

// --- Saídas PWM (LEDC) — 50Hz, 14-bit ---
static constexpr uint8_t PIN_SERVO_AIL_ESQ     = 13;   // CH 0
static constexpr uint8_t PIN_SERVO_AIL_DIR     = 12;   // CH 1
static constexpr uint8_t PIN_SERVO_ELEVATOR    = 15;   // CH 2
static constexpr uint8_t PIN_ESC_MOTOR         = 27;   // CH 3

static constexpr uint8_t LEDC_CH_AIL_ESQ       = 0;
static constexpr uint8_t LEDC_CH_AIL_DIR       = 1;
static constexpr uint8_t LEDC_CH_ELEVATOR      = 2;
static constexpr uint8_t LEDC_CH_MOTOR         = 3;

static constexpr uint32_t LEDC_FREQ_HZ         = 50;
static constexpr uint8_t  LEDC_RESOLUTION_BITS = 14;   // 16384 counts
static constexpr uint32_t LEDC_MAX_DUTY        = (1 << LEDC_RESOLUTION_BITS) - 1;

// --- ADC Bateria ---
static constexpr uint8_t PIN_VBAT_ADC          = 34;

// ============================================================
//  CONSTANTES FÍSICAS E LIMITES AERODINÂMICOS
// ============================================================

static constexpr float GRAVITY                 = 9.80665f;  // m/s²

// Limites PWM dos servos (µs)
static constexpr uint16_t PWM_MIN_US           = 1000;
static constexpr uint16_t PWM_MAX_US           = 2000;
static constexpr uint16_t PWM_CENTER_US        = 1500;
static constexpr uint16_t PWM_ESC_ARM_US       = 1000;  // Throttle mínimo (desarmado)

// Proteção de Tip Stall — Limite absoluto de bank em curvas (L1)
static constexpr float MAX_BANK_ANGLE_DEG      = 35.0f;
static constexpr float MAX_BANK_ANGLE_RAD      = MAX_BANK_ANGLE_DEG * DEG_TO_RAD;

// Pre-Arm Checks
static constexpr float PREARM_MIN_VBAT         = 7.0f;   // Volts
static constexpr float PREARM_MAX_TILT_DEG     = 15.0f;  // Graus

// TECS — Limite de estol com histerese
static constexpr float STALL_SPEED_MS          = 8.5f;   // m/s

// Velocidade mínima para fusão COG (Course Over Ground)
static constexpr float MIN_COG_FUSION_SPEED    = 5.0f;   // m/s

// ============================================================
//  TAXAS DE EXECUÇÃO DAS TASKS
// ============================================================

static constexpr uint32_t FLIGHT_CTRL_HZ       = 250;    // Core 1
static constexpr uint32_t FLIGHT_CTRL_PERIOD_US = 1000000 / FLIGHT_CTRL_HZ;  // 4000µs
static constexpr float    FLIGHT_CTRL_DT        = 1.0f / FLIGHT_CTRL_HZ;     // 0.004s

static constexpr uint32_t NAV_HZ               = 50;
static constexpr float    NAV_DT               = 1.0f / NAV_HZ;  // 0.02s

static constexpr uint32_t LORA_HZ              = 10;
static constexpr uint32_t SYSTEM_MON_HZ        = 1;

// ============================================================
//  ENUMS — Modos de Voo e Estados
// ============================================================

/**
 * Modos de voo da FSM (Finite State Machine).
 * Cada modo define o nível de automação e a fonte de comandos.
 */
enum class FlightMode : uint8_t {
    MODE_MANUAL = 0,   // Bypass total — RC direto para servos, sem PID
    MODE_ANGLE  = 1,   // Estabilizado — PIDs ativos, piloto comanda inclinação
    MODE_HOLD   = 2,   // L1/TECS seguram rumo/altitude, override por stick
    MODE_AUTO   = 3,   // Navegação autônoma por waypoints
    MODE_RTH    = 4    // Return-To-Home (failsafe ou comando)
};

/**
 * Estado de armamento do sistema.
 * Transição DISARMED→ARMED requer passagem em todos Pre-Arm Checks.
 */
enum class ArmState : uint8_t {
    DISARMED = 0,
    ARMED    = 1
};

/**
 * Estados de failsafe.
 * NOMINAL: Tudo OK. WARN: Degradação detectada. CRITICAL: RTH forçado.
 */
enum class FailsafeState : uint8_t {
    NOMINAL  = 0,
    WARN     = 1,
    CRITICAL = 2
};

// ============================================================
//  ENUM — IDs de Parâmetros NVS (Tuning Remoto via LoRa)
// ============================================================

/**
 * Cada parâmetro tem um ID numérico único.
 * O pacote LoRa de Tuning (0xDD) envia ParamID + float value.
 * O valor é aplicado em RAM imediatamente e agendado para 
 * gravação persistente na Flash (NVS) pela Task_System_Mon.
 */
enum class ParamID : uint8_t {
    ROLL_KP        = 0,
    ROLL_KI        = 1,
    ROLL_KD        = 2,
    ROLL_FF        = 3,
    PITCH_KP       = 4,
    PITCH_KI       = 5,
    PITCH_KD       = 6,
    PITCH_FF       = 7,
    ANGLE_KP       = 8,
    THR_CRUISE     = 9,
    L1_PERIOD      = 10,
    LORA_TIMEOUT   = 11,
    TPA_BREAKPOINT = 12,
    PARAM_COUNT    = 13   // Sentinela — total de parâmetros
};

/**
 * Mapeamento ParamID → chave NVS (max 15 chars por limitação Preferences)
 * e valores default.
 */
struct ParamMeta {
    const char* nvsKey;
    float       defaultValue;
};

// Tabela de metadados dos parâmetros (definida no main.cpp)
// Declaração extern para uso em múltiplos módulos
extern const ParamMeta PARAM_TABLE[];

// Array de valores em RAM (acessível globalmente, protegido por mutex)
extern float paramValues[];

// Flag indicando que há parâmetros pendentes de gravação NVS
extern volatile bool nvsFlushPending;
extern volatile bool nvsMissionFlushPending;


// ============================================================
//  ESTRUTURAS DE PACOTES LORA (Binárias, packed, sem padding)
// ============================================================

/**
 * 0xBB — Uplink: Comandos RC do piloto
 * Roll/Pitch: -1000 a +1000 (centésimos de grau ou taxa)
 * Throttle:   0 a 1000 (0-100% em décimos)
 * Mode: FlightMode enum
 * Arm: 0=Disarm, 1=Arm
 */
struct __attribute__((packed)) PacketUplinkLoRa_t {
    uint8_t  header;    // 0xBB
    uint8_t  systemId;
    int16_t  roll;      // -1000..+1000
    int16_t  pitch;     // -1000..+1000
    uint16_t throttle;  // 0..1000
    uint8_t  mode;      // FlightMode cast
    uint8_t  arm;       // 0 ou 1
};

/**
 * 0xCC — Uplink: Waypoint de missão
 * Cada waypoint contém índice, coordenadas e parâmetros.
 */
struct __attribute__((packed)) PacketWaypointLoRa_t {
    uint8_t  header;    // 0xCC
    uint8_t  systemId;  // 0x42
    uint8_t  index;     // Index of waypoint (0..31)
    int32_t  lat;       // Latitude * 1e7
    int32_t  lon;       // Longitude * 1e7
    int16_t  alt;       // Target relative altitude in decimeters
    uint16_t speed;     // Target speed in cm/s
    uint8_t  cmd;       // Command type: 0=WAYPOINT, 1=LOITER_TIME, 2=RTL
    uint16_t cmd_val;   // Command value (e.g. loiter time in seconds)
};

struct __attribute__((packed)) PacketMissionControlLoRa_t {
    uint8_t  header;    // 0xCE
    uint8_t  systemId;  // 0x42
    uint8_t  cmd;       // 1=START_UPLOAD, 2=VERIFY_CHECKSUM, 3=ACK, 4=NACK, 5=CLEAR
    uint8_t  data1;     // e.g. wp_count or waypoint index
    uint32_t checksum;  // Simple checksum (sum of all waypoints' data)
    uint8_t  _unused;   // Unused padding to guarantee 9-byte layout
};

/**
 * 0xDD — Uplink: Tuning remoto de parâmetros
 * Permite ajustar ganhos PID e outros parâmetros em voo.
 */
struct __attribute__((packed)) PacketTuningLoRa_t {
    uint8_t  header;    // 0xDD
    uint8_t  systemId;
    uint8_t  paramId;   // ParamID enum
    float    value;     // Novo valor do parâmetro
};

/**
 * 0xAA — Downlink: Telemetria completa do VANT
 * Enviada a cada ciclo da Task_LoRa (10Hz).
 */
struct __attribute__((packed)) PacketTelemetryLoRa_t {
    uint8_t  header;      // 0xAA
    uint8_t  systemId;
    int16_t  roll;        // Roll × 100 (centésimos de grau)
    int16_t  pitch;       // Pitch × 100
    int16_t  yaw;         // Yaw × 100
    int16_t  altitude;    // Altitude em decímetros
    uint16_t vbat;        // Tensão × 100 (centésimos de Volt)
    int32_t  lat;         // Latitude × 1e7
    int32_t  lon;         // Longitude × 1e7
    uint8_t  gps_sats;    // Número de satélites
    uint8_t  mode;        // FlightMode
    uint8_t  armState;    // ArmState
    uint8_t  failsafe;    // FailsafeState
    int8_t   rssi;        // RSSI do último pacote recebido (dBm)
    uint16_t groundSpeed; // Velocidade sobre o solo em cm/s
};

// ============================================================
//  ESTRUTURA PRINCIPAL IPC — FlightState
// ============================================================

/**
 * FlightState — Estado Global do VANT
 * 
 * Esta struct é o ÚNICO mecanismo de comunicação inter-tasks.
 * TODA leitura ou escrita DEVE ser protegida pelo stateMutex.
 * 
 * Padrão de acesso:
 *   if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
 *       // Ler/escrever globalState
 *       xSemaphoreGive(stateMutex);
 *   }
 */
struct FlightState {
    // --- Atitude (Mahony AHRS, Core 1) ---
    float roll_deg;       // Ângulo de rolagem (graus, +dir)
    float pitch_deg;      // Ângulo de arfagem (graus, +nariz cima)
    float yaw_deg;        // Proa magnética/inercial (graus, 0-360)
    
    // --- Taxas angulares filtradas (graus/s) ---
    float gyro_roll_dps;  // Taxa de rolagem
    float gyro_pitch_dps; // Taxa de arfagem
    float gyro_yaw_dps;   // Taxa de guinada
    
    // --- Acelerações lineares (m/s², body frame) ---
    float accel_x;        // Longitudinal (frente = +)
    float accel_y;        // Lateral (direita = +)
    float accel_z;        // Vertical (baixo = +)
    
    // --- Navegação (Core 0) ---
    float altitude_m;     // Altitude barométrica filtrada (Kalman)
    float vario_ms;       // Velocidade vertical (m/s, subindo = +)
    float groundSpeed_ms; // Velocidade sobre o solo (GPS, m/s)
    float cogDeg;         // Course Over Ground (GPS, graus)
    
    // --- GPS ---
    double gps_lat;           // Latitude (graus decimais)
    double gps_lon;           // Longitude (graus decimais)
    float  gps_alt_m;         // Altitude GPS (m, MSL)
    uint8_t gps_sats;         // Satélites rastreados
    bool   gps_fix;           // GPS tem fix 3D válido
    
    // --- Posição de Home (para RTH) ---
    double home_lat;
    double home_lon;
    float  home_alt_m;        // Altitude do Home (MSL GPS)
    float  home_baro_alt_m;   // Altitude barométrica (Kalman) no momento do Home set
    bool   home_set;          // True se o home foi definido

    // --- Missão (Waypoints) ---
    static constexpr uint8_t MAX_WAYPOINTS = 32;
    struct Waypoint {
        double lat;
        double lon;
        float  alt_m;
        float  speed_ms;
        uint8_t cmd;
        uint16_t cmd_val;
        bool   valid;
    };
    Waypoint waypoints[MAX_WAYPOINTS];
    uint8_t  wp_count;       // Total de waypoints na missão
    uint8_t  wp_current;     // Índice do waypoint alvo atual
    
    // --- Comandos RC (via LoRa) ---
    float rc_roll;        // -1.0 a +1.0 (normalizado)
    float rc_pitch;       // -1.0 a +1.0
    float rc_throttle;    // 0.0 a 1.0
    
    // --- Saídas do Controlador ---
    float cmd_roll;       // Saída PID Roll (-1.0 a +1.0)
    float cmd_pitch;      // Saída PID Pitch (-1.0 a +1.0)
    float cmd_throttle;   // Comando de throttle (0.0 a 1.0)
    
    // --- Saídas de Navegação (L1 + TECS) ---
    float nav_roll_deg;   // Comando de roll do L1 (graus)
    float nav_pitch_deg;  // Comando de pitch do TECS (graus)
    float nav_throttle;   // Comando de throttle do TECS (0-1)
    float xtrack_error;
    float alt_error;
    
    // --- Sistema ---
    float    vbat_V;         // Tensão da bateria (Volts)
    FlightMode   mode;       // Modo de voo atual
    ArmState     armState;   // Estado de armamento
    FailsafeState failsafe;  // Estado de failsafe
    int8_t   lora_rssi;      // RSSI do último pacote LoRa
    uint32_t lastLoRaRxMs;   // Timestamp do último pacote recebido (millis)
    
    // --- Velocidade inercial integrada (TECS anti-estol) ---
    float inertialSpeed_ms;  // Velocidade longitudinal integrada do acelerômetro
    
    // --- Earth Frame Acceleration Projection ---
    float accelZ_world;      // Aceleração vertical projetada no frame da Terra
    
    // --- Flags de saturação do mixer (para anti-windup PID) ---
    bool mixerSaturated;
    
    // --- Flags de Requisição IPC (Evita Race Conditions) ---
    bool request_failsafe;   // Core 0 solicita que Core 1 avalie o failsafe
    bool new_pid_gains;      // Core 0 sinaliza que novos ganhos PID chegaram
    uint8_t requested_mode;  // Core 0 (LoRa) solicita novo modo (cast para FlightMode)
    bool requested_arm;      // Core 0 (LoRa) solicita novo estado de armamento
    bool imu_calibrated;     // Indica se a calibração inicial teve sucesso
    bool baro_healthy;       // S-06: Indica se o barômetro BMP280 está saudável e respondendo
};

// ============================================================
//  DECLARAÇÕES EXTERNAS GLOBAIS
// ============================================================

// Estado global do VANT (definido em main.cpp)
extern FlightState globalState;

// Mutex para acesso thread-safe ao globalState
extern SemaphoreHandle_t stateMutex;

// ============================================================
//  FUNÇÕES UTILITÁRIAS INLINE
// ============================================================

/**
 * Converte microsegundos de pulso PWM para duty cycle LEDC (14-bit).
 * PWM servo padrão: período = 20ms (50Hz).
 * Duty = (pulso_us / 20000us) * 16383
 */
inline uint32_t pwmUsToDuty(uint16_t us) {
    return (uint32_t)((float)us / 20000.0f * LEDC_MAX_DUTY);
}

/**
 * Clamp genérico — restringe valor ao intervalo [lo, hi].
 * Essencial para limitar saídas de atuadores e comandos PID.
 */
template <typename T>
inline T clampValue(T val, T lo, T hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * Normaliza ângulo para o intervalo [-180, +180] graus.
 * Necessário para cálculos de erro de heading (evita wrap-around).
 */
inline float wrapAngle180(float angle) {
    if (!isfinite(angle)) return 0.0f;
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle - 180.0f;
}

inline float wrapAngle360(float angle) {
    if (!isfinite(angle)) return 0.0f;
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

/**
 * Interpolação linear simples.
 */
inline float lerpf(float a, float b, float t) {
    return a + t * (b - a);
}

/**
 * Converte graus para radianos.
 */
inline float degToRad(float deg) {
    return deg * (PI / 180.0f);
}

/**
 * Converte radianos para graus.
 */
inline float radToDeg(float rad) {
    return rad * (180.0f / PI);
}

/**
 * Distância Haversine entre dois pontos (lat/lon em graus).
 * Retorna distância em metros. Precisão suficiente para navegação VANT.
 * 
 * Fórmula: a = sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)
 *          c = 2·atan2(√a, √(1-a))
 *          d = R·c
 */
inline float haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0;  // Raio médio da Terra (metros)
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return (float)(R * c);
}

/**
 * Bearing (azimute) de ponto 1 para ponto 2 (graus, 0=Norte, CW).
 * Usado pelo L1 Guidance para calcular a direção ao próximo waypoint.
 */
inline float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double y = sin(dLon) * cos(lat2 * DEG_TO_RAD);
    double x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) -
               sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(dLon);
    float brng = radToDeg((float)atan2(y, x));
    return wrapAngle360(brng);
}

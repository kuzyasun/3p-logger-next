#pragma once

#include "io/io.h"
#include "protocols/protocol_parser.h"
#include "util/time-util.h"

// Forward declaration
typedef struct atp_s atp_t;

#define MSP_BUFFER_SIZE 64

// --- MSP Command Definitions ---
#define MSP_IDENT 100       // Out: Головна інформація про прошивку
#define MSP_STATUS 101      // Out: Статус системи
#define MSP_RAW_IMU 102     // Out: Дані з акселерометра, гіроскопа, магнітометра
#define MSP_SERVO 103       // Out: Позиції сервоприводів
#define MSP_MOTOR 104       // Out: Рівні газу моторів
#define MSP_RC 105          // Out: Канали пульта керування
#define MSP_RAW_GPS 106     // Out: GPS дані
#define MSP_COMP_GPS 107    // Out: Дистанція/напрямок до "дому"
#define MSP_ATTITUDE 108    // Out: Кути (pitch, roll, yaw)
#define MSP_ALTITUDE 109    // Out: Висота
#define MSP_ANALOG 110      // Out: Напруга, струм, RSSI
#define MSP_RC_TUNING 111   // Out: Налаштування RC
#define MSP_PID 112         // Out: Коефіцієнти PID
#define MSP_BOX 113         // Out: Статуси режимів (BOX)
#define MSP_MISC 114        // Out: Різна інша інформація
#define MSP_MOTOR_PINS 115  // Out: Кількість моторів та їх піни
#define MSP_BOXNAMES 116    // Out: Назви режимів (BOX)
#define MSP_PIDNAMES 117    // Out: Назви PID
#define MSP_WP 118          // Out: Точка маршруту
#define MSP_BOXIDS 119      // Out: ID режимів (BOX)
#define MSP_SERVO_CONF 120  // Out: Конфігурація сервоприводів

#define MSP_SET_RAW_RC 200       // In: Встановити значення каналів
#define MSP_SET_RAW_GPS 201      // In: Встановити дані GPS (для "повернення додому")
#define MSP_SET_PID 202          // In: Встановити коефіцієнти PID
#define MSP_SET_BOX 203          // In: Встановити режими (BOX)
#define MSP_SET_RC_TUNING 204    // In: Встановити налаштування RC
#define MSP_ACC_CALIBRATION 205  // In: Команда для калібрування акселерометра
#define MSP_MAG_CALIBRATION 206  // In: Команда для калібрування магнітометра
#define MSP_SET_MISC 207         // In: Встановити різну іншу інформацію
#define MSP_RESET_CONF 208       // In: Скинути конфігурацію
#define MSP_SET_WP 209           // In: Встановити точку маршруту
#define MSP_SELECT_SETTING 210   // In: Вибрати профіль налаштувань
#define MSP_SET_HEAD 211         // In: Встановити напрямок (yaw)
#define MSP_SET_SERVO_CONF 212   // In: Встановити конфігурацію сервоприводів
#define MSP_SET_MOTOR 213        // In: Встановити рівні газу моторів

// Internal MSP parser state
typedef struct {
    io_t *io;
    uint8_t buf[MSP_BUFFER_SIZE];
    uint8_t buf_pos;
    uint8_t step;
    uint8_t data_len;
    uint8_t cmd;
    uint8_t checksum;
    time_micros_t last_frame_recv;
} msp_state_t;

// Public MSP parser structure
typedef struct {
    protocol_parser_t parser;
    msp_state_t state;
    atp_t *atp_ctx;
} msp_parser_t;

void msp_parser_init(msp_parser_t *instance, atp_t *atp_ctx);
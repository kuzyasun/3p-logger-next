#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <util/time-util.h>

#include "protocols/protocol_parser.h"

#define MSP2_BUFFER_SIZE 256  // Максимальний розмір корисного навантаження

// --- Деякі поширені команди MSPv2 (ID > 0x1000) ---
#define MSP2_INAV_STATUS 0x1501  // Out: Розширений статус (iNav)
#define MSP2_SENSOR_GPS 0x1502   // Out: Сирі дані з GPS-сенсора (iNav)
#define MSP2_NAV_POSLL 0x1504    // Out: Поточна позиція (широта, довгота)
#define MSP2_NAV_STATUS 0x1505   // Out: Статус навігації (iNav)
#define MSP2_ATTITUDE 0x2002     // Out: Кути (Pitch, Roll, Yaw)
#define MSP2_ALTITUDE 0x2003     // Out: Висота
#define MSP2_ANALOG 0x2006       // Out: Аналогові дані (батарея, RSSI, струм)
#define MSP2_RC_CHANNELS 0x2008  // Out: Значення RC-каналів

// Внутрішній стан парсера MSPv2
typedef struct {
    uint8_t step;          // Поточний крок кінцевого автомата
    uint8_t flags;         // Прапори з заголовка
    uint16_t cmd;          // ID команди (16-біт)
    uint16_t payload_len;  // Довжина корисного навантаження (16-біт)
    uint16_t payload_pos;  // Поточна позиція в буфері
    uint8_t checksum;      // Розрахована контрольна сума (CRC8)

    uint8_t buf[MSP2_BUFFER_SIZE];
    time_micros_t last_frame_recv;

} msp_v2_state_t;

/**
 * @brief Ініціалізує екземпляр парсера MSPv2.
 * @param parser Вказівник на загальний парсер.
 */
void msp_v2_parser_init(protocol_parser_t *parser);
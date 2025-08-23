#include "msp_v2.h"

#include <log.h>
#include <string.h>

#include "app_state.h"

static const char *TAG = "MSPv2";

// --- Прототипи внутрішніх функцій ---
static void msp_v2_parser_internal_init(void *parser_state);
static void msp_v2_parser_internal_process_byte(void *parser_state, uint8_t byte);
static void msp_v2_parser_internal_destroy(void *parser_state);
static void msp_v2_process_packet(msp_v2_state_t *state, uint16_t cmd, const uint8_t *payload, uint16_t len);

/**
 * @brief Розрахунок контрольної суми CRC8-DVB-S2 (поліном 0xD5).
 * Такий самий, як у CRSF.
 */
static uint8_t msp_v2_crc8(const uint8_t *data, size_t len, uint8_t crc) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// --- Публічна функція ініціалізації ---
void msp_v2_parser_init(msp_v2_parser_t *instance) {
    if (!instance) {
        LOG_E(TAG, "Invalid instance pointer");
        return;
    }

    // Ініціалізація базової структури парсера
    instance->parser.state = &instance->state;
    instance->parser.vtable.init = msp_v2_parser_internal_init;
    instance->parser.vtable.process_byte = msp_v2_parser_internal_process_byte;
    instance->parser.vtable.destroy = msp_v2_parser_internal_destroy;
    instance->parser.is_initialized = false;

    // Ініціалізація внутрішнього стану
    memset(&instance->state, 0, sizeof(msp_v2_state_t));

    LOG_I(TAG, "parser instance initialized");
}

// --- Реалізація vtable-функцій ---

static void msp_v2_parser_internal_init(void *parser_state) {
    msp_v2_state_t *state = (msp_v2_state_t *)parser_state;
    if (!state) return;
    memset(state, 0, sizeof(msp_v2_state_t));
    LOG_D(TAG, "parser state reset");
}

static void msp_v2_parser_internal_destroy(void *parser_state) {
    // Для простоти, destroy робить те саме, що й init
    msp_v2_parser_internal_init(parser_state);
}

static void msp_v2_parser_internal_process_byte(void *parser_state, uint8_t byte) {
    msp_v2_state_t *state = (msp_v2_state_t *)parser_state;
    if (!state) return;

    // Оновлюємо CRC для всіх байтів після преамбули
    if (state->step > 2) {
        state->checksum = msp_v2_crc8(&byte, 1, state->checksum);
    }

    switch (state->step) {
        case 0:  // Sync '$'
            if (byte == '$') state->step = 1;
            break;
        case 1:  // Sync 'X'
            state->step = (byte == 'X') ? 2 : 0;
            break;
        case 2:  // Direction '<' або '>'
            if (byte == '<' || byte == '>') {
                state->checksum = 0;  // Починаємо розрахунок CRC з цього байта
                state->checksum = msp_v2_crc8(&byte, 1, state->checksum);
                state->step = 3;
            } else {
                state->step = 0;
            }
            break;
        case 3:  // Flags
            state->flags = byte;
            state->step = 4;
            break;
        case 4:  // Command LSB
            state->cmd = byte;
            state->step = 5;
            break;
        case 5:  // Command MSB
            state->cmd |= (uint16_t)byte << 8;
            state->step = 6;
            break;
        case 6:  // Payload Length LSB
            state->payload_len = byte;
            state->step = 7;
            break;
        case 7:  // Payload Length MSB
            state->payload_len |= (uint16_t)byte << 8;
            if (state->payload_len > MSP2_BUFFER_SIZE) {
                LOG_W(TAG, "Payload too large: %d", state->payload_len);
                state->step = 0;  // Помилка, скидаємо
            } else if (state->payload_len == 0) {
                state->step = 9;  // Немає даних, переходимо до CRC
            } else {
                state->payload_pos = 0;
                state->step = 8;
            }
            break;
        case 8:  // Payload
            state->buf[state->payload_pos++] = byte;
            if (state->payload_pos >= state->payload_len) {
                state->step = 9;
            }
            break;
        case 9:  // Checksum
            // 'byte' - це отриманий CRC. 'state->checksum' - це розрахований CRC
            // для всіх попередніх байтів. Ми повинні "видалити" останній байт (отриманий CRC),
            // щоб порівняти його.
            // Наш розрахований CRC вже включає все до payload включно.
            // Отриманий CRC не повинен входити в розрахунок.
            // Тому state->checksum має бути розрахований на байтах до цього.

            // Правильна логіка: CRC розраховується на Direction, Flags, Cmd, Len, Payload.
            // Отриманий байт (byte) є фінальним CRC для порівняння.

            // У кроці 2 ми починаємо розрахунок CRC. На кроці 8 він завершується.
            // Тепер ми порівнюємо.
            if (state->checksum == byte) {
                state->last_frame_recv = time_micros_now();
                msp_v2_process_packet(state, state->cmd, state->buf, state->payload_len);
            } else {
                LOG_W(TAG, "checksum mismatch! Got: 0x%02X, Calc: 0x%02X", byte, state->checksum);
            }
            state->step = 0;  // Готові до наступного пакета
            break;
        default:
            state->step = 0;
            break;
    }
}

// --- Обробка вмісту пакетів ---

static void msp_v2_process_packet(msp_v2_state_t *state, uint16_t cmd, const uint8_t *payload, uint16_t len) {
    app_state_t *app_state = app_state_get_instance();

    switch (cmd) {
        case MSP2_SENSOR_GPS: {
            if (len >= 20) {
                // iNav структура: lat, lon, alt, gnd_speed, gnd_course, sats, hdop, ...
                int32_t lat, lon;
                int32_t alt_msl;      // Висота над рівнем моря в мм
                uint32_t gnd_speed;   // Швидкість в мм/с
                uint16_t gnd_course;  // Курс в 0.1 градуса
                uint8_t sats = payload[18];
                uint8_t fix_type = payload[19];

                memcpy(&lat, payload, sizeof(int32_t));
                memcpy(&lon, payload + 4, sizeof(int32_t));
                memcpy(&alt_msl, payload + 8, sizeof(int32_t));
                memcpy(&gnd_speed, payload + 12, sizeof(uint32_t));
                memcpy(&gnd_course, payload + 16, sizeof(uint16_t));

                if (fix_type >= 3) {  // 3D-Fix
                    app_state_begin_update();
                    app_state_set_i32(APP_STATE_FIELD_PLANE_LATITUDE, &app_state->plane_latitude, lat);
                    app_state_set_i32(APP_STATE_FIELD_PLANE_LONGITUDE, &app_state->plane_longitude, lon);
                    app_state_set_i32(APP_STATE_FIELD_PLANE_ALTITUDE, &app_state->plane_altitude, alt_msl / 1000);  // в метри
                    app_state_set_i16(APP_STATE_FIELD_PLANE_SPEED, &app_state->plane_speed, gnd_speed / 10);        // в см/с
                    app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, gnd_course / 10);   // в градуси
                    app_state_set_u8(APP_STATE_FIELD_PLANE_STAR, &app_state->plane_star, sats);
                    app_state_set_u8(APP_STATE_FIELD_PLANE_FIX, &app_state->plane_fix, fix_type);
                    app_state_end_update();

                    LOG_D(TAG, "GPS: Lat=%d Lon=%d Alt=%dm Sats=%d", lat, lon, alt_msl / 1000, sats);
                }
            }
            break;
        }

        case MSP2_ATTITUDE: {
            if (len >= 8) {
                int32_t roll, pitch, yaw;  // Кути в 1/1000000 радіан
                memcpy(&roll, payload, 4);
                memcpy(&pitch, payload + 4, 4);
                // Yaw може бути в іншому пакеті в деяких прошивках

                const float rad_to_deg = 57.295779513f;
                float roll_deg = (float)roll * 1e-6f * rad_to_deg;
                float pitch_deg = (float)pitch * 1e-6f * rad_to_deg;

                app_state_begin_update();
                app_state_set_i16(APP_STATE_FIELD_PLANE_ROLL, &app_state->plane_roll, (int16_t)roll_deg);
                app_state_set_i16(APP_STATE_FIELD_PLANE_PITCH, &app_state->plane_pitch, (int16_t)pitch_deg);
                app_state_end_update();
                LOG_D(TAG, "Attitude: Roll=%.1f Pitch=%.1f", roll_deg, pitch_deg);
            }
            break;
        }

        case MSP2_ANALOG: {
            if (len >= 4) {
                // Напруга (мВ), струм (мА), RSSI (%), використано (мАг)
                uint16_t vbat_mv = payload[0] | (payload[1] << 8);
                uint16_t mah_drawn = payload[2] | (payload[3] << 8);

                app_state_begin_update();
                app_state_set_u16(APP_STATE_FIELD_PLANE_VBAT, &app_state->plane_vbat, vbat_mv);
                app_state_set_u16(APP_STATE_FIELD_PLANE_BATTERY, &app_state->plane_battery, mah_drawn);
                // Інші поля (RSSI, amps) можуть бути далі в пакеті, якщо len > 4
                app_state_end_update();
                LOG_D(TAG, "Analog: VBat=%.2fV, mAh=%u", (float)vbat_mv / 1000.0f, mah_drawn);
            }
            break;
        }

        default:
            LOG_D(TAG, "Unhandled cmd 0x%04X with len %u", cmd, len);
            break;
    }
}
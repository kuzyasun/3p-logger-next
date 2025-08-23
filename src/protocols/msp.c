#include "msp.h"

#include <log.h>
#include <stdio.h>
#include <string.h>

#include "app_state.h"

static const char *TAG = "MSP";

static void msp_parser_internal_init(void *parser_state);
static void msp_parser_internal_process_byte(void *parser_state, uint8_t byte);
static void msp_parser_internal_destroy(void *parser_state);
static void msp_process_packet(msp_state_t *state, uint8_t cmd, const uint8_t *payload, size_t len);

// --- Глобальні змінні для стану, що не залежить від одного пакета ---
static char box_names[256] = {0};      // Буфер для зберігання назв режимів
static uint32_t active_box_flags = 0;  // Бітова маска активних режимів

void msp_parser_init(msp_parser_t *instance) {
    if (!instance) {
        LOG_E(TAG, "Invalid instance pointer");
        return;
    }

    instance->parser.state = &instance->state;
    instance->parser.vtable.init = msp_parser_internal_init;
    instance->parser.vtable.process_byte = msp_parser_internal_process_byte;
    instance->parser.vtable.destroy = msp_parser_internal_destroy;
    instance->parser.is_initialized = false;

    memset(&instance->state, 0, sizeof(msp_state_t));

    LOG_I(TAG, "parser instance initialized");
}

static void msp_parser_internal_init(void *parser_state) {
    msp_state_t *state = (msp_state_t *)parser_state;
    if (!state) return;

    state->buf_pos = 0;
    state->step = 0;
    state->data_len = 0;
    state->cmd = 0;
    state->checksum = 0;
    state->last_frame_recv = 0;
    memset(state->buf, 0, sizeof(state->buf));

    LOG_D(TAG, "parser state initialized");
}

static void msp_parser_internal_process_byte(void *parser_state, uint8_t byte) {
    msp_state_t *state = (msp_state_t *)parser_state;
    if (!state) return;

    switch (state->step) {
        case 0:  // '$'
            if (byte == '$') state->step = 1;
            break;
        case 1:  // 'M'
            if (byte == 'M')
                state->step = 2;
            else
                state->step = 0;
            break;
        case 2:  // direction
            if (byte == '>' || byte == '<')
                state->step = 3;
            else
                state->step = 0;
            break;
        case 3:  // data length
            state->data_len = byte;
            state->checksum = byte;
            state->buf_pos = 0;
            state->step = 4;
            break;
        case 4:  // command
            state->cmd = byte;
            state->checksum ^= byte;
            state->step = (state->data_len > 0) ? 5 : 6;
            break;
        case 5:  // payload
            if (state->buf_pos < MSP_BUFFER_SIZE) {
                state->buf[state->buf_pos++] = byte;
            }
            state->checksum ^= byte;
            if (state->buf_pos >= state->data_len) {
                state->step = 6;
            }
            break;
        case 6:  // checksum
            if (byte == state->checksum) {
                time_micros_t now = time_micros_now();
                state->last_frame_recv = now;
                msp_process_packet(state, state->cmd, state->buf, state->data_len);
            } else {
                LOG_W(TAG, "checksum mismatch");
            }
            state->step = 0;
            break;
        default:
            state->step = 0;
            break;
    }
}

static void msp_parser_internal_destroy(void *parser_state) {
    msp_state_t *state = (msp_state_t *)parser_state;
    if (!state) return;
    memset(state, 0, sizeof(msp_state_t));
    LOG_D(TAG, "parser state destroyed");
}

static void update_flight_mode_string(app_state_t *app_state) {
    if (box_names[0] == '\0') {
        // Якщо назви режимів ще не завантажені, нічого не робимо
        return;
    }

    char flight_mode_str[64] = {0};
    char temp_box_names[256];
    strncpy(temp_box_names, box_names, sizeof(temp_box_names) - 1);

    char *name = strtok(temp_box_names, ";");
    int i = 0;
    while (name != NULL) {
        if ((active_box_flags & (1 << i))) {
            if (flight_mode_str[0] != '\0') {
                strncat(flight_mode_str, ", ", sizeof(flight_mode_str) - strlen(flight_mode_str) - 1);
            }
            strncat(flight_mode_str, name, sizeof(flight_mode_str) - strlen(flight_mode_str) - 1);
        }
        name = strtok(NULL, ";");
        i++;
    }

    if (flight_mode_str[0] != '\0') {
        app_state_begin_update();
        strncpy(app_state->plane_flight_mode, flight_mode_str, sizeof(app_state->plane_flight_mode) - 1);
        app_state->plane_flight_mode[sizeof(app_state->plane_flight_mode) - 1] = '\0';
        app_state_set_u32(APP_STATE_FIELD_PLANE_FLIGHT_MODE, (uint32_t *)&app_state->plane_flight_mode, 1);
        app_state_end_update();
    }
}

static void msp_process_packet(msp_state_t *state, uint8_t cmd, const uint8_t *payload, size_t len) {
    app_state_t *app_state = app_state_get_instance();

    switch (cmd) {
        case MSP_IDENT: {
            if (len >= 7) {
                uint8_t version = payload[0];
                uint8_t multitype = payload[1];
                uint8_t msp_version = payload[2];
                LOG_I(TAG, "IDENT: FW Ver: %d.%d, Protocol Ver: %d, CopterType: %d", version / 100, (version % 100) / 10, msp_version, multitype);
            }
            break;
        }

        case MSP_STATUS: {
            if (len >= 11) {
                uint16_t cycleTime, i2cErrors, sensors;
                uint32_t flightModeFlags;

                memcpy(&cycleTime, payload, 2);
                memcpy(&i2cErrors, payload + 2, 2);
                memcpy(&sensors, payload + 4, 2);
                memcpy(&flightModeFlags, payload + 6, 4);

                // Прапор ARM зазвичай є першим бітом (BOXARM)
                bool is_armed = (flightModeFlags & 1);

                app_state_begin_update();
                app_state_set_u8(APP_STATE_FIELD_PLANE_ARMED, &app_state->plane_armed, is_armed ? 1 : 0);
                app_state_end_update();

                LOG_D(TAG, "STATUS: Armed: %d, CycleTime: %uus, I2C Errors: %u, Sensors: 0x%X", is_armed, cycleTime, i2cErrors, sensors);
            }
            break;
        }

        case MSP_RAW_IMU: {
            if (len >= 18) {
                int16_t acc[3], gyro[3], mag[3];
                memcpy(acc, payload, 6);
                memcpy(gyro, payload + 6, 6);
                memcpy(mag, payload + 12, 6);

                LOG_D(TAG, "RAW_IMU: ACC(x,y,z)=%d,%d,%d | GYRO(x,y,z)=%d,%d,%d | MAG(x,y,z)=%d,%d,%d", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2],
                      mag[0], mag[1], mag[2]);
            }
            break;
        }

        case MSP_SERVO: {
            if (len > 0 && (len % 2 == 0)) {
                const uint8_t num_servos = len / 2;
                uint16_t servos[16];
                char log_str[128] = "SERVOS: ";

                for (int i = 0; i < num_servos && i < 16; i++) {
                    memcpy(&servos[i], payload + (i * 2), sizeof(uint16_t));
                    char temp[12];
                    sprintf(temp, "S%d=%u ", i + 1, servos[i]);
                    strcat(log_str, temp);
                }
                LOG_D(TAG, "%s", log_str);
            }
            break;
        }

        case MSP_MOTOR: {
            if (len > 0 && (len % 2 == 0)) {
                const uint8_t num_motors = len / 2;
                uint16_t motors[8];
                char motor_log[128] = "MOTORS: ";
                for (int i = 0; i < num_motors && i < 8; i++) {
                    memcpy(&motors[i], payload + (i * 2), sizeof(uint16_t));
                    char temp[10];
                    sprintf(temp, "M%d=%u ", i + 1, motors[i]);
                    strcat(motor_log, temp);
                }
                LOG_D(TAG, "%s", motor_log);
            }
            break;
        }

        case MSP_RC: {
            if (len > 0 && (len % 2 == 0)) {
                const uint8_t num_channels = len / 2;
                app_state_update_rc_channels((const uint16_t *)payload, num_channels);
                LOG_D(TAG, "RC: Processed %d channels.", num_channels);
            }
            break;
        }

        case MSP_RAW_GPS: {
            if (len >= 16) {
                uint8_t fix = payload[0];
                uint8_t numsat = payload[1];
                int32_t lat = 0, lon = 0;
                int16_t alt = 0;
                uint16_t spd = 0, course = 0;

                memcpy(&lat, payload + 2, sizeof(int32_t));
                memcpy(&lon, payload + 6, sizeof(int32_t));
                memcpy(&alt, payload + 10, sizeof(int16_t));
                memcpy(&spd, payload + 12, sizeof(uint16_t));
                memcpy(&course, payload + 14, sizeof(uint16_t));

                if (fix) {
                    app_state_begin_update();
                    app_state_set_i32(APP_STATE_FIELD_PLANE_LONGITUDE, &app_state->plane_longitude, lon);
                    app_state_set_i32(APP_STATE_FIELD_PLANE_LATITUDE, &app_state->plane_latitude, lat);
                    app_state_set_i32(APP_STATE_FIELD_PLANE_ALTITUDE, &app_state->plane_altitude, (int32_t)alt);
                    app_state_set_i16(APP_STATE_FIELD_PLANE_SPEED, &app_state->plane_speed, (int16_t)(spd));
                    app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, (int16_t)(course));
                    app_state_set_u8(APP_STATE_FIELD_PLANE_STAR, &app_state->plane_star, numsat);
                    app_state_set_u8(APP_STATE_FIELD_PLANE_FIX, &app_state->plane_fix, fix);
                    app_state_end_update();
                    LOG_D(TAG, "RAW_GPS: Lat=%d Lon=%d Alt=%dm Speed=%dcm/s Course=%ddeg", lat, lon, alt, spd, course);
                }
            }
            break;
        }

        case MSP_COMP_GPS: {
            if (len >= 4) {
                int16_t dist_to_home, dir_to_home;
                memcpy(&dist_to_home, payload, sizeof(int16_t));
                memcpy(&dir_to_home, payload + 2, sizeof(int16_t));

                app_state_begin_update();
                app_state_set_i16(APP_STATE_FIELD_PLANE_HOME_DIST, &app_state->plane_home_dist, dist_to_home);
                app_state_set_i16(APP_STATE_FIELD_PLANE_HOME_DIR, &app_state->plane_home_dir, dir_to_home);
                app_state_end_update();
                LOG_D(TAG, "COMP_GPS: DistHome=%dm, DirHome=%ddeg", dist_to_home, dir_to_home);
            }
            break;
        }

        case MSP_ATTITUDE: {
            if (len >= 6) {
                int16_t roll, pitch, yaw;
                memcpy(&roll, payload, sizeof(int16_t));
                memcpy(&pitch, payload + 2, sizeof(int16_t));
                memcpy(&yaw, payload + 4, sizeof(int16_t));

                app_state_begin_update();
                app_state_set_i16(APP_STATE_FIELD_PLANE_ROLL, &app_state->plane_roll, roll / 10);
                app_state_set_i16(APP_STATE_FIELD_PLANE_PITCH, &app_state->plane_pitch, pitch / 10);
                app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, yaw);
                app_state_end_update();
                LOG_D(TAG, "ATTITUDE: Roll=%.1f Pitch=%.1f Yaw=%d", (float)roll / 10.0f, (float)pitch / 10.0f, yaw);
            }
            break;
        }

        case MSP_ALTITUDE: {
            if (len >= 6) {
                int32_t alt;
                int16_t vario;
                memcpy(&alt, payload, sizeof(int32_t));
                memcpy(&vario, payload + 4, sizeof(int16_t));

                app_state_begin_update();
                app_state_set_i32(APP_STATE_FIELD_PLANE_BARO_ALTITUDE, &app_state->plane_baro_altitude, alt / 100);
                app_state_set_i16(APP_STATE_FIELD_PLANE_VSPEED, &app_state->plane_vspeed, vario);
                app_state_end_update();
                LOG_D(TAG, "ALTITUDE: Alt=%.2fm Vario=%.2fm/s", (float)alt / 100.0f, (float)vario / 100.0f);
            }
            break;
        }

        case MSP_ANALOG: {
            if (len >= 7) {
                uint8_t vbat = payload[0];
                uint16_t mah_drawn, rssi;
                int16_t amps;

                memcpy(&mah_drawn, payload + 1, sizeof(uint16_t));
                memcpy(&rssi, payload + 3, sizeof(uint16_t));
                memcpy(&amps, payload + 5, sizeof(int16_t));

                app_state_begin_update();
                app_state_set_u16(APP_STATE_FIELD_PLANE_VBAT, &app_state->plane_vbat, vbat * 100);
                app_state_set_u16(APP_STATE_FIELD_PLANE_BATTERY, &app_state->plane_battery, mah_drawn);
                app_state_set_u8(APP_STATE_FIELD_PLANE_RSSI, &app_state->plane_rssi, (uint8_t)(rssi * 100 / 1023));
                app_state_set_u16(APP_STATE_FIELD_PLANE_ESC_CURRENT, &app_state->plane_esc_current, (uint16_t)(amps * 10));
                app_state_end_update();
                LOG_D(TAG, "ANALOG: VBat=%.1fV, mAh=%u, RSSI=%u, Amps=%.2fA", (float)vbat / 10.0f, mah_drawn, rssi, (float)amps / 100.0f);
            }
            break;
        }

        case MSP_BOXNAMES: {
            if (len > 0) {
                memset(box_names, 0, sizeof(box_names));
                memcpy(box_names, payload, len < sizeof(box_names) ? len : sizeof(box_names) - 1);
                LOG_I(TAG, "BOXNAMES: %s", box_names);
            }
            break;
        }

        case MSP_BOX: {
            if (len > 0 && (len % 2 == 0)) {
                uint32_t flags = 0;
                if (len >= 2) {
                    uint16_t flags_low;
                    memcpy(&flags_low, payload, sizeof(uint16_t));
                    flags = flags_low;
                }
                if (len >= 4) {
                    uint16_t flags_high;
                    memcpy(&flags_high, payload + 2, sizeof(uint16_t));
                    flags |= ((uint32_t)flags_high << 16);
                }
                active_box_flags = flags;
                update_flight_mode_string(app_state);
                LOG_D(TAG, "BOX: Active flags=0x%08X", active_box_flags);
            }
            break;
        }

        default: {
            LOG_D(TAG, "Unhandled cmd %u with len %u", cmd, len);
            break;
        }
    }
}
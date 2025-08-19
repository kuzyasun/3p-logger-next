#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct protocol_parser_vtable_s {
    void (*init)(void *parser_state);
    void (*process_byte)(void *parser_state, uint8_t byte);
    void (*destroy)(void *parser_state);
    void (*process_packet)(void *parser_state, const uint8_t *data, size_t len);
} protocol_parser_vtable_t;

typedef struct {
    void *state;
    protocol_parser_vtable_t vtable;
    bool is_initialized;
} protocol_parser_t;

#ifdef __cplusplus
}
#endif

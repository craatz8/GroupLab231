/**************************************************************************//**
 *
 * @file rotary-encoder.c
 *
 * @author Connor Raatz
 * @author Daniel Kasparek
 *
 * @brief Code to determine the direction that a rotary encoder is turning.
 *
 ******************************************************************************/

/*
 * ComboLock GroupLab assignment and starter code (c) 2022-24 Christopher A. Bohn
 * ComboLock solution (c) the above-named students
 */

#include <CowPi.h>
#include "interrupt_support.h"
#include "rotary-encoder.h"

#define A_WIPER_PIN         (16)
#define B_WIPER_PIN         (A_WIPER_PIN + 1)

typedef enum {
    HIGH_HIGH, HIGH_LOW, LOW_LOW, LOW_HIGH, UNKNOWN
} rotation_state_t;

static rotation_state_t volatile state;
static direction_t volatile direction = STATIONARY;
static int volatile clockwise_count = 0;
static int volatile counterclockwise_count = 0;

static void handle_quadrature_interrupt();
static rotation_state_t get_state_from_quadrature(uint8_t quadrature);

void initialize_rotary_encoder() {
    cowpi_set_pullup_input_pins((1 << A_WIPER_PIN) | (1 << B_WIPER_PIN));
    state = UNKNOWN;
    register_pin_ISR((1 << A_WIPER_PIN) | (1 << B_WIPER_PIN), handle_quadrature_interrupt);
}

uint8_t get_quadrature() {
    volatile cowpi_ioport_t *ioport = (cowpi_ioport_t *)(0xD0000000);
    uint32_t input_pins_state = ioport->input;
    uint8_t quadrature = 0;

    // Extract the state of A_WIPER_PIN and B_WIPER_PIN
    quadrature |= ((input_pins_state >> A_WIPER_PIN) & 0x1) << 1; // A in bit 1
    quadrature |= ((input_pins_state >> B_WIPER_PIN) & 0x1) << 0; // B in bit 0

    return quadrature;
}

static rotation_state_t get_state_from_quadrature(uint8_t quadrature) {
    switch (quadrature) {
        case 0b11: // A=1, B=1
            return HIGH_HIGH;
        case 0b10: // A=1, B=0
            return LOW_HIGH;
        case 0b00: // A=0, B=0
            return LOW_LOW;
        case 0b01: // A=0, B=1
            return HIGH_LOW;
        default:
            return UNKNOWN;
    }
}

static void handle_quadrature_interrupt() {
    static rotation_state_t last_state = UNKNOWN;
    uint8_t quadrature = get_quadrature();
    rotation_state_t current_state = get_state_from_quadrature(quadrature);

    // Ignore if the state hasn't changed
    if (current_state == state) {
        return;
    }

    // Debouncing logic
    switch (current_state) {
        case LOW_LOW:
            if ((state == HIGH_LOW && last_state == HIGH_HIGH)) {
                // Valid clockwise rotation
                clockwise_count++;
                direction = CLOCKWISE;
            } else if ((state == LOW_HIGH && last_state == HIGH_HIGH)) {
                // Valid counterclockwise rotation
                counterclockwise_count++;
                direction = COUNTERCLOCKWISE;
            } else {
                // Invalid transition due to bounce; ignore
                current_state = state; // Stay in the same state
                break;
            }
            break;

        case HIGH_HIGH:
            // Always allow transition into HIGH_HIGH
            break;

        case HIGH_LOW:
        case LOW_HIGH:
            // Allow transitions if they follow the expected sequence
            break;

        default:
            // Unknown state; ignore
            current_state = state; // Stay in the same state
            break;
    }

    // Update state variables
    last_state = state;
    state = current_state;
}


char *count_rotations(char *buffer) {
    sprintf(buffer, "CW:%d CCW:%d", clockwise_count, counterclockwise_count);
    return buffer;
}

direction_t get_direction() {
    direction_t current_direction = direction;
    direction = STATIONARY;
    return current_direction;
}
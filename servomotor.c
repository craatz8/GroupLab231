/**************************************************************************//**
 *
 * @file rotary-encoder.c
 *
 * @author (STUDENTS -- TYPE YOUR NAME HERE)
 * @author (STUDENTS -- TYPE YOUR NAME HERE)
 *
 * @brief Code to control a servomotor.
 *
 ******************************************************************************/

/*
 * ComboLock GroupLab assignment and starter code (c) 2022-24 Christopher A. Bohn
 * ComboLock solution (c) the above-named students
 */

#include <CowPi.h>
#include "servomotor.h"
#include "interrupt_support.h"

#define SERVO_PIN           (22)
#define PULSE_INCREMENT_uS  (500)
#define SIGNAL_PERIOD_uS    (20000)

static int volatile pulse_width_us;

//static cowpi_ioport_t volatile *ioport;
//ioport = (cowpi_ioport_t *) (0xD0000000);
static cowpi_ioport_t *ioport = (cowpi_ioport_t *)(0xD0000000);


static u_int32_t time_until_next_rising_edge = SIGNAL_PERIOD_uS;
static u_int32_t time_until_next_falling_edge = 0;

static void handle_timer_interrupt();

void initialize_servo() {
    cowpi_set_output_pins(1 << SERVO_PIN);
    center_servo();
    register_periodic_timer_ISR(0, PULSE_INCREMENT_uS, handle_timer_interrupt);
}

char *test_servo(char *buffer) {
    if (cowpi_left_button_is_pressed()) {
        center_servo();
        //write to buffer
        sprintf(buffer, "SERVO: center");
    }
    else if (!cowpi_left_button_is_pressed()) {
        if (cowpi_left_switch_is_in_left_position()) {
            rotate_full_clockwise();
            sprintf(buffer, "SERVO: right");
        }
        else {
            rotate_full_counterclockwise();
            sprintf(buffer, "SERVO: left");
        }
    }


    ;
    return buffer;
}

void center_servo() {
    pulse_width_us = 1500;
    handle_timer_interrupt()
    ;
}

void rotate_full_clockwise() {
    pulse_width_us = 500;
    handle_timer_interrupt()
    ;
}

void rotate_full_counterclockwise() {
    pulse_width_us = 2500;
    handle_timer_interrupt()
    ;
}

static void handle_timer_interrupt() {
    // Update remaining times for the rising and falling edges
    if (time_until_next_rising_edge > 0) {
        time_until_next_rising_edge -= PULSE_INCREMENT_uS;
    }
    if (time_until_next_falling_edge > 0) {
        time_until_next_falling_edge -= PULSE_INCREMENT_uS;
    }

    // Handle rising edge
    if (time_until_next_rising_edge == 0) {
        ioport->output |= (1 << SERVO_PIN); // Set SERVO_PIN to 1
        time_until_next_rising_edge = SIGNAL_PERIOD_uS; // Reset time for the next pulse
        time_until_next_falling_edge = pulse_width_us;  // Set time for the pulse width
    }

    // Handle falling edge
    if (time_until_next_falling_edge == 0) {
        ioport->output &= ~(1 << SERVO_PIN); // Set SERVO_PIN to 0
    }

    ;
}

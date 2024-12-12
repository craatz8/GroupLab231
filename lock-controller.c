/**************************************************************************//**
 *
 * @file lock-controller.c
 *
 * @author Connor Raatz
 * @author Daniel Kasparek
 *
 * @brief Code to implement the "combination lock" mode.
 *
 ******************************************************************************/

/*
 * ComboLock GroupLab assignment and starter code (c) 2022-24 Christopher A. Bohn
 * ComboLock solution (c) the above-named students
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include <CowPi.h>             // For LED and input functions
#include "display.h"           // For display_string()
#include "lock-controller.h"
#include "rotary-encoder.h"    // direction_t, get_direction()
#include "servomotor.h"        // rotate_full_clockwise(), rotate_full_counterclockwise()

// Lock states
typedef enum {
    LOCKED,
    UNLOCKED,
    ALARMED,
    CHANGING,
    SHOWING_BAD_TRY // New state to handle bad attempts
} lock_state_t;

static lock_state_t lock_state;

// Combination arrays
static uint8_t combination[3] = {5, 10, 15};        // Default combination
static uint8_t entered_combination[3] = {0, 0, 0}; // Entered combination

// Current digit being set (0: first, 1: second, 2: third)
static uint8_t current_digit = 0;

// Attempt count for incorrect tries
static uint8_t attempt_count = 0;

// Expected rotation directions for each digit
static direction_t expected_directions[3] = {CLOCKWISE, COUNTERCLOCKWISE, CLOCKWISE};

// Timer variables for non-blocking delays
static uint32_t bad_try_start_time = 0;
static uint8_t bad_try_blinks_remaining = 0;
static bool is_bad_try_blinking = false;

// Button state tracking for edge detection
static bool last_left_button_state = false;

static uint8_t num_first_entry_turns = 1;
static uint8_t num_second_entry_turns = 1;
static uint8_t num_third_entry_turns = 1;

// Helper function prototypes
static void illuminate_locked_state_leds(void);
static void illuminate_unlocked_state_leds(void);
static void toggle_alarm_leds(void);
static void display_combination(void);
static void display_open_message(void);
static void display_locked_message(void);
static void display_alarm_message(void);
static void display_bad_try(uint8_t attempt);
static void unlock_system(void);
static void lock_system(void);
static void handle_incorrect_attempt(uint8_t attempt);
static void handle_combination_entry(void);
static bool evaluate_combination(void);
static void increment_current_digit(void);

// Alarm LED toggle tracking
static bool alarm_leds_on = false;

// LED Control Functions
static void illuminate_locked_state_leds() {
    cowpi_illuminate_left_led();    // Internal LED on
    cowpi_deluminate_right_led();   // External LED off
}

static void illuminate_unlocked_state_leds() {
    cowpi_deluminate_left_led();    // Internal LED off
    cowpi_illuminate_right_led();   // External LED on
}

static void toggle_alarm_leds() {
    if (alarm_leds_on) {
        cowpi_deluminate_left_led();
        cowpi_deluminate_right_led();
    } else {
        cowpi_illuminate_left_led();
        cowpi_illuminate_right_led();
    }
    alarm_leds_on = !alarm_leds_on;
}

// Display Helper Functions
static void display_combination() {
    char display_buffer[32]; // Increased buffer size to prevent overflow

    if (current_digit == 0) {
        snprintf(display_buffer, sizeof(display_buffer), "%02d-  -  ", entered_combination[0]);
        display_string(0, display_buffer);
    }
    else if (current_digit == 1) {
        snprintf(display_buffer, sizeof(display_buffer), "%02d-%02d-  ", entered_combination[0], entered_combination[1]);
        display_string(0, display_buffer);
    }
    else if (current_digit == 2) {
        snprintf(display_buffer, sizeof(display_buffer), "%02d-%02d-%02d",
                entered_combination[0],
                entered_combination[1],
                entered_combination[2]);
        display_string(0, display_buffer);
    }
}

static void display_open_message() {
    display_string(0, "OPEN");
}

static void display_locked_message() {
    display_string(0, "  -  -  ");
}

static void display_alarm_message() {
    display_string(0, "ALERT!");
}

static void display_bad_try(uint8_t attempt) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "Bad Try %d", attempt);
    display_string(0, buffer);
}

// Initialization
void initialize_lock_controller(void) {
    lock_state = LOCKED;
    current_digit = 0;
    memset(entered_combination, 0, sizeof(entered_combination));
    illuminate_locked_state_leds();
    rotate_full_clockwise(); // Move servo to locked position
    display_locked_message();
}

// Control logic
void control_lock(void) {
    switch (lock_state) {
        case LOCKED:
            handle_combination_entry();
            break;

        case UNLOCKED:
            if (cowpi_left_button_is_pressed() && cowpi_right_button_is_pressed()) {
                lock_system();
            }

            if (cowpi_left_switch_is_in_right_position() && cowpi_right_button_is_pressed()) {

            }

            
            break;

        case ALARMED:
            // Blink both LEDs continuously
            toggle_alarm_leds();
            busy_wait_ms(250);
            break;

        case CHANGING:
            display_string(0, "CHANGE COMBO");
            busy_wait_ms(500);
            // Reset entered combination and current digit
            memset(entered_combination, 0, sizeof(entered_combination));
            current_digit = 0;
            lock_state = LOCKED;
            illuminate_locked_state_leds();
            rotate_full_clockwise();
            display_locked_message();
            break;

        case SHOWING_BAD_TRY:
            // Handle the "Bad Try X" display and LED blinking
            if (!is_bad_try_blinking) {
                // Initialize blinking
                is_bad_try_blinking = true;
                bad_try_start_time = millis();
                bad_try_blinks_remaining = 2; // Number of blinks
            } else {
                uint32_t current_time = millis();
                if (current_time - bad_try_start_time >= 500) { // 500 ms per blink
                    // Toggle LEDs
                    toggle_alarm_leds();
                    bad_try_start_time = current_time;
                    if (--bad_try_blinks_remaining == 0) {
                        // Blinking finished
                        is_bad_try_blinking = false;
                        // Reset combination and digit
                        memset(entered_combination, 0, sizeof(entered_combination));
                        current_digit = 0;
                        lock_state = LOCKED;
                        illuminate_locked_state_leds();
                        display_locked_message(); // Shows "  -  -  "
                    }
                }
            }
            break;

        default:
            break;
    }

    // Button Edge Detection for LOCKED state
    if (lock_state == LOCKED) {
        bool current_left_button_state = cowpi_left_button_is_pressed();

        // Detect rising edge (button pressed)
        if (current_left_button_state && !last_left_button_state) {
            if (evaluate_combination()) {
                unlock_system();
            } else {
                handle_incorrect_attempt(++attempt_count);
            }
        }

        last_left_button_state = current_left_button_state;
    }
}

// Unlock and Lock functions
static void unlock_system(void) {
    lock_state = UNLOCKED;
    illuminate_unlocked_state_leds();
    rotate_full_counterclockwise(); // Move servo to unlocked position
    display_open_message();
}

static void lock_system(void) {
    lock_state = LOCKED;
    illuminate_locked_state_leds();
    rotate_full_clockwise(); // Move servo to locked position
    display_locked_message();
}

// Handling incorrect attempts
static void handle_incorrect_attempt(uint8_t attempt) {
    display_bad_try(attempt); // Display "Bad Try X" on row 0

    if (attempt >= 3) {
        // Transition to ALARMED state
        lock_state = ALARMED;
        display_alarm_message();
    } else {
        // Transition to SHOWING_BAD_TRY state
        lock_state = SHOWING_BAD_TRY;
        is_bad_try_blinking = false; // Initialize blinking in the new state
    }
}

// Handle combination entry using rotary encoder
static void handle_combination_entry() {
    direction_t dir = get_direction(); // from rotary-encoder.h

    if (dir != STATIONARY) {
        direction_t expected_dir = expected_directions[current_digit];
        if (dir == expected_dir) {
            // Rotate in expected direction: set digit's value
            entered_combination[current_digit]++;
            if (entered_combination[current_digit] > 15) {
                entered_combination[current_digit] = 0;

                if (current_digit == 0) {
                    num_first_entry_turns++;
                }
                else if (current_digit == 1) {
                    num_second_entry_turns++;
                }
                else if (current_digit == 2) {
                    num_third_entry_turns++;
                }
            }
            display_combination();
        }
        else {
            // Rotation in opposite direction: confirm digit and move to next digit
            increment_current_digit();
            display_combination();
        }
        busy_wait_ms(200); // Small delay to prevent multiple triggers
    }
}

// Increment current_digit safely
static void increment_current_digit() {
    if (current_digit < 2) {
        current_digit++;
    }
    else {
        // Wrap around to the first digit
        current_digit = 0;
    }
}

// Evaluate combination
static bool evaluate_combination() {
    bool is_correct_combination = (memcmp(combination, entered_combination, sizeof(combination)) == 0);
    bool has_correct_num_turns = (num_first_entry_turns == 3) && (num_second_entry_turns == 2) && (num_third_entry_turns == 1);
    return is_correct_combination && has_correct_num_turns;
}

uint8_t const *get_combination(void) {
    return combination;
}

void force_combination_reset(void) {
    combination[0] = 5;
    combination[1] = 10;
    combination[2] = 15;
    // Optionally, reset entered combination and current digit
    memset(entered_combination, 0, sizeof(entered_combination));
    current_digit = 0;
}

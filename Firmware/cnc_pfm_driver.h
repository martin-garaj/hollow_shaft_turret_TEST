/* Pulse-Frequency-Modulation for CNC Shield
 * 
 * This file defines the operation of PFM for controling 3-axis CNC Shield.
 * 
 * Functions defined in this file include:
 *      void    init_pfm_vars(int32_t pfm_delta_steps_x, int32_t pfm_delta_steps_y, int32_t pfm_delta_steps_z)
 *      void    set_pfm_delta_steps(uint8_t this_pfm, int32_t pfm_delta_steps_x)
 *      int32_t get_pfm_delta_steps(uint8_t this_pfm)
 *      void    init_pfm_isr(void)
 *      void    set_pfm_freq(uint32_t freq)
 *              ISR(TIMER1_COMPA_vect)
 *      void    init_cnc_pins(void)
 *      void    set_cnc_enable(void)
 *      void    set_cnc_disable(void)
 *      void    setup_pfm(int32_t pfm_delta_steps_x, int32_t pfm_delta_steps_y, int32_t pfm_delta_steps_z)
 *      void    setup_pfm(void)
 *      void    control_pfm_target_freq(uint8_t this_pfm, uint16_t pfm_target_freq, bool pfm_direction)
 *      void    control_pfm_target_delta(uint8_t this_pfm, uint16_t pfm_target_freq, int32_t pfm_target_delta)
 * 
 * Variables defined in this file:
 *      static pfm_vars pfm[NUM_PFM]
 *      static volatile uint8_t _isr_pfm_busy
 * 
*/


// ensure this library description is only included once
#ifndef pulse_freq_modul_h
#define pulse_freq_modul_h

// import Arduino headers
#include <Arduino.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
// import constants regardin CNC Shield
#include "cnc_shield.h"
// import constants regardin CNC Shield
#include "pfm_config.h"


// declare structure representing single PFM
typedef struct {
    // internal - ISR variables
    // interrupt routine counter
    volatile uint16_t _isr_pfm_counter;
    // cnc shield pin for steps
    volatile uint8_t  _cnc_shield_step_pin;
    // cnc shield pin for direction
    volatile uint8_t  _cnc_shield_dir_pin;

    // input - read/write only
    // true: target delta, false: target frequency
    volatile bool pfm_control_target_delta;
    // frequency of PFM if pfm_control_target_delta[X]=false
    volatile uint16_t pfm_target_freq;
    // target delta if if pfm_control_target_delta[X]=true
    volatile int32_t pfm_target_delta;
    // direction 
    volatile bool pfm_direction;

    // output - read only
    // number of steps from default position
    volatile int32_t pfm_delta_steps;
} pfm_vars;
static pfm_vars pfm[NUM_PFM];

// flag wehn ISR is busy
static volatile uint8_t _isr_pfm_busy;

// Initialize PFM variables
void init_pfm_vars(int32_t pfm_delta_steps_x, int32_t pfm_delta_steps_y, int32_t pfm_delta_steps_z)
{
    /*
    * Function: init_pfm_vars
    * ----------------------------
    *   Initializes Pulse-Frequency-Modulation parameters 
    *
    *   pfm_delta_steps_x: offest from default position of steper motor on x-axis
    *   pfm_delta_steps_y: offest from default position of steper motor on y-axis
    *   pfm_delta_steps_z: offest from default position of steper motor on z-axis
    *
    *   returns: void
    */
    for(uint8_t this_pfm=0; this_pfm<NUM_PFM; this_pfm++)
    {
        // for all axis
        pfm[this_pfm]._isr_pfm_counter = 0;
        pfm[this_pfm].pfm_control_target_delta = false;
        pfm[this_pfm].pfm_target_freq = INACTIVE_FREQ;
        pfm[this_pfm].pfm_target_delta = 0;
        pfm[this_pfm].pfm_direction = true;

        // X axis
        if(this_pfm==PFM_X)
        {
            pfm[this_pfm]._cnc_shield_step_pin = X_STP_PIN;
            pfm[this_pfm]._cnc_shield_dir_pin = X_DIR_PIN;
            pfm[this_pfm].pfm_delta_steps = pfm_delta_steps_x;
        }
        // Y axis
        if(this_pfm==PFM_Y)
        {
            pfm[this_pfm]._cnc_shield_step_pin = Y_STP_PIN;
            pfm[this_pfm]._cnc_shield_dir_pin = Y_DIR_PIN;
            pfm[this_pfm].pfm_delta_steps = pfm_delta_steps_y;
        }
        // Z axis
        if(this_pfm==PFM_Z)
        {
            pfm[this_pfm]._cnc_shield_step_pin = Z_STP_PIN;
            pfm[this_pfm]._cnc_shield_dir_pin = Z_DIR_PIN;
            pfm[this_pfm].pfm_delta_steps = pfm_delta_steps_z;
        }
    }
}

// setter for pfm_delta_steps_x
void set_pfm_delta_steps(uint8_t this_pfm, int32_t pfm_delta_steps_x)
{
    /*
    * Function: set_pfm_delta_steps_x
    * ----------------------------
    *   Set delta setps of stepper motor on x-axis
    *
    *   pfm_delta_steps_x: offest from default position of stepper motor on x-axis
    *
    *   returns: void
    */
    _isr_pfm_busy = true;
    pfm[this_pfm].pfm_delta_steps = pfm_delta_steps_x;
    _isr_pfm_busy = false;
}

// setter for pfm_delta_steps
int32_t get_pfm_delta_steps(uint8_t this_pfm)
{
    /// @brief Get the delta setps of a stepper motor
    /// @param this_pfm target pfm, values: PMF_X, PFM_Y, PFM_Z
    /// @return int32_t of current delta_steps
    return pfm[this_pfm].pfm_delta_steps;
}

// Initialize pfm isr
void init_pfm_isr(void)
{
    /// @brief Initialize Interrupt-Service-Routine
    /// @param void  
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0; 
    OCR1A = INTERRUPT_COUNTER;
    TCCR1B = (1<<WGM12) | (1<<CS10); // 1<<CS10 -> prescaler 1:1
    TIMSK1 = (1<<OCIE1A); 
    interrupts(); 
}

// set 
void set_pfm_freq(uint32_t freq)
{
    /*
    * Function: set_pfm_freq
    * ----------------------------
    *   Set maximum Pulse-Frequency-Modulation frequency
    *
    *   freq: frequency
    *
    *   returns: void
    */
    OCR1A = F_CPU/2/freq;
}

// Pulse-Frequency-Modulation Interrupt Service Routine (PFM ISR)
ISR(TIMER1_COMPA_vect)
{
    /*
    * Function: ISR
    * ----------------------------
    *   Interrupt-Service-Routine of Pulse-Frequency-Modulation
    */
    if (_isr_pfm_busy) { return; } // The busy-flag is used to avoid reentering this interrupt
    _isr_pfm_busy = true;

    for(uint8_t this_pfm=0; this_pfm<NUM_PFM; this_pfm++)
    {
        // default state
        if(pfm[this_pfm]._isr_pfm_counter==0 || pfm[this_pfm]._isr_pfm_counter==INACTIVE_FREQ)
        {
            // reset counter
            pfm[this_pfm]._isr_pfm_counter=0;
            // reset/default step signal
            digitalWrite(pfm[this_pfm]._cnc_shield_step_pin, LOW);
            // change direction when the step pin is reset
            digitalWrite(pfm[this_pfm]._cnc_shield_dir_pin, pfm[this_pfm].pfm_direction);
        }

        // increment counter 
        pfm[this_pfm]._isr_pfm_counter++;

        // produce signal
        if(pfm[this_pfm]._isr_pfm_counter>pfm[this_pfm].pfm_target_freq)
        {
            // reset counter
            pfm[this_pfm]._isr_pfm_counter=0;
            // assert signal
            digitalWrite(pfm[this_pfm]._cnc_shield_step_pin, HIGH);
            // update step counter (add/subtract bool)
            if(pfm[this_pfm].pfm_direction)
            {
                pfm[this_pfm].pfm_delta_steps++;
            }
            else
            {
                pfm[this_pfm].pfm_delta_steps--;
            }
        }

        // run until delta steps reached
        if(pfm[this_pfm].pfm_control_target_delta)
        {
            //if target delta steps are reached
            if(pfm[this_pfm].pfm_delta_steps==pfm[this_pfm].pfm_target_delta)
            {
                // set speed to 0
                pfm[this_pfm].pfm_target_freq = INACTIVE_FREQ;
                // control back to default
                pfm[this_pfm].pfm_control_target_delta = false;
            }
        }
    }

    // reset busy flag
    _isr_pfm_busy = false;
}

void init_cnc_pins(void)
{
    /*
    * Function: init_cnc_pins
    * ----------------------------
    *   Initialize CNC Shield pins
    *
    *   returns: void
    */

    // disable cnc shield
    pinMode(SHIELD_EN_PIN, OUTPUT);
    digitalWrite(SHIELD_EN_PIN, HIGH);
    // set pins to 
    for(uint8_t this_pfm=0; this_pfm<NUM_PFM; this_pfm++)
    {
        pinMode(pfm[this_pfm]._cnc_shield_step_pin, OUTPUT);
        digitalWrite(pfm[this_pfm]._cnc_shield_step_pin, LOW);
        pinMode(pfm[this_pfm]._cnc_shield_dir_pin, OUTPUT);
        digitalWrite(pfm[this_pfm]._cnc_shield_dir_pin, LOW);
    }
}

void set_cnc_enable(void)
{
    /*
    * Function: set_cnc_enable
    * ----------------------------
    *   Enable CNC Shield
    * 
    *   returns: void
    */
    digitalWrite(SHIELD_EN_PIN, LOW);
}

void set_cnc_disable(void)
{
    /*
    * Function: set_cnc_disable
    * ----------------------------
    *   Disable CNC Shield
    * 
    *   returns: void
    */
    digitalWrite(SHIELD_EN_PIN, HIGH);
}


void setup_pfm(int32_t pfm_delta_steps_x, int32_t pfm_delta_steps_y, int32_t pfm_delta_steps_z)
{
    /*
    * Function: setup_pfm
    * ----------------------------
    *   Setup Pulse-Frequency-Modulation
    *
    *   pfm_delta_steps_x: offest from default position of steper motor on x-axis
    *   pfm_delta_steps_y: offest from default position of steper motor on y-axis
    *   pfm_delta_steps_z: offest from default position of steper motor on z-axis
    *
    *   returns: void
    */
    init_cnc_pins();
    init_pfm_vars(pfm_delta_steps_x, pfm_delta_steps_y, pfm_delta_steps_z);
    init_pfm_isr();
}


void setup_pfm(void)
{
    /*
    * Function: set_pfm_delta_steps_x
    * ----------------------------
    *   Setup Pulse-Frequency-Modulation to default state
    *
    *   returns: void
    */
    setup_pfm(0, 0, 0);
}

void control_pfm_target_freq(uint8_t this_pfm, uint16_t pfm_target_freq, bool pfm_direction)
{
    _isr_pfm_busy = true;
    // give control to freq
    pfm[this_pfm].pfm_control_target_delta = false;
    // set frequency and direction
    pfm[this_pfm].pfm_target_freq = pfm_target_freq;
    pfm[this_pfm].pfm_direction = pfm_direction;
    // assure the change takes immediate effect
    // pfm[this_pfm]._isr_pfm_counter=0;
    _isr_pfm_busy = false;

}

void control_pfm_target_delta(uint8_t this_pfm, uint16_t pfm_target_freq, int32_t pfm_target_delta)
{

    _isr_pfm_busy = true;
    // the current pfm_delta_steps is exactly at pfm_target_delta
    if(pfm[this_pfm].pfm_delta_steps == pfm_target_delta) 
    {
        // prevent any movement
        pfm[this_pfm].pfm_target_freq = INACTIVE_FREQ;
        // control back to default
        pfm[this_pfm].pfm_control_target_delta = false;
    }
    // discrepancy 
    else
    {
        // give control to delta 
        pfm[this_pfm].pfm_control_target_delta = true;
        pfm[this_pfm].pfm_target_delta = pfm_target_delta;
        pfm[this_pfm].pfm_target_freq = pfm_target_freq;
        // decide shortest direction
        if(pfm[this_pfm].pfm_delta_steps < pfm_target_delta)
        {
            // set delta and direction
            pfm[this_pfm].pfm_direction = true;
        } 
        if(pfm[this_pfm].pfm_delta_steps > pfm_target_delta)
        {
            // set delta and direction
            pfm[this_pfm].pfm_direction = false;
        } 

        // assure the change takes immediate effect
        pfm[this_pfm]._isr_pfm_counter=0;
    }
    _isr_pfm_busy = false;
}

#endif

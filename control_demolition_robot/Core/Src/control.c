/**
  ******************************************************************************
  * @file           : control.c
  * @brief          : Demolition Robot Control Logic Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "control.h"
#include "pwm.h"
#include "main.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define JOYSTICK_CENTER     127     // Center position of joystick (0-255 range)
#define JOYSTICK_DEADZONE   5       // Deadzone around center (reduced for wider range, smoother control)
#define INVALID_MODE_NOISE_FILTER 5 // Require 5 consecutive invalid modes before emergency stop

/* Smoothing parameters for solenoid control */
#define PWM_RAMPING_ENABLED 1       // Enable PWM ramping for smooth transitions
#define MAX_PWM_CHANGE_PER_CYCLE 10 // Max PWM change per cycle (% per 50ms) - lower = smoother but slower response
#define SMOOTH_CURVE_ENABLED 1      // Enable non-linear curve for smooth start/stop

/* Private variables ---------------------------------------------------------*/
// PWM limits per output channel (min, max) - adjustable per solenoid
typedef struct {
    uint8_t min;  // Minimum PWM output (%)
    uint8_t max;  // Maximum PWM output (%)
} PWM_Limits_t;

// PWM limits for each output - indexed by PWM channel enum
static PWM_Limits_t pwm_limits[20] = {
    [PWM_1_BRAKE]                    = {00, 70},  // Brake
    [PWM_2_CYLINDER_1_ON]            = {00, 70},  // Cylinder 1 ON
    [PWM_3_CYLINDER_2_OUT]           = {00, 70},  // Cylinder 2 OUT
    [PWM_4_CYLINDER_2_IN]            = {00, 70},  // Cylinder 2 IN
    [PWM_5_CYLINDER_3_OUT]           = {00, 70},  // Cylinder 3 OUT (Bucket)
    [PWM_6_CYLINDER_3_IN]            = {00, 70},  // Cylinder 3 IN (Bucket)
    [PWM_7_CYLINDER_4_OUT]           = {00, 70},  // Cylinder 4 OUT
    [PWM_8_CYLINDER_4_IN]            = {00, 70},  // Cylinder 4 IN
    [PWM_9_TOOL_1]                   = {00, 70},  // Tool 1 (Reserved)
    [PWM_10_TOOL_2]                  = {00, 70},  // Tool 2 (Reserved)
    [PWM_11_SLEW_CW]                 = {00, 70},  // Slew CW
    [PWM_12_SLEW_CCW]                = {00, 70},  // Slew CCW
    [PWM_13_OUTRIGGER_LEFT_UP]       = {00, 100},  // Outrigger Left UP
    [PWM_14_OUTRIGGER_LEFT_DOWN]     = {00, 100},  // Outrigger Left DOWN
    [PWM_15_OUTRIGGER_RIGHT_UP]      = {00, 100},  // Outrigger Right UP
    [PWM_16_OUTRIGGER_RIGHT_DOWN]    = {00, 100},  // Outrigger Right DOWN
    [PWM_17_TRACK_RIGHT_FORWARD]     = {00, 70},  // Track Right FORWARD
    [PWM_18_TRACK_RIGHT_BACKWARD]    = {00, 70},  // Track Right BACKWARD
    [PWM_19_TRACK_LEFT_FORWARD]      = {00, 70},  // Track Left FORWARD
    [PWM_20_TRACK_LEFT_BACKWARD]     = {00, 70},  // Track Left BACKWARD
};

/* Private variables - noise filters -----------------------------------------*/
static uint8_t invalid_mode_counter = 0;  // Count consecutive invalid modes

/* Private variables - PWM smoothing -----------------------------------------*/
#if PWM_RAMPING_ENABLED
static uint8_t prev_pwm_target[20] = {0};  // Previous PWM target values for ramping
#endif

/* Private function prototypes -----------------------------------------------*/
static uint8_t MapJoystickToPWM(uint8_t joystick_value, bool inverse, PWM_Channel_t channel);
static uint8_t ApplySmoothing(uint8_t new_pwm, PWM_Channel_t channel);

/**
  * @brief  Initialize control system
  * @retval None
  */
void Control_Init(void)
{
    // Initialize PWM system
    PWM_Init();

    // Set all outputs to safe state (0%)
    Control_EmergencyStop();
}

/**
  * @brief  Update control outputs based on LoRa data
  * @param  lora_data: Pointer to received LoRa data structure
  * @retval None
  */
void Control_Update(LoRa_ReceivedData_t *lora_data)
{
    if (lora_data == NULL) return;

    // ========================================================================
    // MODE DETECTION
    // ========================================================================
    // Mode UPPER: s5_1 = 0, s5_2 = 0 → Excavator controls (cylinders, slew)
    // Mode DUAL:  s5_1 = 1, s5_2 = 0 → Reserved for future implementation
    // Mode LOWER: s5_1 = 0, s5_2 = 1 → Mobility controls (tracks, outriggers)

    bool mode_upper = (lora_data->s5_1 == 0) && (lora_data->s5_2 == 0);
    bool mode_dual  = (lora_data->s5_1 == 1) && (lora_data->s5_2 == 0);
    bool mode_lower = (lora_data->s5_1 == 0) && (lora_data->s5_2 == 1);

    // ========================================================================
    // MODE UPPER - EXCAVATOR CONTROLS
    // ========================================================================
    if (mode_upper)
    {
        // Valid mode detected - reset invalid mode counter
        invalid_mode_counter = 0;
        // --------------------------------------------------------------------
        // LEFT STICK Y-AXIS: CYLINDER 3 (Bucket)
        // --------------------------------------------------------------------
        // joy_left_y: 127→255 = Cylinder 3 UP   (PWM_5) 0→100%
        //             127→0   = Cylinder 3 DOWN (PWM_6) 0→100%

        if (lora_data->joy_left_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving DOWN (0-117) → Cylinder 3 DOWN
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, true, PWM_6_CYLINDER_3_IN);
            PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, pwm_value);
            PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 0);
        }
        else if (lora_data->joy_left_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving UP (137-255) → Cylinder 3 UP
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, false, PWM_5_CYLINDER_3_OUT);
            PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, pwm_value);
            PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 0);
            PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, 0);
        }

        // --------------------------------------------------------------------
        // LEFT STICK X-AXIS: SLEW ROTATION
        // --------------------------------------------------------------------
        // joy_left_x: 127→255 = Slew CW  (PWM_11) 0→100%
        //             127→0   = Slew CCW (PWM_12) 0→100%

        if (lora_data->joy_left_x < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving LEFT (0-117) → Slew CCW
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, true, PWM_12_SLEW_CCW);
            PWM_SetDutyCycle(PWM_12_SLEW_CCW, pwm_value);
            PWM_SetDutyCycle(PWM_11_SLEW_CW, 0);
        }
        else if (lora_data->joy_left_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Slew CW
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, false, PWM_11_SLEW_CW);
            PWM_SetDutyCycle(PWM_11_SLEW_CW, pwm_value);
            PWM_SetDutyCycle(PWM_12_SLEW_CCW, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_11_SLEW_CW, 0);
            PWM_SetDutyCycle(PWM_12_SLEW_CCW, 0);
        }

        // --------------------------------------------------------------------
        // RIGHT STICK Y-AXIS: CYLINDER 2 (Stick)
        // --------------------------------------------------------------------
        // Normal mode (joy_right_btn1 = 0):
        //   joy_right_y: 127→0   = Cylinder 2 UP   (PWM_3) 0→100%
        //                127→255 = Cylinder 2 DOWN (PWM_4) 0→100%
        // Reversed mode (joy_right_btn1 = 1 / Cylinder 1 ON pressed):
        //   joy_right_y: 127→0   = Cylinder 2 DOWN (PWM_4) 0→100%
        //                127→255 = Cylinder 2 UP   (PWM_3) 0→100%

        bool cylinder_2_reversed = (lora_data->joy_right_btn1 == 1);

        if (lora_data->joy_right_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Stick DOWN (0-117)
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, true,
                                                 cylinder_2_reversed ? PWM_4_CYLINDER_2_IN : PWM_3_CYLINDER_2_OUT);
            if (cylinder_2_reversed)
            {
                // Reversed: Stick DOWN → Cylinder 2 DOWN
                PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, pwm_value);
                PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, 0);
            }
            else
            {
                // Normal: Stick DOWN → Cylinder 2 UP
                PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, pwm_value);
                PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, 0);
            }
        }
        else if (lora_data->joy_right_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Stick UP (137-255)
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, false,
                                                 cylinder_2_reversed ? PWM_3_CYLINDER_2_OUT : PWM_4_CYLINDER_2_IN);
            if (cylinder_2_reversed)
            {
                // Reversed: Stick UP → Cylinder 2 UP
                PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, pwm_value);
                PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, 0);
            }
            else
            {
                // Normal: Stick UP → Cylinder 2 DOWN
                PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, pwm_value);
                PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, 0);
            }
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, 0);
            PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, 0);
        }

        // --------------------------------------------------------------------
        // RIGHT STICK X-AXIS: CYLINDER 4 (Reversed mapping!)
        // --------------------------------------------------------------------
        // joy_right_x: 127→0   = Cylinder 4 DOWN (PWM_7) 0→100%
        //              127→255 = Cylinder 4 UP   (PWM_8) 0→100%

        if (lora_data->joy_right_x < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Stick LEFT (0-117) → Cylinder 4 DOWN
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, true, PWM_7_CYLINDER_4_OUT);
            PWM_SetDutyCycle(PWM_7_CYLINDER_4_OUT, pwm_value);
            PWM_SetDutyCycle(PWM_8_CYLINDER_4_IN, 0);
        }
        else if (lora_data->joy_right_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Stick RIGHT (137-255) → Cylinder 4 UP
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, false, PWM_8_CYLINDER_4_IN);
            PWM_SetDutyCycle(PWM_8_CYLINDER_4_IN, pwm_value);
            PWM_SetDutyCycle(PWM_7_CYLINDER_4_OUT, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_7_CYLINDER_4_OUT, 0);
            PWM_SetDutyCycle(PWM_8_CYLINDER_4_IN, 0);
        }

        // --------------------------------------------------------------------
        // BRAKE - Always ON in UPPER mode (100% PWM)
        // --------------------------------------------------------------------
        PWM_SetDutyCycle(PWM_1_BRAKE, 100);

        // --------------------------------------------------------------------
        // CYLINDER_1_ON - Controlled by joy_right_btn1
        // Valve solenoid parallel between cylinder 1 and 2
        // --------------------------------------------------------------------
        if (lora_data->joy_right_btn1 == 1)
        {
            PWM_SetDutyCycle(PWM_2_CYLINDER_1_ON, 100);  // Open valve
        }
        else
        {
            PWM_SetDutyCycle(PWM_2_CYLINDER_1_ON, 0);    // Close valve
        }

        // Stop all mobility controls in UPPER mode
        PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, 0);
        PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, 0);
        PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, 0);
        PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, 0);
        PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, 0);
        PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, 0);
        PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, 0);
        PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, 0);
    }

    // ========================================================================
    // MODE LOWER - MOBILITY CONTROLS
    // ========================================================================
    else if (mode_lower)
    {
        // Valid mode detected - reset invalid mode counter
        invalid_mode_counter = 0;
        // --------------------------------------------------------------------
        // LEFT STICK Y-AXIS: TRACK LEFT
        // --------------------------------------------------------------------
        // joy_left_y: 127→255 = Track Left Forward  (PWM_19) 0→100%
        //             127→0   = Track Left Backward (PWM_20) 0→100%

        if (lora_data->joy_left_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving DOWN (0-117) → Track Left Backward
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, true, PWM_20_TRACK_LEFT_BACKWARD);
            PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, pwm_value);
            PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, 0);
        }
        else if (lora_data->joy_left_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving UP (137-255) → Track Left Forward
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, false, PWM_19_TRACK_LEFT_FORWARD);
            PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, pwm_value);
            PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, 0);
            PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, 0);
        }

        // --------------------------------------------------------------------
        // LEFT STICK X-AXIS: OUTRIGGER LEFT
        // --------------------------------------------------------------------
        // joy_left_x: 127→255 = Outrigger Left Up   (PWM_13) 0→100%
        //             127→0   = Outrigger Left Down (PWM_14) 0→100%

        if (lora_data->joy_left_x < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving LEFT (0-117) → Outrigger Left Down
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, true, PWM_14_OUTRIGGER_LEFT_DOWN);
            PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, pwm_value);
            PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, 0);
        }
        else if (lora_data->joy_left_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Outrigger Left Up
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, false, PWM_13_OUTRIGGER_LEFT_UP);
            PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, pwm_value);
            PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, 0);
            PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, 0);
        }

        // --------------------------------------------------------------------
        // RIGHT STICK Y-AXIS: TRACK RIGHT
        // --------------------------------------------------------------------
        // joy_right_y: 127→255 = Track Right Forward  (PWM_17) 0→100%
        //              127→0   = Track Right Backward (PWM_18) 0→100%

        if (lora_data->joy_right_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving DOWN (0-117) → Track Right Backward
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, true, PWM_18_TRACK_RIGHT_BACKWARD);
            PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, pwm_value);
            PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, 0);
        }
        else if (lora_data->joy_right_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving UP (137-255) → Track Right Forward
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, false, PWM_17_TRACK_RIGHT_FORWARD);
            PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, pwm_value);
            PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, 0);
            PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, 0);
        }

        // --------------------------------------------------------------------
        // RIGHT STICK X-AXIS: OUTRIGGER RIGHT
        // --------------------------------------------------------------------
        // joy_right_x: 127→255 = Outrigger Right Down (PWM_16) 0→100%
        //              127→0   = Outrigger Right Up   (PWM_15) 0→100%

        if (lora_data->joy_right_x < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving LEFT (0-117) → Outrigger Right Up
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, true, PWM_15_OUTRIGGER_RIGHT_UP);
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, pwm_value);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, 0);
        }
        else if (lora_data->joy_right_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Outrigger Right Down
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, false, PWM_16_OUTRIGGER_RIGHT_DOWN);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, pwm_value);
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, 0);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, 0);
        }

        // Stop all excavator controls in LOWER mode
        PWM_SetDutyCycle(PWM_1_BRAKE, 0);              // Stop brake
        PWM_SetDutyCycle(PWM_2_CYLINDER_1_ON, 0);      // Close valve
        PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, 0);
        PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, 0);
        PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 0);
        PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, 0);
        PWM_SetDutyCycle(PWM_11_SLEW_CW, 0);
        PWM_SetDutyCycle(PWM_12_SLEW_CCW, 0);
    }

    // ========================================================================
    // MODE DUAL - RESERVED FOR FUTURE IMPLEMENTATION
    // ========================================================================
    else if (mode_dual)
    {
        // TODO: Implement dual mode in the future
        // This mode will combine both excavator and mobility controls
        // For now, treat as invalid mode and use noise filter

        // Increment invalid mode counter
        if (invalid_mode_counter < INVALID_MODE_NOISE_FILTER)
        {
            invalid_mode_counter++;
        }

        // Only stop if we've had INVALID_MODE_NOISE_FILTER consecutive invalid modes
        if (invalid_mode_counter >= INVALID_MODE_NOISE_FILTER)
        {
            Control_EmergencyStop();
        }
    }

    // ========================================================================
    // INVALID MODE - EMERGENCY STOP (with noise filter)
    // ========================================================================
    else
    {
        // Unknown mode combination - use noise filter before emergency stop

        // Increment invalid mode counter
        if (invalid_mode_counter < INVALID_MODE_NOISE_FILTER)
        {
            invalid_mode_counter++;
        }

        // Only stop if we've had INVALID_MODE_NOISE_FILTER consecutive invalid modes
        if (invalid_mode_counter >= INVALID_MODE_NOISE_FILTER)
        {
            Control_EmergencyStop();
        }
    }

    // ========================================================================
    // PE6 (MOTOR STARTER) - GPIO CONTROL
    // ========================================================================
    // Motor starter controlled by S1_1 hold
    // - S1_1 = 1 (hold) → PE6 = HIGH
    // - Otherwise → PE6 = LOW
    // Note: PE6 is forced LOW during sleep mode in main.c

    if (lora_data->s1_1 == 1)
    {
        GPIOE->BSRR = (1<<6);      // BS6 = set PE6 to HIGH
    }
    else
    {
        GPIOE->BSRR = (1<<(6+16)); // BR6 = reset PE6 to LOW
    }
}

/**
  * @brief  Emergency stop - set all PWM outputs to 0% and motor starter OFF
  * @retval None
  */
void Control_EmergencyStop(void)
{
    PWM_StopAll();  // This will set all PWM channels to 0%
}

/**
  * @brief  Apply smooth curve to PWM value for gradual start/stop
  * @param  pwm_value: Linear PWM value (0-100%)
  * @retval Smoothed PWM value (0-100%)
  */
static uint8_t ApplySmoothCurve(uint8_t pwm_value)
{
#if SMOOTH_CURVE_ENABLED
    if (pwm_value == 0) return 0;

    // Apply exponential curve for smooth start
    // Formula: output = input^1.5 for smooth acceleration
    // Using integer math: pwm * sqrt(pwm) / 10
    uint16_t temp = pwm_value * pwm_value;  // pwm^2
    temp = temp / 100;                       // Normalize

    // Approximate sqrt using lookup or linear interpolation
    // For simplicity, use quadratic curve: pwm^2 / 100
    uint8_t result = (uint8_t)temp;

    return result;
#else
    return pwm_value;
#endif
}

/**
  * @brief  Apply PWM ramping for smooth transitions
  * @param  new_pwm: Target PWM value
  * @param  channel: PWM channel
  * @retval Ramped PWM value (limited rate of change)
  */
static uint8_t ApplySmoothing(uint8_t new_pwm, PWM_Channel_t channel)
{
#if PWM_RAMPING_ENABLED
    uint8_t prev_pwm = prev_pwm_target[channel];
    uint8_t ramped_pwm = new_pwm;

    // Calculate difference
    int16_t diff = (int16_t)new_pwm - (int16_t)prev_pwm;

    // Apply rate limiting
    if (diff > MAX_PWM_CHANGE_PER_CYCLE)
    {
        ramped_pwm = prev_pwm + MAX_PWM_CHANGE_PER_CYCLE;
    }
    else if (diff < -MAX_PWM_CHANGE_PER_CYCLE)
    {
        ramped_pwm = prev_pwm - MAX_PWM_CHANGE_PER_CYCLE;
    }

    // Store for next cycle
    prev_pwm_target[channel] = ramped_pwm;

    return ramped_pwm;
#else
    return new_pwm;
#endif
}

/**
  * @brief  Map joystick value (0-255) to PWM duty cycle with per-channel limiting
  * @param  joystick_value: Raw joystick value (0-255)
  * @param  inverse: true = map 0-127 to 0-100%, false = map 127-255 to 0-100%
  * @param  channel: PWM channel to get min/max limits for
  * @retval PWM duty cycle percentage (min-max% when active, 0% when stopped)
  */
static uint8_t MapJoystickToPWM(uint8_t joystick_value, bool inverse, PWM_Channel_t channel)
{
    uint8_t pwm_value = 0;

    if (inverse)
    {
        // Map 0-127 to 100-0% (inverse: 0 = max, 127 = 0)
        if (joystick_value <= JOYSTICK_CENTER)
        {
            pwm_value = ((JOYSTICK_CENTER - joystick_value) * 100) / JOYSTICK_CENTER;
        }
    }
    else
    {
        // Map 127-255 to 0-100% (normal: 127 = 0, 255 = max)
        if (joystick_value >= JOYSTICK_CENTER)
        {
            pwm_value = ((joystick_value - JOYSTICK_CENTER) * 100) / JOYSTICK_CENTER;
        }
    }

    // Clamp to 0-100%
    if (pwm_value > 100) pwm_value = 100;

    // Apply smooth curve for gradual start/stop
    pwm_value = ApplySmoothCurve(pwm_value);

    // Apply PWM limiting: 0% stays 0%, 1-100% maps to channel's min-max
    if (pwm_value > 0)
    {
        // Get limits for this specific channel
        uint8_t pwm_min = pwm_limits[channel].min;
        uint8_t pwm_max = pwm_limits[channel].max;

        // Scale from 0-100% to pwm_min-pwm_max
        // Formula: output = min + (input * (max - min) / 100)
        pwm_value = pwm_min + ((pwm_value * (pwm_max - pwm_min)) / 100);
    }

    // Apply ramping for smooth transitions
    pwm_value = ApplySmoothing(pwm_value, channel);

    return pwm_value;
}

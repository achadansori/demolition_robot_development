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

/* Smoothing parameters for solenoid control */
#define PWM_RAMPING_ENABLED 0       // OFF - instant response (no delay)
#define MAX_PWM_CHANGE_PER_CYCLE 10 // Not used when ramping OFF
#define SMOOTH_CURVE_ENABLED 0      // OFF - linear mapping (no delay)

/* Private variables ---------------------------------------------------------*/
// PWM limits per output channel (min, max) - adjustable per solenoid
typedef struct {
    uint8_t min;  // Minimum PWM output (%)
    uint8_t max;  // Maximum PWM output (%)
} PWM_Limits_t;

// PWM limits for each output - indexed by PWM channel enum
// NOTE: Max values capped at 90% to prevent TIP122 deep saturation (stuck ON issue)
// TIP122 Darlington transistors accumulate base charge at 100% duty cycle,
// causing them to stay ON even when PWM returns to 0%. Limiting to 90% max
// ensures proper switching and prevents the "stuck ON" problem.
static PWM_Limits_t pwm_limits[20] = {
    [PWM_1_BRAKE]                    = {0, 100},  // Brake (always 100% in UPPER mode)
    [PWM_2_CYLINDER_1_ON]            = {0, 100},  // Cylinder 1 ON valve (digital ON/OFF)
    [PWM_3_CYLINDER_2_OUT]           = {30, 55},  // Cylinder 2 OUT
    [PWM_4_CYLINDER_2_IN]            = {20, 50},  // Cylinder 2 IN
    [PWM_5_CYLINDER_3_OUT]           = {32, 70},  // Cylinder 3 OUT (Bucket)
    [PWM_6_CYLINDER_3_IN]            = {32, 60},  // Cylinder 3 IN (Bucket)
    [PWM_7_CYLINDER_4_OUT]           = {32, 60},  // Cylinder 4 OUT
    [PWM_8_CYLINDER_4_IN]            = {32, 65},  // Cylinder 4 IN
    [PWM_9_TOOL_1]                   = {0, 60},   // Tool 1 (Reserved) - max 60%
    [PWM_10_TOOL_2]                  = {0, 50},   // Tool 2 (Reserved)
    [PWM_11_SLEW_CW]                 = {25, 45},  // Slew CW
    [PWM_12_SLEW_CCW]                = {25, 45},  // Slew CCW
    [PWM_13_OUTRIGGER_LEFT_UP]       = {10, 70},  // Outrigger Left UP - max 70%
    [PWM_14_OUTRIGGER_LEFT_DOWN]     = {10, 70},  // Outrigger Left DOWN - max 70%
    [PWM_15_OUTRIGGER_RIGHT_UP]      = {10, 70},  // Outrigger Right UP - max 70%
    [PWM_16_OUTRIGGER_RIGHT_DOWN]    = {10, 70},  // Outrigger Right DOWN - max 70%
    [PWM_17_TRACK_RIGHT_FORWARD]     = {11, 46},  // Track Right FORWARD
    [PWM_18_TRACK_RIGHT_BACKWARD]    = {11, 46},  // Track Right BACKWARD
    [PWM_19_TRACK_LEFT_FORWARD]      = {18, 53},  // Track Left FORWARD
    [PWM_20_TRACK_LEFT_BACKWARD]     = {21, 66},  // Track Left BACKWARD
};

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
    PWM_StopAll();
}

/**
  * @brief  Update control outputs based on NRF24 data
  * @param  lora_data: Pointer to received NRF24 data structure
  * @retval None
  */
void Control_Update(NRF24_ReceivedData_t *lora_data)
{
    if (lora_data == NULL) return;

    // ========================================================================
    // EMERGENCY MODE - S0 = 0 (HIGHEST PRIORITY!)
    // ========================================================================
    if (lora_data->s0 == 0)
    {
        GPIOB->BSRR = (1<<(8+16)); // BR8 = reset PB8 to LOW
        PWM_StopAll();
        GPIOE->BSRR = (1<<(6+16)); // BR6 = reset PE6 to LOW
        GPIO_SetTool1(0);
        return;  // Exit immediately - no further processing
    }
    else
    {
        GPIOB->BSRR = (1<<8);      // BS8 = set PB8 to HIGH
    }

    // ========================================================================
    // SLEEP MODE - motor_active = 0 (ALL PWM = 0)
    // ========================================================================
    // When transmitter is in sleep mode or motor not yet started,
    // all PWM outputs must be 0 for safety
    if (lora_data->motor_active == 0)
    {
        PWM_StopAll();
        GPIOE->BSRR = (1<<(6+16)); // BR6 = reset PE6 to LOW
        GPIO_SetTool1(0);
        return;  // Exit immediately - no further processing
    }

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
        // --------------------------------------------------------------------
        // BRAKE CONTROL - Always ON in UPPER mode (100% PWM)
        // --------------------------------------------------------------------
        PWM_SetDutyCycle(PWM_1_BRAKE, 100);

        // --------------------------------------------------------------------
        // CYLINDER_1_ON - Controlled by joy_right_btn1
        // --------------------------------------------------------------------
        // Valve solenoid parallel between cylinder 1 and 2
        // This valve controls access to cylinder 1
        if (lora_data->joy_right_btn1 == 1)
        {
            PWM_SetDutyCycle(PWM_2_CYLINDER_1_ON, 100);  // Open valve
        }
        else
        {
            PWM_SetDutyCycle(PWM_2_CYLINDER_1_ON, 0);    // Close valve
        }

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
        // RIGHT STICK Y-AXIS: CYLINDER 2
        // --------------------------------------------------------------------
        // joy_right_y: 127→0   = Cylinder 2 OUT (PWM_3) 0→100%
        //              127→255 = Cylinder 2 IN  (PWM_4) 0→100%
        // Note: Cylinder 1 is now controlled by CYLINDER_1_ON valve (joy_right_btn1)

        if (lora_data->joy_right_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Stick DOWN (0-117) → Cylinder 2 OUT
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, true, PWM_3_CYLINDER_2_OUT);
            PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, pwm_value);
            PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, 0);
        }
        else if (lora_data->joy_right_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Stick UP (137-255) → Cylinder 2 IN
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, false, PWM_4_CYLINDER_2_IN);
            PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, pwm_value);
            PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, 0);
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
        // BREAKER - 2 Valve Control
        // --------------------------------------------------------------------
        // Valve 1 (GPIO PB1): ON/OFF digital trigger by joy_left_btn2
        // Valve 2 (PWM_10): Flow control by R8 potentiometer (proportional)
        //
        // joy_left_btn2 = 1 → Breaker ON (PB1 = HIGH)
        // joy_left_btn2 = 0 → Breaker OFF (PB1 = LOW)
        // R8: 0-255 → PWM_10: min-max% (respects pwm_limits)

        if (lora_data->joy_left_btn2 == 1)
        {
            GPIO_SetTool1(1);  // Valve 1: Breaker ON (digital HIGH)
        }
        else
        {
            GPIO_SetTool1(0);  // Valve 1: Breaker OFF (digital LOW)
        }

        // Valve 2: Flow control (proportional from R8)
        // R8 range: 0-255 → PWM range: min-max% (respects pwm_limits)
        uint8_t breaker_flow_raw = (lora_data->r8 * 100) / 255;
        uint8_t breaker_flow = 0;
        if (breaker_flow_raw > 0)
        {
            // Scale from 0-100% to min-max%
            breaker_flow = pwm_limits[PWM_10_TOOL_2].min +
                ((breaker_flow_raw * (pwm_limits[PWM_10_TOOL_2].max - pwm_limits[PWM_10_TOOL_2].min)) / 100);
        }
        PWM_SetDutyCycle(PWM_10_TOOL_2, breaker_flow);

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
        // LEFT STICK X-AXIS: OUTRIGGER RIGHT (swapped due to electrical wiring)
        // --------------------------------------------------------------------
        // joy_left_x: 127→255 = Outrigger Right Up   (PWM_15) 0→100%
        //             127→0   = Outrigger Right Down (PWM_16) 0→100%

        if (lora_data->joy_left_x < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving LEFT (0-117) → Outrigger Right Down
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, true, PWM_16_OUTRIGGER_RIGHT_DOWN);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, pwm_value);
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, 0);
        }
        else if (lora_data->joy_left_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Outrigger Right Up
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, false, PWM_15_OUTRIGGER_RIGHT_UP);
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, pwm_value);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, 0);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, 0);
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
        // RIGHT STICK X-AXIS: OUTRIGGER LEFT (swapped due to electrical wiring)
        // --------------------------------------------------------------------
        // joy_right_x: 127→255 = Outrigger Left Down (PWM_14) 0→100%
        //              127→0   = Outrigger Left Up   (PWM_13) 0→100%

        if (lora_data->joy_right_x < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving LEFT (0-117) → Outrigger Left Up
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, true, PWM_13_OUTRIGGER_LEFT_UP);
            PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, pwm_value);
            PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, 0);
        }
        else if (lora_data->joy_right_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Outrigger Left Down
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, false, PWM_14_OUTRIGGER_LEFT_DOWN);
            PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, pwm_value);
            PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, 0);
            PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, 0);
        }

        // Stop all excavator controls in LOWER mode
        PWM_SetDutyCycle(PWM_1_BRAKE, 0);
        PWM_SetDutyCycle(PWM_2_CYLINDER_1_ON, 0);
        PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, 0);
        PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, 0);
        PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 0);
        PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, 0);
        PWM_SetDutyCycle(PWM_11_SLEW_CW, 0);
        PWM_SetDutyCycle(PWM_12_SLEW_CCW, 0);
        GPIO_SetTool1(0);                         // Stop Breaker Valve 1 (digital OFF)
        PWM_SetDutyCycle(PWM_10_TOOL_2, 0);       // Stop Breaker Valve 2
    }

    // ========================================================================
    // MODE DUAL - COMBINED TRACK CONTROL
    // ========================================================================
    // When joy_right_btn1 is pressed, both tracks move together
    // joy_right_y controls forward/backward for both tracks simultaneously
    else if (mode_dual)
    {
        // --------------------------------------------------------------------
        // BOTH TRACKS - Controlled by joy_right_btn1 + joy_right_y
        // --------------------------------------------------------------------
        // joy_right_btn1 = 1 (pressed): Enable track control
        //   joy_right_y: 127→255 = Both tracks FORWARD
        //                127→0   = Both tracks BACKWARD

        if (lora_data->joy_right_btn1 == 1)
        {
            if (lora_data->joy_right_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
            {
                // Moving DOWN (0-117) → Both tracks BACKWARD
                uint8_t pwm_left = MapJoystickToPWM(lora_data->joy_right_y, true, PWM_20_TRACK_LEFT_BACKWARD);
                uint8_t pwm_right = MapJoystickToPWM(lora_data->joy_right_y, true, PWM_18_TRACK_RIGHT_BACKWARD);

                PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, pwm_left);
                PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, pwm_right);
                PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, 0);
                PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, 0);
            }
            else if (lora_data->joy_right_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
            {
                // Moving UP (137-255) → Both tracks FORWARD
                uint8_t pwm_left = MapJoystickToPWM(lora_data->joy_right_y, false, PWM_19_TRACK_LEFT_FORWARD);
                uint8_t pwm_right = MapJoystickToPWM(lora_data->joy_right_y, false, PWM_17_TRACK_RIGHT_FORWARD);

                PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, pwm_left);
                PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, pwm_right);
                PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, 0);
                PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, 0);
            }
            else
            {
                // Deadzone - stop all tracks
                PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, 0);
                PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, 0);
                PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, 0);
                PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, 0);
            }
        }
        else
        {
            // joy_right_btn1 not pressed - stop all tracks
            PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, 0);
            PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, 0);
            PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, 0);
            PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, 0);
        }

        // Stop all excavator controls in DUAL mode
        PWM_SetDutyCycle(PWM_1_BRAKE, 0);
        PWM_SetDutyCycle(PWM_2_CYLINDER_1_ON, 0);
        PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, 0);
        PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, 0);
        PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 0);
        PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, 0);
        PWM_SetDutyCycle(PWM_7_CYLINDER_4_OUT, 0);
        PWM_SetDutyCycle(PWM_8_CYLINDER_4_IN, 0);
        PWM_SetDutyCycle(PWM_11_SLEW_CW, 0);
        PWM_SetDutyCycle(PWM_12_SLEW_CCW, 0);
        GPIO_SetTool1(0);                          // Stop Breaker Valve 1 (digital OFF)
        PWM_SetDutyCycle(PWM_10_TOOL_2, 0);        // Stop Breaker Valve 2
        PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, 0);
        PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, 0);
        PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, 0);
        PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, 0);
    }

    // ========================================================================
    // PE6 (MOTOR STARTER) - GPIO CONTROL
    // ========================================================================
    // Motor starter controlled by motor_active flag from transmitter
    // - motor_active = 1 → PE6 = HIGH (motor ON)
    // - motor_active = 0 → PE6 = LOW  (motor OFF)
    // Note: PE6 is also forced LOW during emergency (S0=0) and sleep mode

    if (lora_data->motor_active == 1)
    {
        GPIOE->BSRR = (1<<6);      // BS6 = set PE6 to HIGH
    }
    else
    {
        GPIOE->BSRR = (1<<(6+16)); // BR6 = reset PE6 to LOW
    }
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

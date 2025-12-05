/**
  ******************************************************************************
  * @file           : control.c
  * @brief          : Demolition Robot Control Logic Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "control.h"
#include "pwm.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define JOYSTICK_CENTER     127     // Center position of joystick (0-255 range)
#define JOYSTICK_DEADZONE   10      // Deadzone around center to prevent drift

/* Private function prototypes -----------------------------------------------*/
static uint8_t MapJoystickToPWM(uint8_t joystick_value, bool inverse);

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
        // --------------------------------------------------------------------
        // LEFT STICK Y-AXIS: CYLINDER 3 (Bucket)
        // --------------------------------------------------------------------
        // joy_left_y: 127→255 = Cylinder 3 OUT (PWM_5) 0→100%
        //             127→0   = Cylinder 3 IN  (PWM_6) 0→100%

        if (lora_data->joy_left_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving DOWN (0-117) → Cylinder 3 IN
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, true);
            PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, pwm_value);
            PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 0);
        }
        else if (lora_data->joy_left_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving UP (137-255) → Cylinder 3 OUT
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, false);
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
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, true);
            PWM_SetDutyCycle(PWM_12_SLEW_CCW, pwm_value);
            PWM_SetDutyCycle(PWM_11_SLEW_CW, 0);
        }
        else if (lora_data->joy_left_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Slew CW
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, false);
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
        // joy_right_y: 127→255 = Cylinder 2 IN  (PWM_4) 0→100%
        //              127→0   = Cylinder 2 OUT (PWM_3) 0→100%

        if (lora_data->joy_right_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving DOWN (0-117) → Cylinder 2 OUT
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, true);
            PWM_SetDutyCycle(PWM_3_CYLINDER_2_OUT, pwm_value);
            PWM_SetDutyCycle(PWM_4_CYLINDER_2_IN, 0);
        }
        else if (lora_data->joy_right_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving UP (137-255) → Cylinder 2 IN
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, false);
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
        // RIGHT STICK X-AXIS: CYLINDER 1 (Boom)
        // --------------------------------------------------------------------
        // joy_right_x: 127→255 = Cylinder 1 IN  (PWM_2) 0→100%
        //              127→0   = Cylinder 1 OUT (PWM_1) 0→100%

        if (lora_data->joy_right_x < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving LEFT (0-117) → Cylinder 1 OUT
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, true);
            PWM_SetDutyCycle(PWM_1_CYLINDER_1_OUT, pwm_value);
            PWM_SetDutyCycle(PWM_2_CYLINDER_1_IN, 0);
        }
        else if (lora_data->joy_right_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Cylinder 1 IN
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, false);
            PWM_SetDutyCycle(PWM_2_CYLINDER_1_IN, pwm_value);
            PWM_SetDutyCycle(PWM_1_CYLINDER_1_OUT, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_1_CYLINDER_1_OUT, 0);
            PWM_SetDutyCycle(PWM_2_CYLINDER_1_IN, 0);
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
        // --------------------------------------------------------------------
        // LEFT STICK Y-AXIS: TRACK LEFT
        // --------------------------------------------------------------------
        // joy_left_y: 127→255 = Track Left Forward  (PWM_19) 0→100%
        //             127→0   = Track Left Backward (PWM_20) 0→100%

        if (lora_data->joy_left_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving DOWN (0-117) → Track Left Backward
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, true);
            PWM_SetDutyCycle(PWM_20_TRACK_LEFT_BACKWARD, pwm_value);
            PWM_SetDutyCycle(PWM_19_TRACK_LEFT_FORWARD, 0);
        }
        else if (lora_data->joy_left_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving UP (137-255) → Track Left Forward
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, false);
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
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, true);
            PWM_SetDutyCycle(PWM_14_OUTRIGGER_LEFT_DOWN, pwm_value);
            PWM_SetDutyCycle(PWM_13_OUTRIGGER_LEFT_UP, 0);
        }
        else if (lora_data->joy_left_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Outrigger Left Up
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_x, false);
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
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, true);
            PWM_SetDutyCycle(PWM_18_TRACK_RIGHT_BACKWARD, pwm_value);
            PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, 0);
        }
        else if (lora_data->joy_right_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving UP (137-255) → Track Right Forward
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_y, false);
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
        // joy_right_x: 127→255 = Outrigger Right Up   (PWM_15) 0→100%
        //              127→0   = Outrigger Right Down (PWM_16) 0→100%

        if (lora_data->joy_right_x < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
        {
            // Moving LEFT (0-117) → Outrigger Right Down
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, true);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, pwm_value);
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, 0);
        }
        else if (lora_data->joy_right_x > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
        {
            // Moving RIGHT (137-255) → Outrigger Right Up
            uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_right_x, false);
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, pwm_value);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, 0);
        }
        else
        {
            // Deadzone - stop both
            PWM_SetDutyCycle(PWM_15_OUTRIGGER_RIGHT_UP, 0);
            PWM_SetDutyCycle(PWM_16_OUTRIGGER_RIGHT_DOWN, 0);
        }

        // Stop all excavator controls in LOWER mode
        PWM_SetDutyCycle(PWM_1_CYLINDER_1_OUT, 0);
        PWM_SetDutyCycle(PWM_2_CYLINDER_1_IN, 0);
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
        // Stop all outputs for now
        Control_EmergencyStop();
    }

    // ========================================================================
    // INVALID MODE - EMERGENCY STOP
    // ========================================================================
    else
    {
        // Unknown mode combination - stop all outputs for safety
        Control_EmergencyStop();
    }
}

/**
  * @brief  Emergency stop - set all PWM outputs to 0%
  * @retval None
  */
void Control_EmergencyStop(void)
{
    PWM_StopAll();
}

/**
  * @brief  Map joystick value (0-255) to PWM duty cycle (0-100%)
  * @param  joystick_value: Raw joystick value (0-255)
  * @param  inverse: true = map 0-127 to 0-100%, false = map 127-255 to 0-100%
  * @retval PWM duty cycle percentage (0-100)
  */
static uint8_t MapJoystickToPWM(uint8_t joystick_value, bool inverse)
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

    return pwm_value;
}

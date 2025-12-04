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
    // CYLINDER 3 CONTROL - Left Joystick Y-axis
    // ========================================================================
    // joystick_left_y: 0-127 = Cylinder 3 IN (PWM 6)
    //                  127-255 = Cylinder 3 OUT (PWM 5)

    if (lora_data->joy_left_y < (JOYSTICK_CENTER - JOYSTICK_DEADZONE))
    {
        // Moving DOWN (0-117) → Cylinder 3 IN
        uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, true);
        PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, pwm_value);
        PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 0);  // Stop opposite direction
    }
    else if (lora_data->joy_left_y > (JOYSTICK_CENTER + JOYSTICK_DEADZONE))
    {
        // Moving UP (137-255) → Cylinder 3 OUT
        uint8_t pwm_value = MapJoystickToPWM(lora_data->joy_left_y, false);
        PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, pwm_value);
        PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, 0);   // Stop opposite direction
    }
    else
    {
        // Deadzone - stop both
        PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 0);
        PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, 0);
    }

    // ========================================================================
    // TODO: Add more controls here as needed
    // ========================================================================
    // Example for other cylinders, tracks, tools, etc.
    // Follow the same pattern as Cylinder 3 above
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

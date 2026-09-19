#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

#define STEPPER_DRV_MAX_NUM_STEPPERS        (2)
#define STEPPER_DRV_ACTIVE_PIN_VALUE        (0)
#define STEPPER_DRV_RMT_QUEUE_SIZE          (4)         /* Number of RMT events to queue for each stepper */ 
#define STEPPER_DRV_RMT_RESOLUTION_HZ       (1000000)   /* 1 MHz tick rate (1 tick = 1 µs) */
#define STEPPER_DRV_MAX_STEP_FREQUENCY_HZ   (20000)     /* Actual physical step-rate ceiling */
#define STEPPER_DRV_STEP_DUTY_CYCLE_PERCENT (25)        /* Step pulse duty cycle in percentage */    

typedef enum stepper_drv_direction
{
    STEPPER_DRV_DIRECTION_FWD,
    STEPPER_DRV_DIRECTION_REV  
} stepper_drv_direction_t;

typedef struct stepper_drv_stepper_config
{
    uint8_t stepper_id;
    uint8_t pin_step;
    uint8_t pin_dir;
    uint8_t pin_enable;
} stepper_drv_stepper_config_t;

/*
 * Initialize the stepper driver with the provided configurations.
 *
 * @param stepper_configs Array of stepper configurations.
 * @param num_steppers Number of steppers to initialize.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_init(const stepper_drv_stepper_config_t *stepper_configs, const size_t num_steppers);

/* -------------- Raw step control -------------- */

/*
 * Starts a raw step on the specified stepper motor.
 *
 * @param stepper_id The ID of the stepper motor.
 * @param direction The direction of the step.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_raw_step_start(const uint8_t stepper_id, stepper_drv_direction_t direction);

/*
 * Sets the frequency of the raw step for the specified stepper motor.
 *
 * @param stepper_id The ID of the stepper motor.
 * @param freq_hz The frequency of the step in Hz.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_raw_step_set_frequency(const uint8_t stepper_id, uint32_t freq_hz);

/*
 * Stops the raw step for the specified stepper motor.
 *
 * @param stepper_id The ID of the stepper motor.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_raw_step_stop(const uint8_t stepper_id);

/*
 * Gets the current position (in steps) of the specified stepper motor.
 *
 * @param stepper_id The ID of the stepper motor.
 * @return The current step position of the stepper motor.
 */
int32_t stepper_drv_get_position(const uint8_t stepper_id);

/* --------------- Motor control ---------------- */

/*
 * Enables the specified stepper motor, allowing it to execute moves.
 * Puts the stepper into an enabled state.
 * While in the enabled state, the stepper can execute moves and respond to halt/resume commands.
 * 
 * @param stepper_id The ID of the stepper motor to enable.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_enable(const uint8_t stepper_id);

/*
 * Disables the specified stepper motor, preventing it from executing moves.
 * Puts the stepper into a disabled state.
 * While in the disabled state, the stepper will not execute any moves and will ignore halt/resume commands.
 * Additionally, the physical stepper will be powered down; allowing for hand manipulation (useful for calibration).
 *
 * @param stepper_id The ID of the stepper motor to disable.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_disable(const uint8_t stepper_id);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_DRIVER_H */
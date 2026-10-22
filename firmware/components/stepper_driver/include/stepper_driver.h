#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/rmt_tx.h"

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

#define STEPPER_DRV_ACTIVE_PIN_VALUE        (0)
#define STEPPER_DRV_RMT_QUEUE_SIZE          (4)         /* Number of RMT events to queue for each stepper */ 
#define STEPPER_DRV_RMT_RESOLUTION_HZ       (1000000)   /* 1 MHz tick rate (1 tick = 1 µs) */
#define STEPPER_DRV_MAX_STEP_FREQUENCY_HZ   (20000)     /* Actual physical step-rate ceiling */
#define STEPPER_DRV_STEP_DUTY_CYCLE_PERCENT (25)        /* Step pulse duty cycle in percentage */    

typedef struct stepper_drv_config
{
    uint8_t pin_step;
    uint8_t pin_dir;
    uint8_t pin_enable;
} stepper_drv_config_t;

typedef enum stepper_drv_state
{
    STEPPER_DRV_STATE_ENABLED, 
    STEPPER_DRV_STATE_DISABLED,   
    STEPPER_DRV_STATE_MOVING
} stepper_drv_state_t;

typedef enum stepper_drv_direction
{
    STEPPER_DRV_DIRECTION_FWD,
    STEPPER_DRV_DIRECTION_REV  
} stepper_drv_direction_t;

/*
 * Stepper driver handle structure.
 * It is advised to treat this structure as opaque and not access its members directly.
 * Use the provided API functions to interact with the stepper driver.
 */
typedef struct stepper_drv_handle
{
    stepper_drv_config_t config;
    rmt_channel_handle_t rmt_channel;
    rmt_encoder_t *copy_encoder;
    _Atomic uint16_t step_frequency_hz;
    _Atomic int32_t step_position;
    _Atomic stepper_drv_state_t state;
    stepper_drv_direction_t direction;
} stepper_drv_handle_t;

/*
 * Initialize the stepper driver with the provided configurations.
 *
 * @param stepper_configs Stepper configuration structure.
 * @param stepper_handle Pointer to the stepper handle.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_init(const stepper_drv_config_t stepper_config, stepper_drv_handle_t* stepper_handle);

/* -------------- Raw step control -------------- */

/*
 * Starts a raw step on the specified stepper motor.
 *
 * @param stepper_handle Pointer to the stepper handle.
 * @param direction The direction of the step.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_raw_step_start(stepper_drv_handle_t* stepper_handle, const stepper_drv_direction_t direction);

/*
 * Sets the frequency of the raw step for the specified stepper motor.
 *
 * @param stepper_handle Pointer to the stepper handle.
 * @param freq_hz The frequency of the step in Hz.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_raw_step_set_frequency(stepper_drv_handle_t* stepper_handle, const uint32_t freq_hz);

/*
 * Stops the raw step for the specified stepper motor.
 *
 * @param stepper_handle Pointer to the stepper handle.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_raw_step_stop(stepper_drv_handle_t* stepper_handle);

/*
 * Gets the current position (in steps) of the specified stepper motor.
 *
 * @param stepper_handle Pointer to the stepper handle.
 * @param position Pointer to the position variable.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_get_position(stepper_drv_handle_t* stepper_handle, int32_t *position);

/* --------------- Motor control ---------------- */

/*
 * Enables the specified stepper motor, allowing it to execute moves.
 * Puts the stepper into an enabled state.
 * While in the enabled state, the stepper can execute moves and respond to halt/resume commands.
 * 
 * @param stepper_handle Pointer to the stepper handle.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_enable(stepper_drv_handle_t* stepper_handle);

/*
 * Disables the specified stepper motor, preventing it from executing moves.
 * Puts the stepper into a disabled state.
 * While in the disabled state, the stepper will not execute any moves and will ignore halt/resume commands.
 * Additionally, the physical stepper will be powered down; allowing for hand manipulation (useful for calibration).
 *
 * @param stepper_handle Pointer to the stepper handle.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t stepper_drv_disable(stepper_drv_handle_t* stepper_handle);

/*
 * Gets the current state of the specified stepper motor.
 *
 * @param stepper_handle Pointer to the stepper handle.
 * @return True if the stepper motor is enabled, false otherwise.
 */
bool stepper_drv_is_enabled(stepper_drv_handle_t* stepper_handle);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_DRIVER_H */
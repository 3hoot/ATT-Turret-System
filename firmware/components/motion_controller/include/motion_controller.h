#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"

#include "as5600.h"
#include "ina226.h"
#include "stepper_driver.h"

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

/* ------------------- Logging ------------------ */

#define MOTION_CTRL_LOG_SAMPLE_COUNT            (5000)
#define MOTION_CTRL_LOG_SAMPLE_TASK_PRIORITY    (11)
#define MOTION_CTRL_LOG_SAMPLE_TASK_STACK_SIZE  (2048)
#define MOTION_CTRL_LOG_SAMPLE_TASK_INTERVAL_MS (10)

/*  
 * When the motion controller's state is set to moving, 
 * the controller will log samples and dump them continuously to the log buffer.
 * Each motion_ctrl handle has its own log buffer.
 */

typedef struct __attribute__((packed)) motion_ctrl_log_sample
{
    int64_t timestamp_us;
    float current;
    float voltage;
    float angle;
} motion_ctrl_log_sample_t;

/* -------------- Main loop / init -------------- */

#define MOTION_CTRL_LOOP_TASK_PRIORITY          (12)
#define MOTION_CTRL_LOOP_TASK_STACK_SIZE        (2048)
#define MOTION_CTRL_LOOP_TASK_INTERVAL_MS       (10)
#define MOTION_CTRL_LOOP_TASK_ERROR_RESOLVE_MS  (100)

#define MOTION_CTRL_MOVE_QUEUE_LENGTH (10)
#define MOTION_CTRL_MOVE_QUEUE_WAIT_MS (50)

#define MOTION_CTRL_TIMEOUT_US (30000000)

typedef enum motion_ctrl_state
{
    MOTION_CTRL_STATE_IDLE = 0,
    MOTION_CTRL_STATE_CALIBRATING,
    MOTION_CTRL_STATE_MOVING,
    MOTION_CTRL_STATE_ERROR
} motion_ctrl_state_t;

/*
 * The motion controller internal logic works off of -90 to +90 degrees, with 0 being the origin position.
 * The min and max angular positions set by user are then mapped to this range.
 */
typedef struct motion_ctrl_limits
{
    float max_position;
    float int_position;
    float min_position;
} motion_ctrl_limits_t;

typedef struct motion_ctrl_config 
{
    stepper_drv_handle_t* stepper;
    as5600_handle_t* position_sensor;
    ina226_handle_t* power_sensor;
    motion_ctrl_limits_t limits;
    float position_threshold;
} motion_ctrl_config_t;

/*
 * Motion controller move structure.
 * If step_frequency_hz is set to 0, the motion controller will adaptively control the stepper.
 */
typedef struct motion_ctrl_move
{
    float end_position;
    uint32_t step_frequency_hz;
    bool block_until_complete;
} motion_ctrl_move_t;

/*
 * Motion controller handle structure.
 * It is advised to treat this structure as opaque and not access its members directly.
 * Use the provided API functions to interact with the motion controller.
 */
typedef struct motion_ctrl_handle
{
    motion_ctrl_config_t config;
    float origin_position;
    float position_range;
    bool flip_sensor_direction;
    _Atomic motion_ctrl_state_t state;
    RingbufHandle_t log_buffer;
    QueueHandle_t move_queue;
} motion_ctrl_handle_t;

/*
 * Initializes the motion controller.
 *
 * @param handle Pointer to the motion controller handle.
 * @param config Pointer to the motion controller configuration.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_init(const motion_ctrl_config_t *config, motion_ctrl_handle_t *ctrl_handle);

/*
 * Gets the current position of the motion controller.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param position Pointer to the variable that will hold the current position.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_get_position(motion_ctrl_handle_t *ctrl_handle, float *position);

/*
 * Moves the motion controller to a specific position.
 *
 * @param ctrl_handle Pointer to the motion controller handle.
 * @param move Pointer to the move structure containing the end position and step frequency (this will be copied internally -> free after use).
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_move(motion_ctrl_handle_t *ctrl_handle, motion_ctrl_move_t *move);

/* ------------------- Logging ------------------ */

/*
 * Dumps the log samples from the motion controller's log buffer.
 * Will wait for the state to be idle before dumping the log (i.e. will block until the motion controller is idle).
 *
 * @param handle Pointer to the motion controller handle.
 * @param samples Pointer to the buffer where the log samples will be stored.
 * @param sample_count How many samples to dump. If the log buffer has fewer samples than this or the value is -1, it will dump all available samples.
 * @param actual_sample_count Pointer to the variable that will hold the actual number of samples dumped.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_dump_log(motion_ctrl_handle_t *handle, motion_ctrl_log_sample_t *samples, int32_t sample_count, size_t* actual_sample_count);

/* ------------ Calibration functions ------------*/

/*
 * Registers the current position of motion controller as the maximum / intermediate / minimum position.
 * After running these, the origin position will be recalculated (look-up motion_ctrl_handle_t comments).
 * Call motion_ctrl_register_max_limit() first, then motion_ctrl_register_int_limit(), then motion_ctrl_register_min_limit().
 * 
 * @param ctrl_handle Pointer to the motion controller handle.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t motion_ctrl_register_max_limit(motion_ctrl_handle_t *ctrl_handle);
esp_err_t motion_ctrl_register_int_limit(motion_ctrl_handle_t *ctrl_handle);
esp_err_t motion_ctrl_register_min_limit(motion_ctrl_handle_t *ctrl_handle);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_H */
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "esp_err.h"
#include "esp_check.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"

#include "stepper_driver.h"

static const char *TAG = "stepper_driver";

/*
 * Build an RMT symbol for a given frequency.
 * Since the RMT peripheral operates at a fixed resolution, 
 * the symbol is constructed to represent a single step pulse with the specified frequency and duty cycle.
 *
 * @param frequency_hz The frequency in Hertz.
 * @param symbol Pointer to the RMT symbol to be built.
 */
static void stepper_drv_rmt_build_symbol(const uint32_t frequency_hz, rmt_symbol_word_t *symbol);

/*
 * Callback function for RMT transmission done event.
 * In this context, it fires when a single step pulse has been completed.
 * 
 * @param channel The RMT channel handle that triggered the event.
 * @param event_data Pointer to the RMT transmission done event data.
 * @param arg User-defined argument, expected to be a pointer to the stepper_drv_stepper_t structure.
 * @return True if a higher priority task was woken, false otherwise.
 */
static bool IRAM_ATTR stepper_drv_raw_step_done(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *event_data, void *arg);

static uint8_t stepper_drv_stepper_instance_count = 0;

esp_err_t stepper_drv_init(const stepper_drv_config_t stepper_config, stepper_drv_handle_t* stepper_handle)
{
    ESP_RETURN_ON_FALSE((stepper_config.pin_step != GPIO_NUM_NC) && (stepper_config.pin_dir != GPIO_NUM_NC) && (stepper_config.pin_enable != GPIO_NUM_NC),
                        ESP_ERR_INVALID_ARG, TAG, "Invalid stepper pin configuration");

    *stepper_handle = (stepper_drv_handle_t) {
        .config = stepper_config
    };

    ESP_LOGI(TAG, "Initializing stepper %d EN and DIR pins", stepper_drv_stepper_instance_count);
    uint64_t gpio_config_mask = ((1ULL << stepper_handle->config.pin_dir) | (1ULL << stepper_handle->config.pin_enable));
    gpio_config_t io_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = gpio_config_mask};
    ESP_ERROR_CHECK(gpio_config(&io_config));
    ESP_ERROR_CHECK(gpio_set_level(stepper_handle->config.pin_enable, !STEPPER_DRV_ACTIVE_PIN_VALUE));  /* Disable stepper by default */

    ESP_LOGI(TAG, "Initializing RMT channel for stepper %d", stepper_drv_stepper_instance_count);
    rmt_tx_channel_config_t rmt_tx_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = stepper_handle->config.pin_step,
        .mem_block_symbols = 64,
        .trans_queue_depth = STEPPER_DRV_RMT_QUEUE_SIZE,
        .resolution_hz = STEPPER_DRV_RMT_RESOLUTION_HZ,
        .intr_priority = 3,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_tx_config, &stepper_handle->rmt_channel));

    rmt_copy_encoder_config_t copy_encoder_config = {0};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &stepper_handle->copy_encoder));

    rmt_tx_event_callbacks_t cbs = { .on_trans_done = stepper_drv_raw_step_done };
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(stepper_handle->rmt_channel, &cbs, stepper_handle));

    ESP_ERROR_CHECK(rmt_enable(stepper_handle->rmt_channel));

    atomic_init(&stepper_handle->step_frequency_hz, 0);
    atomic_init(&stepper_handle->step_position, 0);
    atomic_init(&stepper_handle->state, STEPPER_DRV_STATE_DISABLED);
    stepper_handle->direction = STEPPER_DRV_DIRECTION_FWD;

    ESP_LOGI(TAG, "Stepper %d initialized", stepper_drv_stepper_instance_count++);
    return ESP_OK;
}

esp_err_t stepper_drv_raw_step_start(stepper_drv_handle_t* stepper_handle, const stepper_drv_direction_t direction)
{
    ESP_RETURN_ON_FALSE(stepper_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid stepper handle");
    ESP_RETURN_ON_FALSE(stepper_handle->state != STEPPER_DRV_STATE_DISABLED, ESP_ERR_INVALID_STATE, TAG, "Stepper is disabled");

    if (atomic_load(&stepper_handle->state) == STEPPER_DRV_STATE_MOVING)
    {
        /* Already running, just update the direction */
        stepper_handle->direction = direction;
        ESP_LOGI(TAG, "Stepper already moving, updated direction to %s", direction == STEPPER_DRV_DIRECTION_FWD ? "FWD" : "REV");
        gpio_set_level(stepper_handle->config.pin_dir, direction == STEPPER_DRV_DIRECTION_FWD ? STEPPER_DRV_ACTIVE_PIN_VALUE : !STEPPER_DRV_ACTIVE_PIN_VALUE);
        return ESP_OK;
    }

    stepper_handle->direction = direction;
    gpio_set_level(stepper_handle->config.pin_dir, direction == STEPPER_DRV_DIRECTION_FWD ? STEPPER_DRV_ACTIVE_PIN_VALUE : !STEPPER_DRV_ACTIVE_PIN_VALUE);

    uint32_t frequency_hz = atomic_load(&stepper_handle->step_frequency_hz);
    if (frequency_hz == 0)
    {
        frequency_hz = STEPPER_DRV_MAX_STEP_FREQUENCY_HZ; /* Fallback so that starting the stepper doesn't fail if frequency is not set */
        atomic_store(&stepper_handle->step_frequency_hz, frequency_hz);
    }

    rmt_symbol_word_t symbol;
    stepper_drv_rmt_build_symbol(frequency_hz, &symbol);
    rmt_transmit_config_t tx_config = { .loop_count = 1 }; /* Single pulse transmission */

    ESP_ERROR_CHECK(rmt_transmit(stepper_handle->rmt_channel, stepper_handle->copy_encoder, &symbol, sizeof(symbol), &tx_config));
    atomic_store(&stepper_handle->state, STEPPER_DRV_STATE_MOVING);

    return ESP_OK;
}

esp_err_t stepper_drv_raw_step_set_frequency(stepper_drv_handle_t* stepper_handle, const uint32_t freq_hz)
{
    ESP_RETURN_ON_FALSE(stepper_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid stepper handle");
    ESP_RETURN_ON_FALSE(freq_hz >= 0 && freq_hz <= STEPPER_DRV_MAX_STEP_FREQUENCY_HZ, ESP_ERR_INVALID_ARG, TAG, "Frequency %d Hz is out of range", freq_hz);

    atomic_store(&stepper_handle->step_frequency_hz, freq_hz);

    return ESP_OK;
}

esp_err_t stepper_drv_raw_step_stop(stepper_drv_handle_t* stepper_handle)
{
    ESP_RETURN_ON_FALSE(stepper_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid stepper handle");

    atomic_store(&stepper_handle->state, STEPPER_DRV_STATE_ENABLED);   /* Callback will see this and not re-arm */

    return ESP_OK;
}

esp_err_t stepper_drv_get_position(stepper_drv_handle_t* stepper_handle, int32_t *position)
{
    ESP_RETURN_ON_FALSE(stepper_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid stepper handle");
    ESP_RETURN_ON_FALSE(position != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid position pointer");

    *position = atomic_load(&stepper_handle->step_position);
    return ESP_OK;
}

esp_err_t stepper_drv_enable(stepper_drv_handle_t* stepper_handle)
{
    ESP_RETURN_ON_FALSE(stepper_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid stepper handle");

    gpio_set_level(stepper_handle->config.pin_enable, STEPPER_DRV_ACTIVE_PIN_VALUE); /* Enable stepper */
    atomic_store(&stepper_handle->state, STEPPER_DRV_STATE_ENABLED);

    return ESP_OK;
}

esp_err_t stepper_drv_disable(stepper_drv_handle_t* stepper_handle)
{
    ESP_RETURN_ON_FALSE(stepper_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid stepper handle");

    gpio_set_level(stepper_handle->config.pin_enable, !STEPPER_DRV_ACTIVE_PIN_VALUE); /* Disable stepper */
    atomic_store(&stepper_handle->state, STEPPER_DRV_STATE_DISABLED);

    return ESP_OK;
}

static void stepper_drv_rmt_build_symbol(const uint32_t frequency_hz, rmt_symbol_word_t *symbol)
{
    uint32_t period_ticks = STEPPER_DRV_RMT_RESOLUTION_HZ / frequency_hz;
    uint32_t high_ticks = (period_ticks * STEPPER_DRV_STEP_DUTY_CYCLE_PERCENT) / 100;
    
    /* Guard against zero or full duty cycle */
    if (high_ticks == 0)
    {
        high_ticks = 1;
    }
    if (high_ticks >= period_ticks)
    {
        high_ticks = period_ticks - 1;
    }

    symbol->level0 = 1;
    symbol->duration0 = high_ticks;
    symbol->level1 = 0;
    symbol->duration1 = period_ticks - high_ticks;
} 

static bool IRAM_ATTR stepper_drv_raw_step_done(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *event_data, void *arg)
{
    stepper_drv_handle_t* stepper_handle = (stepper_drv_handle_t *)arg;
    if (stepper_handle == NULL)
    {
        return false;
    }
    
    atomic_fetch_add(&stepper_handle->step_position, (stepper_handle->direction == STEPPER_DRV_DIRECTION_FWD) ? 1 : -1);

    if (!(atomic_load(&stepper_handle->state) == STEPPER_DRV_STATE_MOVING))
    {
        return false;
    }

    uint16_t frequency_hz = atomic_load(&stepper_handle->step_frequency_hz);
    if (frequency_hz == 0)
    {
        return false; /* Frequency is zero, stop stepping */
    }

    rmt_symbol_word_t symbol;
    stepper_drv_rmt_build_symbol(frequency_hz, &symbol);
    rmt_transmit_config_t tx_config = { .loop_count = 1 }; /* Single pulse transmission */ 
    ESP_ERROR_CHECK(rmt_transmit(stepper_handle->rmt_channel, stepper_handle->copy_encoder, &symbol, sizeof(symbol), &tx_config));

    return false; /* No higher level task woken here */
}
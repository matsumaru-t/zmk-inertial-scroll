/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_inertial_scroll

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(inertial_scroll, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct inertial_scroll_config {
    uint32_t friction;
    uint32_t min_velocity;
    uint32_t max_velocity;
    uint32_t max_duration_ms;
    uint32_t velocity_scale;
};

struct inertial_scroll_data {
    const struct device *dev;
    struct k_work_delayable inertia_work;
    
    /* Velocity tracking */
    int32_t velocity_x;
    int32_t velocity_y;
    
    /* Accumulator for sub-pixel movement */
    int32_t accum_x;
    int32_t accum_y;
    
    /* Timing */
    int64_t last_event_time;
    int64_t inertia_start_time;
    bool inertia_active;
    
    /* Last input values for velocity calculation */
    int32_t last_x;
    int32_t last_y;
    int64_t last_x_time;
    int64_t last_y_time;
};

/* Emit scroll event */
static void emit_scroll_event(uint16_t code, int32_t value) {
    if (value == 0) {
        return;
    }
    
    /* Create and emit input event */
    struct input_event ev = {
        .type = INPUT_EV_REL,
        .code = code,
        .value = value,
        .sync = true,
    };
    
    /* Use ZMK's input system to emit the event */
    input_report(NULL, ev.type, ev.code, ev.value, true, K_NO_WAIT);
}

/* Inertia work handler - called periodically during inertial scrolling */
static void inertia_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct inertial_scroll_data *data = 
        CONTAINER_OF(dwork, struct inertial_scroll_data, inertia_work);
    
    const struct device *dev = data->dev;
    const struct inertial_scroll_config *config = dev->config;
    
    int64_t now = k_uptime_get();
    int64_t elapsed = now - data->inertia_start_time;
    
    /* Check if max duration exceeded */
    if (elapsed > config->max_duration_ms) {
        LOG_DBG("Inertia max duration reached");
        data->inertia_active = false;
        data->velocity_x = 0;
        data->velocity_y = 0;
        return;
    }
    
    /* Apply friction to velocity */
    int32_t friction_factor = 100 - config->friction;
    data->velocity_x = (data->velocity_x * friction_factor) / 100;
    data->velocity_y = (data->velocity_y * friction_factor) / 100;
    
    /* Check if velocity is below threshold */
    if (abs(data->velocity_x) < config->min_velocity && 
        abs(data->velocity_y) < config->min_velocity) {
        LOG_DBG("Inertia velocity below threshold, stopping");
        data->inertia_active = false;
        data->velocity_x = 0;
        data->velocity_y = 0;
        return;
    }
    
    /* Calculate scroll amount from velocity */
    /* Scale down velocity to reasonable scroll values */
    int32_t scroll_x = data->velocity_x / 100;
    int32_t scroll_y = data->velocity_y / 100;
    
    /* Accumulate sub-pixel movement */
    data->accum_x += data->velocity_x % 100;
    data->accum_y += data->velocity_y % 100;
    
    /* Convert accumulated sub-pixels to pixels */
    if (abs(data->accum_x) >= 100) {
        scroll_x += data->accum_x / 100;
        data->accum_x = data->accum_x % 100;
    }
    if (abs(data->accum_y) >= 100) {
        scroll_y += data->accum_y / 100;
        data->accum_y = data->accum_y % 100;
    }
    
    /* Emit scroll events */
    if (scroll_x != 0) {
        emit_scroll_event(INPUT_REL_WHEEL, scroll_x);
    }
    if (scroll_y != 0) {
        emit_scroll_event(INPUT_REL_WHEEL, scroll_y);
    }
    
    LOG_DBG("Inertia scroll: vx=%d vy=%d sx=%d sy=%d", 
            data->velocity_x, data->velocity_y, scroll_x, scroll_y);
    
    /* Schedule next tick */
    k_work_schedule(&data->inertia_work, 
                    K_MSEC(CONFIG_ZMK_INERTIAL_SCROLL_TICK_MS));
}

static int inertial_scroll_handle_event(const struct device *dev,
                                        struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);
    
    struct inertial_scroll_data *data = dev->data;
    const struct inertial_scroll_config *config = dev->config;
    
    /* Only process scroll wheel events */
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    
    if (event->code != INPUT_REL_WHEEL && event->code != INPUT_REL_HWHEEL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    
    int64_t now = k_uptime_get();
    
    /* Cancel any ongoing inertia when new input arrives */
    if (data->inertia_active) {
        k_work_cancel_delayable(&data->inertia_work);
        data->inertia_active = false;
    }
    
    /* Calculate velocity based on event value and time delta */
    int32_t value = event->value;
    int64_t time_delta;
    int32_t *last_value;
    int64_t *last_time;
    int32_t *velocity;
    
    if (event->code == INPUT_REL_WHEEL) {
        last_value = &data->last_y;
        last_time = &data->last_y_time;
        velocity = &data->velocity_y;
    } else {
        last_value = &data->last_x;
        last_time = &data->last_x_time;
        velocity = &data->velocity_x;
    }
    
    time_delta = now - *last_time;
    if (time_delta < 1) {
        time_delta = 1;
    }
    
    /* Calculate instantaneous velocity */
    /* velocity = distance * scale / time */
    int32_t instant_velocity = (value * config->velocity_scale * 10) / time_delta;
    
    /* Smooth velocity using exponential moving average */
    if (*last_time > 0 && time_delta < 200) {
        /* Blend with previous velocity */
        *velocity = (*velocity * 30 + instant_velocity * 70) / 100;
    } else {
        /* First event or long gap, use instant velocity */
        *velocity = instant_velocity;
    }
    
    /* Clamp velocity */
    if (*velocity > (int32_t)config->max_velocity) {
        *velocity = config->max_velocity;
    } else if (*velocity < -(int32_t)config->max_velocity) {
        *velocity = -config->max_velocity;
    }
    
    /* Update tracking state */
    *last_value = value;
    *last_time = now;
    data->last_event_time = now;
    
    LOG_DBG("Scroll event: code=%d value=%d velocity=%d", event->code, value, *velocity);
    
    /* Let the original event pass through */
    return ZMK_INPUT_PROC_CONTINUE;
}

/* Called when no more input events (detected via sync) */
static int inertial_scroll_handle_sync(const struct device *dev,
                                       struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);
    
    struct inertial_scroll_data *data = dev->data;
    const struct inertial_scroll_config *config = dev->config;
    
    /* Check if we should start inertia */
    int64_t now = k_uptime_get();
    int64_t since_last = now - data->last_event_time;
    
    /* Start inertia if:
     * 1. Not already active
     * 2. Some time has passed since last event (debounce)
     * 3. Velocity is above threshold
     */
    if (!data->inertia_active && 
        since_last >= 30 && since_last < 150 &&
        (abs(data->velocity_x) >= config->min_velocity ||
         abs(data->velocity_y) >= config->min_velocity)) {
        
        LOG_DBG("Starting inertia: vx=%d vy=%d", data->velocity_x, data->velocity_y);
        
        data->inertia_active = true;
        data->inertia_start_time = now;
        data->accum_x = 0;
        data->accum_y = 0;
        
        k_work_schedule(&data->inertia_work, 
                        K_MSEC(CONFIG_ZMK_INERTIAL_SCROLL_TICK_MS));
    }
    
    return ZMK_INPUT_PROC_CONTINUE;
}

static int inertial_scroll_init(const struct device *dev) {
    struct inertial_scroll_data *data = dev->data;
    
    data->dev = dev;
    data->velocity_x = 0;
    data->velocity_y = 0;
    data->accum_x = 0;
    data->accum_y = 0;
    data->last_event_time = 0;
    data->inertia_active = false;
    data->last_x = 0;
    data->last_y = 0;
    data->last_x_time = 0;
    data->last_y_time = 0;
    
    k_work_init_delayable(&data->inertia_work, inertia_work_handler);
    
    LOG_INF("Inertial scroll input processor initialized");
    return 0;
}

static const struct zmk_input_processor_driver_api inertial_scroll_driver_api = {
    .handle_event = inertial_scroll_handle_event,
};

#define INERTIAL_SCROLL_INST(n)                                                    \
    static struct inertial_scroll_data inertial_scroll_data_##n = {};              \
    static const struct inertial_scroll_config inertial_scroll_config_##n = {      \
        .friction = DT_INST_PROP_OR(n, friction, 15),                              \
        .min_velocity = DT_INST_PROP_OR(n, min_velocity, 50),                      \
        .max_velocity = DT_INST_PROP_OR(n, max_velocity, 2000),                    \
        .max_duration_ms = DT_INST_PROP_OR(n, max_duration_ms, 1500),              \
        .velocity_scale = DT_INST_PROP_OR(n, velocity_scale, 100),                 \
    };                                                                             \
    DEVICE_DT_INST_DEFINE(n, inertial_scroll_init, NULL,                           \
                          &inertial_scroll_data_##n,                               \
                          &inertial_scroll_config_##n,                             \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,        \
                          &inertial_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INERTIAL_SCROLL_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

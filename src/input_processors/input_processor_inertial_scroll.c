/*
 * Copyright (c) 2024 The ZMK Contributors
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
    int32_t velocity_x;
    int32_t velocity_y;
    int32_t accum_x;
    int32_t accum_y;
    int64_t last_event_time;
    int64_t inertia_start_time;
    bool inertia_active;
    int64_t last_x_time;
    int64_t last_y_time;
};

static void emit_scroll_event(uint16_t code, int32_t value) {
    if (value == 0) return;
    input_report(NULL, INPUT_EV_REL, code, value, true, K_NO_WAIT);
}

/* Smooth exponential decay - very gentle friction curve */
static int32_t get_smooth_friction(const struct inertial_scroll_config *config, int32_t velocity) {
    int32_t abs_vel = abs(velocity);
    int32_t base = config->friction;
    
    /* Gentler curve: high speed slightly more friction, low speed very low friction */
    if (abs_vel > 500) {
        return base + (base / 2);  /* 1.5x at high speed */
    } else if (abs_vel > 200) {
        return base;               /* Normal */
    } else if (abs_vel > 50) {
        return base / 2;           /* Half at medium-low */
    } else {
        return base / 4;           /* Quarter at very low - long lingering */
    }
}

static void inertia_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct inertial_scroll_data *data = CONTAINER_OF(dwork, struct inertial_scroll_data, inertia_work);
    const struct device *dev = data->dev;
    const struct inertial_scroll_config *config = dev->config;
    
    int64_t now = k_uptime_get();
    int64_t elapsed = now - data->inertia_start_time;
    
    if (elapsed > config->max_duration_ms) {
        data->inertia_active = false;
        data->velocity_x = 0;
        data->velocity_y = 0;
        return;
    }
    
    /* Apply smooth friction */
    int32_t max_vel = (abs(data->velocity_x) > abs(data->velocity_y)) ? 
                      abs(data->velocity_x) : abs(data->velocity_y);
    int32_t friction = get_smooth_friction(config, max_vel);
    int32_t factor = 1000 - friction;  /* Use 1000 scale for smoother decay */
    if (factor < 100) factor = 100;
    if (factor > 999) factor = 999;
    
    data->velocity_x = (data->velocity_x * factor) / 1000;
    data->velocity_y = (data->velocity_y * factor) / 1000;
    
    if (abs(data->velocity_x) < config->min_velocity && 
        abs(data->velocity_y) < config->min_velocity) {
        data->inertia_active = false;
        data->velocity_x = 0;
        data->velocity_y = 0;
        return;
    }
    
    /* Calculate scroll with sub-pixel accumulation */
    int32_t scroll_x = data->velocity_x / 100;
    int32_t scroll_y = data->velocity_y / 100;
    
    data->accum_x += data->velocity_x % 100;
    data->accum_y += data->velocity_y % 100;
    
    if (abs(data->accum_x) >= 100) {
        scroll_x += data->accum_x / 100;
        data->accum_x = data->accum_x % 100;
    }
    if (abs(data->accum_y) >= 100) {
        scroll_y += data->accum_y / 100;
        data->accum_y = data->accum_y % 100;
    }
    
    if (scroll_x != 0) emit_scroll_event(INPUT_REL_HWHEEL, scroll_x);
    if (scroll_y != 0) emit_scroll_event(INPUT_REL_WHEEL, scroll_y);
    
    k_work_schedule(&data->inertia_work, K_MSEC(CONFIG_ZMK_INERTIAL_SCROLL_TICK_MS));
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
    
    if (event->type != INPUT_EV_REL) return ZMK_INPUT_PROC_CONTINUE;
    if (event->code != INPUT_REL_WHEEL && event->code != INPUT_REL_HWHEEL) return ZMK_INPUT_PROC_CONTINUE;
    
    int64_t now = k_uptime_get();
    
    if (data->inertia_active) {
        k_work_cancel_delayable(&data->inertia_work);
        data->inertia_active = false;
    }
    
    int32_t value = event->value;
    int64_t time_delta;
    int64_t *last_time;
    int32_t *velocity;
    
    if (event->code == INPUT_REL_WHEEL) {
        last_time = &data->last_y_time;
        velocity = &data->velocity_y;
    } else {
        last_time = &data->last_x_time;
        velocity = &data->velocity_x;
    }
    
    time_delta = now - *last_time;
    if (time_delta < 1) time_delta = 1;
    
    /* Gentler velocity calculation */
    int32_t instant_velocity = (value * config->velocity_scale * 8) / time_delta;
    
    /* Smooth velocity with more weight on previous value */
    if (*last_time > 0 && time_delta < 200) {
        *velocity = (*velocity * 40 + instant_velocity * 60) / 100;
    } else {
        *velocity = instant_velocity;
    }
    
    if (*velocity > (int32_t)config->max_velocity) *velocity = config->max_velocity;
    else if (*velocity < -(int32_t)config->max_velocity) *velocity = -config->max_velocity;
    
    *last_time = now;
    data->last_event_time = now;
    
    return ZMK_INPUT_PROC_CONTINUE;
}

static int inertial_scroll_handle_sync(const struct device *dev,
                                       struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    ARG_UNUSED(event);
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);
    
    struct inertial_scroll_data *data = dev->data;
    const struct inertial_scroll_config *config = dev->config;
    
    int64_t now = k_uptime_get();
    int64_t since_last = now - data->last_event_time;
    
    if (!data->inertia_active && 
        since_last >= 10 && since_last < 200 &&
        (abs(data->velocity_x) >= config->min_velocity ||
         abs(data->velocity_y) >= config->min_velocity)) {
        
        data->inertia_active = true;
        data->inertia_start_time = now;
        data->accum_x = 0;
        data->accum_y = 0;
        k_work_schedule(&data->inertia_work, K_MSEC(CONFIG_ZMK_INERTIAL_SCROLL_TICK_MS));
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
    data->last_x_time = 0;
    data->last_y_time = 0;
    k_work_init_delayable(&data->inertia_work, inertia_work_handler);
    return 0;
}

static const struct zmk_input_processor_driver_api inertial_scroll_driver_api = {
    .handle_event = inertial_scroll_handle_event,
};

#define INERTIAL_SCROLL_INST(n) \
    static struct inertial_scroll_data inertial_scroll_data_##n = {}; \
    static const struct inertial_scroll_config inertial_scroll_config_##n = { \
        .friction = DT_INST_PROP_OR(n, friction, 15), \
        .min_velocity = DT_INST_PROP_OR(n, min_velocity, 50), \
        .max_velocity = DT_INST_PROP_OR(n, max_velocity, 2000), \
        .max_duration_ms = DT_INST_PROP_OR(n, max_duration_ms, 1500), \
        .velocity_scale = DT_INST_PROP_OR(n, velocity_scale, 100), \
    }; \
    DEVICE_DT_INST_DEFINE(n, inertial_scroll_init, NULL, \
                          &inertial_scroll_data_##n, \
                          &inertial_scroll_config_##n, \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
                          &inertial_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INERTIAL_SCROLL_INST)

#endif

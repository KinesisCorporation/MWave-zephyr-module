/*
 * Copyright (c) 2024 Polarity Works
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_advertising_mode

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zmk/ble_add.h>
#include <dt-bindings/advertising_mode.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)


static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {

    switch (binding->param1) {
    case EXT_POWER_OFF_CMD:
        return adv_mode_disable(adv_mode);
    case EXT_POWER_ON_CMD:
        return adv_mode_enable(adv_mode);
    case EXT_POWER_TOGGLE_CMD:
        if (adv_mode_get(adv_mode) > 0)
            return adv_mode_disable(adv_mode);
        else
            return adv_mode_enable(adv_mode);
    default:
        LOG_ERR("Unknown adv_mode command: %d", binding->param1);
    }

    return -ENOTSUP;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_adv_mode_init(const struct device *dev) { return 0; };

static const struct behavior_driver_api behavior_adv_mode_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
};

BEHAVIOR_DT_INST_DEFINE(0, behavior_adv_mode_init, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_adv_mode_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <math.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/led_strip.h>

#include <zmk/activity.h>
#include <zmk/usb.h>
#include <zmk/ble.h>
#include <zmk/battery.h>
#include <zmk/hid_indicators.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/endpoints.h>
#include <zmk/indicators.h>

#include <zmk/workqueue.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if !DT_HAS_CHOSEN(zmk_indicators)

#error "A zmk,indicators chosen node must be declared"

#endif

#define ZMK_LED_NUMLOCK_BIT BIT(0)
#define ZMK_LED_CAPSLOCK_BIT BIT(1)
#define ZMK_LED_SCROLLLOCK_BIT BIT(2)
#define ZMK_LED_COMPOSE_BIT BIT(3)
#define ZMK_LED_KANA_BIT BIT(4)

struct zmk_led_hsb {
    uint16_t h;
    uint8_t s;
    uint8_t b;
};

#define STRIP_CHOSEN DT_CHOSEN(zmk_indicators)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_CHOSEN, chain_length)

#define HUE_MAX 360
#define SAT_MAX 100
#define BRT_MAX 100

struct zmk_stp_ble {
    uint8_t prof;
    bool open;
    bool connected;
};

static struct zmk_led_hsb color0; // LED0
static struct zmk_led_hsb color1; // LED1

static struct zmk_stp_ble ble_status;
static bool numl;
static bool usb;
static bool battery;

static bool on;

static const struct device *led_strip;

static struct led_rgb pixels[STRIP_NUM_PIXELS];

static struct led_rgb hsb_to_rgb(struct zmk_led_hsb hsb) {
    float r, g, b;

    uint8_t i = hsb.h / 60;
    float v = hsb.b / ((float)BRT_MAX);
    float s = hsb.s / ((float)SAT_MAX);
    float f = hsb.h / ((float)HUE_MAX) * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
        r = v;
        g = p;
        b = q;
        break;
    }

    struct led_rgb rgb = {r : r * 255, g : g * 255, b : b * 255};

    return rgb;
}

static void zmk_stp_indicators_batt(struct k_work *work) {
    // Get state of charge
    uint8_t soc = zmk_battery_state_of_charge();
    LOG_DBG("State of charge: %d", soc);
    struct led_rgb rgb;
    if (soc > 80) {
        rgb.r = 0;
        rgb.g = 255;
        rgb.b = 0;
    } else if (soc > 50 && soc < 80) {
        rgb.r = 255;
        rgb.g = 255;
        rgb.b = 0;
    } else if (soc > 20 && soc < 51) {
        rgb.r = 255;
        rgb.g = 140;
        rgb.b = 0;
    } else {
        rgb.r = 255;
        rgb.g = 0;
        rgb.b = 0;
    }
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = rgb;
    }
    int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }
}

K_WORK_DEFINE(battery_ind_work, zmk_stp_indicators_batt);

int zmk_stp_indicators_enable_batt() {
    // Stop blinking timers
    k_timer_stop(&slow_blink_timer);
    k_timer_stop(&fast_blink_timer);
    k_timer_stop(&connected_timeout_timer);
    // Set battery flag to prevent other things overriding
    battery = true;
    // Submit battery work to queue
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &battery_ind_work);
    return 0;
}
int zmk_stp_indicators_disable_batt() {
    // Unset battery flag to allow other events to override
    battery = false;
    // Submit works to update both LEDs
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &bluetooth_ind_work);
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &numl_ind_work);
    return 0;
}

static void zmk_stp_indicators_blink_work(struct k_work *work) {
    LOG_DBG("Blink work triggered");
    // If LED on turn off and vice cersa
    if (color0.b)
        color0.b = 0;
    else
        color0.b = CONFIG_ZMK_STP_INDICATORS_BRT_MAX;
    // Convert HSB to RGB and update LEDs
    pixels[IS_ENABLED(CONFIG_ZMK_STP_INDICATORS_SWITCH_LEDS)?1:0] = hsb_to_rgb(color0);
    int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }
}

K_WORK_DEFINE(blink_work, zmk_stp_indicators_blink_work);

static void zmk_stp_indicators_blink_handler(struct k_timer *timer) {
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &blink_work);
}

// Define timers for blinking and led timeout
K_TIMER_DEFINE(fast_blink_timer, zmk_stp_indicators_blink_handler, NULL);
K_TIMER_DEFINE(slow_blink_timer, zmk_stp_indicators_blink_handler, NULL);
K_TIMER_DEFINE(connected_timeout_timer, zmk_stp_indicators_blink_handler, NULL);

static void zmk_stp_indicators_bluetooth(struct k_work *work) {
    // Set LED to blue if profile one, set sat to 0 if profile 0 (white)
    LOG_DBG("BLE PROFILE: %d", ble_status.prof);
    color0.h = 240;
    if (ble_status.prof) {
        color0.s = 100;
    } else
        color0.s = 0;
    // If in USB HID mode
    if (usb) {
        LOG_DBG("USB MODE");
        // Stop all timers
        k_timer_stop(&slow_blink_timer);
        k_timer_stop(&fast_blink_timer);
        k_timer_start(&connected_timeout_timer, K_SECONDS(3), K_NO_WAIT);
        // Set LED to green
        color0.h = 120;
        color0.s = 100;
        color0.b = CONFIG_ZMK_STP_INDICATORS_BRT_MAX;
    } else if (ble_status.open) {
        LOG_DBG("BLE PROF OPEN");
        // If profile is open (unpaired) start fast blink timer and ensure LED turns on
        color0.b = CONFIG_ZMK_STP_INDICATORS_BRT_MAX;
        k_timer_stop(&slow_blink_timer);
        k_timer_stop(&connected_timeout_timer);
        k_timer_start(&fast_blink_timer, K_NO_WAIT, K_MSEC(200));
    } else if (!ble_status.connected) {
        LOG_DBG("BLE PROF NOT CONN");
        // If profile paired but not connected start slow blink timer and ensure LED on
        color0.b = CONFIG_ZMK_STP_INDICATORS_BRT_MAX;
        k_timer_stop(&fast_blink_timer);
        k_timer_stop(&connected_timeout_timer);
        k_timer_start(&slow_blink_timer, K_NO_WAIT, K_MSEC(750));
    } else {
        LOG_DBG("BLE PROF CONN");
        // If connected start the 3 second timeout to turn LED off
        color0.b = CONFIG_ZMK_STP_INDICATORS_BRT_MAX;
        k_timer_stop(&slow_blink_timer);
        k_timer_stop(&fast_blink_timer);
        if(!zmk_usb_is_powered())
            k_timer_start(&connected_timeout_timer, K_SECONDS(3), K_NO_WAIT);
    }
    // Convert HSB to RGB and update the LEDs

    pixels[IS_ENABLED(CONFIG_ZMK_STP_INDICATORS_SWITCH_LEDS)?1:0] = hsb_to_rgb(color0);
    LOG_DBG("Setting LED:%d", IS_ENABLED(CONFIG_ZMK_STP_INDICATORS_SWITCH_LEDS)?1:0);
    int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }
}

static void zmk_stp_indicators_numl(struct k_work *work) {
    color1.s = 0;
    // Set LED on if numllock pressed
    if (numl)
        color1.b = CONFIG_ZMK_STP_INDICATORS_BRT_MAX;
    else
        color1.b = 0;
    // Convert HSB to RGB and update the LEDs
    pixels[IS_ENABLED(CONFIG_ZMK_STP_INDICATORS_SWITCH_LEDS)?0:1] = hsb_to_rgb(color1);
    LOG_DBG("Setting LED:%d", IS_ENABLED(CONFIG_ZMK_STP_INDICATORS_SWITCH_LEDS)?0:1);
    int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }
}

// Define work to update LEDs
K_WORK_DEFINE(bluetooth_ind_work, zmk_stp_indicators_bluetooth);
K_WORK_DEFINE(numl_ind_work, zmk_stp_indicators_numl);

static void zmk_stp_indicators_battery_blink_work(struct k_work *work) {
    LOG_DBG("Blink work triggered");
    // If LED on turn off and vice cersa
    color0.h = 0;
    color0.s = 100;
    color1.h = 0;
    color1.s = 100;
    if (color0.b){
        color0.b = 0;
        color1.b = 0;
    }
    else{
        color0.b = CONFIG_ZMK_STP_INDICATORS_BRT_MAX;
        color1.b = CONFIG_ZMK_STP_INDICATORS_BRT_MAX;
        }
    // Convert HSB to RGB and update LEDs
    pixels[IS_ENABLED(CONFIG_ZMK_STP_INDICATORS_SWITCH_LEDS)?1:0] = hsb_to_rgb(color0);
    pixels[IS_ENABLED(CONFIG_ZMK_STP_INDICATORS_SWITCH_LEDS)?0:1] = hsb_to_rgb(color1);
    int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }
}

K_WORK_DEFINE(battery_blink_work, zmk_stp_indicators_battery_blink_work);

static void zmk_stp_indicators_battery_blink_handler(struct k_timer *timer) {
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &battery_blink_work);
}

// Define timers for blinking and led timeout
K_TIMER_DEFINE(battery_blink_timer, zmk_stp_indicators_battery_blink_handler, NULL);

static void zmk_stp_indicators_battery_timer_handler(struct k_timer *timer) {
    //do some battery stuf here  
    battery = false;
    k_timer_stop(&battery_blink_timer); 
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &bluetooth_ind_work);
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &numl_ind_work);
}

// Define timers for blinking and led timeout
K_TIMER_DEFINE(battery_timeout_timer, zmk_stp_indicators_battery_timer_handler, NULL);

static void zmk_stp_indicators_battery_low_timer_handler(struct k_timer *timer) {
    //do some battery stuf here
    battery = true;
    k_timer_start(&battery_blink_timer, K_NO_WAIT, K_MSEC(750));
    k_timer_start(&battery_timeout_timer, K_SECONDS(5), K_NO_WAIT);
}

// Define timers for blinking and led timeout
K_TIMER_DEFINE(battery_low_timer, zmk_stp_indicators_battery_low_timer_handler, NULL);

void zmk_stp_indicators_resample(){
    ble_status.connected = zmk_ble_active_profile_is_connected();
    ble_status.open = zmk_ble_active_profile_is_open();
    ble_status.prof = zmk_ble_active_profile_index();
    usb = (zmk_endpoints_preferred().transport==ZMK_TRANSPORT_USB);

    numl = (zmk_hid_indicators_get_current_profile() & ZMK_LED_NUMLOCK_BIT);
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &bluetooth_ind_work);
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &numl_ind_work);
}

static void zmk_stp_indicators_resample_work(struct k_work *work) {
    LOG_DBG("Resample work triggered");
    zmk_stp_indicators_resample();
}

K_WORK_DELAYABLE_DEFINE(resample_work, zmk_stp_indicators_resample_work);


static int zmk_stp_indicators_init(void) {

    LOG_DBG("Initialising STP indicators");

    led_strip = DEVICE_DT_GET(STRIP_CHOSEN);

    color0 = (struct zmk_led_hsb){
        h : 240,
        s : 100,
        b : CONFIG_ZMK_STP_INDICATORS_BRT_MAX,
    };

    color1 = (struct zmk_led_hsb){
        h : 240,
        s : 0,
        b : CONFIG_ZMK_STP_INDICATORS_BRT_MAX,
    };

    ble_status = (struct zmk_stp_ble){
        prof : zmk_ble_active_profile_index(),
        open : zmk_ble_active_profile_is_open(),
        connected : zmk_ble_active_profile_is_connected()
    };
    numl = (zmk_hid_indicators_get_current_profile() & ZMK_LED_NUMLOCK_BIT);
    usb = false;
    battery = false;

    on = true;

    k_work_schedule(&resample_work, K_MSEC(500));

    return 0;
}

int zmk_stp_indicators_on() {
    if (!led_strip)
        return -ENODEV;

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &bluetooth_ind_work);
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &numl_ind_work);

    return 0;
}

static void zmk_stp_indicators_off_handler(struct k_work *work) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = (struct led_rgb){r : 0, g : 0, b : 0};
    }

    led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
}

K_WORK_DEFINE(underglow_off_work, zmk_stp_indicators_off_handler);

int zmk_stp_indicators_off() {
    if (!led_strip)
        return -ENODEV;

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_off_work);
    on = false;

    return 0;
}

static int stp_indicators_auto_state(bool *prev_state, bool new_state) {
    if (on == new_state) {
        return 0;
    }
    if (new_state) {
        on = *prev_state;
        *prev_state = false;
        return zmk_stp_indicators_on();
    } else {
        on = false;
        *prev_state = true;
        return zmk_usb_is_powered() ? 0 : zmk_stp_indicators_off();
    }
}

static int stp_indicators_event_listener(const zmk_event_t *eh) {
    // If going idle or waking up
    if (as_zmk_activity_state_changed(eh)) {
        static bool prev_state = false;
        return stp_indicators_auto_state(&prev_state,
                                         zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    }
    // If USB state changed
    if (as_zmk_endpoint_changed(eh)) {

        // Get new USB state, HID state and set local flags
        usb = (zmk_endpoints_preferred().transport==ZMK_TRANSPORT_USB);
        LOG_DBG("ENDPOINT EVENT: %d", usb);

        numl = (zmk_hid_indicators_get_current_profile() & ZMK_LED_NUMLOCK_BIT);
        //  Update LEDs
        //
        if (!battery) {
            k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &bluetooth_ind_work);
            k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &numl_ind_work);
        }
        return 0;
    }

    // If BLE state changed
    if (as_zmk_ble_active_profile_changed(eh)) {
        LOG_DBG("BLE CHANGE LOGGED");
        // Get BLE information, Numl state and set local flags
        ble_status.connected = zmk_ble_active_profile_is_connected();
        ble_status.open = zmk_ble_active_profile_is_open();
        ble_status.prof = zmk_ble_active_profile_index();
        numl = (zmk_hid_indicators_get_current_profile() & ZMK_LED_NUMLOCK_BIT);
        // Update LEDs
        if (!battery) {
            k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &bluetooth_ind_work);
            k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &numl_ind_work);
        }
        return 0;
    }

    if (as_zmk_hid_indicators_changed(eh)) {
        // Get new HID state, set local flags
        numl = (zmk_hid_indicators_get_current_profile() & ZMK_LED_NUMLOCK_BIT);
        LOG_DBG("INDICATOR CHANGED, num %d", numl);
        if (!battery) {
            k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &numl_ind_work);
            // k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &bluetooth_ind_work);
        }
        return 0;
    }
    if (as_zmk_battery_state_changed(eh)) {
        // Get battery state, if low blinky blinky

        LOG_DBG("Battery event");
        if(zmk_battery_state_of_charge() < CONFIG_ZMK_STP_INDICATORS_BATTERY_THRESHOLD) {
            LOG_DBG("LOW BATTERY WARNING");
            battery=true; 
            k_timer_stop(&slow_blink_timer);
            k_timer_stop(&fast_blink_timer);
            k_timer_stop(&connected_timeout_timer);
            k_timer_start(&battery_low_timer, K_NO_WAIT, K_MINUTES(1));

        }
        return 0;
    }


    return -ENOTSUP;
}

ZMK_LISTENER(stp_indicators, stp_indicators_event_listener);

ZMK_SUBSCRIPTION(stp_indicators, zmk_activity_state_changed);
ZMK_SUBSCRIPTION(stp_indicators, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(stp_indicators, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(stp_indicators, zmk_hid_indicators_changed);
ZMK_SUBSCRIPTION(stp_indicators, zmk_battery_state_changed);

SYS_INIT(zmk_stp_indicators_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
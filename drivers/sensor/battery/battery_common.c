/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <zephyr/drivers/sensor.h>

#include "battery_common.h"

int battery_channel_get(const struct battery_value *value, enum sensor_channel chan,
                        struct sensor_value *val_out) {
    switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val_out->val1 = value->millivolts / 1000;
        val_out->val2 = (value->millivolts % 1000) * 1000U;
        break;

    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val_out->val1 = value->state_of_charge;
        val_out->val2 = 0;
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

uint8_t lithium_ion_mv_to_pct(int16_t bat_mv) {
    // Simple linear approximation of a battery based off adafruit's discharge graph:
    // https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages

    if (bat_mv >= 4200) {
        return 100;
    } else if (bat_mv <= 3450) {
        return 0;
    }

    return bat_mv * 2 / 15 - 459;
}

uint8_t alkaline_mv_to_pct(int16_t bat_mv) {

    // Discharge data from https://lygte-info.dk/info/BatteriesLowCurrentDischarge2%20UK.html

       struct lookup_point {
        int16_t millivolts;
        int16_t percent;
    };

    static const struct lookup_point battery_lookup[] = {
        {.millivolts = (1600*2), .percent = 100}, 
        {.millivolts = (1500*2), .percent = 94},
        {.millivolts = (1450*2), .percent = 88},
        {.millivolts = (1300*2), .percent = 40},
        {.millivolts = (1260*2), .percent = 28},
        {.millivolts = (1200*2), .percent = 16},
        {.millivolts = (1140*2), .percent = 10},
        {.millivolts = (900*2), .percent = 0},
    };

    if (bat_mv > battery_lookup[0].millivolts) {
        return battery_lookup[0].percent;
    }

    for (int i = 1; i < ARRAY_SIZE(battery_lookup); i++) {
        struct lookup_point one = battery_lookup[i - 1];
        struct lookup_point two = battery_lookup[i];
        if (bat_mv >= two.millivolts) {
            const int t = bat_mv - one.millivolts;
            const int dx = two.millivolts - one.millivolts;
            const int dy = two.percent - one.percent;
            return one.percent + dy * t / dx;
        }
    }

    return battery_lookup[ARRAY_SIZE(battery_lookup) - 1].percent;
}

uint8_t cr2032_mv_to_pct(int16_t bat_mv) {

       struct lookup_point {
        int16_t millivolts;
        int16_t percent;
    };

    static const struct lookup_point battery_lookup[] = {
        {.millivolts = 2900, .percent = 100}, 
        {.millivolts = 2700, .percent = 30},
        {.millivolts = 2600, .percent = 20},
        {.millivolts = 2500, .percent = 10},
        {.millivolts = 2200, .percent = 5},
        {.millivolts = 2000, .percent = 0},
    };

    if (bat_mv > battery_lookup[0].millivolts) {
        return battery_lookup[0].percent;
    }

    for (int i = 1; i < ARRAY_SIZE(battery_lookup); i++) {
        struct lookup_point one = battery_lookup[i - 1];
        struct lookup_point two = battery_lookup[i];
        if (bat_mv >= two.millivolts) {
            const int t = bat_mv - one.millivolts;
            const int dx = two.millivolts - one.millivolts;
            const int dy = two.percent - one.percent;
            return one.percent + dy * t / dx;
        }
    }

    return battery_lookup[ARRAY_SIZE(battery_lookup) - 1].percent;
}
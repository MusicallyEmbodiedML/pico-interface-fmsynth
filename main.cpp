/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
extern "C" {
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
}

#include <cmath>
#include <initializer_list>
#include <vector>
#include "MedianFilter.h"
#include "MEMLSerial_Pico.hpp"
#include "MaxiTrigger.hpp"

#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwipopts.h"
#include "cgi.h"
#include "ssi.h"

#include "pico/multicore.h"

void run_server() {
    httpd_init();
    ssi_init();
    cgi_init();
    // printf("Http server initialized.\n");
    // infinite loop for now
    for (;;) {
        tight_loop_contents();
    }
}

void core1_entry() {

    if (cyw43_arch_init()) {
        // printf("failed to initialise\n");
        // return 1;
    }
    cyw43_arch_enable_sta_mode();
    // this seems to be the best be can do using the predefined `cyw43_pm_value` macro:
    // cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
    // however it doesn't use the `CYW43_NO_POWERSAVE_MODE` value, so we do this instead:
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1, 1, 1));

    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("This Hotspot Is Hotter Than You", "nemoreno", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect.\n");
        return;
    } else {
        printf("Connected.\n");

        extern cyw43_t cyw43_state;
        auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
        printf("IP Address: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
    }
    // turn on LED to signal connected
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);


    run_server();
}


const int NBUTTONS = 4;
// size_t buttonPins[NBUTTONS] = {13,14,15,3,4,5};
size_t buttonPins[NBUTTONS] = {13,14,15,16};
bool buttonValues[NBUTTONS]={0,0,0,0};

const int NADCS = 3;
maxiTrigger<uint16_t> adcChangeTrigs[NADCS];
float adcValue[NADCS];
float adcValue_smoothed[NADCS];

MedianFilter<float> adcFilters[NADCS];
MedianFilter<int> buttonFilters[NBUTTONS];
MedianFilter<int> clockFilter;


static int pulseCount=0;
static uint64_t lastPulse=0;
auto serial = std::make_unique<MEMLSerial>();


void gpio_callback(uint gpio, uint32_t events) {
    absolute_time_t t0;
    t0 = get_absolute_time();
    // int64_t now = to_us_since_boot(t0);
    uint64_t diff = absolute_time_diff_us(lastPulse, t0);
    // printf("pulse %d, %lld\n", pulseCount++, diff);
    lastPulse = t0;
    diff = clockFilter.process(diff);

    serial->sendMessage(MEMLSerial::msgType::pulse_period, 0, diff);
}

int main() {
    stdio_init_all();
    printf("MEML FM Synth Interface\n");

    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE , true, &gpio_callback);

    multicore_launch_core1(core1_entry);

    //serial cx
    const unsigned int kGPIO_UART_TX = 0;
    const unsigned int kGPIO_UART_RX = 1;
    gpio_set_function(kGPIO_UART_TX, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(kGPIO_UART_RX, UART_FUNCSEL_NUM(uart0, 1));

    //serialSLIP serial;


    for (auto &v: adcFilters) {
        v.init(25);
    }
    for (auto &v: buttonFilters) {
        v.init(10);
    }
    clockFilter.init(7);
    //setup adc
    constexpr unsigned int kN_adc = 3;
    adc_init();
    for(auto& i: {26,27,28}) {
        adc_gpio_init(i);
    }

    //setup buttons
    for(auto& i: buttonPins) {
        gpio_init(i);
        gpio_set_dir(i, 0);
        gpio_pull_up(i);
    }

    // Set up polling and smoothing
    constexpr int kInterval_us = 100;
    constexpr double kInterval_s = static_cast<double>(kInterval_us*kN_adc) * 1e-6;
    constexpr float kSample_rate = static_cast<float>(1./kInterval_s);
    constexpr float kSmoothing_time_ms = 10;

    while (1) {
        //read the ADCs
        for(auto& i: {0,1,2}) {
            adc_select_input(i);
            //give time for ADC to settle
            sleep_us(kInterval_us);
            adcValue[i] = static_cast<float>(adc_read());
        }
        for (auto& i: {0,1,2}) {
            int16_t smoothed_adc_value = static_cast<int16_t>(adcFilters[i].process(adcValue[i]));
            if (adcChangeTrigs[i].onChanged(smoothed_adc_value, 20)) {
                serial->sendMessage(MEMLSerial::joystick, i, smoothed_adc_value << 4);
            }
        }
        size_t idx=0;
        for(auto& i: buttonPins) {
            bool buttonValue = buttonFilters[idx].process(gpio_get(i));
            if (buttonValue != buttonValues[idx]) {
                buttonValues[idx] = buttonValue;
                //printf("button: %d: %d\n", i, buttonValue);
                //serial.sendMessage(static_cast<serialSLIP::messageTypes>(serialSLIP::messageTypes::TRAINMODE+idx), buttonValue);
                // TODO AM Button indexes should be reversed properly
                static const std::vector<size_t> button_idx_translate{2, 1, 0, 3};
                serial->sendMessage(MEMLSerial::button, button_idx_translate[idx], buttonValue);
            }
            idx++;
        }

        sleep_ms(10);
    }
}

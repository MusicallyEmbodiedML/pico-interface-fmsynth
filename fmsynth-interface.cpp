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
#include "hardware/uart.h"
}

#include <cmath>
#include <initializer_list>


template<typename T>
class maxiTrigger
{
public:
    maxiTrigger() {};
    /*! Generate a trigger when a signal changes beyond a certain amount \param input A signal \param tolerance The amount of chance allowed before a trigger is generated*/
    double onChanged(T input, T tolerance)
    {
        T changed = 0;
        if (abs(input - previousValue) > tolerance)
        {
            changed = 1;
            previousValue = input;
        }
        return changed;
    }

private:
    T previousValue = 1;
    bool firstTrigger = 1;
};

maxiTrigger<uint16_t> changeTrigs[3];

uint16_t adcValue[3];

int main() {
    stdio_init_all();
    printf("MEML FM Synth Interface\n");

    //serial cx
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
    
    uart_init(uart0, 115200);
    uart_puts(uart0, "Hello world!");

    adc_init();


    for(auto& i: {26,27,28}) {
        adc_gpio_init(i);
    }

    while (1) {
        for(auto& i: {0,1,2}) {
            adc_select_input(i);
            //give time for ADC to settle
            sleep_us(100);
            adcValue[i] = adc_read();
            if (changeTrigs[i].onChanged(adcValue[i], 40)) {
                printf("ADC %d:\t", i);
                for(int j=0; j < i; j++) printf("\t");
                printf("%d\n", adcValue[i]);
            }
        }
        sleep_ms(10);
    }
}

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
#include <vector>


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

maxiTrigger<uint16_t> adcChangeTrigs[3];
bool buttonValues[3]={0,0,0};

uint16_t adcValue[3];
size_t buttonPins[3] = {13,14,15};

struct serialSLIP {
public:
    enum messageTypes {JOYSTICKX=0, JOYSTICKY, JOYSTICKZ, TRAINMODE, RANDOMISE, RECORD};

    //SLIP encoding
    static const uint8_t eot = 0300;
	static const uint8_t slipesc = 0333;
	static const uint8_t slipescend = 0334;
	static const uint8_t slipescesc = 0335;

    void sendDatagram(uint8_t bytes[3]) {
        uart_putc_raw(uart0, eot);
        for(int i=0; i < 3; i++) {
            if(bytes[i] == eot){ 
                uart_putc_raw(uart0, slipesc);
                uart_putc_raw(uart0, slipescend); 
            } else if(bytes[i]==slipesc) {  
                uart_putc_raw(uart0, slipesc);
                uart_putc_raw(uart0, slipescesc); 
            } else {
                uart_putc_raw(uart0, bytes[i]);
            }	
        }
        uart_putc_raw(uart0, eot);
    }

    void sendMessage(messageTypes msgType, uint16_t value) {
        //___encode to bytes

        //send message type
        uint8_t msgData[3];
        msgData[0] = static_cast<uint8_t>(msgType);

        //decode and send val
        uint8_t* byteArray = reinterpret_cast<uint8_t*>(&value);
        msgData[1] = byteArray[0];
        msgData[2] = byteArray[1];

        printf("%d %d %d\n", msgData[0], msgData[1], msgData[2]);
        //___send
        sendDatagram(msgData);
    }
};

int main() {
    stdio_init_all();
    printf("MEML FM Synth Interface\n");

    //serial cx
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
    
    uart_init(uart0, 115200);

    serialSLIP serial;

    //setup adc
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



    while (1) {
        for(auto& i: {0,1,2}) {
            adc_select_input(i);
            //give time for ADC to settle
            sleep_us(100);
            adcValue[i] = adc_read();
            if (adcChangeTrigs[i].onChanged(adcValue[i], 40)) {
                printf("ADC %d:\t", i);
                for(int j=0; j < i; j++) printf("\t");
                printf("%d\n", adcValue[i]);
                serial.sendMessage(static_cast<serialSLIP::messageTypes>(serialSLIP::messageTypes::JOYSTICKX + i), adcValue[i]);
            }
        }
        size_t idx=0;
        for(auto& i: buttonPins) {
            bool buttonValue = gpio_get(i);
            if (buttonValue != buttonValues[idx]) {
                buttonValues[idx] = buttonValue;
                printf("button: %d: %d\n", i, buttonValue);
                serial.sendMessage(static_cast<serialSLIP::messageTypes>(serialSLIP::messageTypes::TRAINMODE+idx), buttonValue);
            }
            idx++;
        }

        sleep_ms(10);
    }
}

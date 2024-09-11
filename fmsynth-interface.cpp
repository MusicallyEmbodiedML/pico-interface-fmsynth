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
#include <array>


template<typename T>
class maxiTrigger
{
public:
    maxiTrigger() {};
    /*! Generate a trigger when a signal changes beyond a certain amount
    \param input A signal
    \param tolerance The amount of chance allowed before a trigger is generated*/
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

float adcValue[3];
float adcValue_smoothed[3];
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


/**
 * MEML Serial implementation
 * 
 */

#include <cstring>
#include <string>
#include <memory>



class MEMLSerial {

 public:

    enum msgType
    {
        joystick='j',
        button='b'
    };

    const char delim = '\n';

    MEMLSerial(uart_inst_t *uart_hw = uart0) :
        uart_hw_(uart_hw),
        datagram_buffer_({ '\0' }) {
        unsigned int requested_baudrate = 115200;

        // Init UART internally
        if(uart_init(uart_hw_, requested_baudrate) != requested_baudrate) {
            printf("ERROR - Uart not initialised!\n");
            uart_is_init_ = false;
        }
        uart_set_format(uart_hw_, 8, 2, UART_PARITY_NONE);
        uart_is_init_ = true;
    }

    void sendMessage(msgType type, uint8_t index, std::string &value) {
        std::sprintf(datagram_buffer_.data(),
            "%c,%d,%s%c",
            type,
            index,
            value.c_str(),
            delim);
        if (uart_is_init_) {
            //printf("%s", datagram_buffer_.data());

            //uart_puts(uart0, datagram_buffer_.data());
            //memcpy(datagram_buffer_.data(), "ABCDE\n", sizeof(char)*7);
            for (auto &c : datagram_buffer_) {
                if (c == '\0') {
                    break;
                }
                uart_putc_raw(uart0, c);
                printf("%c", c);
            }
        } else {
            printf("No echo - UART not init\n");
        }
    }
    // Overloads for common types of messages
    void sendMessage(msgType type, uint8_t index, uint16_t value) {
        std::string value_str;
        value_str.reserve(8);
        std::sprintf(value_str.data(), "%d", value);
        //printf("Value:%d, string: %s\n", value, value_str.c_str());
        sendMessage(type, index, value_str);
    }

 private:

    static constexpr unsigned int kDatagram_buffer_length = 128;
    uart_inst_t *uart_hw_;
    std::array<char, kDatagram_buffer_length> datagram_buffer_;
    bool uart_is_init_;
};


template<size_t n_channels>
class OnePoleSmoother {
 public:
    OnePoleSmoother(float time_ms, float sample_rate) :
        sample_rate_(sample_rate),
        y_ { 0 } {
        SetTimeMs(time_ms);
    }
    void SetTimeMs(float time_ms) {
        //b1_ = std::exp(
        //    std::log(0.01) /
        //    time_ms * sample_rate_ * 0.001
        //);
        b1_ = std::pow(0.1, 1.f/ (time_ms * 0.001 * sample_rate_));
    }
    __attribute__((always_inline)) void Process(const float * x_ptr, float *y_ptr) {
        float *y2_ptr = y_;
        for (unsigned int c = 0; c < n_channels; c++) {
            const float x = *x_ptr; 
            *y2_ptr = *y_ptr = x + b1_ * (*y2_ptr - x);
            ++x_ptr;
            ++y_ptr;
            ++y2_ptr;
        }
    }

 protected:
    float sample_rate_;
    float b1_;
    float y_[n_channels];
};



int main() {
    stdio_init_all();
    printf("MEML FM Synth Interface\n");

    //serial cx
    const unsigned int kGPIO_UART_TX = 0;
    const unsigned int kGPIO_UART_RX = 1;
    gpio_set_function(kGPIO_UART_TX, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(kGPIO_UART_RX, UART_FUNCSEL_NUM(uart0, 1));

    //serialSLIP serial;
    auto serial = std::make_unique<MEMLSerial>();


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
    constexpr float kSmoothing_time_ms = 200;
    auto smoother = std::make_unique< OnePoleSmoother<kN_adc> >(kSmoothing_time_ms, kSample_rate);

    while (1) {
        for(auto& i: {0,1,2}) {
            adc_select_input(i);
            //give time for ADC to settle
            sleep_us(kInterval_us);
            adcValue[i] = static_cast<float>(adc_read());
        }
        smoother->Process(adcValue, adcValue_smoothed);
        for (auto& i: {0,1,2}) {
            if (adcChangeTrigs[i].onChanged(adcValue[i], 50)) {
                int16_t smoothed_adc_value = static_cast<int16_t>(adcValue_smoothed[i]);
                serial->sendMessage(MEMLSerial::joystick, i, smoothed_adc_value << 4);
            }
        }
        size_t idx=0;
        for(auto& i: buttonPins) {
            bool buttonValue = gpio_get(i);
            if (buttonValue != buttonValues[idx]) {
                buttonValues[idx] = buttonValue;
                //printf("button: %d: %d\n", i, buttonValue);
                //serial.sendMessage(static_cast<serialSLIP::messageTypes>(serialSLIP::messageTypes::TRAINMODE+idx), buttonValue);
                // TODO AM Button indexes should be reversed properly
                serial->sendMessage(MEMLSerial::button, 2-idx, buttonValue);
            }
            idx++;
        }

        sleep_ms(10);
    }
}

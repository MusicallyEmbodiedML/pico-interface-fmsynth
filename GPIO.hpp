#ifndef __GPIO_HPP__
#define __GPIO_HPP__


extern "C" {
#include "hardware/gpio.h"
}

enum kGPIO_LEDs {
    GPIO_midi_LED = 21,
    GPIO_training_LED = 2,
};


#endif  // __GPIO_HPP__

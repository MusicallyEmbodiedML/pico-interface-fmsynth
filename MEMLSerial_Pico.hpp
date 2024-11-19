#ifndef __MEML_SERIAL_PICO_H__
#define __MEML_SERIAL_PICO_H__

extern "C" {
#include "hardware/uart.h"
#include "hardware/irq.h"
}
#include <cstring>
#include <string>
#include <memory>
#include <sstream>
#include <iostream>
#include <array>


class MEMLSerial {

 public:

    enum msgType
    {
        joystick='j',
        button='b',
        pulse_period='p',
        slider='s',
    };

    const char delim = '\n';

    MEMLSerial(uart_inst_t *uart_hw = uart0);

    void sendMessage(msgType type, uint8_t index, std::string &value);
    void sendMessage(msgType type, uint8_t index, uint64_t value);
    void sendFloatMessage(msgType type, uint8_t index, float value);

 private:

    static constexpr unsigned int kDatagram_buffer_length = 128;
    uart_inst_t *uart_hw_;
    std::array<char, kDatagram_buffer_length> datagram_buffer_;
    bool uart_is_init_;
};

#endif  // __MEML_SERIAL_PICO_H__
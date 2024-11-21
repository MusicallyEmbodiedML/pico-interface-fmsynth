#include "MEMLSerial_Pico.hpp"
#include "UART_Common.hpp"
#include "common_defs.h"
#include "pico/stdlib.h"

#include <iostream>


ts_app_state GAppState;// Global serial singleton
extern std::shared_ptr<MEMLSerial> serial;


static void uart0_irq_routine(void) {

    while(uart_is_readable(uart0)) {
        char ch = uart_getc(uart0);
        serial->StoreMessage(ch);
    }
}


MEMLSerial::MEMLSerial(uart_inst_t *uart_hw) :
    uart_hw_(uart_hw),
    datagram_buffer_({ '\0' }) {
    unsigned int requested_baudrate = 115200;

    // Init UART internally
    if(uart_init(uart_hw_, requested_baudrate) != requested_baudrate) {
        printf("ERROR - Uart not initialised!\n");
        uart_is_init_ = false;
    }
    uart_set_format(uart_hw_, 8, 2, UART_PARITY_NONE);

    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_routine);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);

    uart_is_init_ = true;
}


void MEMLSerial::StoreMessage(char c)
{
    if (c == '\n') {
        // Process buffer
        _ProcessMessage(rx_buffer_);
        // ...and clear
        rx_buffer_ = std::string();
    } else if (c) {
        rx_buffer_.push_back(c);
        //printf("%c", c);
    }
}


void MEMLSerial::_ProcessMessage(const std::string &msg)
{
    // printf("UART RX- size %d, content: ", msg.size());
    // for (auto &c: msg) {
    //     printf("%c", c);
    // }

    std::vector<std::string> tokens = UART_Common::SplitMessage(msg);
    char msgtype = tokens[0][0];

    std::vector<std::string> msg_payload(tokens.begin()+1, tokens.end());

    switch(msgtype) {

        case UART_Common::state_dump: {
            if (UART_Common::ExtractAppState(msg_payload, GAppState)) {
                printf("UART- App state received.\n");
            } else {
                printf("UART- App state corrupted!.\n");
                // TODO Query for another app state
            }
        } break;

        default: {
            std::cout << "UART- Message type '" << msgtype << "' not implemented." << std::endl;
        }
    }
}


void MEMLSerial::sendMessage(UART_Common::msgType type, uint8_t index, std::string &value) {
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
void MEMLSerial::sendMessage(UART_Common::msgType type, uint8_t index, uint64_t value) {
    std::string value_str;
    value_str.reserve(8);
    std::sprintf(value_str.data(), "%d", value);
    //printf("Value:%d, string: %s\n", value, value_str.c_str());
    sendMessage(type, index, value_str);
}

void MEMLSerial::sendFloatMessage(UART_Common::msgType type, uint8_t index, float value)
{
    std::string value_str;
    value_str.reserve(32);
    std::sprintf(value_str.data(), "%.9g", value);
    //printf("Value:%f, string: %s\n", value, value_str.c_str());
    sendMessage(type, index, value_str);
}

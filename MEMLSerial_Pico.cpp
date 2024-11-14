#include "MEMLSerial_Pico.hpp"


static void uart0_irq_routine(void) {

    std::stringstream message;

    while(uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        message << ch;
    }

    std::cout << "UART IRQ: message - ";
    std::cout << message.str();
    std::cout << std::endl;
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

void MEMLSerial::sendMessage(msgType type, uint8_t index, std::string &value) {
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
void MEMLSerial::sendMessage(msgType type, uint8_t index, uint64_t value) {
    std::string value_str;
    value_str.reserve(8);
    std::sprintf(value_str.data(), "%d", value);
    //printf("Value:%d, string: %s\n", value, value_str.c_str());
    sendMessage(type, index, value_str);
}

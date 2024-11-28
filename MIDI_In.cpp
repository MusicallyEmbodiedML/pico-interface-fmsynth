#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <cstdio>

#include "MEMLSerial_Pico.hpp"
#include "UART_Common.hpp"
#include "GPIO.hpp"

extern std::shared_ptr<MEMLSerial> serial;


// UART1 and GPIO pin configuration
#define UART_ID uart1
#define BAUD_RATE 31250 // Standard MIDI baud rate
#define UART_TX_PIN 4  // GPIO pin for TX (not used for MIDI IN)
#define UART_RX_PIN 5  // GPIO pin for RX (MIDI IN)

#define NOTE_ON   0x90
#define NOTE_OFF  0x80

// Forward declaration
static void parse_midi_message(uint8_t byte);
static void midi_callback(void);
static void process_midi_message(uint8_t status, uint8_t pitch, uint8_t velocity);


void midi_init()
{
    // Initialize UART1 for MIDI input
    uart_init(UART_ID, BAUD_RATE);
    // Configure UART RX pin (MIDI input)
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    // Optional: Disable TX if not needed
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // Enable callback
    irq_set_exclusive_handler(UART1_IRQ, midi_callback);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    std::printf("MIDI- callback set up.\n");
}

void midi_callback()
{
    if (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID); // Read the MIDI byte
        parse_midi_message(byte);
    }
}

// Function to parse incoming MIDI bytes
void parse_midi_message(uint8_t byte) {
    static uint8_t midi_message[3] = {0}; // Buffer for the current MIDI message
    static int message_length = 0;       // Expected number of bytes in the message
    static int received_bytes = 0;       // Number of bytes received so far

    //std::printf("0x%02x ", byte);

    // Determine if the byte is a status byte (0x80 to 0xFF)
    if (byte & 0x80) {
        // Status byte received, reset the message
        midi_message[0] = byte;
        received_bytes = 1;

        // Determine the expected message length based on the status byte
        uint8_t status = byte & 0xF0; // Ignore channel bits (lower nibble)
        if (status == 0xC0 || status == 0xD0) {
            // Program Change or Channel Pressure (2 bytes total)
            message_length = 2;
        } else {
            // Most other messages (3 bytes total)
            message_length = 3;
        }
    } else {
        // Data byte received, append it to the message buffer
        if (received_bytes > 0 && received_bytes < 3) {
            midi_message[received_bytes++] = byte;
        }
    }

    // Check if the complete message has been received
    if (received_bytes == message_length) {
        // Parse the message
        uint8_t status = midi_message[0] & 0xF0; // Status type
        uint8_t channel = midi_message[0] & 0x0F; // Channel number
        uint8_t data1 = midi_message[1];         // First data byte
        uint8_t data2 = (message_length == 3) ? midi_message[2] : 0; // Second data byte (if applicable)

        // Print the parsed message
        //std::printf("MIDI Message: Status=0x%X, Channel=%d, Data1=%d, Data2=%d\n",
        //       status, channel, data1, data2);
        //std::printf("\n");
        process_midi_message(status, data1, data2);

        // Reset for the next message
        received_bytes = 0;
    }
}

void process_midi_message(uint8_t status, uint8_t pitch, uint8_t velocity)
{
    switch(status) {
        case NOTE_ON: {
            serial->sendMessage(UART_Common::midi_note, pitch, velocity);
            gpio_put(GPIO_midi_LED, true);
        } break;

        case NOTE_OFF: {
            serial->sendMessage(UART_Common::midi_note, pitch, 0);
            gpio_put(GPIO_midi_LED, false);
        } break;

        default: {}
    }
}
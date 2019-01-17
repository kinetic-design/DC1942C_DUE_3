//! @todo Review this file.
/*
UserInterface.h

UserInterface routines are used to read and write user data through the Arduino's
serial interface.

Copyright 2018(c) Analog Devices, Inc.

*/

#ifndef USERINTERFACE_H
#define USERINTERFACE_H

#include <stdint.h>

#define UI_BUFFER_SIZE 64
#define SERIAL_TERMINATOR '\n'

// io buffer
extern char ui_buffer[UI_BUFFER_SIZE];

// Read data from the serial interface into the ui_buffer buffer
uint8_t read_data();

// Read a float value from the serial interface
float read_float();

// Read an integer from the serial interface.
// The routine can recognize Hex, Decimal, Octal, or Binary
// Example:
// Hex:     0x11 (0x prefix)
// Decimal: 17
// Octal:   O21 (leading letter O prefix)
// Binary:  B10001 (leading letter B prefix)
int32_t read_int();

// Read a string from the serial interface.  Returns a pointer to the ui_buffer.
char *read_string();

// Read a character from the serial interface
int8_t read_char();

void print_menu();


#endif  // USERINTERFACE_H

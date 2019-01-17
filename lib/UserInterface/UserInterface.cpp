//! @todo Review this file.
/*
UserInterface.cpp

UserInterface routines are used to read and write user data through the Arduino's
serial interface.

*/


#include <Arduino.h>
#include <stdint.h>
#include "UserInterface.h"
#include "Linduino.h"

char ui_buffer[UI_BUFFER_SIZE];

// Read data from the serial interface into the ui_buffer
uint8_t read_data()
{
  uint8_t index = 0; //index to hold current location in ui_buffer
  int c; // single character used to store incoming keystrokes
  while (index < UI_BUFFER_SIZE-1)
  {
    c = Serial.read(); //read one character
    if (((char) c == '\r') || ((char) c == '\n')) break; // if carriage return or linefeed, stop and return data
    if ( ((char) c == '\x7F') || ((char) c == '\x08') )  // remove previous character (decrement index) if Backspace/Delete key pressed      index--;
    {
      if (index > 0) index--;
    }
    else if (c >= 0)
    {
      ui_buffer[index++]=(char) c; // put character into ui_buffer
    }
  }
  ui_buffer[index]='\0';  // terminate string with NULL

  if ((char) c == '\r')    // if the last character was a carriage return, also clear linefeed if it is next character
  {
    delay(10);  // allow 10ms for linefeed to appear on serial pins
    if (Serial.peek() == '\n') Serial.read(); // if linefeed appears, read it and throw it away
  }

  return index; // return number of characters, not including null terminator
}

// Read a float value from the serial interface
float read_float()
{
  float data;
  read_data();
  data = atof(ui_buffer);
  return(data);
}

// Read an integer from the serial interface.
// The routine can recognize Hex, Decimal, Octal, or Binary
// Example:
// Hex:     0x11 (0x prefix)
// Decimal: 17
// Octal:   021 (leading zero prefix)
// Binary:  B10001 (leading B prefix)
int32_t read_int()
{
  int32_t data;

  read_data();
  if (ui_buffer[0] == 'm')
    return('m');
  if ((ui_buffer[0] == 'B') || (ui_buffer[0] == 'b'))
  {
    data = strtol(ui_buffer+1, NULL, 2);
  }
  else
    data = strtol(ui_buffer, NULL, 0);
  return(data);
}

// Read a string from the serial interface.  Returns a pointer to the ui_buffer.
char *read_string()
{
  read_data();
  return(ui_buffer);
}

// Read a character from the serial interface
int8_t read_char()
{
  read_data();
  return(ui_buffer[0]);
}

/*!*********************************
  \brief Prints the main menu
***********************************/
void print_menu()
{
    Serial.println("+ command+function ------------------+");
    Serial.println("|     0    | Display Menu            |");
    Serial.println("|     1    | Balancing Off: default  |");
    Serial.println("|     2    | Balancing  On           |");
    Serial.println("|     3    | Cell, Battery Voltages  |");
    Serial.println("|     4    | Analog Readings         |");
    Serial.println("|     5    | 6804-2 Config Status    |");
    Serial.println("|     6    | Cell Voltage Limits     |");
    Serial.println("+----------+-------------------------+");
    Serial.println("| Please enter command: ");
    Serial.println();
}

// M115 Capabilities String

#include "MarlinFirmware.h"
#include "host_interface/host_io.h"
#include "fastio.h"

const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M45

void gcode_M45() {
  millis_t stop_ms = millis() + (8 * 1000UL);
  while ((unsigned int)(millis()) < stop_ms) {
	millis_t last_ms = millis();
	void idle();
    while (((unsigned int)(millis() - last_ms)) < (500UL)) idle();
    for (uint8_t pin_number = 80; pin_number < NUM_DIGITAL_PINS; pin_number++) {
      uint8_t v;
      // Arduino pin numbering
      pinMode(pin_number, INPUT);
      digitalRead(pin_number);
	  v = digitalRead(pin_number);
      if (v)  SERIAL_PROTOCOL('1');
      else  SERIAL_PROTOCOL('0');
      if (pin_number % 10 == 9)
        SERIAL_PROTOCOL(' '); //Adds space after 10 characters
    }
    SERIAL_PROTOCOL('\n');
  }
  SERIAL_PROTOCOLPGM("Button Test Ended"); //Double quotes for a text string
  SERIAL_EOL;
}

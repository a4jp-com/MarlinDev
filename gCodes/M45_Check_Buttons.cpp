// M115 Capabilities String

#include "MarlinFirmware.h"
#include "host_interface/host_io.h"
#include "fastio.h"

const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M45

void gcode_M45() {
  millis_t start_ms = millis();
  while (((unsigned int)(millis() - start_ms)) < (10 * 1000UL)) {
    for (uint8_t pin_number = 50; pin_number < NUM_DIGITAL_PINS; pin_number++) {
      uint8_t v;
      // Arduino pin numbering
      pinMode(pin_number, INPUT);
      v = digitalRead(pin_number);
      if (v)  SERIAL_PROTOCOL('1');
      else  SERIAL_PROTOCOL('0');
      if (pin_number % 10 == 0)
        SERIAL_PROTOCOL(' ');
      if ((pin_number-50) % 30 == 0)
        SERIAL_PROTOCOL('\n');
    }
    SERIAL_PROTOCOL('\n');
  }
}

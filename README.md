SDI-12 environmental sensor and data logger for Arduino Due. Utilises non-blocking sensor sampling via a hardware timer ISR while handling SDI-12 command strings. Smooths data from a BME680 and BH1750 via ring buffers over I2C, logging time-stamped CSV data via SPI to an SD card while displaying real-time metrics on a TFT display.

Supported SDI-12 commands:

?! - address query,
aAb! - change address where 'a' is the current address and 'b' is the new address,
aM! - start measurement,
aD1! - send data (exculding lux),
aD2! - send data (only lux),
aR0! - continuous live measurement

Buttons:

Button 1 - used to log a reading to the sd card,
Button 2 - clear sd card memory

Sample output:

Serial output (aR0!) - 0+23.45+55.10+1013.25+48.72+310.50,
SD card log entry (Button 1) - 2026-07-12 19:06:07, 23.44, 54.98, 1013.21, 48.55, 298.75

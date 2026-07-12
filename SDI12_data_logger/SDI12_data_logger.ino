// arduino and sensors
#include <Arduino.h>          // core arduino framework
#include <Wire.h>             // I2C
#include <Adafruit_Sensor.h>  // adafruit unified sensor library
#include <Adafruit_BME680.h>  // BME680 sensor
#include <BH1750.h>           // BH1750
// lcd 
#include <Adafruit_GFX.h>    // core graphics library
#include <Adafruit_ST7735.h> // hardware-specific library for ST7735
// sd card
#include <RTClib.h>   // RTC
#include <SdFat.h>

// define values to initialise display
// need J13 connected
#define TFT_CS    10
#define TFT_RST   6 
#define TFT_DC    7 
#define TFT_SCLK 13   
#define TFT_MOSI 11 

// initialise display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// how often to redraw the dashboard, starting value
unsigned long drawTarget = 0;     

// last displayed values, to optimize display redraw
String prevTemp = "";
String prevHumidity = "";
String prevPressure = "";
String prevGas = "";
String prevLux = "";

// sd card                    
SdFs sd;
FsFile dataFile;                // file to read/write to
//^ configuration for FAT16/FAT32 and exFAT.

// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = A3;
//
// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 12;
const uint8_t SOFT_MOSI_PIN = 11;
const uint8_t SOFT_SCK_PIN  = 13;

// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)                      // file to read/write to

// init rtc 
RTC_DS1307 rtc;                     

unsigned long lastLogTime = 0;
const unsigned long logInterval = 60000;

// button pins
const int LOG_BUTTON = 2;
const int CLEAR_BUTTON = 3;

// button states
bool lastLogState = HIGH;
bool lastClearState = HIGH;

// button debounce variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

//  sensor init
Adafruit_BME680 bme;                // BME680: temperature, humidity, pressure
BH1750 lightMeter(0x23);            // BH1750: light level (I2C address 0x23)

// SDI-12 settings
// pin 8 controls receive/transmit direction (rewired from pin 7)
// HIGH = listening mode (waiting for commands)
// LOW = talking mode (sending a reply)

#define DIRO 8

#define LED_PIN LED_BUILTIN  // onboard LED (pin 13 = PA17) used as ISR visual indicator

String command;                      // stores incoming command one character at a time
int deviceAddress = 0;               // this sensor's SDI-12 address (0-9)

// measurement buffer — filled by aM!, returned by aD1! and aD2!
float measTemp = 0.0;                // temperature (°C)
float measHumidity = 0.0;            // humidity (%)
float measPressure = 0.0;            // pressure (hPa)
float measGas = 0.0;                 // gas (kOhms)
float measLux = -2.0;                // light level (lux), start at -2 so that 0D2! returns the error value instead of 0

// buffers used to average readings during aM!, fills measurement buffers
float luxBuffer[100];
float tmpBuffer[10];
float humBuffer[10];
float prsBuffer[10];
float gasBuffer[10];

int luxIndex = 0; // index of next write slot in the lux ring buffer
int luxCount = 0; // number of valid samples currently in the lux buffer
int bmeIndex = 0;
int bmeCount = 0;

// Sensor status flags
bool luxPresent = true;
bool bmePresent = true;

// isr flags — set inside tc3_handler(), cleared and acted on inside loop()
// i2c cannot be called inside an isr because i2c itself uses interrupts,
// and you cannot run an interrupt inside another interrupt — so flags are used instead
volatile bool luxFlag = false;
volatile bool bmeStartFlag = false;
volatile int isrTick = 0;

// bme680 non-blocking read state (non-blocking means it can gather sensor data whilst other code is running)
bool bmeReading = false; // true while a bme680 forced-mode measurement is in progress
unsigned long bmeReadyMs = 0; // millis() value at which the measurement result will be ready

// configures and starts TC0 for a precise 10ms interrupt period using MCK/128 (84MHz / 128 = 656,250Hz)
void setupTimer() {

  // enable the peripheral clock for TC0
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);

  // waveform mode, count up, reset on RC compare, clock = MCK/128 (656,250Hz)
  TC_Configure(TC0, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);

  // counter resets every 6563 ticks — 6563 / 656,250Hz = exactly 10ms per interrupt
  TC_SetRC(TC0, 0, 6563);

  // enable RC compare interrupt (fires when counter reaches RC)
  TC0->TC_CHANNEL[0].TC_IER =  TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;

  NVIC_EnableIRQ(TC0_IRQn);  // register TC0 with the cpu interrupt controller

  TC_Start(TC0, 0);  // counter begins running immediately after this line
}

void setup() {

  // serial monitor (USB debug output to PC)
  Serial.begin(9600);

  Wire.begin();                      // start the I2C bus — on Due: SDA=20, SCL=21 (use J9 jumper wires on lab board)

  // BME680 (tries to start at address 0x76 => 118)
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    bmePresent = false;
  }

  // BME680_OS_#X samples the last '#' readings and calculates averages
  bme.setTemperatureOversampling(BME680_OS_8X);   // average 8 temperature readings
  bme.setPressureOversampling(BME680_OS_8X);      // average 8 pressure readings
  bme.setHumidityOversampling(BME680_OS_2X);      // average 2 humidity readings

  // configure gas sensor
  bme.setGasHeater(320, 150);        // heat the plate to 320°C for 150ms

  // BH1750
  if (!lightMeter.begin()) {          // start the light sensor
    Serial.println("Could not find a valid BH1750 sensor, check wiring!");
    luxPresent = false;
  }

  // SDI-12 UART — on Due, Serial1 is on pins 18(TX)/19(RX)
  Serial1.begin(1200, SERIAL_7E1);   // 1200 baud, 7 data bits, even parity, 1 stop bit
  pinMode(DIRO, OUTPUT);             // pin 7 is an output (controls direction chip)
  digitalWrite(DIRO, HIGH);          // HIGH = receive mode (listening)

  //initialise display
  tft.initR(INITR_BLACKTAB); 
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setRotation(1);

  // rtc
  if (!rtc.begin()) {
    Serial.println("RTC not found");
  } else {
    if (!rtc.isrunning()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // sd card
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("SD card failed");
  } else {
    Serial.println("SD card initialized");
  }

  // create and write first line of file if it doesn't already exist
  if (!sd.exists("data.csv")) {
    dataFile = sd.open("data.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Timestamp, Temperature, Humidity, Pressure, Gas, Lux");
      dataFile.close();
    }
  }

  // configure button pins as inputs
  pinMode(LOG_BUTTON, INPUT_PULLUP);
  pinMode(CLEAR_BUTTON, INPUT_PULLUP);

  // configure the LED pin as an output before the timer starts
  pinMode(LED_PIN, OUTPUT);

  // start the hardware timer
  setupTimer();
}


void loop() {

  // bh1750 (light sensor) sampling — every 10ms, triggered by luxFlag set in tc3 isr
  if (luxFlag) {

    luxFlag = false; // clear flag

    if (luxPresent) {                       // checks if bh1750 is active
      float v = lightMeter.readLightLevel(); // reads light level
      if (v >= 0) {

        luxBuffer[luxIndex] = v;            // write the sample at the current buffer index position
        luxIndex = (luxIndex + 1) % 100;    // advance to next index position

        if (luxCount < 100) {
          luxCount++;
        }
      }
    }
  }

  // bme680 measurement start — every 500ms, triggered by bmeStartFlag set in tc3 isr
  // beginReading() is non-blocking which means it starts the sensor measurement and returns immediately
  if (bmeStartFlag && !bmeReading) {

    bmeStartFlag = false; // clear flag

    if (bmePresent) {         // checks if bme680 is active
      // start measurement, beginReading() returns ms until result is ready
      unsigned long dur = bme.beginReading();
      bmeReading = true;
      bmeReadyMs = millis() + dur + 10; // record when the result will be ready, with 10ms margin
    }
  }

  // bme680 result collection — once the measurement time has finished
  if (bmeReading && millis() >= bmeReadyMs) {
    if (bmePresent && bme.endReading()) {

      tmpBuffer[bmeIndex] = bme.temperature;
      humBuffer[bmeIndex] = bme.humidity;
      prsBuffer[bmeIndex] = bme.pressure / 100.0f;
      gasBuffer[bmeIndex] = bme.gas_resistance / 1000.0f;

      bmeIndex = (bmeIndex + 1) % 10; // advance the index position

      if (bmeCount < 10) {
        bmeCount++;
      }

    }
    bmeReading = false; // no longer measuring/reading bme
  }

  int byteIn;

  // checks for input on SDI-12 line, reads byte by byte
  if (Serial1.available()) {

    byteIn = Serial1.read();

    if (byteIn == 33) { // 33 = ASCII '!' = end of command, full command is ready to process
      SDI12Receive(command);
      command = "";

    } else if (byteIn != 0) { // if byte is not null (0), add to string
      command += char(byteIn);
    }
  }

  // write to sd card every 60 seconds
  if (millis() - lastLogTime >= logInterval) {
    logDataToSD();
    lastLogTime = millis();
  }

  // log to and clear sd card using buttons
  bool logButtonState = digitalRead(LOG_BUTTON);
  bool clearButtonState = digitalRead(CLEAR_BUTTON);

  // if its been 200ms since last button press
  if ((millis() - lastDebounceTime) > debounceDelay) {

    // if button 1 is pressed
    if (lastLogState == HIGH && logButtonState == LOW) {
      Serial.println("Manual log triggered.");
      logDataToSD();
      lastDebounceTime = millis();
    }

    // clear sd card if button 2 is pressed 
    if (lastClearState == HIGH && clearButtonState == LOW) {
      Serial.println("Clearing SD card log file...");
      sd.remove("data.csv");

      // Re-initialise file with header
      dataFile = sd.open("data.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println("Timestamp, Temperature, Humidity, Pressure, Gas, Lux");
        dataFile.close();
      }
      Serial.println("SD card memory cleared.");
      lastDebounceTime = millis();
    }
  }
  //update state trackers
  lastLogState = logButtonState;
  lastClearState = clearButtonState;

  // real-time display
  if (millis() >= drawTarget) {     // if enough time has passed since last redraw

    // overwrite previous values in black to erase without clearing the whole screen
    tft.setTextColor(ST77XX_BLACK);
    LCDPrint("Temperature: " + prevTemp + " C", 20);
    LCDPrint("Humidity: " + prevHumidity + "%", 40);
    LCDPrint("Pressure: " + prevPressure + " hPa", 60);
    LCDPrint("Gas: " + prevGas + " kOhms", 80);
    LCDPrint("Light level: " + prevLux + " lux", 100);

    // get averaged sensor values from ring buffers
    String displayTemp = String(bufferAverage(tmpBuffer, bmeCount));
    String displayHumidity = String(bufferAverage(humBuffer, bmeCount));
    String displayPres = String(bufferAverage(prsBuffer, bmeCount));
    String displayGas = String(bufferAverage(gasBuffer, bmeCount));
    String displayLight = String(bufferAverage(luxBuffer, luxCount));

    // print live values
    tft.setTextColor(ST77XX_WHITE);
    LCDPrint("Temperature: " + displayTemp + " C", 20); 
    LCDPrint("Humidity: " + displayHumidity + "%", 40);
    LCDPrint("Pressure: " + displayPres + " hPa", 60);
    LCDPrint("Gas: " + displayGas + " kOhms", 80);
    LCDPrint("Light level: " + displayLight + " lux", 100);
    
    //store for next loop redraw
    prevTemp = displayTemp;
    prevHumidity = displayHumidity;
    prevPressure = displayPres;
    prevGas = displayGas;
    prevLux = displayLight;
    
    //increment time until next redraw, millis() ensures next draw is always 5 seconds away
    drawTarget = millis() + 5000;
  }
}


//  SDI12Receive  (processes a completed command)
void SDI12Receive(String cmd) {

  String addr = String(deviceAddress);  // convert address int to string for comparison

  // ?! = address query — returns our address to the recorder
  if (cmd == "?") {
    SDI12Send(addr);
  }

  // aAb! = change address — updates sensor address to b and confirms
  else if (cmd.charAt(0) == addr.charAt(0) && cmd.charAt(1) == 'A') {
    char newAddr = cmd.charAt(2);      // new address to change to
    deviceAddress = newAddr - '0';     // update stored address
    SDI12Send(String(newAddr));
  }

  // aM! = start measurement — reads sensors into buffer, returns time and value count
  else if (cmd == addr + "M") {
    measTemp = bufferAverage(tmpBuffer, bmeCount);
    measHumidity = bufferAverage(humBuffer, bmeCount);
    measPressure = bufferAverage(prsBuffer, bmeCount); 
    measGas = bufferAverage(gasBuffer, bmeCount);
    if (luxCount > 0) {           // dont override default lux value if there is no data
    measLux = bufferAverage(luxBuffer, luxCount);
}

    // print number of working sensors
    // check if buffers contain data
    int count = 0;
    if (bmePresent && bmeCount > 0) count += 4;
    if (luxPresent && luxCount > 0) count += 1;

    SDI12Send(addr + "000" + count);      // ready in 0 seconds, n values available 
  }

  // aD1! = send data (exculding lux) — returns buffered values from last aM!
  else if (cmd == addr + "D1") {
    SDI12Send(addr 
      + "+" + String(measTemp, 2)         // temperature to 2 decimal places
      + "+" + String(measHumidity, 2)     // humidity to 2 decimal places
      + "+" + String(measPressure, 2)     // pressure to 2 decimal places 
      + "+" + String(measGas, 2));        // gas to 2 decimal places
  }

  // aD2! = send data (only lux) — returns buffered values from last aM!            
  else if (cmd == addr + "D2") {
    SDI12Send(addr + "+" + String(measLux, 2));      // lux to 2 decimal places
  }

  // aR0! = continuous measurement — reads sensors live and returns values immediately
  // performReading() is blocking: the cpu sits and waits ~200ms doing nothing until
  // the sensor finishes — acceptable here because the recorder expects the delay
  else if (cmd == addr + "R0") {
    bme.performReading();
    SDI12Send(addr
      + "+" + String(bme.temperature, 2)
      + "+" + String(bme.humidity, 2)
      + "+" + String((float)bme.pressure / 100, 2)              // /100 to convert to hPa                 
      + "+" + String((float)bme.gas_resistance / 1000, 2)       // /1000 to convert to kOhms    
      + "+" + String((float)lightMeter.readLightLevel(), 2));   // must be float otherwise "2" converts it to binary
  }

}

//  SDI12Send  (sends a response back over SDI-12)
void SDI12Send(String message) {

  Serial.print("message: ");
  Serial.println(message);            // print to serial monitor for debugging

  digitalWrite(DIRO, LOW);            // switch to transmit mode
  delay(100);                         // wait for line to settle before sending

  Serial1.print(message + String("\r\n"));  // send message
  Serial1.flush();                    // wait until every byte has been physically sent

  Serial1.end();                      // close serial port (clears leftover bytes)
  Serial1.begin(1200, SERIAL_7E1);    // restart fresh

  digitalWrite(DIRO, HIGH);           // switch back to receive mode
}

//LCD print function
void LCDPrint(String text, int y_val) {
  tft.setCursor(10, y_val);
  tft.println(text);
}

// logs date, time and sensor readings to the file
void logDataToSD() {
  measTemp = bufferAverage(tmpBuffer, bmeCount);
  measHumidity = bufferAverage(humBuffer, bmeCount);
  measPressure = bufferAverage(prsBuffer, bmeCount); 
  measGas = bufferAverage(gasBuffer, bmeCount);
  measLux = bufferAverage(luxBuffer, luxCount);

  DateTime now = rtc.now();

  dataFile = sd.open("data.csv", FILE_WRITE);

  if (dataFile) {
    // date
    dataFile.print(now.year());
    dataFile.print("-");
    if (now.month() < 10) dataFile.print("0");
    dataFile.print(now.month());
    dataFile.print("-");
    if (now.day() < 10) dataFile.print("0");
    dataFile.print(now.day());
    dataFile.print(" ");

    // time
    if (now.hour() < 10) dataFile.print("0");
    dataFile.print(now.hour());
    dataFile.print(":");
    if (now.minute() < 10) dataFile.print("0");
    dataFile.print(now.minute());
    dataFile.print(":");
    if (now.second() < 10) dataFile.print("0");
    dataFile.print(now.second());

    // sensor data
    dataFile.print(", ");
    dataFile.print(measTemp, 2);

    dataFile.print(", ");
    dataFile.print(measHumidity, 2);

    dataFile.print(", ");
    dataFile.print(measPressure, 2);

    dataFile.print(", ");
    dataFile.print(measGas, 2);

    dataFile.print(", ");
    dataFile.println(measLux, 2);

    dataFile.close();

    Serial.println("Data logged");
  } else {
    Serial.println("Error opening data.csv");
  }
}

// TC0 interrupt service routine — fires exactly every 10ms when counter matches RC
// TC0_Handler is a CPU reserved function name that is automatically called upon RC match
// meaning the function is run despite not being explicitly called in the code
void TC0_Handler() {

  // read SR to acknowledge and clear the interrupt flag
  TC_GetStatus(TC0, 0);

  luxFlag = true;  // enable lux flag, tells loop() to collect lux reading

  // collect new bme680 reading every 500ms (10ms reset * 50 = 500ms)
  if (++isrTick >= 50) {
    isrTick      = 0;
    bmeStartFlag = true;

    // toggle the onboard LED every 500ms as a visual indicator that the ISR is running
    // use PIO SODR/CODR instead of digitalWrite() — digitalWrite() is not ISR-safe
    // LED_BUILTIN on Due is PB27 (pin 13)
    if (PIOB->PIO_ODSR & PIO_PB27) {
      PIOB->PIO_CODR = PIO_PB27;            // LED on  → turn off
    } else {
      PIOB->PIO_SODR = PIO_PB27;     // LED off → turn on
    }
  }
}


float bufferAverage(float* buf, int count) {

  if (count == 0) {
    return 0.0f;
  }

  // calculates average by summing buffer values then dividing by count
  float s = 0;
  for (int i = 0; i < count; i++) {s += buf[i];}
  return s / count;
}

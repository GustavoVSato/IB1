#include <Arduino.h>

#include <SPI.h>
#include "esp32-hal-timer.h"

#include "esp_dsp.h"

#define PIN_CS 5      // CS do ADS8326
#define PIN_SCLK 18   // SCLK
#define PIN_MISO 19   // DOUT

#define MUX_IN_PIN 25 // controle do MUX


#define DELTA
#define THETA
#define ALPHA
#define BETA
#define BETA_HIGH 
#define SMR

SPISettings adsSettings(6000000, MSBFIRST, SPI_MODE0);

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool muxState = false;


void setup() {
  
}

void loop() {
  
}

// put function definitions here:
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  muxState = !muxState;
  digitalWrite(MUX_IN_PIN, muxState);

  portEXIT_CRITICAL_ISR(&timerMux);
}


// Inicialização do ADC
void ads_init() {

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);   // ADC inativo

  SPI.begin(PIN_SCLK, PIN_MISO, -1, PIN_CS);
}


// Inicialização do MUX (125 kHz)
void mux_init() {

  pinMode(MUX_IN_PIN, OUTPUT);
  digitalWrite(MUX_IN_PIN, LOW);

 
  timer = timerBegin(0, 80, true);

  // Interrupção a cada 4 µs
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 4, true);
  timerAlarmEnable(timer);
}


// Leitura do ADS8326
uint16_t ads_read() {

  uint8_t hi, lo;

  SPI.beginTransaction(adsSettings);
  digitalWrite(PIN_CS, LOW);

  hi = SPI.transfer(0x00);
  lo = SPI.transfer(0x00);

  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  return (uint16_t)(hi << 8) | lo;
}
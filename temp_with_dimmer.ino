#pragma SPARK_NO_PREPROCESSOR // disable spark preprocssor to avoid issues with callbacks
#include "application.h"

// debugging options
#define CLOUD_DEBUG_ON
#define WEBHOOKS_DEBUG_ON
//#define STATE_DEBUG_ON
#define DATA_DEBUG_ON
#define SERIAL_DEBUG_ON
//#define LCD_DEBUG_ON

// keep track of installed version
#define STATE_VERSION    1 // update whenver structure changes
#define DEVICE_VERSION  "T 0.1.0" // update with every code update

// scale controller
#include "device/SerialDeviceState.h"
#include "TempController.h"

// lcd
DeviceDisplay* lcd = &LCD_16x2;

// initial state of the scale
SerialDeviceState* state = new SerialDeviceState(
  /* locked */                    false,
  /* state_logging */             false,
  /* data_logging */              false,
  /* data_reading_period_min */   500, // in ms
  /* data_reading_period */       1000, // in ms
  /* data_logging_period */       1800, // in seconds
  /* data_logging_type */         LOG_BY_TIME
);

// scale controller
TempController* tc = new TempController(
  /* one wire pin */      D4,
  /* reset pin */         A5,
  /* lcd screen */        lcd,
  /* pointer to state */  state
);

// using system threading to speed up restart after power out
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// setup
void setup() {

  // serial
  Serial.begin(9600);

  // lcd temporary messages
  lcd->setTempTextShowTime(3); // how many seconds temp time

  // mfc
  tc->init();

  // dimmer
  const double LIGHT_LEVEL = 84.72; // light level in percent (100% is 3.3V on the digital to analogue converter pin DAC)
  pinMode(DAC, OUTPUT);
  analogWrite(DAC, (int) LIGHT_LEVEL/100.00 * 4095);

  // connect device to cloud
  Serial.println("INFO: connecting to cloud");
  Particle.connect();
}

// loop
void loop() {
  tc->update();
}

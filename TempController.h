#pragma once
#include "device/SerialDeviceController.h"
#include <OneWire.h>
// included directly because it does not look for OneWire at the right place
#include <spark-dallas-temperature.h>

// FIXME: implement an intermediate class between DeviceController and SerialDeviceController that has time based data querying (e.g. AnalogDeviceController)

// controller class
class TempController : public SerialDeviceController {

  private:

    // temp sensor
    const int one_wire_pin;
    OneWire oneWire = 0;//(ONE_WIRE_BUS);  // oneWire instance to communicate with any OneWire devices
    DallasTemperature sensors = 0;//(&oneWire);  // oneWire temperature sensor
    DeviceAddress sensor; // probe address

    // state
    SerialDeviceState* state;
    DeviceState* ds = state;

  public:

    // constructors
    TempController();

    // without LCD
    TempController (int one_wire_pin, int reset_pin, SerialDeviceState* state) :
      SerialDeviceController(reset_pin, 0, 0, "", 500), one_wire_pin(one_wire_pin), state(state) { construct(); }

    // with LCD
    TempController (int one_wire_pin, int reset_pin, DeviceDisplay* lcd, SerialDeviceState* state) :
      SerialDeviceController(reset_pin, lcd, 0, 0, "", 500), one_wire_pin(one_wire_pin), state(state) { construct(); }

    // construct, setup & loop
    void construct();
    void init();
    void update();

    // data
    void sendRequestCommand();
    int processSerialData(byte b);
    void completeSerialData(int error_count);
    void updateDataInformation();

    // state
    DeviceState* getDS() { return(ds); }; // return device state
    SerialDeviceState* getDSS() { return(state); }; // return device state serial
    void saveDS(); // save device state to EEPROM
    bool restoreDS(); // load device state from EEPROM
};

/**** CONSTRUCTION & init ****/

void TempController::construct() {
  // construct one wire communication
  oneWire = OneWire(one_wire_pin);
  sensors = DallasTemperature(&oneWire);

  // start data vector
  data.resize(1);
  data[0] = DeviceData(1, "T", "DegC", 2);
}

// init function
void TempController::init() {
  DeviceController::init(); // NOTE: skips SerialDeviceController

  // initialize sensors
  sensors.begin(); //initialize
  sensors.getAddress(sensor, 0); //storing address in sensor variable
  sensors.setResolution(sensor, 12);

  last_request = millis();
  last_data_log = millis(); // unlike the base class data log is time based
}

void TempController::update() {
  SerialDeviceController::update();
  // don't wait for serial data
  if (serial_data_status == SERIAL_DATA_WAITING) {
    serial_data_status = processSerialData(0);
    completeSerialData(error_counter);
  }
}

/**** DATA QUERY ****/

void TempController::sendRequestCommand() {
  sensors.requestTemperatures();
  SerialDeviceController::startSerialData();
}

int TempController::processSerialData(byte b) {
  double tempc_read = sensors.getTempC(sensor);
  if (tempc_read > -127) data[0].setNewestValue(tempc_read);
  else error_counter++;
  return(SERIAL_DATA_COMPLETE);
}

void TempController::completeSerialData(int error_count) {
  // only save values if all of them where received properly
  if (error_count == 0) {
    data[0].saveNewestValue(true);
    updateDataInformation();
  }
}


/***** DATA INFORMATION ****/

void TempController::updateDataInformation() {
  SerialDeviceController::updateDataInformation();
  // LCD update
  if (lcd) {

    int i = 0;
    if (data[i].getN() > 0)
      getDataDoubleText(data[i].variable, data[i].getValue(), data[i].units, data[i].getN(), lcd_buffer, sizeof(lcd_buffer), PATTERN_KVUN_SIMPLE, data[i].getDecimals());
    else
      getInfoKeyValue(lcd_buffer, sizeof(lcd_buffer), data[i].variable, "no data yet", PATTERN_KV_SIMPLE);
    lcd->printLine(2, lcd_buffer);

  }
}

/**** STATE PERSISTENCE ****/

// save device state to EEPROM
void TempController::saveDS() {
  EEPROM.put(STATE_ADDRESS, *state);
  #ifdef STATE_DEBUG_ON
    Serial.println("INFO: temp controller state saved in memory (if any updates were necessary)");
  #endif
}

// load device state from EEPROM
bool TempController::restoreDS(){
  SerialDeviceState saved_state;
  EEPROM.get(STATE_ADDRESS, saved_state);
  bool recoverable = saved_state.version == STATE_VERSION;
  if(recoverable) {
    EEPROM.get(STATE_ADDRESS, *state);
    Serial.printf("INFO: successfully restored state from memory (version %d)\n", STATE_VERSION);
  } else {
    Serial.printf("INFO: could not restore state from memory (found version %d), sticking with initial default\n", saved_state.version);
    saveDS();
  }
  return(recoverable);
}

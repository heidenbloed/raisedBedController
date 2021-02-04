#ifndef RAISED_BED_CONTROLLER_H_
#define RAISED_BED_CONTROLLER_H_

#include "Arduino.h"
#include "SPI.h"
#include <nRF24L01.h>
#include <RF24.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DS3231.h>

void initTft();
void handleRadioMsg(long nowUnix);
void logMoistureValue();
void updatePumpVoltage();
void initWatering(bool manual);
void wateringLoop();
void endWatering();
void updateCurrentTankStatus(int currentPumpVal);
void checkIfTankWasRefilled();
void updateClockAndWateringTime(RTCDateTime now, long nextWateringTime);
void sendViaRadio(String msg);
void updateTftStatus(String status);
void updateTftWatering(String nextWatering);
void updateTftMoisture(String moistureLevel);
void updateTftPumpVoltage(String pumpVoltage);
void updateTftTime(String timeStr);
void tankRefillISR();
void manualWateringISR();

#endif /* RAISED_BED_CONTROLLER_H_ */

#include "raised_bed_controller.h"

#define DEBUGGING

const int ledWaterPin = 0;
const int ledRefillPin = 1;
const int btnRefillPin = 2;
const int btnWateringPin = 3;
const int btnLimitPin = 4;
const int pumpPin = 5;
const int tftRstPin = 6;
const int tftCsPin = 7;
const int tftDcPin = 8;
const int nrf24CePin = 9;
const int nrf24CsnPin = 10;
const int mosiPin = 11;
const int misoPin = 12;
const int spiClkPin = 13;
const int moistureSensor1Pin = A0;
const int moistureSensor2Pin = A1;
const int moistureSensorPowerPin = A2;
const int potiPin = A3;

long lastClockUpdate = 0;

const int moistureMeasuringIntervalInSeconds = 5;
const int moistureSettleInterval = 100;
const int numMoistureReadings = 4;
const int pumpWarmupPeriod = 200;
const int pumpWateringPeriod = 1000;
const int refillCheckDelay = 250;
const int minPumpVal = 120;
float currentPumpVoltage = 0.0;

const int tftWidth = 320;
const int tftHeight = 240;
const int tftHeaderTextSize = 2;
const int tftTimeTextSize = 1;
const int tftValueTextSize = 4;
const int tftMargin = 3;
const int tftTimeBarHeight = 14;

long lastMoistureMeasurement = 0;
long wateringStart = 0;
long nextWateringTime = 0;
bool nextWateringValid = false;
bool watering = true;
bool tankIsEmpty = false;
bool displayIsOn = false;

bool tankRefillWasPressed = false;
long lastTankRefillPress = 0;
bool manualWateringWasPressed = false;
long lastManualWateringPress = 0;
int btnDebouncingPeriod = 500;

const int TANK_UNDECIDED = 0;
const int TANK_FULL = 1;
const int TANK_EMPTY = 2;
int currentTankStatus = TANK_UNDECIDED;
const int minPumpValForTankDecision = 100;
const float tankFullSlope=4.902077801266418;
const float tankFullIntercept=-122.62657410119039;
const float tankEmptySlope=1.779161939543686;
const float tankEmptyIntercept=133.7240951609847;

RF24 radio(nrf24CePin, nrf24CsnPin);
const uint8_t receivingAddress[] = "water";
const uint8_t sendingAddress[] = "wwitc";

Adafruit_ILI9341 tft = Adafruit_ILI9341(tftCsPin, tftDcPin, tftRstPin);

Adafruit_INA219 ina219;

DS3231 ds3231;
RTCDateTime datetime;


void setup() {
  #ifdef DEBUGGING
    Serial.begin(9600);
  #else
    TCCR0B = TCCR0B & 0b11111000 | 0x02;
    pinMode(ledRefillPin, OUTPUT);
    pinMode(ledWaterPin, OUTPUT);
  #endif
  pinMode(btnRefillPin, INPUT_PULLUP);
  pinMode(btnWateringPin, INPUT_PULLUP);
  pinMode(btnLimitPin, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  pinMode(moistureSensor1Pin, INPUT);
  pinMode(moistureSensor2Pin, INPUT);
  pinMode(moistureSensorPowerPin, OUTPUT);
  pinMode(potiPin, INPUT);

  initTft();

  radio.begin();
  radio.setRetries(15,15);
  radio.setAutoAck(true);
  radio.openWritingPipe(sendingAddress);
  radio.openReadingPipe(1, receivingAddress);
  radio.startListening();
  #ifdef DEBUGGING
    Serial.println("Start listening");
  #endif

  ina219.begin();

  ds3231.begin();

  attachInterrupt(digitalPinToInterrupt(btnRefillPin), tankRefillISR, RISING);
  attachInterrupt(digitalPinToInterrupt(btnWateringPin), manualWateringISR, RISING);

  #ifndef DEBUGGING
    digitalWrite(ledRefillPin, LOW);
    digitalWrite(ledWaterPin, LOW);
  #endif
  digitalWrite(moistureSensorPowerPin, LOW);

  updateTftStatus("Idle");
}

void initTft(){
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  tft.fillRect(0, 0, tftWidth/2, tftHeight/2 - tftTimeBarHeight/2, ILI9341_DARKGREEN);
  tft.fillRect(tftWidth/2, 0, tftWidth/2, tftHeight/2 - tftTimeBarHeight/2, ILI9341_RED);
  tft.fillRect(0, tftHeight/2 - tftTimeBarHeight/2, tftWidth/2, tftHeight/2 - tftTimeBarHeight/2, ILI9341_BLUE);
  tft.fillRect(tftWidth/2, tftHeight/2 - tftTimeBarHeight/2, tftWidth/2, tftHeight/2 - tftTimeBarHeight/2, ILI9341_PURPLE);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(tftHeaderTextSize);
  tft.setCursor(tftMargin, tftMargin);
  tft.println("STATUS");
  tft.setCursor(tftWidth/2 + tftMargin, tftMargin);
  tft.println("NEXT WATERING");
  tft.setCursor(tftMargin, tftHeight/2 + tftMargin - tftTimeBarHeight/2);
  tft.println("MOISTURE");
  tft.setCursor(tftWidth/2 + tftMargin, tftHeight/2 + tftMargin - tftTimeBarHeight/2);
  tft.println("PUMP VOLTAGE");

  tft.setTextSize(tftValueTextSize);
  tft.setCursor(tftMargin, tftHeight/4 - 3.5 * tftValueTextSize);
  tft.println("Init");
  tft.setCursor(tftWidth/2 + tftMargin, tftHeight/4 - 3.5 * tftValueTextSize);
  tft.println("Init");
  tft.setCursor(tftMargin, 3*tftHeight/4 - 3.5 * tftValueTextSize);
  tft.println("Init");
  tft.setCursor(tftWidth/2 + tftMargin, 3*tftHeight/4 - 3.5 * tftValueTextSize);
  tft.println("Init");

  tft.setTextSize(tftTimeTextSize);
  tft.setCursor(tftMargin, tftHeight + tftMargin - tftTimeBarHeight);
  tft.println("TIME: 06.09.2020 23:00");
}

void loop()
{
  RTCDateTime now = ds3231.getDateTime();
  long nowUnix = now.unixtime;

  if(radio.available())
  {
    handleRadioMsg(nowUnix);
  }

  if(nowUnix - lastClockUpdate > 1) {
    lastClockUpdate = nowUnix;
    updateClockAndWateringTime(now, nextWateringTime);
  }

  if(nowUnix - lastMoistureMeasurement > moistureMeasuringIntervalInSeconds) {
    lastMoistureMeasurement = nowUnix;
    logMoistureValue();
  }

  if(nextWateringValid && nowUnix >= nextWateringTime){
    nextWateringValid = false;
    initWatering(false);
  }

  updatePumpVoltage();

  if(watering){
    if(millis() - wateringStart < pumpWateringPeriod){
      wateringLoop();
    } else {
      endWatering();
    }
  }

  if(tankRefillWasPressed){
    tankRefillWasPressed = false;
    if(millis() - lastTankRefillPress > btnDebouncingPeriod){
      lastTankRefillPress = millis();
      #ifdef DEBUGGING
        Serial.println("Refill button was pressed.");
      #endif
      if(tankIsEmpty){
        checkIfTankWasRefilled();
      }
    }
  }

  if(manualWateringWasPressed){
    manualWateringWasPressed = false;
    if(millis() - lastManualWateringPress > btnDebouncingPeriod){
      lastManualWateringPress = millis();
      #ifdef DEBUGGING
        Serial.println("Watering button was pressed.");
      #endif
      if(watering){
        endWatering();
      } else {
        initWatering(true);
      }
    }
  }

  if(!displayIsOn && digitalRead(btnLimitPin) == LOW){
    displayIsOn = true;
    //digitalWrite(tftBklPin, HIGH);
  }

  if(displayIsOn && digitalRead(btnLimitPin) == HIGH){
    displayIsOn = false;
    //digitalWrite(tftBklPin, LOW);
  }

  // TODO: turn TFT off + Sync time

  delay(5);
}

void handleRadioMsg(long nowUnix){
  int len = 200;
  #ifdef DEBUGGING
    Serial.print("Radio received message with length: "); Serial.print(len);
  #endif
  char text[len+1];
  radio.read( &text, len );
  text[len+1] = 0;
  #ifdef DEBUGGING
     Serial.print("\t payload: "); Serial.println(text);
  #endif
  String command(text);
  #ifdef DEBUGGING
    Serial.println("Confirm command.");
  #endif
  String logMessage("[confirm] ");
  logMessage.concat(command);
  sendViaRadio(logMessage);
  if(command.startsWith("[nextWatering] "))
  {
    command.replace("[nextWatering] ","");
    int nextWateringReceived = command.toInt();
    #ifdef DEBUGGING
      Serial.print("Next watering unix time: "); Serial.println(nextWateringReceived);
    #endif
    if(nextWateringReceived < nowUnix){
      #ifdef DEBUGGING
        Serial.println("Ignore received watering time, since it is in the past.");
      #endif
    } else {
      nextWateringValid = true;
      nextWateringTime = nextWateringReceived;
    }
  } else {
    #ifdef DEBUGGING
      Serial.println("Unknown command.");
    #endif
  }
}

void logMoistureValue(){
  digitalWrite(moistureSensorPowerPin, HIGH);
  delay(moistureSettleInterval);
  int moistureSensor1Value = 0;
  int moistureSensor2Value = 0;
  for(int readCount = 0; readCount < numMoistureReadings; readCount++){
    moistureSensor1Value += analogRead(moistureSensor1Pin);
    moistureSensor2Value += analogRead(moistureSensor2Pin);
    delay(5);
  }
  digitalWrite(moistureSensorPowerPin, LOW);
  moistureSensor1Value /= numMoistureReadings;
  moistureSensor2Value /= numMoistureReadings;
  #ifdef DEBUGGING
    Serial.print("Moisture 1 value="); Serial.print(moistureSensor1Value); Serial.print("\tMoisture 2 value="); Serial.println(moistureSensor2Value);
  #endif
  updateTftMoisture(String((moistureSensor1Value + moistureSensor2Value) / 2));
  String logMessage("[moisture] ");
  logMessage.concat(moistureSensor1Value);
  logMessage.concat(" ");
  logMessage.concat(moistureSensor2Value);
  sendViaRadio(logMessage);
}

void updatePumpVoltage(){
  int potiValue = analogRead(potiPin);
  float pumpVoltage = 12.0 * minPumpVal * (1 + (potiValue / 1024.0) * (255.0 / minPumpVal - 1)) / 255.0;
  if(abs(currentPumpVoltage - pumpVoltage) > 0.01)
  {
    #ifdef DEBUGGING
      Serial.print("Poti value="); Serial.println(potiValue);
      Serial.print("Pump voltage="); Serial.println(pumpVoltage);
    #endif
    updateTftPumpVoltage(String(pumpVoltage, 2) + "V");
  }
  currentPumpVoltage = pumpVoltage;
}

void initWatering(bool manual){
  if(tankIsEmpty){
    sendViaRadio(String("[watering] skip"));
    #ifdef DEBUGGING
      Serial.println("Skip watering due to empty tank");
    #endif
  } else {
    updateTftStatus("Water");
    if(manual){
      sendViaRadio(String("[watering] manual"));
    } else {
      sendViaRadio(String("[watering] start"));
    }
    #ifdef DEBUGGING
      Serial.println("Start pump warm up period");
    #else
      digitalWrite(ledWaterPin, HIGH);
    #endif
    analogWrite(pumpPin, 255);
    delay(pumpWarmupPeriod);
    #ifdef DEBUGGING
      Serial.println("Start watering period");
    #endif
    wateringStart = millis();
    watering = true;
  }
}

void wateringLoop(){
  int pumpValue = 255.0 * currentPumpVoltage / 12.0;
  if(pumpValue > 252) {
    pumpValue = 255;
  }
  #ifdef DEBUGGING
    Serial.print("Pump value="); Serial.println(pumpValue);
  #endif
  analogWrite(pumpPin, pumpValue);
  updateCurrentTankStatus(pumpValue);
  if(currentTankStatus != TANK_FULL){
    tankIsEmpty = true;
    #ifdef DEBUGGING
      Serial.println("Tank is empty");
    #else
      digitalWrite(ledRefillPin, HIGH);
    #endif
    sendViaRadio(String("[tank] empty"));
    endWatering();
    updateTftStatus("Empty");
  }
}

void endWatering(){
  analogWrite(pumpPin, 0);
  watering = false;
  #ifdef DEBUGGING
    Serial.println("End watering period");
  #else
    digitalWrite(ledWaterPin, LOW);
  #endif
  updateTftStatus("Idle");
  sendViaRadio(String("[watering] end"));
  updateCurrentTankStatus(0);
}

void updateCurrentTankStatus(int currentPumpVal){
  float currentMeasurement = ina219.getCurrent_mA();
  if(currentPumpVal < minPumpValForTankDecision) {
    currentTankStatus = TANK_UNDECIDED;
  } else if(currentTankStatus != TANK_FULL && currentMeasurement > tankFullSlope * currentPumpVal + tankFullIntercept) {
    currentTankStatus = TANK_FULL;
  } else if(currentTankStatus != TANK_EMPTY && currentMeasurement < tankEmptySlope * currentPumpVal + tankEmptyIntercept) {
    currentTankStatus = TANK_EMPTY;
  }
}

void checkIfTankWasRefilled(){
  updateTftStatus("Check");
  analogWrite(pumpPin, 255);
  delay(refillCheckDelay);
  updateCurrentTankStatus(255);
  if(currentTankStatus != TANK_FULL){
    updateTftStatus("Empty");
    #ifdef DEBUGGING
      Serial.println("Tank was not refilled");
    #endif
  } else {
    #ifdef DEBUGGING
      Serial.println("Tank was refilled");
    #endif
    tankIsEmpty = false;
    #ifndef DEBUGGING
      digitalWrite(ledRefillPin, LOW);
    #endif
    updateTftStatus("Idle");
    sendViaRadio(String("[tank] refilled"));
  }
  analogWrite(pumpPin, 0);
  updateCurrentTankStatus(0);
}

void updateClockAndWateringTime(RTCDateTime now, long nextWateringTime){
  int secondsTillNextWatering = nextWateringTime - now.unixtime;
  if(secondsTillNextWatering >= 0){
    int minutesTillNextWatering = secondsTillNextWatering / 60;
    if(minutesTillNextWatering > 0){
      int hoursTillNextWatering = minutesTillNextWatering / 60;
      if(hoursTillNextWatering > 0){
        int leftMinutesTillNextWatering = minutesTillNextWatering % 60;
        updateTftWatering(String(hoursTillNextWatering) + "h" + String(leftMinutesTillNextWatering));
      }
      else {
        int leftSecondsTillNextWatering = secondsTillNextWatering % 60;
        updateTftWatering(String(minutesTillNextWatering) + "min" + String(leftSecondsTillNextWatering));
      }
    } else {
      updateTftWatering(String(secondsTillNextWatering) + "s");
    }
  } else {
    updateTftWatering("nsy");
  }
  String timeStr = String(now.year) + String("-") + String(now.month) + String("-") + String(now.day) + String(" ") + String(now.hour) + String(":") + String(now.minute) + String(":") + String(now.second);
  updateTftTime(timeStr);
}

void sendViaRadio(String msg){
  #ifdef DEBUGGING
    Serial.print("Send message via radio: "); Serial.println(msg);
  #endif
  radio.stopListening();
  char charMsg[msg.length()+1];
  msg.toCharArray(charMsg, msg.length()+1);
  radio.write( &charMsg, strlen(charMsg));
  radio.startListening();
}

void updateTftStatus(String status){
  Serial.print("TFT: set status to "); Serial.println(status);
  tft.fillRect(0, tftHeight/4 - 3.5 * tftValueTextSize, tftWidth/2, 8 * tftValueTextSize, ILI9341_DARKGREEN);
  tft.setTextSize(tftValueTextSize);
  tft.setCursor(tftMargin, tftHeight/4 - 3.5 * tftValueTextSize);
  tft.println(status);
}

void updateTftWatering(String nextWatering){
  Serial.print("TFT: set next watering to "); Serial.println(nextWatering);
  tft.fillRect(tftWidth/2, tftHeight/4 - 3.5 * tftValueTextSize, tftWidth/2, 8 * tftValueTextSize, ILI9341_RED);
  tft.setTextSize(tftValueTextSize);
  tft.setCursor(tftWidth/2 + tftMargin, tftHeight/4 - 3.5 * tftValueTextSize);
  tft.println(nextWatering);
}

void updateTftMoisture(String moistureLevel){
  Serial.print("TFT: set moisture to "); Serial.println(moistureLevel);
  tft.fillRect(0, 3*tftHeight/4 - 3.5 * tftValueTextSize, tftWidth/2, 8 * tftValueTextSize, ILI9341_BLUE);
  tft.setTextSize(tftValueTextSize);
  tft.setCursor(tftMargin, 3*tftHeight/4 - 3.5 * tftValueTextSize);
  tft.println(moistureLevel);
}

void updateTftPumpVoltage(String pumpVoltage){
  Serial.print("TFT: set pump voltage to "); Serial.println(pumpVoltage);
  tft.fillRect(tftWidth/2, 3*tftHeight/4 - 3.5 * tftValueTextSize, tftWidth/2, 8 * tftValueTextSize, ILI9341_PURPLE);
  tft.setTextSize(tftValueTextSize);
  tft.setCursor(tftWidth/2 + tftMargin, 3*tftHeight/4 - 3.5 * tftValueTextSize);
  tft.println(pumpVoltage);
}

void updateTftTime(String timeStr){
  Serial.print("TFT: set time to "); Serial.println(timeStr);
  tft.fillRect(0, tftHeight - tftTimeBarHeight, tftWidth, tftTimeBarHeight, ILI9341_BLACK);
  tft.setTextSize(tftTimeTextSize);
  tft.setCursor(tftMargin, tftHeight - tftTimeBarHeight + tftMargin);
  tft.println(timeStr);
}

void tankRefillISR() {
  tankRefillWasPressed = true;
}

void manualWateringISR() {
  manualWateringWasPressed = true;
}

#include "Arduino.h"
#include "Volume.h"
#include "definitions.hh"

ArduinoInput::ArduinoInput(int pinNumber) {
  arduinoPin = pinNumber;
  pinMode(arduinoPin, INPUT);
}

bool ArduinoInput::GetValue() {
  int pinState = digitalRead(arduinoPin);
  return pinState == HIGH;
}

HoldSensor::HoldSensor(int pinNumber) : ArduinoInput(pinNumber) {
  detected = false;
}
void HoldSensor::Update() {
  detected = this->GetValue();
}
void HoldSensor::Reset() {
  detected = false;
}

MotionSensor::MotionSensor(int sensorPin) : HoldSensor(sensorPin) {

}

ArduinoOutput::ArduinoOutput(int pinNumber) {
  arduinoPin = pinNumber;
  pinMode(arduinoPin, OUTPUT);
}
void ArduinoOutput::SetValue(bool value) {
  digitalWrite(arduinoPin, value);
}

void LED::On() {
  SetValue(HIGH);
}
void LED::Off() {
  SetValue(LOW);
}

Solenoid::Solenoid(int solenoidPin) : ArduinoOutput(solenoidPin) {
  digitalWrite(arduinoPin, HIGH);
}
void Solenoid::Lock() {
  digitalWrite(arduinoPin, HIGH);
}
void Solenoid::Unlock() {
  digitalWrite(arduinoPin, LOW);
}

Buzzer::Buzzer(int buzzerPin, byte initialVolume, Volume* volController) : ArduinoOutput(buzzerPin) { 
  this->volumeController = volController;
  volumeController->begin();
  volumeLevel = initialVolume;
}
void Buzzer::Play() {
  volumeController->tone(BUZZER_FREQUENCY, volumeLevel);
}
void Buzzer::Stop() {
  volumeController->fadeOut(1000);
}
void Buzzer::ChangeVolume(byte newVolume) {
  volumeLevel = newVolume;
  Play();
}

SerialCommunicationDevice::SerialCommunicationDevice(char* prefixText) {
  messagePrefix = prefixText;
  Serial.write("Startup");
}
void SerialCommunicationDevice::SendSignal(char* message) {
  while (!Serial.availableForWrite()) {}
  Serial.write(messagePrefix);
  Serial.write(message);
}
String SerialCommunicationDevice::ReceiveSignal() {
  if (Serial.available()) {
    return Serial.readString();
  }
}

UnlockHandler::UnlockHandler(SerialCommunicationDevice* commObject) {
  locked = true;
  entering = false;
  solenoidLock = new Solenoid(SOLENOID_PIN);
  communication = commObject;

  //rfidSensor = new HoldSensor(DOOR_RFID_PIN);
}
bool UnlockHandler::Update() {
  if (!entering) {
    return true;
  }

  //TODO Send request to confirm identity - either facial or pin
  while (millis() - timerStart < CONFIRM_ENTRY_TIMEOUT_MS) {
    //TODO Receive confirmation from python
    Entry entryState = Success;
    /*if (entryState == Success) {
      SetLock(false);
      return true;
    }
    else if (entryState == Failure) {
      return false;
    }
    */
  }
  return false;
}
void UnlockHandler::ConfirmEntry() {
  timerStart = millis();
  entering = true;
}
void UnlockHandler::SetLock(bool state) {
  locked = state;
  if (!locked) {
    entering = false;
  }
}

ControlPanel::ControlPanel(Buzzer* buzzerObject) {
  buzzer = buzzerObject;
  communication = new SerialCommunicationDevice(PREFIX_MSG);
  windowSensor = new HoldSensor(WINDOW_SENSOR_PIN);
  doorSensor = new HoldSensor(DOOR_RFID_PIN);
  motionSensor = new MotionSensor(MOTION_SENSOR_PIN);
  unlockHandler = new UnlockHandler(communication);

  LEDs[DoorLED] = new LED(LED_DOOR_PIN);
  LEDs[WindowLED] = new LED(LED_WINDOW_PIN);
  LEDs[ArmedLED] = new LED(LED_ARMED_PIN);

  alarmArmed = true;
  alarmActive = false;

}
void ControlPanel::Update() {
  windowSensor->Update();
  doorSensor->Update();
  motionSensor->Update();

  if (alarmActive) {

  }

  if (!alarmArmed) {
    LEDs[ArmedLED]->Off();
    DisarmedUpdate();
    return;
  }
  else {
    LEDs[ArmedLED]->On();
  }

  if (windowSensor->GetState() || motionSensor->GetState()) {
    SoundAlarm();
    return;
  } 

  if (doorSensor->GetState()) {
    unlockHandler->ConfirmEntry();
    unlockHandler->Update();
  }
  
}
void ControlPanel::DisarmedUpdate() {
  if (doorSensor->GetState()) {
    LEDs[DoorLED]->Off();
  }
  else {
    LEDs[DoorLED]->On();
  }
  if (windowSensor->GetState()) {
    LEDs[WindowLED]->Off();
  }
  else {
    LEDs[WindowLED]->On();
  }


  
}

void ControlPanel::SoundAlarm() {
  buzzer->Play();
  windowSensor->Reset();
  doorSensor->Reset();
  motionSensor->Reset();
  alarmActive = true;
}





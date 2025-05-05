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

TriggerSensor::TriggerSensor(int sensorPin) : ArduinoInput(sensorPin) {
  triggered = false; 
  lastState = LOW; 
}
void TriggerSensor::Update() {
  byte state = this->GetValue();
  if (state == HIGH && lastState == LOW) {
    triggered = true;
  }
  lastState = state; 
}
bool TriggerSensor::IsTriggered() {
  return triggered;
}
void TriggerSensor::Reset() {
  triggered = false;
}

ArduinoOutput::ArduinoOutput(int pinNumber) {
  arduinoPin = pinNumber;
  pinMode(arduinoPin, OUTPUT);
}
void ArduinoOutput::SetValue(bool value) {
  digitalWrite(arduinoPin, value);
}

void LED::SetState(byte newState) {
  state = newState;
  SetValue(state);
}
void LED::FlipState() {
  state = state ? LOW : HIGH;
  SetValue(state);
}

Solenoid::Solenoid(int solenoidPin) : ArduinoOutput(solenoidPin) {
  state = HIGH;
  digitalWrite(arduinoPin, state);
}
void Solenoid::ChangeLockState() {
  // Elegantly flip the state with inline if
  state = state ? LOW : HIGH;
  digitalWrite(arduinoPin, state);
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

SerialCommunicationDevice::SerialCommunicationDevice() {
  
}
// Returns true when success
bool SerialCommunicationDevice::SendMessage(char* message) {
  while (!Serial.availableForWrite()) {}
  Serial.print(message);

  while (!Serial.available()) {}
  String ack = Serial.readString();
  if (ack.equals(ACKNOWLEDGE_MSG)) {
    return true;
  }
  return false;
}
String SerialCommunicationDevice::ReceiveMessage() {
  if (Serial.available()) {
    String received = Serial.readString();
    Serial.print(ACKNOWLEDGE_MSG);
    return received;
  }
  return String();
}

ControlPanel::ControlPanel(Buzzer* buzzerObject) {
  buzzer = buzzerObject;
  communication = new SerialCommunicationDevice();
  windowSensor = new HoldSensor(WINDOW_SENSOR_PIN);
  motionSensor = new MotionSensor(MOTION_SENSOR_PIN);
  doorSolenoidButton = new TriggerSensor(DOOR_RFID_BUTTON_PIN);
  solenoidLock = new Solenoid(SOLENOID_PIN);

  LEDs[DoorLED] = new LED(LED_DOOR_PIN);
  LEDs[WindowLED] = new LED(LED_WINDOW_PIN);
  LEDs[ArmedLED] = new LED(LED_ARMED_PIN);

  alarmArmed = true;
  alarmActive = false;
  confirmingIdentity = false;
  reArming = false;
}
void ControlPanel::Update() {
  windowSensor->Update();
  motionSensor->Update();
  doorSolenoidButton->Update();
  
  UnlockHandler();

  if (!alarmArmed) {
    LEDs[ArmedLED]->SetState(LOW);
    DisarmedUpdate();
    return;
  }
  LEDs[ArmedLED]->SetState(HIGH);
  if (windowSensor->GetState() || motionSensor->GetState()) {
    SoundAlarm();
    return;
  } 
}
void ControlPanel::DisarmedUpdate() {
  if (windowSensor->GetState()) {
    LEDs[WindowLED]->SetState(LOW);
  }
  else {
    LEDs[WindowLED]->SetState(HIGH);
  }
}
void ControlPanel::UnlockHandler() {
  if (doorSolenoidButton->IsTriggered()) {
    solenoidLock->ChangeLockState();
    LEDs[DoorLED]->FlipState();
    doorSolenoidButton->Reset();
    //TODO send message saying door changed open state
    //communication->Send
    if (alarmArmed) {
      timerStart = millis();
      confirmingIdentity = true;
    }
  }

  if (!confirmingIdentity && !alarmActive) {
    return;
  }

  if (!alarmActive && millis() - timerStart > CONFIRM_ENTRY_TIMEOUT_MS) {
    SoundAlarm();
  }

  EntryOutcome outcome = CommunicateEntry();
  switch (outcome) {
    Success: {
      StopAlarm();

      break;
    }
    Failure: {
      SoundAlarm();
      break;
    }
    Waiting:
    default: {
      break;
    }
  }
}
ControlPanel::EntryOutcome ControlPanel::CommunicateEntry() {
  //TODO Send request to confirm identity - either facial or pin
  //TODO Receive confirmation from python
  EntryOutcome outcome = Success;
  /*if (outcome == Success) {
    SetLock(false);
    return true;
  }
  else if (outcome == Failure) {
    return false;
  }
  */
  return outcome;
}
void ControlPanel::SoundAlarm() {
  buzzer->Play();
  windowSensor->Reset();
  motionSensor->Reset();
  alarmActive = true;
}
void ControlPanel::StopAlarm() {
  buzzer->Stop();
  alarmActive = false;
  confirmingIdentity = false;
}





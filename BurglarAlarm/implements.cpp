#include "definitions.hh"
#include "Arduino.h"
#include "volume-library/Volume.h"

ArduinoInput::ArduinoInput(int pinNumber) {
  arduinoPin = pinNumber;
  pinMode(pinNumber, INPUT);
}

bool ArduinoInput::GetValue() {
  int pinState = digitalRead(arduinoPin);
  return pinState > 0;
}

/*HoldSensor::HoldSensor(int pinNumber) {
  detected = false;
}*/

ArduinoOutput::ArduinoOutput(int pinNumber) {
  arduinoPin = pinNumber;
  pinMode(pinNumber, OUTPUT);
}
void ArduinoOutput::SetValue(bool value) {
  digitalWrite(arduinoPin, value);
}

void LED::on() {
  SetValue(HIGH);
}
void LED::off() {
  SetValue(LOW);
}

Buzzer::Buzzer(int buzzerPin) : ArduinoOutput(buzzerPin) { 
  volumeController = Volume();
  volumeController.begin();
}
void Buzzer::play() {
  volumeController.tone(BUZZER_FREQUENCY, volumeLevel);
}
void Buzzer::stop() {
  volumeController.fadeOut(1000);
}

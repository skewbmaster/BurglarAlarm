#include "definitions.hh"
#include "Arduino.h"

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
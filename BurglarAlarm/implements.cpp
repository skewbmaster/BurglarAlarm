#include "Arduino.h"
#include "Volume.h"
#include "definitions.hh"

ArduinoInput::ArduinoInput(int pinNumber) {
  arduinoPin = pinNumber;
  pinMode(arduinoPin, INPUT);
}

bool ArduinoInput::GetValue() {
  int pinState = digitalRead(arduinoPin);
  return pinState > 0;
}

HoldSensor::HoldSensor(int pinNumber) : ArduinoInput(pinNumber) {
  detected = false;
}
void HoldSensor::Reset() {

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

UnlockHandler::UnlockHandler() {
  locked = true;
  solenoidLock = Solenoid(SOLENOID_PIN);
  // might not need faceMotion = 
  rfidSensor = HoldSensor(222);
  faceDetector = FacialRecognition(); 
  pinPad = PinPad();
}



ControlPanel::ControlPanel(Buzzer* buzzerObject) {
  buzzer = buzzerObject;
  windowSensor = HoldSensor(WINDOW_SENSOR_PIN);
  doorSensor = MotionSensor(MOTION_SENSOR_PIN);
  unlockHandler = UnlockHandler();

  //LEDs[0] = new LED(LED_DOOR_PIN);
  //LEDs[1] = new LED(LED_WINDOW_PIN);
  //LEDs[2] = new LED(LED_ARMED_PIN);

  systemActive = false;
}

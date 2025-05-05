
#ifndef ALARM_DEFINITIONS_H
#define ALARM_DEFINITIONS_H

#include "Volume.h"
#include <stdio.h>


#define LED_COUNT 3
#define BUZZER_FREQUENCY 400

#define LED_DOOR_PIN 7
#define LED_WINDOW_PIN 6
#define LED_ARMED_PIN 5
#define BUZZER_PIN 4
#define MOTION_SENSOR_PIN 5
#define WINDOW_SENSOR_PIN 8
#define SOLENOID_PIN 11


class ArduinoInput {
  protected:
    int arduinoPin;
  public:
    ArduinoInput(int pinNumber);
    virtual bool GetValue();
};

class HoldSensor : public ArduinoInput {
  protected:
    bool detected;
  public:
    HoldSensor(int pinNumber);
    void Reset();
    void Update();
};

class MotionSensor : public HoldSensor {
  private:
    int detectionDuration;
    int timeThreshold;
  public:
    MotionSensor(int sensorPin);
};

class ArduinoOutput {
  protected:
    int arduinoPin;
  public:
    ArduinoOutput(int pinNumber);
    void SetValue(bool value);
};

class LED : public ArduinoOutput {
  public:
    LED(int ledPin) : ArduinoOutput(ledPin) { };
    void On();
    void Off();
};

class Solenoid : public ArduinoOutput {
  public:
    Solenoid(int solenoidPin);
    void Lock();
    void Unlock();
};

class Buzzer : public ArduinoOutput {
  private:
    byte volumeLevel; 
    Volume* volumeController;
  public:
    Buzzer(int buzzerPin, byte initialVolume, Volume* volumeController);
    void Play();
    void Stop();
    void ChangeVolume(byte newVolume);
};

class SerialCommunicationDevice {
  protected:
    char* messagePrefix;
    bool success;
    void SendSignal(char* message);
    char* ReceiveSignal();
  public:
    virtual SerialCommunicationDevice();
    virtual bool GetUnlocked();
    virtual void Update();
};

class PinPad : SerialCommunicationDevice {
  public:
    PinPad();
    void Reset();
};

class FacialRecognition : SerialCommunicationDevice {
  public:
    FacialRecognition();
    void BeginDetection();
    void EndDetection();
};

class UnlockHandler {
  private:
    PinPad pinPad;
    FacialRecognition faceDetector;
    HoldSensor rfidSensor;
    //MotionSensor faceMotion;
    Solenoid solenoidLock;
    bool locked;

  public:
    UnlockHandler();
    void Update();
    void Lock();
    bool GetLocked();
};

class ControlPanel {
  private:
    Buzzer* buzzer;
    HoldSensor windowSensor;
    HoldSensor doorSensor;
    UnlockHandler unlockHandler;
    //LED LEDs[LED_COUNT];
    bool systemActive;

  public:
    ControlPanel(Buzzer* buzzerObject);
    void Activate();
    void Deactivate();
    void Update();
};



#endif

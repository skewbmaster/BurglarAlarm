

#ifndef ALARM_DEFINITIONS_H
#define ALARM_DEFINITIONS_H
#include "volume-library/Volume.h"

#define LED_COUNT 3
#define BUZZER_FREQUENCY 400

#define LED1_PIN 122


class ArduinoInput {
  private:
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
    MotionSensor();
    //
};

class ArduinoOutput {
  private:
    int arduinoPin;
  public:
    ArduinoOutput(int pinNumber);
    void SetValue(bool value);
};

class LED : public ArduinoOutput {
  public:
    LED(int ledPin) : ArduinoOutput(ledPin) { };
    void on();
    void off();
};

class Buzzer : public ArduinoOutput {
  private:
    Volume volumeController;
    byte volumeLevel; 
  public:
    Buzzer(int buzzerPin);
    void play();
    void stop();
};

class PinPad {
  public:
    PinPad();
    void Reset();
    void Update();
    bool GetUnlocked();
};

class FacialRecognition {
  public:
    FacialRecognition();
    void BeginDetection();
    void EndDetection();
    void Update();
    bool GetUnlocked();
};

class UnlockHandler {
  private:
    PinPad pinPad;
    FacialRecognition faceDetector;
    HoldSensor rfidSensor;
    MotionSensor faceMotion;
    ArduinoOutput solenoidLock;
    bool locked;

  public:
    UnlockHandler();
    void Update();
    void Lock();
    bool GetLocked();
};

class ControlPanel {
  private:
    HoldSensor windowSensor;
    HoldSensor doorSensor;
    UnlockHandler unlockHandler;
    ArduinoOutput LEDs[LED_COUNT];
    bool systemActive;

  public:
    ControlPanel();
    void Activate();
    void Deactivate();
    void Update();

};



#endif

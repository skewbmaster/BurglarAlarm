#ifndef ALARM_DEFINITIONS_H
#define ALARM_DEFINITIONS_H

#include "Volume.h"
#include <stdio.h>

#define LED_COUNT 3
#define BUZZER_FREQUENCY 400
#define CONFIRM_ENTRY_TIMEOUT_MS 45000
#define PREFIX_MSG "ALARM"

#define LED_DOOR_PIN 7
#define LED_WINDOW_PIN 6
#define LED_ARMED_PIN 5
#define BUZZER_PIN 4
#define MOTION_SENSOR_PIN 9
#define DOOR_RFID_PIN 10
#define WINDOW_SENSOR_PIN 8
#define SOLENOID_PIN 11


class ArduinoInput {
  protected:
    int arduinoPin;
    virtual bool GetValue();
  public:
    ArduinoInput(int pinNumber);
};

class HoldSensor : public ArduinoInput {
  protected:
    bool detected;
  public:
    HoldSensor(int pinNumber);
    bool GetState() { return detected; };
    void Update();
    void Reset();
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
    String ReceiveSignal();
  public:
    virtual SerialCommunicationDevice(char* prefixText);
    //virtual bool GetUnlocked();
    //virtual void Update();
};


class UnlockHandler {
  private:
    typedef enum {
      Success,
      Failure,
      Waiting
    } Entry;

    SerialCommunicationDevice* communication;
    Solenoid* solenoidLock;
    bool locked;
    bool entering;
    unsigned long timerStart;

    void SetLock(bool state);

  public:
    UnlockHandler(SerialCommunicationDevice* commObject);
    void ConfirmEntry();
    bool Update();
    bool GetLocked();
};

class ControlPanel {
  private:
    typedef enum {
      DoorLED,
      WindowLED,
      ArmedLED
    } LEDNames;

    Buzzer* buzzer;
    HoldSensor* windowSensor;
    HoldSensor* doorSensor;
    MotionSensor* motionSensor;
    UnlockHandler* unlockHandler;
    SerialCommunicationDevice* communication;
    LED* LEDs[LED_COUNT];
    bool alarmActive, alarmArmed;

    void DisarmedUpdate();
    void SoundAlarm();

  public:
    ControlPanel(Buzzer* buzzerObject);
    void Update();
};



#endif

#ifndef BLUETOOT_GRAZYNA_HEADER_FILE
#define BLUETOOT_GRAZYNA_HEADER_FILE

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "CircularBuffer.hpp"

class ControlCommand
{
public:
  static const uint16_t EMPTY_COMMAND = 0;
  static const uint16_t CALIBRATION_ON = 1;
  static const uint16_t CALIBRATION_OFF = 2;
public:
  ControlCommand();
  ControlCommand(uint16_t command);
  ControlCommand(uint16_t command, float value);
  virtual ~ControlCommand();
  uint16_t getCommand() const;
  float getValue() const;

protected:
  uint16_t m_command;
  float m_value;
};



class Bluetooth
{
public:
  enum class SERIAL_TYPE
  {
    SERIAL_TYPE_SERIAL,
    SERIAL_TYPE_SERIAL1,
    SERIAL_TYPE_SERIAL_SOFTWARE
  };
public:
  static const char* COMM_DELIMETER;
  static const char* CMD_CALIBRATION_ON;
  static const char* CMD_CALIBRATION_OFF;
  static const char* BT_TANS_CALIB_DATA_CMD;

public:
  Bluetooth(uint16_t rxPin, uint16_t txPin, SERIAL_TYPE serialType);
  virtual ~Bluetooth();
  void communication();

  void transfer();
  void addTranserData(const char* data);
  void recieve();
  bool hasNextCommand() const;
  ControlCommand getNextCommand();
  bool isCalibrationOn() const;
  void setCalibration(bool value);
protected:
    void parseRecieveBuffer();
protected:
  void serialBegin(uint32_t bitrate);
  int serialRead();
  size_t serialPrint(const char* data);
  size_t serialPrintln(const char* data);
  int serialAvailable();
protected:
    SERIAL_TYPE m_serialType;
    SoftwareSerial m_serial;
    CircularBuffer<char, 128> m_transferRing;
    CircularBuffer<char, 128> m_recieveRing;
    uint16_t m_tID;
    uint16_t m_rID;
    char m_transferBuffer[128];
    char m_recieveBuffer[128];

    CircularBuffer<ControlCommand, 16> m_commandsRing;

    bool m_calibrationFlag;
};

#endif

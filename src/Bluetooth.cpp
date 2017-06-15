#include "Bluetooth.hpp"

#include <math.h>
#include <stdlib.h>

ControlCommand::ControlCommand() :
m_command(EMPTY_COMMAND),
m_value(0.0f)
{
}

ControlCommand::ControlCommand(uint16_t command) :
m_command(command),
m_value(0.0f)
{
}

ControlCommand::ControlCommand(uint16_t command, float value) :
m_command(command),
m_value(0.0f)
{
}

ControlCommand::~ControlCommand()
{
}

uint16_t ControlCommand::getCommand() const
{
  return m_command;
}

float ControlCommand::getValue() const
{
  return m_value;
}

const char* Bluetooth::COMM_DELIMETER = "\r\n";
const char* Bluetooth::CMD_CALIBRATION_ON = "clON\0";
const char* Bluetooth::CMD_CALIBRATION_OFF= "clOFF\0";
const char* Bluetooth::BT_TANS_CALIB_DATA_CMD = "clDt";

Bluetooth::Bluetooth(uint16_t rxPin, uint16_t txPin, SERIAL_TYPE serialType) :
m_serialType(serialType),
m_serial(rxPin, txPin),
m_tID(0),
m_rID(0),
m_calibrationFlag(false)
{
  serialBegin(9600);
}

Bluetooth::~Bluetooth()
{

}

void Bluetooth::transfer()
{
  m_tID = 0;
  if(m_transferRing.remain())
  {
    while(m_transferRing.remain())
    {
      m_transferBuffer[m_tID++] = m_transferRing.pop();
    }
    m_transferBuffer[m_tID++] = COMM_DELIMETER[0];
    m_transferBuffer[m_tID++] = COMM_DELIMETER[1];
    serialPrintln(m_transferBuffer);
  }
}


void Bluetooth::addTranserData(const char* data)
{
  const char* ptr = data;
  while( (*ptr) != '\0' )
  {
    m_transferRing.push(*ptr);
    ptr++;
  }
}


void Bluetooth::recieve()
{

  if(serialAvailable())
  {
    while(serialAvailable())
    {
      char nextChar = serialRead();
      Serial.print(nextChar);
      m_recieveRing.push(nextChar);
    }
      Serial.println();
  }

  parseRecieveBuffer();
}


void Bluetooth::parseRecieveBuffer()
{
  while(m_recieveRing.remain())
  {
      m_recieveBuffer[m_rID++] = m_recieveRing.pop();
      if(m_recieveBuffer[m_rID-1] == COMM_DELIMETER[1] && m_recieveBuffer[m_rID-2] == COMM_DELIMETER[0])
      {
        m_recieveBuffer[m_rID-2] = '\0';
        m_rID = 0;
        if( strlen(m_recieveBuffer) > 1)
        {
          Serial.println(m_recieveBuffer);
          if(strcmp(m_recieveBuffer, CMD_CALIBRATION_ON) == 0 ){
            Serial.println("same on");
            m_commandsRing.push(ControlCommand(ControlCommand::CALIBRATION_ON));
          }else
          if(strcmp(m_recieveBuffer, CMD_CALIBRATION_OFF) == 0 ){
            Serial.println("same off");
            m_commandsRing.push(ControlCommand(ControlCommand::CALIBRATION_OFF));
          }
        }
      }
  }
}

bool Bluetooth::hasNextCommand() const
{
  return m_commandsRing.remain() > 0;
}

ControlCommand Bluetooth::getNextCommand()
{
  if(hasNextCommand())
    return m_commandsRing.pop();

  return ControlCommand();
}

bool Bluetooth::isCalibrationOn() const
{
  return m_calibrationFlag;
}

void Bluetooth::setCalibration(bool value)
{
  m_calibrationFlag = value;
}

void Bluetooth::serialBegin(uint32_t bitrate)
{
  switch(m_serialType)
  {
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL:
      Serial.begin(bitrate);
    break;
    // case SERIAL_TYPE::SERIAL_TYPE_SERIAL1:
    //   Serial1.begin(bitrate);
    // break;
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL_SOFTWARE:
    default:
          m_serial.begin(bitrate);
  }
}

int Bluetooth::serialRead()
{
  switch(m_serialType)
  {
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL: return Serial.read();
    // case SERIAL_TYPE::SERIAL_TYPE_SERIAL1:
    //   Serial1.begin(bitrate);
    // break;
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL_SOFTWARE:
    default:
          return m_serial.read();
  }
}

size_t Bluetooth::serialPrint(const char* data)
{
  switch(m_serialType)
  {
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL: return Serial.print(data);
    // case SERIAL_TYPE::SERIAL_TYPE_SERIAL1:
    //   Serial1.begin(bitrate);
    // break;
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL_SOFTWARE:
    default:
           return m_serial.print(data);

  }
}

size_t Bluetooth::serialPrintln(const char* data)
{
  switch(m_serialType)
  {
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL: return Serial.println(data);
    // case SERIAL_TYPE::SERIAL_TYPE_SERIAL1:
    //   Serial1.begin(bitrate);
    // break;
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL_SOFTWARE:
    default:
          return m_serial.println(data);
  }
}

int Bluetooth::serialAvailable()
{
  switch(m_serialType)
  {
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL: return Serial.available();
    // case SERIAL_TYPE::SERIAL_TYPE_SERIAL1:
    //   Serial1.begin(bitrate);
    // break;
    case SERIAL_TYPE::SERIAL_TYPE_SERIAL_SOFTWARE:
    default:
          return m_serial.available();
  }
}

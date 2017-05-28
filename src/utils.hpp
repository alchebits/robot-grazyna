#ifndef UTILS_ARDUINO_HPP
#define UTILS_ARDUINO_HPP

#include <Arduino.h>

#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))

#endif //UTILS_ARDUINO_HPP

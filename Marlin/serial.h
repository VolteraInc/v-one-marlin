#pragma once

#include "macros.h"
#include "MarlinSerial.h"

#define MYSERIAL customizedSerial

extern bool logging_enabled;
extern const char endl[];

template <typename T>
MarlinSerial& serialArray(const T A[], size_t size) {
  for (auto i = 0u; i < size; ++i) {
    if (i != 0) {
      MYSERIAL.print(',');
    }
    MYSERIAL.print(A[i]);
  }
  return MYSERIAL;
}

inline MarlinSerial& serialFloat(float f, unsigned int digits) {
  MYSERIAL.print(f, digits);
  return MYSERIAL;
}

template<class T> inline MarlinSerial& operator<<(MarlinSerial &obj, T arg) {
  obj.print(arg);
  return obj;
}

inline MarlinSerial& operator<<(MarlinSerial &obj, MarlinSerial&) {
  return obj;
}

inline MarlinSerial& protocol() { return MYSERIAL; }
#define protocol protocol()

inline MarlinSerial& log() { return MYSERIAL << F("log: "); }
#define log log()

inline MarlinSerial& logNotice() { return MYSERIAL << F("notice: "); }
#define logNotice logNotice()

inline MarlinSerial& logWarning() { return MYSERIAL << F("warning: "); }
#define logWarning logWarning()

inline MarlinSerial& logError() { return MYSERIAL << F("error: "); }
#define logError logError()

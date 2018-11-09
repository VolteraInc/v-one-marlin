#pragma once

#include "macros.h"
#include "MarlinSerial.h"

#define MYSERIAL customizedSerial

extern bool logging_enabled;
extern const char endl[];

namespace logging {
  extern bool inISR;

  inline const __FlashStringHelper* isrPrefix() {
    if (inISR) {
      return F("~~~");
    } else {
      return F("");
    }
  }

  inline static const __FlashStringHelper* openBrace() { return F("{ "); }
  inline static const __FlashStringHelper* closeBrace() { return F(" }"); }

  inline static const __FlashStringHelper* endString() { return F("\""); }
  inline static const __FlashStringHelper* endStringComma() { return F("\", "); }

  inline static const __FlashStringHelper* errorName() { return F("\"name\": \""); }
  inline static const __FlashStringHelper* errorContext() { return F("\"context\": \""); }
  inline static const __FlashStringHelper* errorReason() { return F("\"reason\": \""); }
}

inline MarlinSerial& operator<<(MarlinSerial &obj, unsigned long arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj, unsigned  int arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,          long arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,           int arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,         float arg) { obj.print(arg, 6); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,        double arg) { obj.print(arg, 6); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,          char arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,   const char* arg) { obj.print(arg); return obj; }

class __FlashStringHelper;
inline MarlinSerial& operator<<(MarlinSerial &obj, const __FlashStringHelper* arg) { obj.print(arg); return obj; }


template<typename T>
struct ArrayWithSize {
  const T* arr;
  size_t size;
  ArrayWithSize(const T a[], unsigned int n) : arr(a), size(n) {}
};
template<typename T>
inline MarlinSerial& operator<<(MarlinSerial &obj, ArrayWithSize<T> a) {
  for (auto i = 0u; i < a.size; ++i) {
    if (i != 0) {
      obj << ',';
    }
    obj << a.arr[i];
  }
  return obj;
}

struct FloatWithFormat {
  float value;
  unsigned int digits;
  FloatWithFormat(float v, unsigned int d) : value(v), digits(d) {}
};
inline MarlinSerial& operator<<(MarlinSerial &obj, const FloatWithFormat& st) {
  obj.print(st.value, st.digits);
  return obj;
}


inline MarlinSerial& protocol() { return MYSERIAL; }
#define protocol protocol()

inline MarlinSerial& log() {
  return MYSERIAL << logging::isrPrefix() << F("log: ");
}
#define log log()

inline MarlinSerial& logNotice() {
  return MYSERIAL << logging::isrPrefix() << F("notice: ");
}
#define logNotice logNotice()

inline MarlinSerial& logWarning() {
  return MYSERIAL << logging::isrPrefix() << F("warning: ");
}
#define logWarning logWarning()

inline MarlinSerial& logError() {
  return MYSERIAL << logging::isrPrefix() << F("error: ");
}
#define logError logError()

inline MarlinSerial& reportError_(const __FlashStringHelper* name, const __FlashStringHelper* context) {
  using namespace logging;
  return logError
    << openBrace()
    << errorName() << name << endStringComma()
    << errorContext() << context << endStringComma()
    << errorReason();
}
#define reportError(name, context, reason) \
   do { \
    reportError_(name, context) reason << logging::endString() << logging::closeBrace() << endl; \
  } while(0)

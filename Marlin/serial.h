#pragma once

#include "macros.h"
#include "MarlinSerial.h"

#define MYSERIAL customizedSerial

extern bool logging_enabled;
extern const char endl[];

inline MarlinSerial& operator<<(MarlinSerial &obj, unsigned long arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj, unsigned  int arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,          long arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,           int arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,         float arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,        double arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,          char arg) { obj.print(arg); return obj; }
inline MarlinSerial& operator<<(MarlinSerial &obj,   const char* arg) { obj.print(arg); return obj; }

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

inline MarlinSerial& log() { return MYSERIAL << F("log: "); }
#define log log()

inline MarlinSerial& logNotice() { return MYSERIAL << F("notice: "); }
#define logNotice logNotice()

inline MarlinSerial& logWarning() { return MYSERIAL << F("warning: "); }
#define logWarning logWarning()

inline MarlinSerial& logError() { return MYSERIAL << F("error: "); }
#define logError logError()

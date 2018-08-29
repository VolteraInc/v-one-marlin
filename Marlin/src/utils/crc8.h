#pragma once

// ref https://stackoverflow.com/questions/29214301/ios-how-to-calculate-crc-8-dallas-maxim-of-nsdata
unsigned char crc8(const void * data, const unsigned int size) {
  const auto bytes = reinterpret_cast<const unsigned char*>(data);
  unsigned char crc = 0;
  for ( unsigned int i = 0; i < size; ++i ) {
    unsigned char inbyte = bytes[i];
    for ( unsigned char j = 0; j < 8; ++j ) {
      unsigned char mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if ( mix ) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}
// #pragma once
//
// #define WRITE_ADDRESS B01110110
// #define READ_ADDRESS B01110110
//
// #define RESET_CMD B00011110
//
// #define CONVERT_D1_256_CMD B01000000
// #define CONVERT_D1_512_CMD B01000010
// #define CONVERT_D1_1024_CMD B01000100
// #define CONVERT_D1_2048_CMD B01000110
// #define CONVERT_D1_4096_CMD B01001000
// #define CONVERT_D1_8192_CMD B01001010
//
// #define CONVERT_D2_256_CMD B01010000
// #define CONVERT_D2_512_CMD B01010010
// #define CONVERT_D2_1024_CMD B01010100
// #define CONVERT_D2_2048_CMD B01000110
// #define CONVERT_D2_4096_CMD B01010110
// #define CONVERT_D2_8192_CMD B01011010
//
// #define CONVERT_256_DELAY 600 //microseconds, as per datasheet
// #define CONVERT_512_DELAY 1170  //microseconds, as per datasheet
// #define CONVERT_1024_DELAY 2280 //microseconds, as per datasheet
// #define CONVERT_2048_DELAY 4540 //microseconds, as per datasheet
// #define CONVERT_4096_DELAY 9040 //microseconds, as per datasheet
// #define CONVERT_8192_DELAY 18080  //microseconds, as per datasheet
//
// #define TEMP_OCR 256
// #define PRESSURE_OCR 4096
//
// #define ADC_READ_CMD B00000000
//
// #define PROM_READ_C0_CMD B10100000
// #define PROM_READ_C1_CMD B10100010
// #define PROM_READ_C2_CMD B10100100
// #define PROM_READ_C3_CMD B10100110
// #define PROM_READ_C4_CMD B10101000
// #define PROM_READ_C5_CMD B10101010
// #define PROM_READ_C6_CMD B10101100
//
// // Address calibratin parameters in promValues
// #define C1 0
// #define C2 1
// #define C3 2
// #define C4 3
// #define C5 4
// #define C6 5
//
// class FlowSensor {
//   public:
//     FlowSensor();
//     void periodic_work();
//     long pCompensated = 0; //measured in mBar
//     long tempCompensated = 0; //measured in degrees C
//
//   private:
//     unsigned int promValues[6] = {};
//     unsigned long pressureValueD1 = 0;
//     unsigned long temperatureValueD2 = 0;
//
//     long dT = 0; // "Some parameter" - James
//     long temp = 0; //measured in degrees C*100
//     long p = 0; //measured in mBar*10
//
//     unsigned long getData(int byteCount);
//     uint8_t crc4(unsigned int n_prom[]);
//     bool readPROM();
//     bool resetSensor();
//     bool secondOrderCompensation();
//     bool readTemp(int OSR);
//     bool readPressure(int OSR);
//
// };

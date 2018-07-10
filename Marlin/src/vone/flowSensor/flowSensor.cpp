// #include <Wire.h>
// #include "flowSensor.h"
// #include "../../../serial.h"
//
//
// FlowSensor::FlowSensor() {
//   Wire.begin(); // Configures the I2C Bus.
//   resetSensor();
//   readPROM();
// }
//
// void FlowSensor::periodic_work() {
//   static int state = 0;
//
//   switch(state) {
//     case 0: {
//       readTemp(TEMP_OCR);
//       state ++;
//       break;
//     }
//     case 1: {
//       readPressure(PRESSURE_OCR);
//       state ++;
//       break;
//     }
//     case 2: {
//       secondOrderCompensation();
//       state ++;
//       break;
//     }
//     case 3: {
//       log << F("Temperature is: ") << tempCompensated << F(" Pressure is: ") << pCompensated << endl;
//       state = 0;
//       break;
//     }
//   }
// }
//
// bool FlowSensor::readPressure(int OSR) {
//
//   uint8_t conversionCmd = 0;
//   int conversionDelay = 0;
//
//   switch (OSR){
//     case 256:
//       conversionCmd = CONVERT_D1_256_CMD;
//       conversionDelay = CONVERT_256_DELAY;
//       break;
//     case 512:
//       conversionCmd = CONVERT_D1_512_CMD;
//       conversionDelay = CONVERT_512_DELAY;
//       break;
//     case 1024:
//       conversionCmd = CONVERT_D1_1024_CMD;
//       conversionDelay = CONVERT_1024_DELAY;
//       break;
//     case 2048:
//       conversionCmd = CONVERT_D1_2048_CMD;
//       conversionDelay = CONVERT_2048_DELAY;
//       break;
//     case 4096:
//       conversionCmd = CONVERT_D1_4096_CMD;
//       conversionDelay = CONVERT_4096_DELAY;
//       break;
//     case 8192:
//       conversionCmd = CONVERT_D1_8192_CMD;
//       conversionDelay = CONVERT_8192_DELAY;
//       break;
//   }
//
//   Wire.beginTransmission(WRITE_ADDRESS);
//   Wire.write((uint8_t)conversionCmd);
//
//   if (Wire.endTransmission()!= 0){
//     logError << F("Flow Sensor: Unable to read pressure") << endl;
//     return false;
//   }
//
//   delayMicroseconds(conversionDelay);
//
//   Wire.beginTransmission(WRITE_ADDRESS);
//   Wire.write((uint8_t)ADC_READ_CMD);
//   if (Wire.endTransmission()!= 0){
//     logError << F("Flow Sensor: Unable to read pressure") << endl;
//     return false;
//   }
//
//   //should there be a check here to make sure data is good?
//   pressureValueD1 = getData(Wire.requestFrom(READ_ADDRESS, 3));
//
//   long long OFF = ((long long)promValues[C2]<<16)+(long long)((promValues[C4]*(long long)dT)>>7);
//
//   if(OFF <-17179344900 || OFF > 25769410560 ){
//     //out of bounds
//     logError << F("Flow Sensor: Error OFF out of bounds") << endl;
//     return false;
//   }
//
//   long long SENS = ((long long)promValues[C1]<<15)+(long long)((promValues[C3]*(long long)dT)>>8);
//
//   if(SENS < -8589672450 || SENS > 12884705280){
//     //out of bounds
//     logError << F("Flow Sensor: Error SENS out of bounds") << endl;
//     return false;
//   }
//
//   p = (long)((((long long)(pressureValueD1*SENS)>>21)-OFF)>>13);
//
//   if(p < 0 || p > 300000){
//     //out of bounds
//     logError << F("Flow Sensor: Error Presusre out of bounds") << endl;
//     return false;
//   }
//
//   return true;
//
// }
//
//
// bool FlowSensor::readTemp(int OSR){
//   uint8_t conversionCmd = 0;
//   int conversionDelay = 0;
//   switch (OSR){
//     case 256:
//       conversionCmd = CONVERT_D2_256_CMD;
//       conversionDelay = CONVERT_256_DELAY;
//       break;
//     case 512:
//       conversionCmd = CONVERT_D2_512_CMD;
//       conversionDelay = CONVERT_512_DELAY;
//       break;
//     case 1024:
//       conversionCmd = CONVERT_D2_1024_CMD;
//       conversionDelay = CONVERT_1024_DELAY;
//       break;
//     case 2048:
//       conversionCmd = CONVERT_D2_2048_CMD;
//       conversionDelay = CONVERT_2048_DELAY;
//       break;
//     case 4096:
//       conversionCmd = CONVERT_D2_4096_CMD;
//       conversionDelay = CONVERT_4096_DELAY;
//       break;
//     case 8192:
//       conversionCmd = CONVERT_D2_8192_CMD;
//       conversionDelay = CONVERT_8192_DELAY;
//       break;
//   }
//
//   Wire.beginTransmission(WRITE_ADDRESS);
//   Wire.write((uint8_t)conversionCmd);
//
//   if (Wire.endTransmission()!= 0){
//    logError << F("Flow Sensor: Unable to read temperature.") << endl;
//    return false;
//   }
//
//   delayMicroseconds(conversionDelay);
//
//   Wire.beginTransmission(WRITE_ADDRESS);
//   Wire.write((uint8_t)ADC_READ_CMD);
//
//   if (Wire.endTransmission()!= 0){
//     logError << F("Flow Sensor: Unable to read temperature.") << endl;
//     return false;
//   }
//
//   //should there be a check here to make sure data is good?
//   temperatureValueD2 = getData(Wire.requestFrom(READ_ADDRESS, 3));
//
//   dT = temperatureValueD2-(((long)promValues[C5])<<8);
//
//   if(dT < -16776960 || dT > 16777215){
//     //out of bounds
//     logError << F("Flow Sensor: dT out of bounds.") << endl;
//     return false;
//   }
//
//   temp =(((long long)dT*promValues[C6])>>23) + 2000;
//
//   if(temp < -4000 || temp > 8500){
//     //out of bounds
//     logError << F("Flow Sensor: temperature out of bounds.") << endl;
//     return false;
//   }
//
//   return true;
// }
//
//
// bool FlowSensor::secondOrderCompensation(){
//   long long OFF = ((long long)promValues[C2]<<16)+(long long)((promValues[C4]*(long long)dT)>>7);
//   long long SENS = ((long long)promValues[C1]<<15)+(long long)((promValues[C3]*(long long)dT)>>8);
//
//   long Ti = 0;
//   long long OFFi = 0;
//   long long SENSi = 0;
//
//   if((temp/100) >= 20){
//     Ti = 2*(dT*dT)>>37;
//     OFFi = (long long)(((temp-2000)*(temp-2000))>>4);
//   }
//   else{
//     Ti = (long)(3*((long long)dT*dT)>>33);
//     OFFi = (long long)(3*((temp-2000)*(temp-2000))>>1);
//     SENSi = (long long)(5*((temp-2000)*(temp-2000))>>3);
//
//     if((temp/100)<-15){
//       OFFi = OFFi+(long long)(7*(temp+1500)*(temp+1500));
//       SENSi = SENSi+(long long)(4*(temp+1500)*(temp+1500));
//     }
//   }
//
//   tempCompensated = (temp-Ti)/100.0;
//   pCompensated = (((long)((pressureValueD1*(SENS-SENSi))>>21)-(long)(OFF-OFFi))>>13)/10;
//   return;
// }
//
// bool FlowSensor::resetSensor() {
//
//   Wire.beginTransmission(WRITE_ADDRESS);
//   Wire.write((uint8_t) RESET_CMD);
//   if(Wire.endTransmission() != 0){
//     logError << F("FlowSensor: Sensor Reset Fail!") << endl;
//     return false;
//   }
//   return true;
// }
//
// bool FlowSensor::readPROM(){
//   // Extracts configuration parameters.
//
//  unsigned int n_prom[8] = {}; // Extract the calibration data in the registers.
//  uint8_t factoryCRC = 0; // Used to store the factory calculated CRC.
//
//  uint8_t promCmds[7] = { // Address to extract the proper content.
//      PROM_READ_C0_CMD, // Used for CRC
//      PROM_READ_C1_CMD,
//      PROM_READ_C2_CMD,
//      PROM_READ_C3_CMD,
//      PROM_READ_C4_CMD,
//      PROM_READ_C5_CMD,
//      PROM_READ_C6_CMD
//    };
//
//
//   for(int i=0; i < sizeof(promCmds); i++){
//     Wire.beginTransmission(WRITE_ADDRESS);
//     Wire.write(promCmds[i]);
//     if (Wire.endTransmission() != 0){
//       logError << F("Flow Sensor: Unable to read flow sensor PROM") << endl;
//       return false;
//     }
//
//     // Extract information from it.
//     n_prom[i] = getData(Wire.requestFrom(READ_ADDRESS, 2));
//
//     if (i == 0) {
//       factoryCRC = n_prom[0] >> 12; // C0 is used to get CRC
//     } else {
//       promValues[i-1] = n_prom[i];  // C1-C6 is used in calibration parameters.
//     }
//
//   }
//
//   uint8_t calculatedCRC = crc4(n_prom); // calculate crc4.
//
//   if(calculatedCRC != factoryCRC) {
//     logError << F("Flow Sensor: Invalid CRC from PROM") << endl;
//     return false;
//   }
//
//   log << F("Flow Sensor: Valid CRC from PROM found") << endl;
//   return true;
// }
//
// uint8_t FlowSensor::crc4(unsigned int n_prom[]){
//   int count;
//   unsigned int n_rem = 0;
//   uint8_t n_bit;
//
//   n_prom[0] = n_prom[0] & 0x0FFF; // Erase the CRC.
//   n_prom[7] = 0;
//
//   for (count = 0; count < 16; count ++){
//     if (count %2 ==1) {
//       n_rem ^= (unsigned int)((n_prom[count>>1]) & 0x00FF);
//     }
//     else {
//       n_rem ^= (unsigned short) (n_prom[count>>1]>>8);
//     }
//     for (n_bit = 8; n_bit > 0; n_bit--){
//       if (n_rem & (0x8000)){
//         n_rem = (n_rem << 1) ^ 0x3000;
//       }
//       else {
//         n_rem = (n_rem << 1);
//       }
//     }
//   }
//   n_rem = ((n_rem >> 12) & 0x000F);
//   return (n_rem ^ 0x00);
// }
//
// unsigned long FlowSensor::getData(int byteCount){
//   unsigned long data = 0;
//
//   if(byteCount == 0){
//     return 0; //TODO Do something better
//   }
//
//   // Read in one byte at a time.
//   for(int i = 0; i < byteCount; i++){
//     data = data << 8;
//     data |= Wire.read();
//   }
//
//   return data;
// }

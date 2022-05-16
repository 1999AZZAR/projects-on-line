//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 05 Arduino Line Follower with PID EEPROM Calibration
//----------------------------------------Include Library
#include <EEPROM.h>
//----------------------------------------

//----------------------------------------Retrieve calibration result data that has been stored in the EEPROM
// If you want to use the EEPROM for another use, in order to prevent overlapping data, use the EEPROM address from 20 and above.
// Because the EEPROM address from 20 and below may have been used to store calibration result data.
for (byte i=0; i<=7; i++) {
  ResultRef[i] = readIntFromEEPROM(i);
}
//----------------------------------------

writeIntIntoEEPROM(i, ResultRef[i]); //--> Save the calibration result value in EEPROM

//========================================================================void writeIntIntoEEPROM(int address, int number)
// Subroutine for writing calibration result values in EEPROM
// Source : https://roboticsbackend.com/arduino-store-int-into-eeprom/
void writeIntIntoEEPROM(int address, int number)
{ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}
//========================================================================

//========================================================================int readIntFromEEPROM(int address)
// Subroutine to read calibration result value from EEPROM
// Source : https://roboticsbackend.com/arduino-store-int-into-eeprom/
int readIntFromEEPROM(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}
//========================================================================
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
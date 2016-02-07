#include <Arduino.h>

#include <time.h>

#include <SPI.h>
#include <SPIRAM.h>

/* Working buffer */
#define CMD_BUFFER_SIZE 8096  // size of working buffer (on heap)
uint8_t * Buff = NULL;  /* Put a working buffer on heap later (with malloc). */

int8_t testSPIRAM(uint_farptr_t p1, uint16_t p2)
{
  int8_t ReturnCode;

  if (Buff == NULL) // if there is no Buff buffer allocated (pointer is NULL), then allocate buffer.
    if ( !(Buff = (uint8_t *) malloc( sizeof(uint8_t) * CMD_BUFFER_SIZE )))
    {
      Serial.println(F("malloc for *Buff fail..!"));
      return SPIRAM_ERROR;
    }

  if (p2 >= CMD_BUFFER_SIZE) p2 = CMD_BUFFER_SIZE;

  srand( p1 % 42 ); // mod 42 is very random. ;-)

  for ( uint16_t i = 0; i < p2; ++i)
  {
    Buff[i] = (uint8_t) rand();  // fill the Buff with some pseudo random numbers
  }

  Serial.print(F("Testing at 0x"));
  Serial.print((uint32_t)p1, HEX);
  Serial.print(F(" for "));
  Serial.print(p2);
  Serial.println(F(" bytes"));

  ReturnCode = SPIRAM_begin();
  if (ReturnCode) {
    Serial.print(ReturnCode); // problem with opening the EEPROM / SRAM
    return ReturnCode;
  }

  Serial.print(F("Testing...  "));

  uint_farptr_t FarAddress = p1;

  // write out from the buffer, larger than EEPROM page size, and starting at different addresses.
  ReturnCode = SPIRAM_write( FarAddress, Buff, p2);    // write out from the buffer filled with random bytes.

  if (ReturnCode) {
    Serial.print(ReturnCode);  /* error or disk full */
    return ReturnCode;
  }

  for (uint16_t i = 0; i < p2; ++i) // now for each byte we wrote to the SRAM / EEPROM...
  {
    uint8_t read_result;

    // we only need to read one byte, because reading is easy.
    ReturnCode = SPIRAM_read( &read_result, (FarAddress + i), 1);  // read back each random byte from the SRAM/EEPROM.

    if (ReturnCode)
    {
      Serial.print(ReturnCode);  /* error or disk full */
      return ReturnCode;
    }

    //    Testing visually check each (supposedly) written byte, and the return byte from SRAM / EEPROM.
    //    Serial.print(F("Written 0x"));
    //    Serial.print(Buff[i], HEX);
    //    Serial.print(F(" Read 0x"));
    //    Serial.println(read_result, HEX);

    if ( Buff[i] != read_result)  // check each random byte read back from the SRAM/EEPROM.
    {
      Serial.print(F("Error at 0x"));
      Serial.print((FarAddress + i), HEX);
      Serial.print(F(" with 0x"));
      Serial.println(read_result, HEX);

      return SPIRAM_ERROR;
    }
  }
  return SPIRAM_SUCCESS;
}


void setup() {
  // put your setup code here, to run once:

  setup_RTC_interrupt();    // initialise the RTC Timer & Interrupt.
  Serial.begin(38400);      // open the serial port at 38400 bps.
  SPI.begin();              // initialise the SPI bus.
}

void loop() {
  // put your main code here, to run repeatedly:

  time_t currentTick;     // set up a location for the current time stamp
  time((time_t *)&currentTick); // get the current time, just to change the test initial address and random data seed.
  Serial.println(currentTick);

  Serial.println(F("SPI SRAM Memory Testing: "));
  if (testSPIRAM(RAM0_ADDR + (currentTick & 0x3ff), CMD_BUFFER_SIZE))
    Serial.println(F("**** FAILED ***\r\n"));
  else
    Serial.println(F("PASSED\r\n"));

  Serial.println(F("SPI EEPROM Memory Testing: "));
  if (testSPIRAM(RAM1_ADDR + (currentTick & 0x03ff), CMD_BUFFER_SIZE))
    Serial.println(F("**** FAILED ***\r\n"));
  else
    Serial.println(F("PASSED\r\n"));

  delay(3000);
}

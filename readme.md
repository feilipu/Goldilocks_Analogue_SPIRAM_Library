This library implements firmware to control SPI SRAM, FRAM, and EEPROM as implemented in the Goldilocks Analogue, a ATmega1284p MCU classic Arduino board.

Also provides Ring Buffer support for all memory types.

## General

The Goldilocks Analogue is configured with 256kByte of SRAM and 256kByte of EEPROM. The firmware supports many arbitary SPI memory devices, depending on the device definitions contained in the library code.

## Reference Manual

Initialise the SPI RAM slave select pin, and clear SPI data buffers.
```
int8_t
SPIRAM_begin(void);
```

Write Length bytes at the Dest (SPI RAM) address starting from the Src (MCU RAM) address.
```
int8_t
SPIRAM_write(
					uint_farptr_t const Dest,
					uint8_t * const Src,
					uint32_t const Length);
```

Read Length bytes from the Src (SPI RAM) address starting into Dest (MCU RAM) address.
```
int8_t
SPIRAM_read(
					uint8_t * const Dest,
					uint_farptr_t const Src,
					uint32_t const Length);
```

Initializes a ring buffer ready for use. Buffers must be initialized via this function
before any operations are called upon them. Already initialized buffers may be reset
by re-initializing them using this function.
NOTE WELL: NO CHECKING THAT THE POINTERS OR SIZE ARE CORRECT !!!
```
void
SPIRAM_ringBuffer_InitBuffer(
					SPIRAM_ringBuffer_t* buffer,
					uint_farptr_t const dataPtr,
					const uint32_t size)
```

Flushes the contents of a ring buffer.
```
inline void
SPIRAM_ringBuffer_Flush(
					SPIRAM_ringBuffer_t* const buffer);
```

Retrieves the current number of bytes stored in a particular buffer. This value is computed
by entering an atomic lock on the buffer, so that the buffer cannot be modified while the
computation takes place. This value should be cached when reading out the contents of the buffer,
so that as small a time as possible is spent in an atomic lock.
```
inline uint32_t
SPIRAM_ringBuffer_GetCount(
					SPIRAM_ringBuffer_t* const buffer);
```

Retrieves the free space in a particular buffer. This value is computed by entering an atomic lock
on the buffer, so that the buffer cannot be modified while the computation takes place.
```
inline uint32_t
SPIRAM_ringBuffer_GetFreeCount(
					SPIRAM_ringBuffer_t* const buffer);
```

Atomically determines if the specified ring buffer contains any data. This should
be tested before removing data from the buffer, to ensure that the buffer does not
underflow.

If the data is to be removed in a loop, store the total number of bytes stored in the
buffer (via a call to the SPIRAM_ringBuffer_GetCount() function) in a temporary variable
to reduce the time spent in atomicity locks.
```
inline uint8_t
SPIRAM_ringBuffer_IsEmpty(
					SPIRAM_ringBuffer_t* const buffer);
```

Atomically determines if the specified ring buffer contains any free space. This should
be tested before storing data to the buffer, to ensure that no data is lost due to a
buffer overrun.
```
inline uint8_t
SPIRAM_ringBuffer_IsFull(
					SPIRAM_ringBuffer_t* const buffer);
```

Inserts an element into the ring buffer.
```
void
SPIRAM_ringBuffer_Poke(
					SPIRAM_ringBuffer_t* buffer,
					uint8_t const data);
```

Removes an element from the ring buffer.
```
uint8_t
SPIRAM_ringBuffer_Pop(
					SPIRAM_ringBuffer_t* buffer);
```

Returns the next element stored in the ring buffer, without removing it.
```
uint8_t
SPIRAM_ringBuffer_Peek(
					SPIRAM_ringBuffer_t* const buffer);
```


## Files & Configuration

* SPIRAM.h  contains definitions for selecting which SPI RAM devices are configured.
* SPIRAM.c  contains the code to write and read SPI RAM devices.
* SPIRAM_ringBuffer.h contains ringbuffer management for SPI RAM devices.
* ringBuffer.h contains normal MCU RAM ringbuffer management.

Example code for using the SPIRAM_write() and SPIRAM_read() functions below.

```
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

  /* write out from the buffer, larger than EEPROM page size, and starting at different addresses. */
  ReturnCode = SPIRAM_write( FarAddress, Buff, p2);    // write out from the buffer filled with random bytes.

  if (ReturnCode) {
    Serial.print(ReturnCode);  /* error or disk full */
    return ReturnCode;
  }

  for (uint16_t i = 0; i < p2; ++i) // now for each byte we wrote to the SRAM / EEPROM.
  {
    uint8_t read_result;

    /* we only need to read one byte, because reading is easy. */
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
```

When using the SPI RAM functions in a sketch, remember to initialise the SPI Bus before use.
```
#include <Arduino.h>

#include <time.h>
#include <SPI.h>
#include <SPIRAM.h>

/* Working buffer */
#define CMD_BUFFER_SIZE 8096  // size of working buffer (on heap)
uint8_t * Buff = NULL;  /* Put a working buffer on heap later (with malloc). */


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
```

/*
 * SPIRAM.h
 *
 *  Created on: 04/01/2016
 *  Author: phillip stevens
 */

#ifndef SPIRAM_H_
#define SPIRAM_H_

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************/

// DEVICE TYPES
#define FRAM_FM25W256				11//  FRAM       32k x 8 bit
#define FRAM_MB85RS64V				12//  FRAM        8k x 8 bit
#define SRAM_23LC1024				21//  SRAM      128k x 8 bit
#define SRAM_23LC512				22//  SRAM       64k x 8 bit
#define EEPROM_AT25M01				31//  EEPROM 131,072 x 8 bit
#define EEPROM_AT25512				32//  EEPROM  65,536 x 8 bit

// SELECT PORT HARDWARE ASSIGNMENT HERE - WHICH DEVICE ARE YOU USING?
// Comment out (undefine) RAM0 or RAM1 port if you are not using it.
// RAM0 is upper (physical) device on the board (lower address)
// RAM1 is lower (physical) device on the board
#define RAM0		SRAM_23LC1024
#define RAM1		EEPROM_AT25M01

/**********************************************************************/
/* Add new device Types here */

#if (RAM0 == FRAM_FM25W256 )
#define RAM0_ADDR_BITS			15
#define RAM0_WRITE_LATCH					// Device has Write Latch
#endif
#if (RAM1 == FRAM_FM25W256 )
#define RAM1_ADDR_BITS			15
#define RAM1_WRITE_LATCH					// Device has Write Latch
#endif

#if ( RAM0 == FRAM_MB85RS64V )
#define RAM0_ADDR_BITS			13
#define RAM0_WRITE_LATCH					// Device has Write Latch
#endif
#if ( RAM1  == FRAM_MB85RS64V )
#define RAM1_ADDR_BITS			13
#define RAM1_WRITE_LATCH					// Device has Write Latch
#endif

#if ( RAM0 == SRAM_23LC1024 )
#define RAM0_ADDR_BITS			17
#define RAM0_PAGE_BYTES			32			// Bytes per Page
#endif
#if ( RAM1 == SRAM_23LC1024 )
#define RAM1_ADDR_BITS			17
#define RAM1_PAGE_BYTES			32			// Bytes per Page
#endif

#if ( RAM0 == SRAM_23LC512 )
#define RAM0_ADDR_BITS			16
#define RAM0_PAGE_BYTES			32			// Bytes per Page
#endif
#if ( RAM1 == SRAM_23LC512)
#define RAM1_ADDR_BITS			16
#define RAM1_PAGE_BYTES			32			// Bytes per Page
#endif

#if ( RAM0 == EEPROM_AT25M01 )
#define RAM0_ADDR_BITS			17
#define RAM0_BUSY_MODE						// Device can be busy writing, so check first
#define RAM0_WRITE_LATCH					// Device has Write Latch
#define RAM0_PAGE_ACCESS					// Device needs Page WRITE Access
#define RAM0_PAGE_BYTES			256			// Bytes per Page
#endif
#if ( RAM1 == EEPROM_AT25M01)
#define RAM1_ADDR_BITS			17
#define RAM1_BUSY_MODE						// Device can be busy writing, so check first
#define RAM1_WRITE_LATCH					// Device has Write Latch
#define RAM1_PAGE_ACCESS					// Device needs Page WRITE Access
#define RAM1_PAGE_BYTES			256			// Bytes per Page
#endif

#if ( RAM0 == EEPROM_AT25512 )
#define RAM0_ADDR_BITS			16
#define RAM0_BUSY_MODE						// Device can be busy writing, so check first
#define RAM0_WRITE_LATCH					// Device has Write Latch
#define RAM0_PAGE_ACCESS					// Device needs Page WRITE Access
#define RAM0_PAGE_BYTES			128			// Bytes per Page
#endif
#if ( RAM1 == EEPROM_AT25512)
#define RAM1_ADDR_BITS			16
#define RAM1_BUSY_MODE						// Device can be busy writing, so check first
#define RAM1_WRITE_LATCH					// Device has Write Latch
#define RAM1_PAGE_ACCESS					// Device needs Page WRITE Access
#define RAM1_PAGE_BYTES			128			// Bytes per Page
#endif


/**********************************************************************/

// SPI Slave Select lines are Port C4 and C5 (Goldilocks Analogue)
#define RAM_PORT	PORTC
#define RAM_DDR		DDRC
#define RAM0_SS		PORTC4
#define RAM1_SS		PORTC5

// High Byte of 32bit address determines which device we're addressing
#define RAM0_DEVICE_ADDR		0x00
#define RAM1_DEVICE_ADDR		0x01
//#define RAM2_DEVICE_ADDR		0x02 // etc

// 32bit address determines which device we're addressing
#define RAM0_ADDR				0x00000000
#define RAM1_ADDR				0x01000000
//#define RAM2_ADDR				0x02000000 // etc


// DEVICE STANDARD COMMAND SET
#define READ			0b00000011	// READ Memory Code
#define WRITE			0b00000010	// WRITE Memory Code

#define WREN			0b00000110	// Set Write Enable Latch
#define WRDI			0b00000100	// Clear Write Enable Latch
#define RDID			0b10011111	// Read Device ID

#define RDSR			0b00000101	// Read MODE / Status Register
#define WRSR			0b00000001	// Write MODE / Status Register

// DEVICE STATUS REGISTER MODES
#define MODE_BYTE		0b00000000	// Status Register Mode for Byte Access
#define MODE_PAGE		0b10000000	// Status Register Mode for Page Access
#define MODE_SEQN		0b01000000	// Status Register Mode for Sequential Access (default)
#define MODE_RESERVED	0b11000000	// Status Register Mode RESERVED
#define MODE_BUSY		0b00000001	// Device is Busy in Write process - EEPROM Only

/*
 * Error Codes
 */

#define SPIRAM_SUCCESS                      (  0)
#define SPIRAM_ERROR						( -1)
#define SPIRAM_INVALID_ARGUMENT		    	( -2)
#define SPIRAM_UNSUPPORTED_OPTION			( -3)
#define SPIRAM_PERMISSION_DENIED			( -4)
#define SPIRAM_FILE_NOT_FOUND				( -5)
#define SPIRAM_NO_FREE_FILE_DESCRIPTOR	    ( -6)
#define SPIRAM_NO_SPACE_LEFT_ON_DEVICE	    ( -7)
#define SPIRAM_NO_SUCH_DEVICE				( -8)
#define SPIRAM_DEVICE_IS_BUSY				( -9)
#define SPIRAM_READ_ONLY_FILE_SYSTEM		(-10)

#define SPIRAM_BUS_IS_NOT_READY			    (-11)

#define SPIRAM_DEVICE_CANT_ALLOCATE_NEW	    (-12)
#define SPIRAM_DEVICE_INVALID_NAME		    (-13)
#define SPIRAM_DEVICE_ADDRESS_CONFIGURED	(-14)
#define SPIRAM_DEVICE_NAME_TOO_LONG		    (-15)
#define SPIRAM_DEVICE_INVALID_FAT			(-16)

#define SPIRAM_VOLUME_CANT_ALLOCATE_NEW	    (-22)
#define SPIRAM_VOLUME_INVALID_NAME		    (-23)
#define SPIRAM_VOLUME_ALREADY_CONFIGURED	(-24)
#define SPIRAM_VOLUME_NAME_TOO_LONG		    (-25)
#define SPIRAM_NO_SUCH_VOLUME				(-28)
#define SPIRAM_VOLUME_IS_BUSY				(-29)



/**********************************************************************/

#ifndef NULL              /* pointer to nothing */
   #define NULL ((void *) 0)
#endif

#ifndef TRUE              /* Boolean true */
   #define TRUE (1)
#endif

#ifndef FALSE              /* Boolean false */
   #define FALSE (0)
#endif

/**********************************************************************/


#include <inttypes.h>

/* make a structure to enable us to break the supplied SPI RAM address into a "device"
 * which will be on different slave select lines, and 24 bits of address.
 *
 * The maximum available 1Mbit device uses 17 address bits,
 * so keeping 24 bits for the address is sufficient.
 * We can then differentiate on 256 different slave select lines (if we had enough I/O).
 *
 * NOTE: Address bytes are stored in reverse order because the ATmega is a little ended device !!!
 */
typedef struct _avrspi_device_ptr_t {
	uint8_t low_byte;
	uint8_t mid_byte;
	uint8_t high_byte;
	uint8_t device_byte;
} avrspi_device_ptr_t;

/* allow the SPI RAM address to be referred to either as byte components or as a 32bit long address.
 */
typedef union _addr_farptr_t {
	avrspi_device_ptr_t	bAddr;
	uint_farptr_t lAddr;
} addr_farptr_t;

int8_t SPIRAM_begin(void);

int8_t SPIRAM_read(uint8_t * const Dest, uint_farptr_t const Src, uint32_t const Length);

int8_t SPIRAM_write(uint_farptr_t const Dest, uint8_t * const Src, uint32_t const Length);

#ifdef __cplusplus
}
#endif

#endif /* SPIRAM_H_ */
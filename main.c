/*-------------------------------------------------------------------------/
/  Stand-alone MMC boot loader  R0.01
/--------------------------------------------------------------------------/
/
/  Copyright (C) 2010, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/--------------------------------------------------------------------------/
/ Dec 6, 2010  R0.01  First release
/--------------------------------------------------------------------------/
/ This is a stand-alone MMC/SD boot loader for megaAVRs. It requires a 4KB
/ boot section for code, four GPIO pins for MMC/SD as shown in sch.jpg and
/ nothing else. To port the boot loader into your project, follow the
/ instruction sdescribed below.
/
/ 1. Setup the hardware. Attach a memory card socket to the any GPIO port
/    where you like. Select boot size at least 4KB for the boot loader with
/    BOOTSZ fuses and enable boot loader with BOOTRST fuse.
/
/ 2. Setup the software. Change the four port definitions in the asmfunc.S.
/    Change MCU_TARGET, BOOT_ADR and MCU_FREQ in the Makefile. The BOOT_ADR
/    is a BYTE address of boot section in the flash. Build the boot loader
/    and write it to the device with a programmer.
/
/ 3. Build the application program and output it in binary form instead of
/    hex format. Rename the file "app.bin" and put it into the memory card.
/
/ 4. Insert the card and turn the target power on. When the boot loader found
/    the application file, the file is written into the flash memory prior to
/    start the application program. On-board LED lights (if exist) during
/    the flash programming operation.
/
/-------------------------------------------------------------------------*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include "integer.h"
#include "diskio.h"

//#define UART_DEBUG
#ifdef UART_DEBUG
	#include "uart.h"
#endif
#define CRC_FLASH

// following very useful defines were originally in pff.c but we're not using that so...
#define BS_jmpBoot			0
#define BS_OEMName			3
#define BPB_BytsPerSec		11
#define BPB_SecPerClus		13
#define BPB_RsvdSecCnt		14
#define BPB_NumFATs			16
#define BPB_RootEntCnt		17
#define BPB_TotSec16		19
#define BPB_Media			21
#define BPB_FATSz16			22
#define BPB_SecPerTrk		24
#define BPB_NumHeads		26
#define BPB_HiddSec			28
#define BPB_TotSec32		32
#define BS_55AA				510

#define BS_DrvNum			36
#define BS_BootSig			38
#define BS_VolID			39
#define BS_VolLab			43
#define BS_FilSysType		54

#define BPB_FATSz32			36
#define BPB_ExtFlags		40
#define BPB_FSVer			42
#define BPB_RootClus		44
#define BPB_FSInfo			48
#define BPB_BkBootSec		50
#define BS_DrvNum32			64
#define BS_BootSig32		66
#define BS_VolID32			67
#define BS_VolLab32			71
#define BS_FilSysType32		82

#define MBR_Table			446

#define	DIR_Name			0
#define	DIR_Attr			11
#define	DIR_NTres			12
#define	DIR_CrtTime			14
#define	DIR_CrtDate			16
#define	DIR_FstClusHI		20
#define	DIR_WrtTime			22
#define	DIR_WrtDate			24
#define	DIR_FstClusLO		26
#define	DIR_FileSize		28

typedef void (*f_ptr)(void);

void flash_erase (DWORD);				/* Erase a flash page (asmfunc.S) */
void flash_write (DWORD, const BYTE*);	/* Program a flash page (asmfunc.S) */

BYTE Buff[512];	/* sector buffer */
register uint8_t myR1 asm("r1"); // just doing this so I can do the normal CRT stuff (R1=0, SREG=R1)

#ifdef CRC_FLASH
	static uint16_t updcrc(uint8_t c, uint16_t crc)
	{
		uint8_t flag;
		for (uint8_t i = 0; i < 8; ++i)
		{
			flag = !!(crc & 0x8000);
			crc <<= 1;
			if (c & 0x80)
				crc |= 1;
			if (flag)
				crc ^= 0x1021;
			c <<= 1;
		}
		return crc;
	}
#endif

static inline
int mem_cmpP(const void* dst, const void* src, int cnt) {
	const char *d = (const char *)dst, *s = (const char *)src;
	int r = 0;
	while (cnt-- && (r = *d++ - pgm_read_byte(s++)) == 0) ;
	return r;
}

__attribute__((naked,section(".vectors"))) void start(void) {
	asm("rjmp main");
}	

__attribute__((naked,section(".init3"))) int main(void) {
	DSTATUS res;
	uint16_t * p16;
	uint32_t * p32;
	uint8_t SecPerClus;
	uint16_t BytesPerSec, RsvdSecCnt, BPBSec, FATSz16, progver, currver, firstclust;
	uint32_t RootDir, lba;

	DDRD = 0xFF;
	myR1 = 0;
	SREG = myR1;
	SPH = RAMEND >> 8;
	SPL = RAMEND & 0xFF;

	currver = eeprom_read_word((const uint16_t *)E2END - 1);
#ifdef UART_DEBUG
	UART_init();
	UART_putsP(PSTR("FlashVer = "));
	UART_puthex(currver >> 8);
	UART_puthex(currver & 0xFF);
	UART_newline();
#endif
	res = disk_initialize();
	if (!res) { // card init was OK so we can go on the hunt...
		BPBSec = 0;
		res = disk_readp(Buff, BPBSec, 0, 512 ); // first sector is either an MBR or a boot sector.
		// Offset 11,12 in BS/BPB is BPB_BytesPerSec and will be 512 (if BPB not MBR)
		p16 = (uint16_t *)&Buff[BPB_BytsPerSec];
		BytesPerSec = *p16;
		if (BytesPerSec != (uint16_t)512) {
			// We almost certainly have an MBR so need to get "sectors preceding partition 1" at offset 0x1C6
			p32 = (uint32_t *)&Buff[MBR_Table + 8];
			// then read the BS/BPB
			BPBSec = *p32;
			disk_readp(Buff, BPBSec, 0, 512 );
		}
		// With any luck we're here with a BS/BPB in the buffer
#ifdef UART_DEBUG
		UART_putsP(PSTR("BPB:\r\n"));
		UART_dumpsector(Buff);
#endif
		// Fatgen103: FirstRootDirSecNum = BPB_ResvdSecCnt + (BPB_NumFATs * BPB_FATSz16);
		p16 = (uint16_t *)&Buff[BPB_RsvdSecCnt];
		RsvdSecCnt = *p16;
		p16 = (uint16_t *)&Buff[BPB_FATSz16];
		FATSz16 = *p16;
		RootDir = BPBSec + RsvdSecCnt + (Buff[BPB_NumFATs] * FATSz16);
		SecPerClus = Buff[BPB_SecPerClus];
		// ready to read the root directory sector
		disk_readp(Buff, RootDir, 0, 512 );
#ifdef UART_DEBUG
		UART_putsP(PSTR("Root:\r\n"));
		UART_dumpsector(Buff);
#endif
		// scan the root dir for "AVRAPnnn", entries are 32 bytes each
		for (uint16_t i=0; i<512; i+=32) {
			if (mem_cmpP(&Buff[i], PSTR("AVRAP"), 5) == 0) {
				progver = ((Buff[i+5]-'0') << 8) | ((Buff[i+6]-'0') << 4) | (Buff[i+7]-'0');
#ifdef UART_DEBUG
				UART_putsP(PSTR("FileVer = "));
				UART_puthex(progver >> 8);
				UART_puthex(progver & 0xFF);
				UART_newline();
#endif
				if ((currver == 0xFFFF) || (currver < progver)) { // either there's no app or it's out of date
					// we're going for it!
					eeprom_update_word((uint16_t *)E2END - 1, 0xFFFF);

					// Now got to find the data cluster for this file entry we found
					p16 = (uint16_t *)&Buff[i + DIR_FstClusLO];
					firstclust = *p16;
					lba = ((firstclust - 1) * SecPerClus) + RootDir;
					for (uint16_t sect=0; sect < (BOOT_ADR/512); sect++ ) {
						uint16_t pgoffset;
						uint16_t faddr;
						disk_readp(Buff, lba + sect, 0, 512);
						faddr = sect * 512;
						for (uint8_t page = 0; page < 512 / SPM_PAGESIZE; page++) {
							pgoffset = page * SPM_PAGESIZE;
							flash_erase(faddr + pgoffset);
							flash_write(faddr + pgoffset, &Buff[pgoffset]);
						}						
					}						
					eeprom_update_word((uint16_t *)E2END - 1, progver);
					break;
				}
			}
		}
	}	

#ifdef CRC_FLASH
	uint16_t crc = 0xFFFF;
	for (uint16_t i=0; i < CODE_LEN; i++) {
		crc = updcrc(pgm_read_byte(i), crc);
	}
	// augment
	crc = updcrc(0, updcrc(0, crc));
#endif
	
#ifdef UART_DEBUG
	UART_putsP(PSTR("App CRC = "));
	UART_puthex(crc >> 8);
	UART_puthex(crc & 0xFF);
	UART_newline();
	UART_putsP(PSTR("Flash CRC = "));
	UART_puthex(pgm_read_word(CODE_LEN) >> 8);
	UART_puthex(pgm_read_word(CODE_LEN) & 0xFF);
	UART_newline();
#endif
	if ((eeprom_read_word((const uint16_t *)E2END - 1) != 0xFFFF) &&		/* Start application if exists */
#ifdef CRC_FLASH	
		(pgm_read_word(CODE_LEN) == crc)) {
#else
		(pgm_read_word(0) == (uint16_t)0xFFFF)) {
#endif
		f_ptr reset = (f_ptr)0;
		reset();
	}		

	while(1) {
		PORTD ^=0xFF;
		_delay_ms(250);
	}
}


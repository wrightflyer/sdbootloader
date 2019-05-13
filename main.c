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
#include <util/delay.h>
#include <string.h>
#include "integer.h"
#include "diskio.h"

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


void flash_erase (DWORD);				/* Erase a flash page (asmfunc.S) */
void flash_write (DWORD, const BYTE*);	/* Program a flash page (asmfunc.S) */

extern BYTE CardType;

//FATFS Fatfs;				/* Petit-FatFs work area */
BYTE Buff[/*SPM_PAGESIZE*/512];	/* Page data buffer */
register uint8_t myR1 asm("r1"); 

#ifdef UART_DEBUG
void UART_init(void) {
	UCSRB = (1 << TXEN);
	UBRRL = 23; // 9600 @ 3.6864MHz
}

void UART_put(uint8_t c) {
	while (!(UCSRA & (1 << UDRE)));
	UDR = c;
}

void UART_puts(uint8_t * str) {
	while (*str) {
		UART_put(*str++);
	}
}

void UART_putsP(uint8_t * str) {
	while (pgm_read_byte(str) != 0) {
		UART_put(pgm_read_byte(str++));
	}
}

void UART_putnibble(uint8_t c) {
	if (c < 10) {
		UART_put('0' + c);
	}
	else {
		UART_put('A' + c - 10);
	}
}

void UART_puthex(uint8_t c) {
	UART_putnibble(c >> 4);
	UART_putnibble(c & 0x0F);
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

__attribute__((naked, section(".init3"))) int main (void)
{
	DWORD fa;	/* Flash address */
	WORD br;	/* Bytes read */
	DSTATUS res;
	uint16_t * p16;
	uint32_t * p32;
	uint16_t BytesPerSec, RsvdSecCnt, BPBSec, FATSz16;
	uint32_t RootDir;

	DDRD = 0xFF;
	myR1 = 0;
	SREG = myR1;
	SPH = RAMEND >> 8;
	SPL = RAMEND & 0xFF;

#ifdef UART_DEBUG
	UART_init();
	UART_putsP(PSTR("hello\r\n"));
#endif
	res = disk_initialize();
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
	UART_puts(&Buff[BS_FilSysType32]); // print "FAT12"/"FAT16"
#endif
	// Fatgen103: FirstRootDirSecNum = BPB_ResvdSecCnt + (BPB_NumFATs * BPB_FATSz16);
	p16 = (uint16_t *)&Buff[BPB_RsvdSecCnt];
	RsvdSecCnt = *p16;
	p16 = (uint16_t *)&Buff[BPB_FATSz16];
	FATSz16 = *p16;
	RootDir = BPBSec + RsvdSecCnt + (Buff[BPB_NumFATs] * FATSz16);
	// ready to read the root directory sector
	disk_readp(Buff, RootDir, 0, 512 );
#if UART_DEBUG
	for (uint16_t i=0; i<512; i++) {
		if ((i % 16) == 0) {
			UART_put('\r');
			UART_put('\n');
		}
		UART_puthex(Buff[i]);
		UART_put(' ');
	}
#endif
	for (uint16_t i=0; i<512; i+=32) {
		if (mem_cmpP(&Buff[i],PSTR("FILES"),5) == 0) {
#ifdef UART_DEBUG
			UART_putsP(PSTR("Found"));
#endif
			break;
		}
	}

	if (pgm_read_word(0) != 0xFFFF)		/* Start application if exist */
		((void(*)(void))0)();

	while(1) {
		PORTD ^=0xFF;
		_delay_ms(100);
	}
}


/*-------------------------------------------------------------------------/
/  Stand-alone MMC boot loader  R0.01
/--------------------------------------------------------------------------/
/
/ NB: you !!!must!!!! read and possibly edit configure.h before building this project.
/
/ Latter parts of this (c) Cliff Lawson, 2013. This uses asmfunc.S, and mmc.c
/ from ChaN's original code but otherwise it's a rewrite of the FAT aware
/ sections to do a very specific job of finding just one file and keeping the
/ code footprint to about 1K..2K suitable for small mega AVRs. The same license
/ applies for newly added code.
/
/ See http://spaces.atmel.com/gf/project/sdbootloader/ for more details
/
/ The following is existing copyright text from Chan's "pfboot" (some parts
/ of this text are old an no longer apply to this project):
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
/ instruction described below.
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
#include <stdlib.h>
#include "integer.h"
#include "diskio.h"

#include "configure.h"

#ifdef UART_DEBUG // may be set in configure.h
#include "uart.h"
#endif

// this is the function pointer type used to define the variable that will be used
// to enter the app code when we're ready
typedef void (*f_ptr)(void);

void flash_erase (DWORD);				/* Erase a flash page (asmfunc.S) */
void flash_write (DWORD, const BYTE*);	/* Program a flash page (asmfunc.S) */

static uint16_t updcrc(uint8_t c, uint16_t crc);
uint8_t crc_app_ok(void);
static inline int mem_cmpP(const void* dst, const void* src, int cnt);
void disk_read(uint32_t sec);

BYTE Buff[512];	/* sector buffer */
register uint8_t myR1 asm("r1"); // just doing this so I can do the normal CRT stuff (R1=0, SREG=R1)

//##########################################################################
//#                                           THE PARTY STARTS HERE ;-)                                           #
//##########################################################################
/*
** The following routine locates to the reset jump at the very start of the bootloader
** so is the first opcode executed at power on. The .vectors ensures (via the linker
** script) that it be positioned first. It is done like this (with an RJMP to "main")
** to ensure that any .progmem data (PSTR() debug strings) can be placed by the linker
** immediately after the reset jump.
**
** (this is all because -nostartfiles is being used to ensure no space is wasted with a
** C Run Time (CRT) including a full size vector table)
*/
__attribute__((naked,section(".vectors"))) void start(void) {
    asm("rjmp main");
}	

/*
** The naked asm("rjmp main") above arrives here which is where the bootloader itself
** really starts. The use of .init3 here is simply historical and given the start()
** code above that now handles the correct reset jump, probably not needed - it could just
** be the default .text in fact
*/
__attribute__((section(".init3"))) int main(void) {
    DSTATUS res;
    uint16_t * p16;
    uint32_t * p32;
    uint8_t SecPerClus;
    uint16_t BytesPerSec, RsvdSecCnt, BPBSec, filever, flashver;
    uint32_t FATSz;
    uint32_t RootDir, lba, firstclust;
#ifdef ROOT_MULTISEC	
    uint16_t root_count, offset;
    uint8_t done_update;
#endif	
#ifdef CLUSTER_SUPPORT	
    uint32_t thisclust;
#endif	
    uint8_t fat_offset;

    // the code will flash an LED to give an idea of activity. The port it is on is defined
    // above as LED_PORT. A standard "trick" is that the DDR for an AVR port is always at the
    // port_address-1 so:
    *(uint8_t*)(&LED_PORT-1) = (1 << LED_BIT);
    // because -nostartfiles is used there is no CRT so nothing to ensure R1=0, clear SREG
    // setup the stack pointer and so on. The following therefore replaces those things from
    // the standard CRT. Note there is no .data or .bss usage in this program so no need to 
    // duplicate those loops.
    myR1 = 0;
    SREG = myR1;
    SPH = RAMEND >> 8;
    SPL = RAMEND & 0xFF;

    // This is a debug thing - it's sometimes useful to flood RAM with a known value to make
    // it easier to see in the debugger exactly what has been written
#ifdef FLOOD_RAM
    for (uint16_t i=RAMSTART; i<RAMEND; i++) {
        *(uint8_t *)i = FLOOD_VALUE;
    }
#endif	

    // Currently loaded application code version is held in EEPROM (or 0xFFFF is nothing there
    // or a previous update was interrupted)
    flashver = eeprom_read_word((const uint16_t *)E2END - 1);
#ifdef UART_DEBUG
    UART_init();
    UART_puts(PSTR("B/L start"));
    UART_newline();
#endif
    res = disk_initialize();
#ifdef UART_DEBUG
    UART_putsP(PSTR("res="), res);
#endif	
    if (!res) { // card init was OK so we can go on the hunt...
        BPBSec = 0;
        disk_read(BPBSec); // first sector is either an MBR or a boot sector.
        // Offset 11,12 in BS/BPB is BPB_BytesPerSec and will be 512 (if BPB not MBR)
        p16 = (uint16_t *)&Buff[BPB_BytsPerSec];
        BytesPerSec = *p16;
        if (BytesPerSec != (uint16_t)512) {
            // We almost certainly have an MBR so need to get "sectors preceding partition 1" at offset 0x1C6
            p32 = (uint32_t *)&Buff[MBR_Table + 8];
            // then read the BS/BPB
            BPBSec = *p32;
            disk_read(BPBSec);
        }
        // With any luck we're here with a BS/BPB in the buffer
        // Fatgen103: FirstRootDirSecNum = BPB_ResvdSecCnt + (BPB_NumFATs * BPB_FATSz16);
        p16 = (uint16_t *)&Buff[BPB_RsvdSecCnt];
        RsvdSecCnt = *p16;
        p16 = (uint16_t *)&Buff[BPB_FATSz16];
        FATSz = *p16;
        
        // adding FAT32 support here
        // If FATSz == 0 then we're looking at FAT32
        fat_offset = 1;
#ifdef FAT32_SUPPORT
        if (FATSz == 0) {
            p32 = (uint32_t *)&Buff[BPB_FATSz32];
            FATSz = *p32;
            fat_offset = 2;
        }
#endif
        
        // this is one of the classic FatGen103 calculations...
        RootDir = BPBSec + RsvdSecCnt + (Buff[BPB_NumFATs] * FATSz);
        SecPerClus = Buff[BPB_SecPerClus];
#ifdef ROOT_MULTISEC		
        root_count = SecPerClus; // going to read for N sectors in the root, not just one
        done_update = 0;
        offset = 0;
        
        while (root_count--) {
#endif			
        // ready to read the root directory sector
#ifdef ROOT_MULTISEC
        disk_read(RootDir + offset++);
#else
        disk_read(RootDir);
#endif		
        
        // scan the root dir for "AVRAPnnn", entries are 32 bytes each
        // Note this will only scan one sector (the first 16 entries) of the root directory so it relies
        // on AVRAPnnn.BIN being positioned there. What's more it stops looking as soon as the AVRAP text
        // is found so if the directory held AVRAP003.BIN then AVRAP007.BIN the latter would never be
        // seen. It's therefore best to use an SD card that only ever contains a single AVRAPnnn.BIN, 
        // replace it completely if updating to a later version. If there are other files then as long as
        // the existing AVRAPnnn.BIN is deleted and then the new one copied it should re-use the early
        // directory slot that was being used previously.
        //
        // There is potential to wrap an outer loop around this one that does disk_read(RootDir + i)'s
        // to consider more than one sector. Someone feeling REALLY brave could flesh this out still further
        // to follow the cluster chain for the root dir but by the time you get to that point you are 
        // probably building code so large that you might as well just use ChaN's original avr_boot within
        // pfsample.zip!
        for (uint16_t i=0; i<512; i+=32) {
            if (mem_cmpP(&Buff[i], PSTR("AVRAP"), 5) == 0) {
                filever = ((Buff[i+5]-'0') << 8) | ((Buff[i+6]-'0') << 4) | (Buff[i+7]-'0');
                if ((flashver == 0xFFFF) || (flashver < filever)) { // either there's no app or it's out of date
                    // we're going for it!
                    // by setting the EEPROM version back to 0xFFFF (like a new AVR with erased EEPROM) this 
                    // ensures that if the update is interrupted then at the next restart there's no chance of
                    // the "half finished" code image being useed.
                    eeprom_update_word((uint16_t *)E2END - 1, 0xFFFF);

                    // Now got to find the data cluster for this file entry we found
                    p16 = (uint16_t *)&Buff[i + DIR_FstClusLO];
                    firstclust = *p16;
#ifdef FAT32_SUPPORT
                    if (fat_offset == 2) {
                        p16 = (uint16_t *)&Buff[i + DIR_FstClusHI];
                        firstclust |=  (uint32_t)*p16 << 16;
                    }
#endif					
                    lba = firstclust - fat_offset;
                    lba *= SecPerClus;
                    lba += RootDir;
                    uint16_t pgoffset;
                    uint16_t faddr;
#ifdef CLUSTER_SUPPORT					
                    uint8_t sec_count = SecPerClus;
                    thisclust = firstclust;
#endif					
                    for (uint16_t sect=0; sect < (BOOT_ADR/512); sect++) {
#ifdef CLUSTER_SUPPORT
                        disk_read(lba + (sect % SecPerClus));
#else
                        disk_read(lba + sect);
#endif
                        faddr = sect * 512;
                        for (uint8_t page = 0; page < 512 / SPM_PAGESIZE; page++) {
                            pgoffset = page * SPM_PAGESIZE;
                            flash_erase(faddr + pgoffset);
                            flash_write(faddr + pgoffset, &Buff[pgoffset]);
                        }
#ifdef CLUSTER_SUPPORT						
                        sec_count--;
                        if (!sec_count) {
                            // used all the sectors in a cluster so time to go hunting
                            ldiv_t fat_pos;
                            fat_pos = ldiv(thisclust, (fat_offset ==1) ? 256 : 128);
#ifdef UART_DEBUG
                            UART_puthex16((uint16_t)fat_pos.quot);
                            UART_put(' ');
                            UART_puthex16((uint16_t)fat_pos.rem);
#endif							
                            fat_pos.quot += BPBSec + RsvdSecCnt;
                            disk_read(fat_pos.quot); // get the FAT sector that 'thisclust' is contained in 
#ifdef FAT32_SUPPORT
                            if (fat_offset == 1) {
#endif								
                                p16 = (uint16_t *)&Buff[fat_pos.rem * 2];
                                lba = *p16;			// get cluster number of new cluster
#ifdef FAT32_SUPPORT
                            }
                            else {
                                p32 = (uint32_t *)&Buff[fat_pos.rem * 4];
                                lba = *p32;			// get cluster number of new cluster
                            }
#endif							
                            // !! what if we just got an end of chain marker instead of a cluster number??
                            thisclust = lba;	// and remember in case we do this again
                            lba -= fat_offset;	// then, as before, convert to a base LBA for the cluster
                            lba *= SecPerClus;
                            lba += RootDir;
                            sec_count = SecPerClus;
                        }
#endif						
                    }
#ifdef CRC_FLASH					
                    if (crc_app_ok()) {
#endif						
                        eeprom_update_word((uint16_t *)E2END - 1, filever);
#ifdef ROOT_MULTISEC
                        done_update = 1;
#endif						
#ifdef CRC_FLASH
                    }
#endif
                    break;
                }
            }
        }
#ifdef ROOT_MULTISEC
        if (done_update) {
            break; // from loop reading root and looking for AVRAP files
        }
        } //  the indentation of this is wrong as additional while() loop only present when reading multi-sector root
#endif		
    }	

    /* Start application if exists */
    // there are three possible tests here:
    // 1) the end of the EEPROM should have a valid version number written once the last
    //    code update completed. If it did not complete the variable will hold 0xFFFF.
    // 2) the real belts and braces test is when CRC_FLASH is set and the app image has been
    //    created with srec_cat which calculated and embedded a CRC. Even if the EEPROM says
    //    there is code there this will check for definite that it's exactly the same code
    //    as when it was built on the PC
    // 3) the real simplistic test is that almost every app has a JMP/RJMP at location 0 (or at
    //    least something who's opcode is not 0xFFFF (erased flash)) so the simple check is to
    //    make sure the words at 0 is not 0xFFFF
    if ((eeprom_read_word((const uint16_t *)E2END - 1) != 0xFFFF) &&
#ifdef CRC_FLASH	
        (crc_app_ok())) {
#else
        (pgm_read_word(0) != (uint16_t)0xFFFF)) {
#endif
        f_ptr reset = (f_ptr)0;
        // This just calls to location 0. It's been suggested (SprinterSB) that this method 
        // is not guaranteed to work but it certainly seems to.
        reset();
        // If it really is the case that a function pointer set to 0 will not get exection to
        // location 0 then other ideas are:
        // 1) just do an asm("jmp 0")
        // 2) encode the Asm to do asm("eor r1,r1; push r1, push r1, push r1; ret")
        // 3) I think the following would work:
        //      uint16_t * sp = SP;
        //      sp--;
        //      *sp = 0;
        //   } // this will RET to 0
    }		

    // Assuming we didn't enter the app just sit flashing the lights a bit to say we are "trapped"
    // in the bootloader (that is the AVRAPnnn.BIN could not be found/programmed and the flash does
    // not appear to already have a valid code image. The user should check the card contents and
    // try again.
    while(1) {
        LED_PORT ^= (1 << LED_BIT);
        _delay_ms(250);
    }
}

//##########################################################################
//#                                             UTILITY FUNCTIONS                                                      #
//##########################################################################

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

uint8_t crc_app_ok(void) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i=0; i < CODE_LEN; i++) {
        crc = updcrc(pgm_read_byte(i), crc);
    }
    // augment
    crc = updcrc(0, updcrc(0, crc));
    #ifdef UART_DEBUG
    UART_putsP(PSTR("App CRC= "), crc);
    UART_putsP(PSTR("Flash CRC= "), pgm_read_word(CODE_LEN));
    #endif
    return (pgm_read_word(CODE_LEN) == crc);
}
#endif

// this came from ChaN code - it's a fairly clever way to do a compare of
// string in RAM (dst) with a string in flash (src) and is used to find "AVRAP"
static inline
int mem_cmpP(const void* dst, const void* src, int cnt) {
    const char *d = (const char *)dst, *s = (const char *)src;
    int r = 0;
    while (cnt-- && (r = *d++ - pgm_read_byte(s++)) == 0) ;
    return r;
}

// this is just wrapping around ChaN's low level disk_readp() in mmc.c allowing some
// debug stuff to be invoked if required
void disk_read(uint32_t sec) {
    // always to Buff[], always offset=0 and always len=512
    #ifdef UART_DEBUG
    UART_puthex16((uint16_t)sec);
    UART_newline();
    #endif
    disk_readp(Buff, sec, 0, 512);
    #ifdef UART_DEBUG
    UART_dumpsector(Buff);
    #endif
}


void __do_copy_data(void) {}
void __do_clear_bss(void) {}
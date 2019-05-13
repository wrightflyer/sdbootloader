/*
 * configure.h
 *
 * Created: 10/10/2013 13:30:24
 *  Author: cliff
 */ 

#ifndef CONFIGURE_H_
#define CONFIGURE_H_

//##########################################################################
//#                                                CONFIGURATION                                                      #
//##########################################################################
//
// The following are used in asmfinc.S for bit-banging SPI to the SD card. These defaults are
// the same as hardware SPI (for mega16/32) so the same wiring can use real SPI once the
// application code starts:
#define SD_PIN      PINB
#define SD_PORT	    PORTB
#define SD_DDR	    DDRB
#define CS_PIN	    4
#define SCLK_PIN    7
#define DI_PIN      5 //i(n to SD card that is)
#define DO_PIN      6

// if the code cannot be programmed/run then the bootloader sits flashing an LED. The
// following two defines say where that LED is
#define LED_PORT	PORTD
#define LED_BIT		6

//  Use the following defines to configure exactly how the bootloader is built. At present the
//  bootloader, built for mega32, with all the optional parts disabled is 1,406 bytes long. It's
//  then up to you to choose which of the additional support you want to enable. But if you are
//  trying to stay under 2048 bytes you may have to be quite selective about which ones you
//  enable. If you have a 4K bootloader section (like this mega32) you can just turn everything
//  on. But otherwise here is your shopping list:
//
//                                                               Enabled by default
//       CRC_FLASH               96 bytes                     Yes
//       FAT32_SUPPORT       112 bytes                     Yes
//       CLUSTER_SUPPORT   442 bytes
//       ROOT_MULTISEC       150 bytes                     Yes
//       FLOOD_RAM              16 bytes
//       UART_DEBUG            374 bytes
//
//  See below for a complete description of each option. With items listed above enabled by
// default the bootloader builds to be 1764 bytes, easily fitting in a 2K BLS.

// define the following if the app images have been built using srec_cat and have a CRC-16 at
// the end of the padded image.
#define CRC_FLASH

// by default the code works for FAT16 (usually up to 2GB cards) to support both (ie include
// larger cards) define the following though it "costs" flash size for the bootloader
#define FAT32_SUPPORT

// if building for mega16 or other 16KB micro then the chances are the clusters on the card
// are also 16KB (32 sectors) so the AVRAPnnn.BIN file will fit within one cluster so there's
// no need for the bootloader to be able to understand FATs and follow FAT chains. If however
// this is built for a micro that might have 30K plus of code but stored on a card with 16KB clusters
// it will be necessary for the bootloader to be able to follow mutiple cluster files through the
// FAT so define this entry, though it "costs" more bootloader flash space-
//#define CLUSTER_SUPPORT

// the original code just read one sector of the root so AVRAP* had to be within the first 16
// directory entries. Make the following define to have it search then entire first cluster of the
// root instead (on FAT32 with 32KB clusters that will be 64 sectors so AVRAP can be in the
// first 1024 files now)
#define ROOT_MULTISEC

// for debug it's often useful to have RAM flooded with a known value so you can stop the debugger
// and see which regions have been written. Define the following to enable this and the following
// entries to say what byte to put in memory and, if necessary what the RAM start address is.
// The C header files have always had RAMEND but only recently do they also define RAMSTART
//#define FLOOD_RAM
#ifdef FLOOD_RAM
#ifndef RAMSTART
#define RAMSTART (0x0060)
#endif
#define FLOOD_VALUE 0x55
#endif

// define the following to pull in and enable the UART debug stuff. See uart.[hc]
#define UART_DEBUG

//##########################################################################
//#                                              END CONFIGURATION                                                #
//##########################################################################

#endif /* CONFIGURE_H_ */
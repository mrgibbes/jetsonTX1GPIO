/*
 * jetsonGPIO.h
 *
 * Copyright (c) 2015 JetsonHacks
 * www.jetsonhacks.com
 *
 * Based on Software by RidgeRun
 * Originally from:
 * https://developer.ridgerun.com/wiki/index.php/Gpio-int-test.c
 * Copyright (c) 2011, RidgeRun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JETSONGPIO_H_
#define JETSONGPIO_H_

 /****************************************************************
 * Constants
 ****************************************************************/
 
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

typedef unsigned int jetsonGPIO ;
typedef unsigned int pinDirection ;
typedef unsigned int pinValue ;

enum pinDirections {
	inputPin  = 0,
	outputPin = 1
} ;

enum pinValues {
    low = 0,
    high = 1,
    off = 0,  // synonym for things like lights
    on = 1
}  ;

enum jetsonGPIONumber {
  gpio57  =  57,    // J3A1 - Pin 50
	gpio160 = 160,	  // J3A2 - Pin 40	
	gpio161 = 161,    // J3A2 - Pin 43
	gpio162 = 162,    // J3A2 - Pin 46
	gpio163 = 163,    // J3A2 - Pin 49
	gpio164 = 164,    // J3A2 - Pin 52
	gpio165 = 165,    // J3A2 - Pin 55
	gpio166 = 166,    // J3A2 - Pin 58
}  ;

enum jetsonTX1GPIONumber {
  gpio36 = 36,      // J21 - Pin 32 - Unused - AO_DMIC_IN_CLK
  gpio37 = 37,      // J21 - Pin 16 - Unused - AO_DMIC_IN_DAT
  gpio38 = 38,      // J21 - Pin 13 - Bidir  - GPIO20/AUD_INT
  gpio63 = 63,      // J21 - Pin 33 - Bidir  - GPIO11_AP_WAKE_BT
  gpio184 = 184,    // J21 - Pin 18 - Input  - GPIO16_MDM_WAKE_AP
  gpio186 = 186,    // J21 - Pin 31 - Input  - GPIO9_MOTION_INT
  gpio187 = 187,    // J21 - Pin 37 - Output - GPIO8_ALS_PROX_INT
  gpio219 = 219,    // J21 - Pin 29 - Output - GPIO19_AUD_RST
} ;

enum jetsonTX2GPIONumber {
  gpio388 = 388,     // J21 - Pin 37 - GPIO8_ALS_PROX_INT
  gpio298 = 298,     // J21 - Pin 31 - GPIO9_MOTION_INT
  gpio389 = 389,     // J21 - Pin 33 - GPIO11_AP_WAKE_BT
  gpio481 = 481,     // J21 - Pin 18 - GPIO16_MDM_WAKE_AP
  gpio398 = 398,     // J21 - Pin 29 - GPIO19_AUD_RST
} ;

enum jetsonTXNanoGPIONumber {
  gpio216 = 216;  // Pin 07 - AUDIO_MCLK
  gpio50 = 50;    // Pin 11 - UART_2_RTS
  gpio79 = 79;    // Pin 12 - I2S_4_SCLK
  gpio14 = 14;    // Pin 13 - SPI_2_SCK
  gpio194 = 194;  // Pin 15 - LCD_TE
  gpio232 = 232;  // Pin 16 - SPI_2_CS1
  gpio15 = 15;    // Pin 18 - SPI_2_CS0
  gpio16 = 16;    // Pin 19 - SPI_1_MOSI
  gpio17 = 17;    // Pin 21 - SPI_1_MISO
  gpio13 = 13;    // Pin 22 - SPI_2_MISO
  gpio18 = 18;    // Pin 23 - SPI_1_SCK
  gpio19 = 19;    // Pin 24 - SPI_1_CS0
  gpio20 = 20;    // Pin 26 - SPI_1_CS1
  gpio149 = 149;  // Pin 29 - CAM_AF_EN
  gpio200 = 200;  // Pin 31 - GPIO_PZ0
  gpio168 = 168;  // Pin 32 - LCD_BL_PWM
  gpio38 = 38;    // Pin 33 - GPIO_PE6
  gpio76 = 76;    // Pin 35 - I2S_4_LRCK
  gpio51 = 51;    // Pin 26 - UART_2_CTS
  gpio12 = 12;    // Pin 37 - SPI_2_MOSI
  gpio77 = 77;    // Pin 38 - I2S_4_SDIN
  gpio78 = 78;    // Pin 40 - I2S_4_SDOUT
} ;

int gpioExport ( jetsonGPIO gpio ) ;
int gpioUnexport ( jetsonGPIO gpio ) ;
int gpioSetDirection ( jetsonGPIO, pinDirection out_flag ) ;
int gpioSetValue ( jetsonGPIO gpio, pinValue value ) ;
int gpioGetValue ( jetsonGPIO gpio, unsigned int *value ) ;
int gpioSetEdge ( jetsonGPIO gpio, char *edge ) ;
int gpioOpen ( jetsonGPIO gpio ) ;
int gpioClose ( int fileDescriptor ) ;
int gpioActiveLow ( jetsonGPIO gpio, unsigned int value ) ;

#endif // JETSONGPIO_H_

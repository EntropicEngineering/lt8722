//! @todo Review this file?  Document with Doxygen?  Time permitting...
/*
DC590B USB to Serial Controller

This file contains the routines to emulate the DC590B USB to Serial Converter. All commands
are supported except Uxxy the Write Port D bus. Added the 'D' delay ms command.
With this program, the Linduino can be used by the QuikEval program running on a PC
to communicate with QuikEval compatible demo boards.

The Kxy bit bang command uses the following pin mappings :
0-Linduino 2
1-Linduino 3
2-Linduino 4
3-Linduino 5
4-Linduino 6
5-Linduino 7


Copyright 2021(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "QuikEval_EEPROM.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LT_I2C.h"
#include "LT8722.h"
#include <Wire.h>
#include <SPI.h>

// timeouts
#define READ_TIMEOUT  20
#define MISO_TIMEOUT  1000

// recording mode constants
#define RECORDING_SIZE 50
const byte off = 0;
const byte playback = 1;

// serial mode constants
const byte spi_mode = 0;
const byte i2c_mode = 1;
const byte i2c_auxiliary_mode = 2;

// hex conversion constants
char hex_digits[16]=
{
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

// global variables
byte serial_mode = spi_mode;  // current serial mode
byte recording_mode = off;        // recording mode off
////////////////////////////////////////////////////
// CHANGED MAJOR VERSION to 2 FOR ENHANCED VERSION//
////////////////////////////////////////////////////
char id_string[51]="USBSPI,PIC,02,01,DC,DC590,----------------------\n\0"; // id string
char hex_to_byte_buffer[5]=
{
  '0', 'x', '0', '0', '\0'
};               // buffer for ASCII hex to byte conversion
char byte_to_hex_buffer[3]=
{
  '\0','\0','\0'
};                     // buffer for byte to ASCII hex conversion
char recording_buffer[RECORDING_SIZE]=
{
  '\0'
}; // buffer for saving recording loop
byte recording_index = 0;                // index to the recording buffer

uint8_t w_data_8[5];
uint8_t r_data_8[8];
float tempFloat = 0;
int analogPin = 4;
uint32_t i_soft = 0;


char get_char();

void output_voltage_dev(float output_voltage)
{
        typedef union
        {
          uint8_t   data_8[4];
          uint32_t  data_32;
          int32_t   data_32s;
        }dac_data;

      dac_data dac_data0;
           
      char tmp[8];
      //float writeVoltage=1.0;
      float float_temp=0.0;
      float_temp=output_voltage/16/1.25*16777216;
      dac_data0.data_32s=(int32_t)float_temp;
      
      w_data_8[0] =       0x4;           //SPI Address
      w_data_8[1] = dac_data0.data_8[3]; //WD3
      w_data_8[2] = dac_data0.data_8[2]; //WD2
      w_data_8[3] = dac_data0.data_8[1]; //WD1
      w_data_8[4] = dac_data0.data_8[0]; //WD0
      
      transfer_8722(DW, w_data_8, r_data_8);

}

// Function to write to a register on 8722 device
void write_reg(uint8_t reg, uint32_t value)
{
        typedef union
        {
          uint8_t   data_8[4];
          uint32_t  data_32;
          int32_t   data_32s;
        }dac_data;

      dac_data dac_data0;
           
      char tmp[8];
      dac_data0.data_32=value;
      
      w_data_8[0] =       reg;           //SPI Address
      w_data_8[1] = dac_data0.data_8[3]; //WD3
      w_data_8[2] = dac_data0.data_8[2]; //WD2
      w_data_8[3] = dac_data0.data_8[1]; //WD1
      w_data_8[4] = dac_data0.data_8[0]; //WD0

      transfer_8722(DW, w_data_8, r_data_8);
}

void byte_to_hex(byte value)
// convert a byte to two hex characters
{
  byte_to_hex_buffer[0]=hex_digits[value>>4];        // get upper nibble
  byte_to_hex_buffer[1]=hex_digits[(value & 0x0F)];  // get lower nibble
  byte_to_hex_buffer[2]='\0';                        // add NULL at end
}

byte read_hex()
// read 2 hex characters from the serial buffer and convert
// them to a byte
{
  byte data;
  hex_to_byte_buffer[2]=get_char();
  hex_to_byte_buffer[3]=get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return(data);
}

char get_char()
// get the next character either from the serial port
// or the recording buffer
{
  char command='\0';
  if (recording_mode != playback)
  {
    // read a command from the serial port
    while (Serial.available() <= 0);
    return(Serial.read());
  }
  else
  {
    // read a command from the recording buffer
    if (recording_index < RECORDING_SIZE)
    {
      command = recording_buffer[recording_index++];
      // disregard loop commands during playback
      if (command == 'w') command='\1';
      if (command == 't') command='\1';
      if (command == 'v') command='\1';
      if (command == 'u') command='\1';
    }
    else
      command = '\0';
    if (command == '\0')
    {
      recording_index = 0;
      recording_mode = off;
    }
    return(command);
  }
}
int i = 0;
unsigned char pseudo_reset = 0;

void setup()
// Setup the program
{
  digitalWrite(QUIKEVAL_GPIO, HIGH);
  digitalWrite(QUIKEVAL_CS, HIGH);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  pinMode(QUIKEVAL_GPIO, OUTPUT);
  pinMode(QUIKEVAL_CS, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(QUIKEVAL_GPIO, HIGH);

  Serial.begin(1000000);
  quikeval_SPI_init();
  quikeval_SPI_connect();   // Connect SPI to main data port

  quikeval_I2C_init();           // Configure the EEPROM I2C port for 100kHz SCK
  //Serial.print("hello\n");
  Serial.flush();
}

void loop()
{
  uint32_t iter=0;
  byte tx_data;
  byte rx_data;
  char command;
  int byte_count;
  long delay_count;
  byte w_ss_step = 0x01;
  char tmp[16];
  String inString = "";

  command = get_char();
  //Serial.print("MISO ");
  switch (command)
  {   
    case 'e':
      //Serial.print("e \n");        
      w_data_8[0] = read_hex(); //ADDR
      transfer_8722(DR, w_data_8, r_data_8);
      //print MISO data
      Serial.print("MISO:");
      for(uint8_t i = 0; i < 8; i++)
      {
        sprintf(tmp, "%02X",r_data_8[i]);
        //Serial.print(i);
        //Serial.print(" ");       
        Serial.print(tmp);
        //Serial.print(" ");       
      }
      Serial.print("\r\n");
      break; 
           
    //DW command
    case 'E':
      //char tmp[16];
      w_data_8[0] = read_hex(); //ADDR
      w_data_8[1] = read_hex(); //WD3
      w_data_8[2] = read_hex(); //WD2
      w_data_8[3] = read_hex(); //WD1
      w_data_8[4] = read_hex(); //WD0
      transfer_8722(DW, w_data_8, r_data_8);
      //print MISO data
      Serial.print("MISO:");
      for(uint8_t i = 0; i < 8; i++)
      {
        sprintf(tmp, "%02X",r_data_8[i]);      
        Serial.print(tmp);
        
        //Serial.print(r_data_8[i], HEX);
        //Serial.print("");
      }
      Serial.print("\r\n");
      break;
    
      /////////////////////////////////////////////////////////////
      //Soft start command
      case 'U':
      Serial.print("U \n");
      //reset dev 0
      write_reg(0x0,0x00004000);
      delayMicroseconds(100);
      write_reg(0x0,0x0000120D);
      delayMicroseconds(100);
      ///////////////////////////////////
      //Clear the SPIS_Status register
      write_reg(0x1,0x00000000);
      delayMicroseconds(100);
      write_reg(0x0,0x0000120D);
      delayMicroseconds(100);
      ///////////////////////////////////////
      // Ramp the LDR output from 0V to VDD/2
      ///////////////////////////////////////
      for(i_soft = 0xFFD00000; i_soft < 0xFFFF0000; i_soft = i_soft + 0x00010000)
      {
        write_reg(0x4,i_soft);
        delayMicroseconds(50);
       }
      // Finish off the ramp      
      write_reg(0x4,0xFFFF0000);
      delayMicroseconds(100);        
      write_reg(0x4,0x00000000);
      // LDR should now be at Vin/2
      ///////////////////////////////////
      //Clear the SPIS_Status register
      write_reg(0x1,0x00000000);
      delayMicroseconds(100);
      ///////////////////////////////////
      //Turn on the switcher
      write_reg(0x0,0x0000120F);     
      ///////////////////////////////////
      Serial.print("The LT8722 has just been soft-started\n");          
      break;       
      /////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////
      //Power down command
      case 'u':
      Serial.print("u \n");
      ///////////////////////////////////
      //Turn off the switcher
      write_reg(0x0,0x0000120D);   
      ///////////////////////////////////
      //Turn off the LDR stage also
      write_reg(0x0,0x0000120C);     
      
      Serial.print("The LT8722 has just had its buck stage and LDR stage powered off\n");   
      break;       
      /////////////////////////////////////////////////////////////

      /////////////////////////////////////////////////////////////
      //DC command
      case 'D':
      iter=0;
      Serial.print("D \n");
      output_voltage_dev(0.0);
      Serial.print("LT8722 differential output should be 0V now \n");
      break; 

      /////////////////////////////////////////////////////////////
      //DC command
      case 'V':
      iter=0;
      //Serial.print("V \n");
      //Serial.print("In DC routine \n");

      delay(1);
      while(Serial.available() > 0)
        {
          int inChar = Serial.read();
          inString += (char)inChar;
        }
      tempFloat = inString.toFloat();
      output_voltage_dev(tempFloat/0.969);
      
      delayMicroseconds(100);  
      Serial.flush();  

      break; 

      case 't':
      // Output Vtemp on the LT8722 Aout pin
      write_reg(0x7,0x00000048);
      break; 
      
      default:
    // statements
       break;
  }
}

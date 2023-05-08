/*
Copyright 2018(c) Analog Devices, Inc.

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

 * Created by Benjamin Riggs on 5/5/23.
 *
 * Copyright (c) 2023 Entropic Engineering. All rights reserved.
 */

#include "lt8722.h"

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SPI_NODE DT_NODELABEL( spi1 )

#define LT7822_SPI_MODE			0
#define LT7822_DATA_XFER_SIZE	8
#define LT7822_STATUS_XFER_SIZE 4

static const uint8_t CRC_8_TABLE[256] = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31,
	0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9,
	0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1,
	0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE,
	0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16,
	0x03, 0x04, 0x0D, 0x0A, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80,
	0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8,
	0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10,
	0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F,
	0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13, 0xAE, 0xA9, 0xA0, 0xA7,
	0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF,
	0xFA, 0xFD, 0xF4, 0xF3 };

static struct spi_dt_spec spec = {
	.bus = DEVICE_DT_GET( SPI_NODE ),
	.config =
		SPI_CONFIG_DT( SPI_NODE, SPI_OP_MODE_MASTER | SPI_WORD_SET( 8 ), 0 ) };
static uint8_t mosi_buffer[LT7822_DATA_XFER_SIZE] = { 0 };
static uint8_t miso_buffer[LT7822_DATA_XFER_SIZE] = { 0 };

static uint8_t calc_crc( uint8_t* buffer, size_t len )
{
	uint8_t result = 0;
	for ( uint8_t i = 0; i < len; i++ )
	{
		result = CRC_8_TABLE[( result ^ buffer[i] )];
	}
	return result;
}

lt7822_err_t lt7822_spi_ready( bool* ready )
{
	*ready = spi_is_ready_dt( &spec );
	return 0;
}

static lt7822_err_t transceive( uint8_t ack, uint8_t crc )
{
	static struct spi_buf	  tx_buf = { .buf = mosi_buffer, .len = 0 };
	static struct spi_buf	  rx_buf = { .buf = miso_buffer, .len = 0 };
	static struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	static struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };
	int						  r;

	r = spi_transceive_dt( &spec, &tx_set, &rx_set );
	if ( r )
	{
		printk( "Transceive error: %d\n", -r );
		return -r;
	}

	r = miso_buffer[ack];
	if ( LT7822_ACK != r )
	{
		printk( "LT7822 ACK error: %02x\n", r );
		return r ? r : 0xFF; /* Stuck at zero is error */
	}

	r = calc_crc( miso_buffer, crc );
	if ( r != miso_buffer[crc] )
	{
		printk(
			"CRC error: calculated %02x, received %02x\n", r,
			miso_buffer[crc] );
		return 0xFF;
	}
	return 0;
}

lt7822_err_t lt7822_spi_transact(
	enum lt7822_command		  kind,
	lt7822_status_register_t* status,
	uint32_t*				  data,
	enum lt7822_spi_address	  address )
{
	int err = 0;

	switch ( kind )
	{
	case LT7822_STATUS_ACQUISITION:
		tx_buf.len = rx_buf.len = LT7822_STATUS_XFER_SIZE;
		memset( miso_buffer, 0, LT7822_STATUS_XFER_SIZE );
		mosi_buffer[0] = LT7822_SQ;
		mosi_buffer[1] = address;
		mosi_buffer[2] = calc_crc( mosi_buffer, 2 );

		err = transceive( 3, 2 );
		break;
	case LT7822_DATA_WRITE:
		tx_buf.len = rx_buf.len = LT7822_DATA_XFER_SIZE;
		memset( miso_buffer, 0, LT7822_DATA_XFER_SIZE );
		mosi_buffer[0] = LT7822_DW;
		mosi_buffer[1] = address;
		// LT8277 spi is big-endian
		mosi_buffer[2] = ( (uint8_t*) data )[3];
		mosi_buffer[3] = ( (uint8_t*) data )[2];
		mosi_buffer[4] = ( (uint8_t*) data )[1];
		mosi_buffer[5] = ( (uint8_t*) data )[0];
		mosi_buffer[6] = calc_crc( mosi_buffer, 6 );

		err = transceive( 7, 2 );
		break;
	case LT7822_DATA_READ:
		tx_buf.len = rx_buf.len = LT7822_DATA_XFER_SIZE;
		memset( miso_buffer, 0, LT7822_DATA_XFER_SIZE );
		mosi_buffer[0] = LT7822_DR;
		mosi_buffer[1] = address;
		mosi_buffer[2] = calc_crc( mosi_buffer, 2 );

		err = transceive( 7, 6 );
		if ( err )
			break;

		// LT8277 spi is big-endian
		( (uint8_t*) data )[0] = miso_buffer[5];
		( (uint8_t*) data )[1] = miso_buffer[4];
		( (uint8_t*) data )[2] = miso_buffer[3];
		( (uint8_t*) data )[3] = miso_buffer[2];
		break;
	}

	// LT8277 spi is big-endian
	( (uint8_t*) status )[0] = miso_buffer[1];
	( (uint8_t*) status )[1] = miso_buffer[0];
	return err;
}

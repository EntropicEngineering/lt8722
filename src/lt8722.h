/*
 * Created by Benjamin Riggs on 5/5/23.
 *
 * Copyright (c) 2023 Entropic Engineering. All rights reserved.
 */

#ifndef LT7822_LT8722_H
#define LT7822_LT8722_H

#include <stdint.h>

#define LT7822_SPI_MODE			 0
#define LT7822_MAX_TRANSFER_SIZE 8

enum lt7822_command {
	LT7822_STATUS_ACQUISITION = 0x00,
	LT7822_SQ				  = LT7822_STATUS_ACQUISITION,
	LT7822_DATA_WRITE		  = 0x02,
	LT7822_DW				  = LT7822_DATA_WRITE,
	LT7822_DATA_READ		  = 0x04,
	LT7822_DR				  = LT7822_DATA_READ
};

enum lt7822_spi_address {
	LT7822_SPIS_COMMAND	  = 0x00,
	LT7822_SPIS_STATUS	  = 0x01,
	LT7822_SPIS_DAC_ILIMN = 0x02,
	LT7822_SPIS_DAC_ILIMP = 0x03,
	LT7822_SPIS_DAC		  = 0x04,
	LT7822_SPIS_OV_CLAMP  = 0x05,
	LT7822_SPIS_UV_CLAMP  = 0x06,
	LT7822_SPIS_AMUX	  = 0x07,
};

enum lt7822_acknowledge {
	LT7822_ACK			 = 0xA5,
	LT7822_NACK			 = 0xC3,
	LT7822_BAD_REGISTER	 = 0x0F,
	LT7822_STUCK_AT_ZERO = 0x00,
	LT7822_STUCK_AT_ONE	 = 0xFF
};

typedef struct {
	/* Ignored */			   : 13;
	LT7822_PWR_LIM			   : 4;
	LT7822_SPI_RST			   : 1;
	LT7822_SW_VC_INT		   : 3;
	/* Unused, must be zero */ : 1;
	LT7822_VCC_VREG			   : 1;
	LT7822_SYS_DC			   : 2;
	LT7822_SW_FRQ_ADJ		   : 2;
	LT7822_SW_FRQ_SET		   : 3;
	LT7822_SWEN_REQ			   : 1;
	LT7822_ENABLE_REQ		   : 1;
} lt7822_command_register_t;

typedef struct {
	/* Ignored */		: 21;
	LT7822_V2P5_UVLO	: 1;
	LT7822_CP_UVLO		: 1;
	LT7822_VDDIO_UVLO	: 1;
	LT7822_VCC_UVLO		: 1;
	LT7822_TSD			: 1;
	LT7822_OVER_CURRENT : 1;
	LT7822_POR_OCC		: 1;
	LT7822_MIN_OT		: 1;
	LT7822_SRVO_PLIM	: 1;
	LT7822_SRVO_ILIM	: 1;
	LT7822_SWEN			: 1;
} lt7822_status_register_t;

typedef size_t lt7822_err_t;

/**
 * Determine if SPI is ready and idle.
 *
 * @param[out] busy 	False if transaction in flight or error
 * @return				0 or error
 */
lt7822_err_t lt7822_spi_idle(bool* busy);

/**
 * Begin SPI transaction.
 *
 * Check for idle & no error before calling.
 *
 * @param[in] kind 			Type of transaction
 * @param[in|out] buffer 	Results buffer if SQ or DR, data buffer if DW
 * @return					0 or error
 */
lt7822_err_t lt7822_spi_transact(enum lt7822_command kind, uint8_t* buffer);

void lt7822_init(void);

#endif	// LT7822_LT8722_H

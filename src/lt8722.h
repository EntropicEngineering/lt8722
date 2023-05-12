/*
 * Created by Benjamin Riggs on 5/5/23.
 *
 * Copyright (c) 2023 Entropic Engineering. All rights reserved.
 */

#ifndef LT8722_LT8722_H
#define LT8722_LT8722_H

#include <zephyr/sys/util.h>

enum lt8722_command {
	LT8722_STATUS_ACQUISITION = 0x00,
	LT8722_SQ				  = LT8722_STATUS_ACQUISITION,
	LT8722_DATA_WRITE		  = 0x02,
	LT8722_DW				  = LT8722_DATA_WRITE,
	LT8722_DATA_READ		  = 0x04,
	LT8722_DR				  = LT8722_DATA_READ
};

enum lt8722_spi_address {
	LT8722_SPIS_COMMAND	  = 0x00,
	LT8722_SPIS_STATUS	  = 0x01,
	LT8722_SPIS_DAC_ILIMN = 0x02,
	LT8722_SPIS_DAC_ILIMP = 0x03,
	LT8722_SPIS_DAC		  = 0x04,
	LT8722_SPIS_OV_CLAMP  = 0x05,
	LT8722_SPIS_UV_CLAMP  = 0x06,
	LT8722_SPIS_AMUX	  = 0x07,
};

enum lt8722_acknowledge {
	LT8722_ACK			 = 0xA5,
	LT8722_NACK			 = 0xC3,
	LT8722_BAD_REGISTER	 = 0x0F,
	LT8722_STUCK_AT_ZERO = 0x00,
	LT8722_STUCK_AT_ONE	 = 0xFF
};

typedef struct {
	uint32_t /* Ignored */ : 13;
	uint32_t PWR_LIM	   : 4;
	uint32_t SPI_RST	   : 1;
	uint32_t SW_VC_INT	   : 3;
	uint32_t /* Unused */  : 1;	 // Must be zero
	uint32_t VCC_VREG	   : 1;
	uint32_t SYS_DC		   : 2;
	uint32_t SW_FRQ_ADJ	   : 2;
	uint32_t SW_FRQ_SET	   : 3;
	uint32_t SWEN_REQ	   : 1;
	uint32_t ENABLE_REQ	   : 1;
} lt8722_command_register_t;

typedef union {
	struct {
		uint32_t /* Ignored	*/ : 21;
		uint32_t V2P5_UVLO	   : 1;
		uint32_t CP_UVLO	   : 1;
		uint32_t VDDIO_UVLO	   : 1;
		uint32_t VCC_UVLO	   : 1;
		uint32_t TSD		   : 1;
		uint32_t OVER_CURRENT  : 1;
		uint32_t POR_OCC	   : 1;
		uint32_t MIN_OT		   : 1;
		uint32_t SRVO_PLIM	   : 1;
		uint32_t SRVO_ILIM	   : 1;
		uint32_t SWEN		   : 1;
	};
	uint32_t bits;
} lt8722_status_register_t;

#define LT8722_STATUS_RESET_MASK BIT( 4 )
#define LT8722_STATUS_FAULT_MASK GENMASK( 10, 5 )

#define LT8722_DAC_MIN	((int32_t) -(1 << 24))
#define LT8722_DAC_MAX	((int32_t) (1 << 24) - 1)

/**
 * Ensure device drivers are ready.
 *
 * @return				0 or error
 */
int lt8722_init( void );

/**
 * Begin SPI transaction.
 *
 * Check for ready & no error before calling.
 *
 * @param[in] kind 			Type of transaction
 * @param[out] status		Status reported by device
 * @param[in|out]data 		Output if DR, input if DW, ignored if SQ
 * @param[in] address		Address for DR or DW, ignored if SQ
 * @return					0 or error
 */
int lt8722_spi_transact(
	enum lt8722_command		  kind,
	lt8722_status_register_t* status,
	uint32_t*				  data,
	enum lt8722_spi_address	  address );

/**
 * Start the output while limiting inrush current
 *
 * V_OUT = 16 * dac_init * 2.5 V / 2**25
 * V_OUT is clamped between +V_IN to almost -V_IN.
 *
 * @param[in] dac_init 	Initial value to set DAC
 * 						Must be in range LT8722_DAC_MIN to LT8722_DAC_MAX
 * @return				0 or error
 */
int lt8722_soft_start( int32_t dac_init );

#endif	// LT8722_LT8722_H

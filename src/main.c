
#include "lt8722.h"

#include <cstdint>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

/**
 * First, apply proper VIN and VDDIO voltages to the LT8722.
 *
 * Second, enable the VCC LDO and other LT8722 circuitry by raising the EN pin
 * above the 0.74V threshold and writing the ENABLE_REQ bit to a 1.
 *
 * Third, configure the output voltage control DAC (SPIS_DAC) to 0xFF000000.
 * This code will force the LDR pin to GND when the linear power stage is
 * later enabled.
 *
 * Fourth, write all SPIS_STATUS registers to 0. This clears all faults and
 * allows the linear power stage to be enabled. Due to the actions in the
 * prior step, when the linear power stage turns on in this step, the output
 * load will be discharged to GND. Pause between this step and the next for
 * ~1 ms to allow any prebiased out- put condition to dissipate.
 *
 * Fifth, ramp the output voltage control DAC (SPIS_DAC) from code 0xFF000000
 * to code 0x00000000 in a controlled manner so that the linear driver output
 * (LDR) ramps from GND to VIN/2. During this ramping period, both the PWM
 * driver output (SFB) and linear driver output (LDR) move together to VIN/2.
 * The ramp time for this controlled movement to VIN/2 should be a minimum of
 * 5 ms.
 *
 * Sixth, enable the PWM switching behavior by raising the SWEN pin above the
 * 1.2V (typ) threshold and writing the SWEN_REQ bit to a 1. With both output
 * terminals at VIN/2, the inrush current through the output load is greatly
 * minimized. After the PWM driver switching activity is enabled, keep the
 * output voltage control DAC (SPIS_DAC) code unchanged for a minimum of 160μs.
 */
int startup_sequence( void )
{
	lt8722_status_register_t status;
	uint32_t				 reg = 0;

	/// Second
	// TODO: Set EN high
	( (lt8722_command_register_t*) &reg )->ENABLE_REQ = 1;

	err = lt8722_spi_transact(
		LT8722_DATA_WRITE, &status, &reg, LT8722_SPIS_COMMAND );
	// TODO: Check status
	if ( err )
	{
		printk(
			"Step 2: failed to set ENABLE_REQ, err: %d, status: %02x\n", err,
			(uint32_t) status );
		return err;
	}

	/// Third
	reg = 0xFF000000;
	err = lt8722_spi_transact(
		LT8722_DATA_WRITE, &status, &reg, LT8722_SPIS_DAC );
	// TODO: Check status
	if ( err )
	{
		printk(
			"Step 3: failed to set DAC, err: %d, status: %02x\n", err,
			(uint32_t) status );
		return err;
	}

	/// Fourth
	reg = 0;
	err = lt8722_spi_transact(
		LT8722_DATA_WRITE, &status, &reg, LT8722_SPIS_STATUS );
	// TODO: Check status
	if ( err )
	{
		printk(
			"Step 4: failed to clear SPIS_STATUS, err: %d, status: %02x\n", err,
			(uint32_t) status );
		return err;
	}
	(void) k_sleep( K_MSEC(1));

	/// Fifth
	// 256 steps over 5+ ms
#define TIMEOUT K_USEC( DIV_ROUND_UP( 5000, 256 ) )
	// reg will overflow to 0x0, only works for uint32.
	for ( reg = 0xFF000000; reg > 0; reg += 0x10000 )
	{
		err = lt8722_spi_transact(
			LT8722_DATA_WRITE, &status, &reg, LT8722_SPIS_DAC );
		// TODO: Check status
		if ( err )
		{
			printk(
				"Step 5: failed to set DAC, err: %d, status: %02x\n", err,
				(uint32_t) status );
			return err;
		}
		(void) k_sleep( TIMEOUT );
	}

	/// Sixth
	// TODO: Pull SWEN high
	( (lt8722_command_register_t*) &reg )->SWEN_REQ = 1;

	err = lt8722_spi_transact(
		LT8722_DATA_WRITE, &status, &reg, LT8722_SPIS_COMMAND );
	// TODO: Check status
	if ( err )
	{
		printk(
			"Step 6: failed to set SWEN_REQ, err: %d, status: %02x\n", err,
			(uint32_t) status );
		return err;
	}
	(void) k_sleep( K_USEC( 200 ) );  // Minimum of 160, 32 x 5 µs


	// TODO: Set switching freq to 3 MHz
	// TODO: Set SW_VC_INT to 0b011
	return 0;
}

/**
 * Finally, the output voltage control DAC (SPIS_DAC) code can be stepped in
 * a controlled manner to the desired code. The LDR and SFB outputs will
 * begin to diverge from one another until the desired differential voltage
 * is developed across the output load, the differential output voltage
 * reaches the preset voltage limit, or the output current reaches the preset
 * current limit.
 */
uint32_t dac_ramp_delay_us = 20;  // TODO: expose to shell
int		 set_dac( uint32_t value )
{
	uint32_t
}

int main( void )
{
	uint32_t err;
	bool	 ready;

	err = lt8722_spi_ready( &ready );
	if ( err || ! ready )
	{
		printk( "SPI device not ready\n" );
		return err;
	}

	err = startup_sequence();

	return err;
}

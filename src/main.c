
#include "lt8722.h"

#include <cstdint>
#include <zephyr/sys/printk.h>

/**
 * First, apply proper VIN and VDDIO voltages to the LT8722.
 *
 * Second, enable the VCC LDO and other LT8722 circuitry by raising the EN pin
 * above the 0.74V threshold and writing the ENABLE_REQ bit to a 1.
 *
 * Third, configure the output voltage control DAC (SPIS_ DAC) to 0xFF000000.
 * This code will force the LDR pin to GND when the linear power stage is
 * later enabled.
 *
 * Fourth, write all SPIS_STATUS registers to 0. This clears all faults and
 * allows the linear power stage to be enabled. Due to the actions in the
 * prior step, when the linear power stage turns on in this step, the output
 * load will be discharged to GND. Pause between this step and the next for
 * ~1ms to allow any prebiased out- put condition to dissipate.
 *
 * Fifth, ramp the output voltage control DAC (SPIS_DAC) from code 0xFF000000
 * to code 0x00000000 in a con- trolled manner so that the linear driver
 * output (LDR) ramps from GND to VIN/2. During this ramping period, both the
 * PWM driver output (SFB) and linear driver output (LDR) move together to
 * VIN/2. The ramp time for this controlled movement to VIN/2 should be a
 * mini- mum of 5ms.
 *
 * Sixth, enable the PWM switching behavior by raising the SWEN pin above the
 * 1.2V (typ) threshold and writ- ing the SWEN_REQ bit to a 1. With both
 * output termi- nals at VIN/2, the inrush current through the output load is
 * greatly minimized. After the PWM driver switch- ing activity is enabled,
 * keep the output voltage control DAC (SPIS_DAC) code unchanged for a
 * minimum of 160μs.
 *
 * Finally, the output voltage control DAC (SPIS_DAC) code can be stepped in
 * a controlled manner to the desired code. The LDR and SFB outputs will
 * begin to diverge from one another until the desired differential voltage
 * is developed across the output load, the differ- ential output voltage
 * reaches the preset voltage limit, or the output current reaches the preset
 * current limit.
 */
int startup_sequence( void )
{
	lt7822_status_register_t status;
	uint32_t				 reg = 0;

	err = lt7822_spi_ready( &ready );
	if ( err )
	{
		printk( "SPI device not ready\n" );
		return err;
	}

	// Second
	// TODO: Set EN high
	( (lt7822_command_register_t*) &reg )->ENABLE_REQ = 1;

	err = lt7822_spi_transact(
		LT7822_DATA_WRITE, &status, &reg, LT7822_SPIS_COMMAND );
	// TODO: Check status
	if ( err )
	{
		printk(
			"Step 2: failed to set ENABLE_REQ, err: %d, status: %02x\n", err,
			(uint32_t) status );
		return err;
	}

	// Third
	reg = 0xFF000000;
	err = lt7822_spi_transact(
		LT7822_DATA_WRITE, &status, &reg, LT7822_SPIS_DAC );
	// TODO: Check status
	if ( err )
	{
		printk(
			"Step 3: failed to set DAC, err: %d, status: %02x\n", err,
			(uint32_t) status );
		return err;
	}

	// Fourth
	reg = 0;
	err = lt7822_spi_transact(
		LT7822_DATA_WRITE, &status, &reg, LT7822_SPIS_STATUS );
	// TODO: Check status
	if ( err )
	{
		printk(
			"Step 4: failed to clear SPIS_STATUS, err: %d, status: %02x\n", err,
			(uint32_t) status );
		return err;
	}
	// TODO: Wait ~1ms

	// Fifth
	

	return 0;
}

int main( void )
{
	uint32_t				 err;
	bool					 ready;
	uint32_t				 data = 0;
	lt7822_status_register_t status;

	err = lt7822_spi_ready( &ready );
	if ( err )
		return err;

	err = lt7822_spi_transact(
		LT7822_DATA_READ, &status, &data, LT7822_SPIS_DAC );

	return err;
}

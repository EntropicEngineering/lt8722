
#include "lt8722.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

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
	//	uint32_t
}

int main( void )
{
	uint32_t err;

	err = lt8722_init();
	if ( err )
	{
		printk( "Init failed\n" );
		return err;
	}

	err = startup_sequence();

	return err;
}

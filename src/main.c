
#include "lt8722.h"

#include <zephyr/sys/printk.h>

int main( void )
{
	uint32_t err;

	err = lt8722_init();
	if ( err )
	{
		printk( "Init failed\n" );
		return err;
	}

	return err;
}

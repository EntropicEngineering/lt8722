
#include "lt8722.h"

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

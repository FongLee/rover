#include <stdio.h>

#include "ap_location.h"

int main()
{
	struct location loc1;
	struct location loc2;

	loc1.lat = 24 * 1e7;
	loc1.lng = 13 * 1e7;

	loc2.lat = 26 * 1e7;
	loc2.lng = 12 * 1e7;

	printf("%f\n", get_distance(&loc1, &loc2));

}

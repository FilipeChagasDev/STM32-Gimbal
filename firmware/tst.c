#include <stdio.h>
#include "Inc/ftoa.h"

int main()
{
	char a[100];
	ftoa(0.5, a, 2);
	printf(a);
	return 0;
}

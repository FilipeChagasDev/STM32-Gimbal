#ifndef FTOA_H
#define FTOA_H

#include <string.h>

/*
 * Implementation obtained on the following page:
 * https://www.microchip.com/forums/m183763.aspx
 * 
 * The STM32CubeMX generated HAL does not have the ANSI C ftoa function.
 * That function was renamed from ftoa to float_to_string to avoid conflicts.
 */

/**************************************************
 *
 *    float_to_string - converts float to string
 *
 ***************************************************
 *
 *    This is a simple implemetation with rigid
 *    parameters:
 *            - Buffer must be 8 chars long
 *            - 3 digits precision max
 *            - absolute range is -524,287 to 524,287 
 *            - resolution (epsilon) is 0.125 and
 *              always rounds down
 **************************************************/

void float_to_string(float Value, char* Buffer);

#endif
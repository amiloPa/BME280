/*
 * common_var.c
 *
 *  Created on: 22.10.2020
 *      Author: Piotr
 */


#include "common_var.h"

/****************************************************************************/
/*      function for calculating abs value in int format of variable        */
/****************************************************************************/
int my_abs(int x)
{
    return x < 0 ? -x : x;
}

/****************************************************************************/
/*      function for calculating abs value in uint32 format of variable	    */
/****************************************************************************/
uint32_t my_abs_uint(uint32_t x)
{
    return x < 0 ? -x : x;
}

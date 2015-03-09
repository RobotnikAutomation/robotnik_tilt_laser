/*
 * main.cpp
 *
 *  Created on: 2013. 12. 10.
 *      Author: chase
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <string.h>
#include <termios.h>
#include <math.h>
#include "dynamixel.h"

using namespace DXL_PRO;

Dynamixel DXL("/dev/ttyUSB0");

int main()
{
    
    int i;

    Dynamixel DXL("/dev/ttyUSB0");
    //BulkRead bulkread(&DXL);


    // Open device
    if( DXL.Connect() == 0 )
    {
        printf( "Failed to open USB2Dynamixel!\n" );
        return 0;
    }
    else
        printf( "Succeed to open USB2Dynamixel!\n" );

	//Close port of USB2DXL
    DXL.Disconnect();
   

    return 0;
}





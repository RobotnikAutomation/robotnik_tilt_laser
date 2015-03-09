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

int _getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}


// Control table address
#define TORQUE_ENABLE      562
#define GOAL_POSITION      596


// Defulat setting
#define DEFAULT_BAUDNUM     1 // 57,600
#define CONTROL_PERIOD      (10) // msec (Large value is more slow)


void PrintCommStatus(int CommStatus);
void PrintErrorCode(int ErrorCode);

int main()
{
    //int GoalPos = 125500; // 9-o-clock position for 54-series
    //int GoalPos = 75938; // 3-o-clock position for 42-series
    // int GoalPos = -85000; // 3-o-clock position for 42-series  // -31000 center
    int GoalPos = 10000; // 3-o-clock position for 42-series

    Dynamixel DXL("/dev/ttyUSB0");

    // Open device
    if( DXL.Connect() == 0 )
    {
        printf( "Failed to open USB2Dynamixel!\n" );
        printf( "Press any key to terminate...\n" );
        _getch();
        return 0;
    }
    else
        printf( "USB2Dynamixel initialized!\n" );


    if(DXL.SetBaudrate(DEFAULT_BAUDNUM) == true)
    {
    	printf( "Succeed to set USB2Dynamixel baud rate to 57600 bps!\n" );
    }
    else
    {
        printf( "Failed to change the baudrate!\n" );
        printf( "Press any key to terminate...\n" );
        _getch();
        return 0;
    }
    // turn Torque Enable (562) on
    DXL.WriteByte(1, TORQUE_ENABLE, 1, 0);
    usleep(CONTROL_PERIOD*100);
    // set initial Goal Position to 0
    DXL.WriteDWord(1, GOAL_POSITION, 0, 0);
    usleep(CONTROL_PERIOD*500);
    printf( "ready...\n" );

    while(1)
    {
        printf( "Press any key to continue (ESC to end program)\n" );
        if(_getch() == 0x1b)
            break;
        DXL.WriteDWord(1, GOAL_POSITION, GoalPos, 0);   //move to 9-o-clock position     
	printf( "moving...\n" );	
	usleep(CONTROL_PERIOD*500);
       
    }
    //DIsconnect USB2DXL before ending program
    DXL.Disconnect();

    return 0;
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
        printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
        break;

    case COMM_TXERROR:
        printf("COMM_TXERROR: Incorrect instruction packet!\n");
        break;

    case COMM_RXFAIL:
        printf("COMM_RXFAIL: Failed get status packet from device!\n");
        break;

    case COMM_RXWAITING:
        printf("COMM_RXWAITING: Now recieving status packet!\n");
        break;

    case COMM_RXTIMEOUT:
        printf("COMM_RXTIMEOUT: There is no status packet!\n");
        break;

    case COMM_RXCORRUPT:
        printf("COMM_RXCORRUPT: Incorrect status packet!\n");
        break;

    default:
        printf("This is unknown error code!\n");
        break;
    }
}

// Print error bit of status packet
void PrintErrorCode(int ErrorCode)
{
    if(ErrorCode & ERRBIT_VOLTAGE)
        printf("Input voltage error!\n");

    if(ErrorCode & ERRBIT_ANGLE)
        printf("Angle limit error!\n");

    if(ErrorCode & ERRBIT_OVERHEAT)
        printf("Overheat error!\n");

    if(ErrorCode & ERRBIT_RANGE)
        printf("Out of range error!\n");

    if(ErrorCode & ERRBIT_CHECKSUM)
        printf("Checksum error!\n");

    if(ErrorCode & ERRBIT_OVERLOAD)
        printf("Overload error!\n");

    if(ErrorCode & ERRBIT_INSTRUCTION)
        printf("Instruction code error!\n");
}



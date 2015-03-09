/*
 * main.cpp
 *
 *  Created on: 2013. 1. 3.
 *      Author: zerom
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
#include "bulkread.h"

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
#define P_PRESENT_POSITION_LL   611
#define P_PRESENT_VELOCITY_LL   615

// Defulat setting
#define DEFAULT_BAUDNUM     1 // 1M
#define NUM_ACTUATOR        2 // Number of actuator

void PrintCommStatus(int CommStatus);
void PrintErrorCode(int ErrorCode);

int main()
{
    int id[NUM_ACTUATOR]    = {1, 2};
    int addr[NUM_ACTUATOR]  = {P_PRESENT_POSITION_LL, P_PRESENT_POSITION_LL};
    int len[NUM_ACTUATOR]   = {4, 4};
    int i;
    int read_val;
    int result;

    Dynamixel DXL("/dev/ttyUSB0");
    BulkRead bulkread(&DXL);


    // Open device
    if( DXL.Connect() == 0 )
    {
        printf( "Failed to open USB2Dynamixel!\n" );
        printf( "Press any key to terminate...\n" );
        _getch();
        return 0;
    }
    else
        printf( "Succeed to open USB2Dynamixel!\n" );


    if(DXL.SetBaudrate(DEFAULT_BAUDNUM) == true)
    {
    	printf( "Succeed to change the baudrate!\n" );
    }
    else
    {
        printf( "Failed to change the baudrate!\n" );
        printf( "Press any key to terminate...\n" );
        _getch();
        return 0;
    }



    // Make bulk read packet
    for(i = 0; i < NUM_ACTUATOR; i++)
    {
        bulkread.AddBulkReadData(id[i], addr[i], len[i]);
    }

    while(1)
    {
        printf( "\n" );
        printf( "Press any key to continue!(press ESC to quit)\n" );
        if(_getch() == 0x1b)
            break;

        printf( "\n" );

        result = bulkread.SendTxPacket();
        if( result != COMM_RXSUCCESS)
        	PrintCommStatus(result);

        // Print Read Data
        for(i = 0; i < NUM_ACTUATOR; i++)
        {
        	bulkread.GetDwordValue(id[i], addr[i], (long*)&read_val);
        	printf("ID : %d,  READ_VALUE : %d\n", id[i], read_val);
        }

    }

    DXL.Disconnect();
    printf( "Press any key to terminate...\n" );
    _getch();

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



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

#define PI                  3.141592f

// Control table address
#define P_TORQUE_ENABLE      562
#define P_GOAL_POSITION_LL   596

// Defulat setting
#define DEFAULT_BAUDNUM     1 // 57,600
#define NUM_ACTUATOR        2 // Number of actuator
#define STEP_THETA          (PI / 100.0f) // Large value is more fast
#define CONTROL_PERIOD      (10) // msec (Large value is more slow)


void PrintCommStatus(int CommStatus);
void PrintErrorCode(int ErrorCode);

int main()
{
    int id[NUM_ACTUATOR];
    float phase[NUM_ACTUATOR];
    float theta = 0;
    int AmpPos = 150000;
    int GoalPos;
    int i;
    int result;

    // Initialize id and phase
    for( i=0; i < NUM_ACTUATOR; i++ )
    {
        id[i] = i+1;
        phase[i] = 2*PI * (float)i / (float)NUM_ACTUATOR;
    }

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

    unsigned char param[NUM_ACTUATOR*(1+4)];

    DXL.WriteByte(1, P_TORQUE_ENABLE, 1, 0);
    DXL.WriteByte(2, P_TORQUE_ENABLE, 1, 0);

    while(1)
    {
        printf( "Press any key to continue!(press ESC to quit)\n" );
        if(_getch() == 0x1b)
            break;

        theta = 0;
        do
        {
            // Make syncwrite packet
            for(i = 0; i < NUM_ACTUATOR; i++)
            {
                GoalPos = (int)((sin(theta+phase[i])) * (double)AmpPos);

                param[i*(1+4)+0] = (unsigned char)id[i];
                param[i*(1+4)+1] = DXL_LOBYTE(DXL_LOWORD(GoalPos));
                param[i*(1+4)+2] = DXL_HIBYTE(DXL_LOWORD(GoalPos));
                param[i*(1+4)+3] = DXL_LOBYTE(DXL_HIWORD(GoalPos));
                param[i*(1+4)+4] = DXL_HIBYTE(DXL_HIWORD(GoalPos));
            }
            printf("%d\n", GoalPos);
            result = DXL.SyncWrite(P_GOAL_POSITION_LL, 4, param, NUM_ACTUATOR*(1+4));
            if( result != COMM_RXSUCCESS )
            {
                PrintCommStatus(result);
                break;
            }
            theta += STEP_THETA;
            usleep(CONTROL_PERIOD*1000);

        }while(theta < 2*PI);
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



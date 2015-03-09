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
#define P_TORQUE_ENABLE         562
#define P_GOAL_POSITION_LL      596
#define P_PRESENT_POSITION_LL   611
#define P_MOVING                610

// Defulat setting
#define DEFAULT_ID              1
#define DEFAULT_BAUDNUM         1 // 57,600

void PrintCommStatus(int CommStatus);
void PrintErrorCode(int ErrorCode);

int main()
{
    int PresentPos = 0;
    int index = 0, result = COMM_TXFAIL, error = 0, Moving = 1;
    int GoalPos[2] = {-150000, 150000};

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

    result = DXL.WriteByte(DEFAULT_ID, P_TORQUE_ENABLE, 1, &error);

    if(result == COMM_RXSUCCESS)
    {
		PrintErrorCode(error);
    }
    else
    	PrintCommStatus(result);

	while(1)
	{
		printf( "Press Enter key to continue!(press ESC and Enter to quit)\n" );
		if(_getch() == 0x1b)
			break;

		// Write goal position
		DXL.WriteDWord( DEFAULT_ID, P_GOAL_POSITION_LL, GoalPos[index], &error);
		do
		{
			// Read present position
			result = DXL.ReadDWord(DEFAULT_ID, P_PRESENT_POSITION_LL, (long*) &PresentPos, &error);
			if( result == COMM_RXSUCCESS )
			{
				printf( "%d   %d\n", GoalPos[index], PresentPos );
				PrintErrorCode(error);
			}
			else
			{
				PrintCommStatus(result);
				break;
			}

			// Check moving done
			result = DXL.ReadByte( DEFAULT_ID, P_MOVING, &Moving, &error);
			if( result == COMM_RXSUCCESS )
			{
				if( Moving == 0 )
				{
					// Change goal position
					if( index == 0 )
						index = 1;
					else
						index = 0;
				}

				PrintErrorCode(error);
			}
			else
			{
				PrintCommStatus(result);
				break;
			}
		}while(Moving == 1);
	}

	// Close device
	DXL.Disconnect();
	printf( "Press Enter key to terminate...\n" );
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



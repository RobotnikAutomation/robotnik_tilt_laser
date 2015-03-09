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

void Usage(char *progname)
{
    fprintf(stderr, "-----------------------------------------------------------------------\n");
    fprintf(stderr, "Usage: %s\n" \
                    " [-h | --help]........: display this help\n" \
                    " [-d | --device]......: port to open                     (/dev/ttyUSB0)\n" \
                    , progname);
    fprintf(stderr, "-----------------------------------------------------------------------\n");
}

void Help()
{
	fprintf(stderr, "\n");
    fprintf(stderr, "COMMAND: \n\n" \
                    " baud [BAUD_NUM]           : Baudrate change to [BAUD_NUM] \n" \
                    "                              0:2400, 1:57600, 2:115200, 3:1M, 4:2M, 5:3M \n" \
                    "                              6:4M, 7:4.5M, 8:10.5M \n" \
                    " scan                      : Output the current status of all DXLs \n" \
                    " ping [ID] [ID] ...        : Output the current status of [ID] \n" \
                    " bp                        : Broadcast Ping \n" \
                    " wrb [ID] [ADDR] [VALUE]   : Write 1 byte [VALUE] to [ADDR] of [ID] \n" \
                    " wrw [ID] [ADDR] [VALUE]   : Write 2 byte [VALUE] to [ADDR] of [ID] \n" \
                    " wrd [ID] [ADDR] [VALUE]   : Write 4 byte [VALUE] to [ADDR] of [ID] \n" \
                    " rdb [ID] [ADDR]           : Read 1 byte value from [ADDR] of [ID] \n" \
                    " rdw [ID] [ADDR]           : Read 2 byte value from [ADDR] of [ID] \n" \
                    " rdd [ID] [ADDR]           : Read 4 byte value from [ADDR] of [ID] \n" \
                    " r [ID] [ADDR] [LENGTH]    : Dumps the control table of [ID] \n" \
                    "                              [LENGTH] bytes from [ADDR] \n" \
                    " mon [ID] [ADDR] b|w|d     : Refresh byte|word|dword from [ADDR] of [ID] \n" \
                    " reboot [ID]               : Reboot the dynamixel of [ID] \n" \
                    " reset [ID] [OPTION]       : Factory reset the dynamixel of [ID] \n" \
                    "                              OPTION: 255(All), 1(Except ID), 2(Except ID&Baud)\n" \
                    " exit                      : Exit this program \n" \
                    );
	fprintf(stderr, "\n");
}

// Print error bit of status packet
void PrintErrorCode(int ErrorCode)
{
	printf("ErrorCode : %d (0x%X)\n", ErrorCode, ErrorCode);
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

void Scan()
{
	fprintf(stderr, "\n");
	PingInfo *data = new PingInfo();
	for(int i = 1; i < 253; i++)
	{
		if(DXL.Ping(i, data, 0) == COMM_RXSUCCESS)
		{
			fprintf(stderr, "\n                                          ... SUCCESS \r");
			fprintf(stderr, " [ID:%.3d] Model No : %.5d (0x%.2X 0x%.2X) \n",
					data->ID, data->ModelNumber, DXL_LOBYTE(data->ModelNumber), DXL_HIBYTE(data->ModelNumber));
		}
		else
			fprintf(stderr, ".");

		if(kbhit())
		{
			char c = _getch();
			if(c == 0x1b)
				break;
		}
	}
	fprintf(stderr, "\n\n");
}

void Write(int id, int addr, long value, int len)
{
	int result = COMM_TXFAIL, error = 0;

	if(len == 1)
		result = DXL.WriteByte(id, addr, (int)value, &error);
	else if(len == 2)
		result = DXL.WriteWord(id, addr, (int)value, &error);
	else if(len == 4)
		result = DXL.WriteDWord(id, addr, value, &error);

	if(result != COMM_RXSUCCESS)
	{
		fprintf(stderr, "\n Fail to write! \n\n");
		return;
	}

	if(error != 0)
		PrintErrorCode(error);

	fprintf(stderr, "\n Success to write! \n\n");
}

void Read(int id, int addr, int len)
{
	int result = COMM_TXFAIL, error = 0;
	int ivalue = 0;
	long lvalue = 0;

	if(len == 1)
		result = DXL.ReadByte(id, addr, &ivalue, &error);
	else if(len == 2)
		result = DXL.ReadWord(id, addr, &ivalue, &error);
	else if(len == 4)
		result = DXL.ReadDWord(id, addr, &lvalue, &error);

	if(result != COMM_RXSUCCESS)
	{
		fprintf(stderr, "\n Fail to read! (result : %d) \n\n", result);
		return;
	}

	if(error != 0)
		PrintErrorCode(error);

	if(len == 1)
		fprintf(stderr, "\n READ VALUE : %d \n\n", ivalue);
	else if(len == 2)
		fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", ivalue, ivalue);
	else
		fprintf(stderr, "\n READ VALUE : (UNSIGNED) %lu , (SIGNED) %d \n\n", lvalue, lvalue);
}

void Dump(int id, int addr, int len)
{
	int result = COMM_TXFAIL, error = 0;
	unsigned char* data = (unsigned char*)calloc(len, sizeof(unsigned char));

	result = DXL.Read(id, addr, len, data, &error);
	if(result != COMM_RXSUCCESS)
	{
		fprintf(stderr, "\n Fail to read! (result : %d) \n\n", result);
		return;
	}

	if(error != 0)
		PrintErrorCode(error);

	if(id != BROADCAST_ID)
	{
		fprintf(stderr, "\n");
		for(int i = addr; i < addr+len; i++)
			fprintf(stderr, "ADDR %.3d [0x%.4X] :     %.3d [0x%.2X] \n", i, i, data[i-addr], data[i-addr]);
		fprintf(stderr, "\n");
	}
}

void Refresh(int id, int addr, int len)
{
	int result = COMM_TXFAIL, error = 0;
	int ivalue = 0;
	long lvalue = 0;

	fprintf(stderr, "\n [ESC] : Quit monitoring \n\n");
	while(1)
	{
		if(len == 1)
			result = DXL.ReadByte(id, addr, &ivalue, &error);
		else if(len == 2)
			result = DXL.ReadWord(id, addr, &ivalue, &error);
		else if(len == 4)
			result = DXL.ReadDWord(id, addr, &lvalue, &error);

		if(result != COMM_RXSUCCESS)
		{
			//fprintf(stderr, "\n Fail to read! (result : %d) \n\n", result);
			continue;
		}

		if(error != 0)
			PrintErrorCode(error);

		if(len == 1)
			fprintf(stderr, " READ VALUE : %.3d [0x%.2X] \r", ivalue, ivalue);
		else if(len == 2)
			fprintf(stderr, " READ VALUE : %.5d [0x%.2X 0x%.2X] \r", ivalue, DXL_LOBYTE(ivalue), DXL_HIBYTE(ivalue));
		else
			fprintf(stderr, " READ VALUE : %.10ld [0x%.2X 0x%.2X 0x%.2X 0x%.2X] \r",
					lvalue, DXL_LOBYTE(DXL_LOWORD(lvalue)), DXL_HIBYTE(DXL_LOWORD(lvalue)), DXL_LOBYTE(DXL_HIWORD(lvalue)), DXL_HIBYTE(DXL_HIWORD(lvalue)));

		if(kbhit())
		{
			char c = _getch();
			if(c == 0x1b)
				break;
		}
	}
	fprintf(stderr, "\n\n");
}

void BroadcastPing()
{
	std::vector<PingInfo> vec_info = std::vector<PingInfo>();
	DXL.BroadcastPing(vec_info);

	if(vec_info.size() > 0)
	{
		fprintf(stderr, "\n");
		for(int i = 0; i < (int)vec_info.size(); i++)
		{
			fprintf(stderr, "                                          ... SUCCESS \r");
			fprintf(stderr, " [ID:%.3d] Model No : %.5d (0x%.2X 0x%.2X) \n",
					vec_info.at(i).ID, vec_info.at(i).ModelNumber, DXL_LOBYTE(vec_info.at(i).ModelNumber), DXL_HIBYTE(vec_info.at(i).ModelNumber));
		}
		fprintf(stderr, "\n");
	}
}

void BeltTest()
{
	int result;
	long lvalue1 , lvalue2;

	while(true)
	{
		if(kbhit())
			break;

		result = DXL.ReadDWord(17, 611, &lvalue1, 0);
		result = DXL.ReadDWord(18, 611, &lvalue2, 0);
		fprintf(stderr, "\n READ VALUE : %d , %d \n\n", lvalue1, lvalue2);
	}
}

int main(int argc, char *argv[])
{
    fprintf(stderr, "\n***********************************************************************\n");
    fprintf(stderr,   "*                     DXL Protocol 2.0 Monitor                        *\n");
    fprintf(stderr,   "***********************************************************************\n\n");

    char *dev = (char*)"/dev/ttyUSB0";

    /* parameter parsing */
    while(1)
    {
    	int option_index = 0, c = 0;
    	static struct option long_options[] = {
                {"h", no_argument, 0, 0},
                {"help", no_argument, 0, 0},
                {"d", required_argument, 0, 0},
                {"device", required_argument, 0, 0},
                {0, 0, 0, 0}
    	};

        /* parsing all parameters according to the list above is sufficent */
        c = getopt_long_only(argc, argv, "", long_options, &option_index);

        /* no more options to parse */
        if(c == -1) break;

        /* unrecognized option */
        if(c == '?') {
        	Usage(argv[0]);
            return 0;
        }

        /* dispatch the given options */
        switch(option_index) {
        /* h, help */
        case 0:
        case 1:
        	Usage(argv[0]);
            return 0;
            break;

            /* d, device */
        case 2:
        case 3:
        	Usage(argv[0]);
            dev = strdup(optarg);
            DXL.ComPort->SetPortName(dev);
            break;

        default:
        	Usage(argv[0]);
            return 0;
        }
    }

    if(DXL.Connect() == false)
    {
    	fprintf(stderr, " Fail to open USB2Dyanmixel! [%s] \n\n", dev);
    	return 0;
    }

    char input[128];
    char cmd[80];
    char param[20][30];
    int num_param;
    char* token;
    while(1)
    {
    	printf("[CMD] ");
    	gets(input);
    	fflush(stdin);

    	if(strlen(input) == 0)
    		continue;

    	token = strtok(input, " ");
    	if(token == 0)
    		continue;

    	strcpy(cmd, token);
    	token = strtok(0, " ");
    	num_param = 0;
    	while(token != 0)
    	{
    		strcpy(param[num_param++], token);
    		token = strtok(0, " ");
    	}

    	if(strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
    	{
    		Help();
    	}
    	else if(strcmp(cmd, "ping") == 0)
    	{
    		if(num_param == 0)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		fprintf(stderr, "\n");
			PingInfo *data = new PingInfo();
    		for(int i = 0; i < num_param; i++)
    		{
    			if(DXL.Ping(atoi(param[i]), data, 0) == COMM_RXSUCCESS)
    			{
        			fprintf(stderr, "                                          ... SUCCESS \r");
        			fprintf(stderr, " [ID:%.3d] Model No : %.5d (0x%.2X 0x%.2X) \n",
        					data->ID, data->ModelNumber, DXL_LOBYTE(data->ModelNumber), DXL_HIBYTE(data->ModelNumber));
    			}
    			else
    			{
        			fprintf(stderr, "                                          ... FAIL \r");
        			fprintf(stderr, " [ID:%.3d] \n", atoi(param[i]));
    			}
    		}
    		fprintf(stderr, "\n");
    	}
    	else if(strcmp(cmd, "scan") == 0)
    	{
    		Scan();
    	}
    	else if(strcmp(cmd, "baud") == 0)
    	{
    		if(num_param != 1)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		if(DXL.SetBaudrate(atoi(param[0])) == false)
    			fprintf(stderr, " Failed to change baudrate! \n");
    		else
    			fprintf(stderr, " Success to change baudrate! [ BAUD NUM: %d ]\n", atoi(param[0]));
    	}
    	else if(strcmp(cmd, "wrb") == 0 || strcmp(cmd, "w") == 0)
    	{
    		if(num_param != 3)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		Write(atoi(param[0]), atoi(param[1]), atoi(param[2]), 1);
    	}
    	else if(strcmp(cmd, "wrw") == 0)
    	{
    		if(num_param != 3)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		Write(atoi(param[0]), atoi(param[1]), atoi(param[2]), 2);
    	}
    	else if(strcmp(cmd, "wrd") == 0)
    	{
    		if(num_param != 3)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		Write(atoi(param[0]), atoi(param[1]), atol(param[2]), 4);
    	}
    	else if(strcmp(cmd, "rdb") == 0)
    	{
    		if(num_param != 2)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		Read(atoi(param[0]), atoi(param[1]), 1);
    	}
    	else if(strcmp(cmd, "rdw") == 0)
    	{
    		if(num_param != 2)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		Read(atoi(param[0]), atoi(param[1]), 2);
    	}
    	else if(strcmp(cmd, "rdd") == 0)
    	{
    		if(num_param != 2)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		Read(atoi(param[0]), atoi(param[1]), 4);
    	}
    	else if(strcmp(cmd, "r") == 0)
    	{
    		if(num_param != 3)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		Dump(atoi(param[0]), atoi(param[1]), atoi(param[2]));
    	}
    	else if(strcmp(cmd, "mon") == 0)
    	{
    		int len = 0;

    		if(num_param > 2 && strcmp(param[2],"b") == 0) len = 1;
    		else if(num_param > 2 && strcmp(param[2],"w") == 0) len = 2;
    		else if(num_param > 2 && strcmp(param[2],"d") == 0) len = 4;

    		if(num_param != 3 || len == 0)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		Refresh(atoi(param[0]), atoi(param[1]), len);
    	}
    	else if(strcmp(cmd, "bp") == 0)
    	{
    		if(num_param != 0)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		BroadcastPing();
    	}
    	else if(strcmp(cmd, "reboot") == 0)
    	{
    		if(num_param != 1)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		int result = DXL.Reboot(atoi(param[0]), 0);
    		if(result != COMM_RXSUCCESS)
    			fprintf(stderr, "\n Fail to reboot! \n\n");
    		else
    			fprintf(stderr, "\n Success to reboot! \n\n");
    	}
    	else if(strcmp(cmd, "reset") == 0)
    	{
    		if(num_param != 2)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		int result = DXL.FactoryReset(atoi(param[0]), atoi(param[1]), 0);
    		if(result != COMM_RXSUCCESS)
    			fprintf(stderr, "\n Fail to reset! \n\n");
    		else
    			fprintf(stderr, "\n Success to reset! \n\n");
    	}
    	else if(strcmp(cmd, "exit") == 0)
    	{
    		DXL.Disconnect();
    		return 0;
    	}
    }
}



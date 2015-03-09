/*
 * serialport.cpp
 *
 *  Created on: 2012. 12. 27.
 *      Author: zerom
 */

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include "serialport.h"

using namespace DXL_PRO;

SerialPort::SerialPort(const char* port_name) {
	DEBUG_PRINT = false;
	SocketFD = -1;
	SetPortName(port_name);
}

SerialPort::~SerialPort() {
	ClosePort();
}

void SerialPort::SetPortName(const char* port_name)
{
	strcpy(PortName, port_name);
}

double SerialPort::GetCurrentTime()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0);
}


bool SerialPort::SetBaudDevisor(int speed)
{
	// try to set a custom divisor
	struct serial_struct ss;
	if(ioctl(SocketFD, TIOCGSERIAL, &ss) != 0)
	{
		if(DEBUG_PRINT == true)
			printf(" TIOCGSERIAL failed!\n");
		return false;
	}

	ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
	ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
	int closest_speed = ss.baud_base / ss.custom_divisor;

	if(closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
	{
		if(DEBUG_PRINT == true)
			printf(" Cannot set speed to %d, closest is %d \n", speed, closest_speed);
		return false;
	}

	if(ioctl(SocketFD, TIOCSSERIAL, &ss) < 0)
	{
		if(DEBUG_PRINT == true)
			printf(" TIOCSSERIAL failed!\n");
		return false;
	}
	return true;
}

int SerialPort::GetBaud(int baud)
{
	switch (baud)
	{
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 500000:
		return B500000;
	case 576000:
		return B576000;
	case 921600:
		return B921600;
	case 1000000:
		return B1000000;
	case 1152000:
		return B1152000;
	case 1500000:
		return B1500000;
	case 2000000:
		return B2000000;
	case 2500000:
		return B2500000;
	case 3000000:
		return B3000000;
	case 3500000:
		return B3500000;
	case 4000000:
		return B4000000;
	default:
		return -1;
	}
}

bool SerialPort::SetupSerialPort(int baud)
{
	struct termios newtio;

	SocketFD = open(PortName, O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(SocketFD < 0)
	{
		if(DEBUG_PRINT == true)
			printf("Error opening serial port!\n");
		return false;
	}

	bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

	newtio.c_cflag = baud | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;

    // clean the buffer and activate the settings for the port
    tcflush(SocketFD, TCIFLUSH);
    tcsetattr(SocketFD, TCSANOW, &newtio);

    return true;
}

bool SerialPort::OpenPort()
{
	return SetBaudrate(DEFAULT_BAUDRATE);
}

void SerialPort::ClosePort()
{
	if(SocketFD != -1)
        close(SocketFD);
	SocketFD = -1;
}

void SerialPort::ClearPort()
{
	tcflush(SocketFD, TCIOFLUSH);
}

bool SerialPort::SetBaudrate(int baudrate)
{
	int baud = GetBaud(baudrate);

	ClosePort();

	if(baud <= 0)
	{
		SetupSerialPort(B38400);
		return SetBaudDevisor(baudrate);
	}
	else
	{
		return SetupSerialPort(baud);
	}
}

int SerialPort::WritePort(unsigned char* packet, int packet_len)
{
	return write(SocketFD, packet, packet_len);
}

int SerialPort::ReadPort(unsigned char* packet, int packet_len)
{
	return read(SocketFD, packet, packet_len);
}



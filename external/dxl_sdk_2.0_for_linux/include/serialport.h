/*
 * serialport.h
 *
 *  Created on: 2012. 12. 27.
 *      Author: zerom
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

namespace DXL_PRO {
	class SerialPort {
	private:
		int SocketFD;
		char PortName[20];

		int GetBaud(int baud);
		bool SetupSerialPort(int baud);
		bool SetBaudDevisor(int speed);

	public:
		static const int DEFAULT_BAUDRATE = 57600;
		bool DEBUG_PRINT;

		SerialPort(const char* port_name);
		virtual ~SerialPort();

		double GetCurrentTime();

		bool OpenPort();
		void ClosePort();
		void ClearPort();
		void SetPortName(const char* port_name);
		bool SetBaudrate(int baudrate);

		int WritePort(unsigned char* packet, int packet_len);
		int ReadPort(unsigned char* packet, int packet_len);
	};
}
#endif /* SERIALPORT_H_ */

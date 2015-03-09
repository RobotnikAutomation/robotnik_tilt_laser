/*
 * dynamixel.h
 *
 * Copyright (c) ROBOTIS
 *
 *  Created on: 2012. 12. 26.
 *      Author: zerom
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include <vector>
#include "serialport.h"

#define MAXNUM_TXPACKET     (65535)
#define MAXNUM_RXPACKET     (65535)
#define BROADCAST_ID		(0xFE)

#define LATENCY_TIME		(10)		// ms (USB2Serial Latency timer)

#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

namespace DXL_PRO {
	enum {
		PKT_HEADER0,
		PKT_HEADER1,
		PKT_HEADER2,
		PKT_RESERVED,
		PKT_ID,
		PKT_LENGTH_L,
		PKT_LENGTH_H,
		PKT_INSTRUCTION,
		PKT_PARAMETER
	};

	enum {
		COMM_TXSUCCESS,
		COMM_RXSUCCESS,
		COMM_TXFAIL,
		COMM_RXFAIL,
		COMM_TXERROR,
		COMM_RXWAITING,
		COMM_RXTIMEOUT,
		COMM_RXCORRUPT
	};

	enum {
		INST_PING			= 1,
		INST_READ			= 2,
		INST_WRITE			= 3,
		INST_REG_WRITE		= 4,
		INST_ACTION			= 5,
		INST_FACTORY_RESET	= 6,
		INST_REBOOT			= 8,
		INST_SYSTEM_WRITE	= 13,	// 0x0D
		INST_STATUS			= 85,	// 0x55
		INST_SYNC_READ		= 130,	// 0x82
		INST_SYNC_WRITE		= 131,	// 0x83
		INST_BULK_READ		= 146,	// 0x92
		INST_BULK_WRITE		= 147	// 0x93
	};

	enum {
		ERRBIT_VOLTAGE		= 1,
		ERRBIT_ANGLE		= 2,
		ERRBIT_OVERHEAT		= 4,
		ERRBIT_RANGE		= 8,
		ERRBIT_CHECKSUM		= 16,
		ERRBIT_OVERLOAD		= 32,
		ERRBIT_INSTRUCTION	= 64
	};

	class PingInfo
	{
	public:
		int ID;
		int ModelNumber;
		int FirmwareVersion;

		PingInfo() : ID(-1), ModelNumber(-1), FirmwareVersion(-1) { };
	};

	class BulkReadData
	{
	public:
	    int             iID;
	    int             iStartAddr;
	    int             iLength;
	    int             iError;
	    unsigned char*  pucTable;

	    BulkReadData()
	    {
			iID = 0; iStartAddr = 0; iLength = 0; iError = 0;
			pucTable = 0;

	    }
		~BulkReadData()
		{
//			if(pucTable != 0)
//				delete[] pucTable;
		}
	};


	class Dynamixel
	{
	private:
		double PacketStartTime;
		double PacketWaitTime;
		double ByteTransferTime;

		int GetBaudrate(int baud_num);

		void SetPacketTimeout(int packet_len);
		void SetPacketTimeout(double msec);
		bool IsPacketTimeout();
		double GetPacketTime();

		unsigned short UpdateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
		void AddStuffing(unsigned char *packet);
		void RemoveStuffing(unsigned char *packet);

		int TxPacket(unsigned char *txpacket);
		int RxPacket(unsigned char *rxpacket);
		int TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, int *error);

	public:
		SerialPort *ComPort;

		Dynamixel(const char* port_name);
		virtual ~Dynamixel();

		bool Connect();
		void Disconnect();
		bool SetBaudrate(int baud);

		int Ping(int id, int *error);
		int Ping(int id, PingInfo *info, int *error);
		int BroadcastPing(std::vector<PingInfo>& vec_info);
		int Reboot(int id, int *error);
		int FactoryReset(int id, int option, int *error);

		int Read(int id, int address, int length, unsigned char* data, int *error);
		int ReadByte(int id, int address, int *value, int *error);
		int ReadWord(int id, int address, int *value, int *error);
		int ReadDWord(int id, int address, long *value, int *error);

		int Write(int id, int address, int length, unsigned char* data, int *error);
		int WriteByte(int id, int address, int value, int *error);
		int WriteWord(int id, int address, int value, int *error);
		int WriteDWord(int id, int address, long value, int *error);

		int SyncWrite(int start_addr, int data_length, unsigned char* param, int param_length);
		int BulkRead(std::vector<BulkReadData>& data);
	};

}

#endif /* DYNAMIXEL_H_ */

/*
 * dynamixel.cpp
 *
 *  Created on: 2012. 12. 26.
 *      Author: zerom
 */

#include <stdio.h>	//TODO: TEST CODE
#include <stdlib.h>
#include <string.h>
#include "dynamixel.h"

using namespace DXL_PRO;

Dynamixel::Dynamixel(const char* port_name)
{
	ComPort = new SerialPort(port_name);
}

Dynamixel::~Dynamixel()
{
	Disconnect();
}

int Dynamixel::GetBaudrate(int baud_num)
{
	if(baud_num >= 2400)
		return baud_num;

	switch(baud_num)
	{
	case 0:
		return 2400;
	case 1:
		return 57600;
	case 2:
		return 115200;
	case 3:
		return 1000000;
	case 4:
		return 2000000;
	case 5:
		return 3000000;
	case 6:
		return 4000000;
	case 7:
		return 4500000;
	case 8:
		return 10500000;
	default:
		return ComPort->DEFAULT_BAUDRATE;
	}
}

void Dynamixel::SetPacketTimeout(int packet_len)
{
	PacketStartTime = ComPort->GetCurrentTime();
	PacketWaitTime = (ByteTransferTime * (double)packet_len) + (2.0 * (double)LATENCY_TIME) + 2.0;
}

void Dynamixel::SetPacketTimeout(double msec)
{
	PacketStartTime = ComPort->GetCurrentTime();
	PacketWaitTime = msec;
}

bool Dynamixel::IsPacketTimeout()
{
	if(GetPacketTime() > PacketWaitTime)
		return true;
	return false;
}

double Dynamixel::GetPacketTime()
{
	double time;

	time = ComPort->GetCurrentTime() - PacketStartTime;
	if(time < 0.0)
		PacketStartTime = ComPort->GetCurrentTime();

	return time;
}

bool Dynamixel::Connect()
{
	if(ComPort->OpenPort() == false)
	{
		// TODO: error print
		return false;
	}

	ByteTransferTime = (1000.0 / (double)ComPort->DEFAULT_BAUDRATE) * 10.0;

	return true;
}

void Dynamixel::Disconnect()
{
	ComPort->ClosePort();
}

bool Dynamixel::SetBaudrate(int baud)
{
	int baudrate = GetBaudrate(baud);

	if(ComPort->SetBaudrate(baudrate) == false)
		return false;

	ByteTransferTime = (1000.0 / baudrate) * 10.0;
	return true;
}

unsigned short Dynamixel::UpdateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {0x0000,
	0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
	0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
	0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
	0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
	0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
	0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
	0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
	0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
	0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
	0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
	0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
	0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
	0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
	0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
	0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
	0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
	0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
	0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
	0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
	0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
	0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
	0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
	0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
	0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
	0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
	0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
	0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
	0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
	0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
	0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
	0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
	0x820D, 0x8207, 0x0202 };

	for(j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

void Dynamixel::AddStuffing(unsigned char *packet)
{
	int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;
    unsigned char temp[MAXNUM_TXPACKET] = {0};

    memcpy(temp, packet, PKT_LENGTH_H+1);    // FF FF FD XX ID LEN_L LEN_H
    index = PKT_INSTRUCTION;
    for( i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        temp[index++] = packet[i+PKT_INSTRUCTION];
        if(packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD
            temp[index++] = 0xFD;
            packet_length_out++;
        }
    }
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];


    //////////////////////////
	if(packet_length_in != packet_length_out)
		packet = (unsigned char*)realloc(packet, index * sizeof(unsigned char));

	///////////////////////////

    memcpy(packet, temp, index);
    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

void Dynamixel::RemoveStuffing(unsigned char *packet)
{
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;

    index = PKT_INSTRUCTION;
    for( i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        if(packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION+1] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD FD
            packet_length_out--;
            i++;
        }
        packet[index++] = packet[i+PKT_INSTRUCTION];
    }
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];

    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

int Dynamixel::TxPacket(unsigned char *txpacket)
{
	int packet_tx_len, real_tx_len;
	int length;
	unsigned short crc = 0;

	//TODO: check port using

	// Byte stuffing
	AddStuffing(txpacket);
	length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]);

	// Check Max packet length
	if(length > MAXNUM_TXPACKET)
	{
		//TODO: port free?
		return COMM_TXERROR;
	}

	// Make Packet Header
	txpacket[PKT_HEADER0]	= 0xFF;
	txpacket[PKT_HEADER1]	= 0xFF;
	txpacket[PKT_HEADER2]	= 0xFD;
	txpacket[PKT_RESERVED]	= 0x00;	// reserved

	// Add CRC16
	crc = UpdateCRC(0, txpacket, length+PKT_LENGTH_H+1-2);	// -2 : CRC16
	txpacket[length+PKT_LENGTH_H-1] = DXL_LOBYTE(crc);		// last - 1
	txpacket[length+PKT_LENGTH_H-0] = DXL_HIBYTE(crc);		// last - 0

	// Tx Packet
	ComPort->ClearPort();
	packet_tx_len = length + PKT_LENGTH_H + 1;
	//printf("txfail1\n");
	real_tx_len = ComPort->WritePort(txpacket, packet_tx_len);
	//printf("txfail2\n");
	if(packet_tx_len != real_tx_len)
	{
		//TODO: port free?
		return COMM_TXFAIL;
	}

	return COMM_TXSUCCESS;
}

int Dynamixel::RxPacket(unsigned char *rxpacket)
{
	int rx_length = 0, wait_length = PKT_LENGTH_H + 4 + 1;	// 4 : INST ERROR CHKSUM_L CHKSUM_H
	int i, result = COMM_RXFAIL;
	unsigned short crc = 0;

	while(1)
	{
		rx_length += ComPort->ReadPort(&rxpacket[rx_length], wait_length - rx_length);
		if(rx_length >= wait_length)	// wait_length minimum : 11
		{
			// find packet header
			for(i = 0; i < (rx_length - 2); i++)
			{
				if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF && rxpacket[i+2] == 0xFD)
					break;
			}

			if(i == 0)
			{
				// check length
				wait_length = DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
				if(rx_length < wait_length)
				{
					// check timeout
					if(IsPacketTimeout() == true)
					{
						if(rx_length == 0)
							result = COMM_RXTIMEOUT;
						else
							result = COMM_RXCORRUPT;

						//TODO: bus free?

						break;
					}

					continue;
				}

				// check CRC16
				crc = DXL_MAKEWORD(rxpacket[wait_length-2], rxpacket[wait_length-1]);
				if(UpdateCRC(0, rxpacket, wait_length-2) == crc) // -2 : except CRC16
					result = COMM_RXSUCCESS;
				else
					result = COMM_RXCORRUPT;

				//TODO: bus free?

				break;
			}
			else
			{
				// remove unnecessary packets
				memcpy(&rxpacket[0], &rxpacket[i], rx_length - i);
				rx_length -= i;
			}
		}
		else
		{
			// check timeout
			if(IsPacketTimeout() == true)
			{
				if(rx_length == 0)
					result = COMM_RXTIMEOUT;
				else
					result = COMM_RXCORRUPT;

				//TODO: bus free?

				break;
			}
		}
	}

	if(result == COMM_RXSUCCESS)
		RemoveStuffing(rxpacket);

	//TODO: port free?
	return result;
}

int Dynamixel::TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, int *error)
{
	int result = COMM_TXFAIL;

	//TODO: check bus idle?

	result = TxPacket(txpacket);

	// Check Tx packet result
	if(result != COMM_TXSUCCESS)
		return result;

	// Set Rx timeout
	if(txpacket[PKT_INSTRUCTION] == INST_READ)
		SetPacketTimeout(DXL_MAKEWORD(txpacket[PKT_PARAMETER+2], txpacket[PKT_PARAMETER+3]) + 11);
	else
		SetPacketTimeout(PKT_LENGTH_H + 4 + 1);	// 4 : INST ERROR CHKSUM_L CHKSUM_H

	// Broadcast ID && !BulkRead == no need to wait for a rxpacket
	if(txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_BULK_READ)
	{
		//TODO: bus free?
		return COMM_RXSUCCESS;
	}

	result = RxPacket(rxpacket);
	if((result == COMM_RXSUCCESS) && (txpacket[PKT_ID] != BROADCAST_ID) && (txpacket[PKT_ID] != rxpacket[PKT_ID]))
		result = RxPacket(rxpacket);

	if(result == COMM_RXSUCCESS && txpacket[PKT_ID] != BROADCAST_ID)
	{
		if(error != 0)
			*error = (int)rxpacket[PKT_PARAMETER];
	}

	return result;
}

int Dynamixel::BroadcastPing(std::vector<PingInfo>& vec_info)
{
	const int PING_STATUS_LENGTH = 14, MAX_ID = 252;
	int result = COMM_TXFAIL;
	int rx_length = 0, wait_length = 0;
	unsigned char txpacket[10]	= {0};
	unsigned char rxpacket[PING_STATUS_LENGTH * MAX_ID]	= {0};

	txpacket[PKT_ID]			= (unsigned char)BROADCAST_ID;
	txpacket[PKT_LENGTH_L]		= 0x03;
	txpacket[PKT_LENGTH_H]		= 0x00;
	txpacket[PKT_INSTRUCTION]	= INST_PING;

	result = TxPacket(txpacket);
	if(result != COMM_TXSUCCESS)
	{
		// TODO: Bus free?
		return result;
	}

	wait_length = PING_STATUS_LENGTH * MAX_ID;

	// Set Rx Timeout
	SetPacketTimeout( (double)((ByteTransferTime * wait_length) + (3 * MAX_ID) + 2 * LATENCY_TIME) );

	while(1)
	{
		int _cnt = ComPort->ReadPort(&rxpacket[rx_length], wait_length - rx_length);
		if(_cnt > 0)
		{
			rx_length += _cnt;
			fprintf(stderr, "cnt: %d, Interval: %f / Wait time: %f \n", _cnt, GetPacketTime(), PacketWaitTime);
		}
		if((IsPacketTimeout() == 1) || (rx_length >= wait_length))
			break;
	}
	// TODO: Bus free?

	if(rx_length == 0)
		return COMM_RXTIMEOUT;

	while(1)
	{
		if(rx_length < PING_STATUS_LENGTH)
			return COMM_RXCORRUPT;

		// find packet header
		int idx = 0;
		while(idx < (rx_length - 2))
		{
			if(rxpacket[idx] == 0xFF && rxpacket[idx + 1] == 0xFF && rxpacket[idx + 2] == 0xFD)
				break;
			else
				idx++;
		}

		if(idx == 0)
		{
			// check CRC16
			int crc = DXL_MAKEWORD(rxpacket[PING_STATUS_LENGTH - 2], rxpacket[PING_STATUS_LENGTH - 1]);
			if(UpdateCRC(0, rxpacket, PING_STATUS_LENGTH - 2) == crc) // -2 : except CRC16
			{
				vec_info.push_back(PingInfo());
				vec_info.at(vec_info.size()-1).ID = rxpacket[PKT_ID];
				vec_info.at(vec_info.size()-1).ModelNumber = DXL_MAKEWORD(rxpacket[PKT_PARAMETER+1], rxpacket[PKT_PARAMETER+2]);
				vec_info.at(vec_info.size()-1).FirmwareVersion = rxpacket[PKT_PARAMETER+3];

				memcpy(&rxpacket[0], &rxpacket[PING_STATUS_LENGTH], rx_length - PING_STATUS_LENGTH);
				rx_length -= PING_STATUS_LENGTH;
			}
			else
			{
				result = COMM_RXCORRUPT;

				// remove header (0xFF 0xFF 0xFD)
				memcpy(&rxpacket[0], &rxpacket[3], rx_length - 3);
				rx_length -= 3;
			}

			if(rx_length < PING_STATUS_LENGTH)
				break;
		}
		else
		{
			// remove unnecessary packets
			memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
			rx_length -= idx;
		}
	}

	return result;
}

int Dynamixel::Ping(int id, int *error)
{
	return Ping(id, 0, error);
}

int Dynamixel::Ping(int id, PingInfo *info, int *error)
{
	int result = COMM_TXFAIL;
	unsigned char txpacket[10] 	= {0};
	unsigned char rxpacket[14] 	= {0};

	txpacket[PKT_ID]			= (unsigned char)id;
	txpacket[PKT_LENGTH_L]		= 0x03;
	txpacket[PKT_LENGTH_H]		= 0x00;
	txpacket[PKT_INSTRUCTION]	= INST_PING;

	result = TxRxPacket(txpacket, rxpacket, error);
	if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
	{
		if(info != 0)
		{
			info->ID = rxpacket[PKT_ID];
			info->ModelNumber = DXL_MAKEWORD(rxpacket[PKT_PARAMETER+1], rxpacket[PKT_PARAMETER+2]);
			info->FirmwareVersion = rxpacket[PKT_PARAMETER+3];
		}
	}

	return result;
}

int Dynamixel::Reboot(int id, int *error)
{
	unsigned char txpacket[10]	= {0};
	unsigned char rxpacket[11]	= {0};

	txpacket[PKT_ID]            = (unsigned char)id;
	txpacket[PKT_LENGTH_L]      = 0x03;
	txpacket[PKT_LENGTH_H]      = 0x00;
	txpacket[PKT_INSTRUCTION]   = INST_REBOOT;

	return TxRxPacket(txpacket, rxpacket, error);
}

int Dynamixel::FactoryReset(int id, int option, int *error)
{
	unsigned char txpacket[11]	= {0};
	unsigned char rxpacket[11]	= {0};

	txpacket[PKT_ID]            = (unsigned char)id;
	txpacket[PKT_LENGTH_L]      = 0x04;
	txpacket[PKT_LENGTH_H]      = 0x00;
	txpacket[PKT_INSTRUCTION]   = INST_FACTORY_RESET;
	txpacket[PKT_PARAMETER]		= (unsigned char)option;

	return TxRxPacket(txpacket, rxpacket, error);
}

int Dynamixel::Read(int id, int address, int length, unsigned char* data, int *error)
{
	int result = COMM_TXFAIL;
	unsigned char txpacket[14]	= {0};
	unsigned char* rxpacket		= (unsigned char*)calloc(length+11, sizeof(unsigned char));

	txpacket[PKT_ID]			= (unsigned char)id;
	txpacket[PKT_LENGTH_L]		= 0x07;
	txpacket[PKT_LENGTH_H]		= 0x00;
	txpacket[PKT_INSTRUCTION]	= INST_READ;
	txpacket[PKT_PARAMETER+0]	= (unsigned char)DXL_LOBYTE(address);
	txpacket[PKT_PARAMETER+1]	= (unsigned char)DXL_HIBYTE(address);
	txpacket[PKT_PARAMETER+2]	= (unsigned char)DXL_LOBYTE(length);
	txpacket[PKT_PARAMETER+3]	= (unsigned char)DXL_HIBYTE(length);

	result = TxRxPacket(txpacket, rxpacket, error);
	if(result == COMM_RXSUCCESS && id != BROADCAST_ID)
		memcpy(data, &rxpacket[PKT_PARAMETER+1], length);

	free(rxpacket);
	return result;
}

int Dynamixel::ReadByte(int id, int address, int *value, int *error)
{
	int result = COMM_TXFAIL;
	unsigned char data[1] = {0};

	result = Read(id, address, 1, data, error);
	if(result == COMM_RXSUCCESS)
		*value = data[0];

	return result;
}

int Dynamixel::ReadWord(int id, int address, int *value, int *error)
{
	int result = COMM_TXFAIL;
	unsigned char data[2] = {0};

	result = Read(id, address, 2, data, error);
	if(result == COMM_RXSUCCESS)
		*value = DXL_MAKEWORD(data[0], data[1]);

	return result;
}

int Dynamixel::ReadDWord(int id, int address, long *value, int *error)
{
	int result = COMM_TXFAIL;
	unsigned char data[4] = {0};

	result = Read(id, address, 4, data, error);
	if(result == COMM_RXSUCCESS)
		*value = DXL_MAKEDWORD(DXL_MAKEWORD(data[0], data[1]), DXL_MAKEWORD(data[2], data[3]));

	return result;
}

int Dynamixel::Write(int id, int address, int length, unsigned char* data, int *error)
{
	unsigned char* txpacket 	= (unsigned char*)calloc(length+12, sizeof(unsigned char));
	unsigned char rxpacket[12] 	= {0};
	int result = COMM_TXFAIL;
	txpacket[PKT_ID]			= (unsigned char)id;
	txpacket[PKT_LENGTH_L]		= DXL_LOBYTE(length+5);
	txpacket[PKT_LENGTH_H]		= DXL_HIBYTE(length+5);
	txpacket[PKT_INSTRUCTION]	= INST_WRITE;
	txpacket[PKT_PARAMETER+0]	= (unsigned char)DXL_LOBYTE(address);
	txpacket[PKT_PARAMETER+1]	= (unsigned char)DXL_HIBYTE(address);

	memcpy(&txpacket[PKT_PARAMETER+2], data, length);

	result = TxRxPacket(txpacket, rxpacket, error);

	//free(txpacket);

	return result;
}

int Dynamixel::WriteByte(int id, int address, int value, int *error)
{
	unsigned char data[1] = {value};
	return Write(id, address, 1, data, error);
}

int Dynamixel::WriteWord(int id, int address, int value, int *error)
{
	unsigned char data[2] = {DXL_LOBYTE(value), DXL_HIBYTE(value)};
	return Write(id, address, 2, data, error);
}

int Dynamixel::WriteDWord(int id, int address, long value, int *error)
{
    unsigned char data[4] = { DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)),
	                          DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value)) };
	return Write(id, address, 4, data, error);
}

int Dynamixel::SyncWrite(int start_addr, int data_length, unsigned char* param, int param_length)
{
	int pkt_length = param_length + 7;	// 7 : INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CHKSUM_L CHKSUM_H
	unsigned char* txpacket 	= (unsigned char*)calloc(pkt_length+7, sizeof(unsigned char));
	unsigned char rxpacket[12] 	= {0};

	txpacket[PKT_ID]			= BROADCAST_ID;
	txpacket[PKT_LENGTH_L]		= DXL_LOBYTE(pkt_length);
	txpacket[PKT_LENGTH_H]		= DXL_HIBYTE(pkt_length);
	txpacket[PKT_INSTRUCTION]	= INST_SYNC_WRITE;
	txpacket[PKT_PARAMETER+0]	= DXL_LOBYTE(start_addr);
	txpacket[PKT_PARAMETER+1]	= DXL_HIBYTE(start_addr);
	txpacket[PKT_PARAMETER+2]	= DXL_LOBYTE(data_length);
	txpacket[PKT_PARAMETER+3]	= DXL_HIBYTE(data_length);
	memcpy(&txpacket[PKT_PARAMETER+4], param, param_length);

	//free(txpacket);

	return TxRxPacket(txpacket, rxpacket, 0);
}

int Dynamixel::BulkRead(std::vector<BulkReadData>& data)
{
	int param_length = data.size()*5;
	if(param_length == 0)
		return COMM_TXFAIL;
    int result = COMM_TXFAIL, n, wait_length = 0;
    int num = data.size(); // each length : 5 (ID ADDR_L ADDR_H LEN_L LEN_H)
    int pkt_length = param_length + 3;  // 3 : INST CHKSUM_L CHKSUM_H
    unsigned char txpacket[MAXNUM_TXPACKET] = {0};
    unsigned char rxpacket[MAXNUM_RXPACKET] = {0};

    txpacket[PKT_ID]            = (unsigned char)BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = (unsigned char)DXL_LOBYTE(pkt_length);
    txpacket[PKT_LENGTH_H]      = (unsigned char)DXL_HIBYTE(pkt_length);
    txpacket[PKT_INSTRUCTION]   = INST_BULK_READ;
    for(unsigned int i = 0; i < data.size(); i++)
    {
    	txpacket[PKT_PARAMETER+5*i+0] = data[i].iID;
    	txpacket[PKT_PARAMETER+5*i+1] = DXL_LOBYTE(data[i].iStartAddr);
    	txpacket[PKT_PARAMETER+5*i+2] = DXL_HIBYTE(data[i].iStartAddr);
    	txpacket[PKT_PARAMETER+5*i+3] = DXL_LOBYTE(data[i].iLength);
    	txpacket[PKT_PARAMETER+5*i+4] = DXL_HIBYTE(data[i].iLength);
    }


    //memcpy(&txpacket[PKT_PARAMETER], param, param_length);

    for(n = 0; n < num; n++)
    {
        wait_length += data[n].iLength + 11;
    }

    /************ TxRxPacket *************/
    // Wait for Bus Idle
//    while(comm->iBusUsing == 1)
//    {
//        //Sleep(0);
//    }
    //usleep(1000);
    //printf("aaaaa\n");

     result = TxPacket(txpacket);


    // Check Tx packet result
    if( result != COMM_TXSUCCESS )
    {
        return result;
    }

    // Set Rx Timeout (BULK_READ)
    SetPacketTimeout(wait_length);


    for(n = 0; n < num; n++)
    {
    	result = RxPacket(rxpacket);

    	if(result == COMM_RXSUCCESS)
    	{
    		data[n].iError = rxpacket[PKT_PARAMETER];
    	}
    	else
    	{
    		//printf("result %d", result);
    		return result;
    	}

    	memcpy(data[n].pucTable, &rxpacket[PKT_PARAMETER+1], data[n].iLength);
    }

    return result;
}



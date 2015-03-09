/*
 * bulkreaddata.cpp
 *
 *  Created on: 2013. 1. 11.
 *      Author: zerom
 */

#include "bulkread.h"

using namespace DXL_PRO;


BulkRead::BulkRead(Dynamixel *dxl)
{
	m_DXL = dxl;
	//m_DXL = new Dynamixel(port_name);
	for(int i= 0; i <255; i++)
		id_list[i] = -1;
}

BulkRead::~BulkRead()
{

}


bool BulkRead::AddBulkReadData(int id, int addr, int length)
{
	if(m_Data.size() != 0)
		for(unsigned int i = 0 ; i < m_Data.size() ; i++)
			if(m_Data[i].iID == id)
			{
				fprintf(stderr, "can not be added the id which is already added ");
				return false;
			}


	BulkReadData tempData;
	tempData.iID = id;
	tempData.iStartAddr = addr;
	tempData.iLength = length;
	m_Data.push_back(tempData);

	m_Data[m_Data.size() - 1].pucTable = new unsigned char[length];

	id_list[id] = m_Data.size() - 1;

	return true;
}

bool BulkRead::ChangeBulkReadData(int id, int addr, int length)
{
	if(m_Data.size() != 0)
		for(unsigned int i = 0 ; i < m_Data.size() ; i++)
			if(m_Data[i].iID == id)
			{
				if(m_Data[i].pucTable != 0)
				{
					m_Data[i].iStartAddr = addr;
					delete[] m_Data[i].pucTable;
					m_Data[i].pucTable = new unsigned char[length];
				}
				else
				{
					m_Data[i].iStartAddr = addr;
					m_Data[i].pucTable = new unsigned char[length];
				}
				return true;
			}
			else
			{
				return false;
			}
	else
		return false;
}

void BulkRead::ClearBulkReadData()
{
	if(m_Data.size() != 0)
	{
		for(unsigned int i = 0 ; i < m_Data.size() ; i++)
		{
			if(m_Data[i].pucTable != 0)
			{
				delete[] m_Data[i].pucTable;
				m_Data[i].pucTable = NULL;
			}
		}
		m_Data.clear();
	}

	for(int i= 0; i <255; i++)
		id_list[i] = -1;

	return;
}

int BulkRead::SendTxPacket()
{
	int result;
	result = m_DXL->BulkRead(m_Data);

	return result;
}

bool BulkRead::GetByteValue(int id, int addr, int *data)
{
    if(id_list[id] == -1)
        return 0;

    if(m_Data[id_list[id]].iError == -1)
    	return 0;

    if(addr < m_Data[id_list[id]].iStartAddr || m_Data[id_list[id]].iStartAddr + m_Data[id_list[id]].iLength-1 < addr)
        return 0;

    *data = m_Data[id_list[id]].pucTable[ ( addr - m_Data[id_list[id]].iStartAddr ) ];

    return 1;
}
bool BulkRead::GetWordValue(int id, int addr, int *data)
{
    if(id_list[id] == -1)
        return false;

    if(m_Data[id_list[id]].iError == -1)
    	return false;

    if(addr < m_Data[id_list[id]].iStartAddr || m_Data[id_list[id]].iStartAddr + m_Data[id_list[id]].iLength-1 < addr)
        return false;

    *data = DXL_MAKEWORD(m_Data[id_list[id]].pucTable[(addr-m_Data[id_list[id]].iStartAddr)],
    					 m_Data[id_list[id]].pucTable[(addr-m_Data[id_list[id]].iStartAddr+1)]);

	return true;
}
bool BulkRead::GetDwordValue(int id, int addr, long *data)
{
    if(id_list[id] == -1)
        return false;

    if(m_Data[id_list[id]].iError == -1)
    	return false;

    if(addr < m_Data[id_list[id]].iStartAddr || m_Data[id_list[id]].iStartAddr + m_Data[id_list[id]].iLength-1 < addr)
        return false;

    *data = DXL_MAKEDWORD(DXL_MAKEWORD(m_Data[id_list[id]].pucTable[(addr - m_Data[id_list[id]].iStartAddr)],
    								   m_Data[id_list[id]].pucTable[(addr - m_Data[id_list[id]].iStartAddr+1)]),
                          DXL_MAKEWORD(m_Data[id_list[id]].pucTable[(addr - m_Data[id_list[id]].iStartAddr+2)],
                        		   	   m_Data[id_list[id]].pucTable[(addr - m_Data[id_list[id]].iStartAddr+3)]));

	return true;
}
bool BulkRead::GetValue(int id, int addr, int length, unsigned char *data)
{
	return true;
}

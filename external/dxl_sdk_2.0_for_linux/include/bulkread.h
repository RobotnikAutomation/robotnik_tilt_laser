/*
 * bulkreaddata.h
 *
 *  Created on: 2013. 1. 11.
 *      Author: zerom
 */

#ifndef BULKREAD_H_
#define BULKREAD_H_

#include <stdio.h>
#include <vector>
#include "dynamixel.h"


namespace DXL_PRO
{


class BulkRead
{
private:
	Dynamixel *m_DXL;
	std::vector<BulkReadData> m_Data;
    int id_list[255];

public:
	BulkRead(Dynamixel *dxl);

	virtual ~BulkRead();

	bool AddBulkReadData(int id, int addr, int length);
	bool ChangeBulkReadData(int id, int addr, int length);
	void ClearBulkReadData();

	int SendTxPacket();

	bool GetByteValue(int id, int addr, int *data);
	bool GetWordValue(int id, int addr, int *data);
	bool GetDwordValue(int id, int addr, long *data);
	bool GetValue(int id, int addr, int length, unsigned char *data);

};

} /* namespace Thor */
#endif /* BULKREADDATA_H_ */

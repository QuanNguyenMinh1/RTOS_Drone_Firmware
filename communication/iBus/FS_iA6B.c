/*
 * FS_iA6B.c
 *
 *  Created on: Nov 4, 2025
 *      Author: Quan
 */

#include "FS_iA6B.h"


FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len)
{
	unsigned short chksum = 0xffff;

	for(int i=0;i<len-2;i++)
	{
		chksum = chksum - data[i];
	}

	return ((chksum&0x00ff)==data[30]) && ((chksum>>8)==data[31]);
}

void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus)
{
	iBus->RH = (data[2] | data[3]<<8) & 0x0fff;	// CH1
	iBus->RV = (data[4] | data[5]<<8) & 0x0fff;	// CH2
	iBus->LV = (data[6] | data[7]<<8) & 0x0fff;	// CH3
	iBus->LH = (data[8] | data[9]<<8) & 0x0fff;	// CH4
	iBus->VrA = (data[10] | data[11]<<8) & 0x0fff;	// CH5
	iBus->VrB = (data[12] | data[13]<<8) & 0x0fff;	// CH6
	iBus->SwA = (data[14] | data[15]<<8) & 0x0fff;	// CH7
	iBus->SwB = (data[16] | data[17]<<8) & 0x0fff;	// CH8
	iBus->SwC = (data[18] | data[19]<<8) & 0x0fff;	// CH9
	iBus->SwD = (data[20] | data[21]<<8) & 0x0fff;	// CH10

	if (iBus->SwC == 1500)
	{
	 	iBus->FailSafe = 1;
	}
	else
	{
	 	iBus->FailSafe = 0;
	}

//	iBus->FailSafe = iBus->SwC == 1500;	// vế phải true thì vế phải trả về 1 và sau đó iBus->FailSafe = 1. Ngược lại iBus->FailSafe = 0
	// SwC được gán thẳng = 1 <=> SwC về 0% (nấc giữa)
}

unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus)
{
	return iBus->FailSafe != 0;
}

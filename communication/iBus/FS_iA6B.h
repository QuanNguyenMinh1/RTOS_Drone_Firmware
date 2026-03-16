/*
 * FS_iA6B.h
 *
 *  Created on: Nov 4, 2025
 *      Author: Quan
 */

#ifndef USERCODE_FS_IA6B_FS_IA6B_H_
#define USERCODE_FS_IA6B_FS_IA6B_H_

typedef struct _FSiA6B_iBus
{
	unsigned short RH; //Right Horizontal
	unsigned short RV; //Right Vertical
	unsigned short LV; //Left Vertical
	unsigned short LH; //Left Horizontal
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned char FailSafe;
}FSiA6B_iBus;

extern FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus);
unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus);

#endif /* USERCODE_FS_IA6B_FS_IA6B_H_ */

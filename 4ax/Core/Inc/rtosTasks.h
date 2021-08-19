#ifndef _RTOSTASKS_H
#define _RTOSTASKS_H

#define MAXTASKS 10
#define READDMP_IDX 0
#define ANGLEPID_IDX 1
#define PKTHDL_IDX 2
#define SENDSTATUS_IDX 3
#define STACKSIZE 512

typedef struct RtosTasks{
    
	unsigned int freq[MAXTASKS];
	uint8_t priority[MAXTASKS];
	uint8_t taskNum;

}tsks;

void tskStatusInit(tsks *tskAry){

	tskAry->freq[0] = 1;
	tskAry->priority[0] = 4;
	tskAry->freq[1] = 2;
	tskAry->priority[1] = 1;
	tskAry->freq[2] = 20;
	tskAry->priority[2] = 2;
	tskAry->freq[3] = 50;
	tskAry->priority[3] = 3;
}

void resetTsk(tsks tskAry , uint8_t index , unsigned int freq , uint8_t prio){

	tskAry.freq[index] = freq;
	tskAry.priority[index] = prio;

}


#endif
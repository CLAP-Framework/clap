/**
 * @file usRadar.h
 * @brief 
 * 	UDAS's ultrasonic radar,
 * 	Front radar is from FA FB .. to  FD,
 * 	rear radar is RA .. RD
 * @author yong.zhu@novauto.com.cn
 * @version 0.0.1
 * @date 2019-11-13
 */


#ifndef __USRADAR_H__
#define __USRADAR_H__

typedef float float32;
typedef double float64;

typedef char int8;
typedef unsigned char uint8;

typedef unsigned short uint16;
typedef short int16;

typedef unsigned int uint32;
typedef int int32;

#define MAP_R_FRAME 0x110	// auto rear radar detect objects
#define MAP_F_FRAME 0x111	// front radar
#define DISTANCES_FRAME 0x112	// distances for every single radar, 
				// FA - FD, RA - RD
//const uint16 radar_msw_ctr = 0x0220;

typedef struct {
	uint8 judge;		// 
	uint8 sort;		
	uint8 traceTimes;
	float mx;		// in metres;
	bool z_judge;		// object is  hight or low
	float my;
} USDetection;

typedef struct {
	uint8 channel;		// which? RA-RD
	uint8 H_S_Dist;		// 8bit hight of single radar detect distance.
	uint8 L_S_Dist;		// low bit
	uint8 H_P1_Dist;	// 8bit high of RB to RA detections.
	uint8 L_P1_Dist;	// low bit
	uint8 H_P2_Dist;	// 8bit high of RB to RC detections.
	uint8 L_P2_Dist;	// low bit
} USDistances;

#endif // __USRADAR__H__


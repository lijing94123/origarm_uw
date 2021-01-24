#ifndef MYDATA_H_
#define MYDATA_H_

#define SEGNUM 4
#define ACTNUM 4
#define SEGMENTNUM 4
#define BELLOWNUM 4

typedef struct QUATERNION_TAG{
 /*unCompressed Quaternion*/
 int16_t imuData[4];
}QUATERNION;

typedef struct SENSORDATA_TAG {
	int16_t pressure;
	int16_t distance;
	QUATERNION quaternion;
}SENSORDATA;

typedef struct COMMANDDATA_TAG {
	int16_t values[5];
	uint16_t commandType;
}COMMANDDATA;

enum COMMAND_MODE{openingCommandType, pressureCommandType};

typedef struct SPIDATA_T_TAG{
	SENSORDATA data[SEGMENTNUM][BELLOWNUM];
	char infos[10];
}SPIDATA_T;

typedef struct SPIDATA_R_TAG{
	COMMANDDATA data[SEGMENTNUM][BELLOWNUM];
	char infos[10];
}SPIDATA_R;


SPIDATA_T sensorData;
SPIDATA_R commandData;

SPIDATA_T sensorData_last;

//ABL->Pressure
float k0 = 3000;
float length0 = 0.055;

float radR = 0.06;
float radr = 0.02;
float crossA = M_PI*radr*radr;
float C1 = 6*k0*radR*0.5/crossA;

//initial parameters for one segment
float x_origin = 0;
float y_origin = 0;
float z_origin = 0.055;

//parameters for one segment
float lengthmax = 0.08;
float lengthmin = 0.03;
float alphamax =  M_PI/2;
float alphamin = -M_PI/2;

#define CONSTRAIN(x, min, max) (((x) < (min)) ? (min) :  (((x) > (max)) ? (max) : (x)))

#endif // MYDATA_H__
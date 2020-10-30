#ifndef _CLOSE_LOOP_H_
#define _CLOSE_LOOP_H_

// extern float posCommand;

typedef struct {

	float out;
	float error;
	float lastError;
	float P;
	float I;
	float D;

}pidProfile_t;

void chassisInit(void);

#endif
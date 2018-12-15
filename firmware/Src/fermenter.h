/*
 * fermenter.h
 *
 *  Created on: 15 Dec 2018
 *      Author: jcera
 */

#ifndef SRC_FERMENTER_H_
#define SRC_FERMENTER_H_

class Fermenter
{
	void (*mSample_cb)(int &cpu, int &temp0, int &temp1);
	void (*mCoolerControl)(bool state);
	void (*mHeaterControl)(bool state);
	uint32_t mCoolerDisableTime;
	int mRunCounter;
	double mSetPoint;

public:
	Fermenter(void (*sample_temp_cb)(int &cpu, int &temp0, int &temp1),
			void (*cooler_control)(bool state),
			void (*heater_control)(bool state));
	void run();
};


#endif /* SRC_FERMENTER_H_ */

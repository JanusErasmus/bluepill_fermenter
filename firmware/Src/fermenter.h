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
	enum eFermenterState
	{
		OFF,
		COOLING,
		HEATING
	};
	void (*mSample_cb)(double &cpu, double &temp0, double &temp1, double &temp2);
	void (*mCoolerControl)(bool state);
	void (*mHeaterControl)(bool state);
	uint32_t mCoolerDisableTime;
	int mRunCounter;
	double mSetPoint;
	eFermenterState mState;

public:
	Fermenter(void (*sample_temp_cb)(double &cpu, double &temp0, double &temp1, double &temp2),
			void (*cooler_control)(bool state),
			void (*heater_control)(bool state));
	void run();
};


#endif /* SRC_FERMENTER_H_ */

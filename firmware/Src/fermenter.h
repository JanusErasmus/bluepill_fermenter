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
	void (*mSample_cb)(double &temperature);
	void (*mCoolerControl)(bool state);
	void (*mHeaterControl)(bool state);
	uint32_t mCoolerDisableTime;
	int mRunCounter;
	double mSetPoint;
	eFermenterState mState;

public:
	Fermenter(int setpoint,
			void (*sample_temp_cb)(double &temperature),
			void (*cooler_control)(bool state),
			void (*heater_control)(bool state));
	void run();

	void set(int temp){ mSetPoint = temp; }
	int getSetpoint(){ return (int)mSetPoint; }
};


#endif /* SRC_FERMENTER_H_ */

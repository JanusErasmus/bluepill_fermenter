/*
 * fermener.c
 *
 *  Created on: 15 Dec 2018
 *      Author: jcera
 */
#include <stdio.h>
#include <math.h>

#include "stm32f1xx_hal.h"

#include "fermenter.h"

#define COOLER_REST_TIME 900000

Fermenter::Fermenter(int setpoint,
		void (*sample_cb)(double &temp),
		void (*cooler_control)(bool state),
		void (*heater_control)(bool state)) :
		mSample_cb(sample_cb),
		mCoolerControl(cooler_control),
		mHeaterControl(heater_control),
		mCoolerDisableTime(0),
		mRunCounter(0),
		mSetPoint(setpoint),
		mState(OFF)
{
	if(mHeaterControl)
		mHeaterControl(false);

	if(mCoolerControl)
		mCoolerControl(false);

	printf("Fermenter created, Set point %d\n", setpoint);
}

//Run should be called every 100ms
void Fermenter::run()
{
	//check temperatures every 10 seconds
	if(mRunCounter++ < 100)
		return;

	mRunCounter = 0;

	//we cannot do anything without temperatures
	if(!mSample_cb)
		return;

	double feedback;
	mSample_cb(feedback);

	printf("Fermenter Ctrl %0.3f -> %0.3f\n", feedback, mSetPoint);
	//try keep set point using temp (outside liquid)
	switch(mState)
	{
	//this will either start cooling or heating the fermenter
	case OFF:
		if(feedback > (mSetPoint + 1))
		{
			//we can only enable the cooler after it was off for the rest time
			if(mCoolerDisableTime)
			{
				uint32_t disbaledDelta = HAL_GetTick() - mCoolerDisableTime;
				if(disbaledDelta > COOLER_REST_TIME)
				{
					if(mCoolerControl)
						mCoolerControl(true);
					mState = COOLING;
				}
				else
				{
					printf("Cooler was only off for %d seconds\n", (int)(disbaledDelta / 1000));
				}
			}
			else
			{
				//start checking cooler disable time
				mCoolerDisableTime = 1;
				if(mCoolerControl)
					mCoolerControl(true);
				mState = COOLING;
			}
		}
		break;
	case COOLING:
		if(feedback < (mSetPoint - 1))
		{
			if(mCoolerControl)
				mCoolerControl(false);
			if(mCoolerDisableTime)
				mCoolerDisableTime = HAL_GetTick();

			mState = OFF;
		}
		break;

	case HEATING:
		mState = OFF;
		break;
	}
}


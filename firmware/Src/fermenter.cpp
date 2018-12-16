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

Fermenter::Fermenter(void (*sample_cb)(double &cpu, double &temp0, double &temp1, double &temp2),
		void (*cooler_control)(bool state),
		void (*heater_control)(bool state)) :
		mSample_cb(sample_cb),
		mCoolerControl(cooler_control),
		mHeaterControl(heater_control),
		mCoolerDisableTime(COOLER_REST_TIME + 1),
		mRunCounter(0),
		mSetPoint(19),
		mState(OFF)
{
	if(mHeaterControl)
		mHeaterControl(false);

	if(mCoolerControl)
		mCoolerControl(false);
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

	double cpu, temp0, temp1, temp2;
	mSample_cb(cpu, temp0, temp1, temp2);
	printf("Fermenter run %0.3f %0.3f %0.3f %0.3f\n", cpu, temp0, temp1, temp2);

	double feedback = round(temp1);
	printf("Fermenter Ctrl %0.3f -> %0.3f\n", feedback, mSetPoint);
	//try keep set point using temp1 (outside liquid)
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
					printf("Cooler was only off for %d seconds\n", (int)disbaledDelta);
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


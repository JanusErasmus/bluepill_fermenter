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

Fermenter::Fermenter(void (*sample_cb)(int &cpu, int &temp0, int &temp1),
		void (*cooler_control)(bool state),
		void (*heater_control)(bool state)) :
		mSample_cb(sample_cb),
		mCoolerControl(cooler_control),
		mHeaterControl(heater_control),
		mCoolerDisableTime(COOLER_REST_TIME + 1),
		mRunCounter(0),
		mSetPoint(19)
{
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

	int cpu, temp0, temp1;
	mSample_cb(cpu, temp0, temp1);
	printf("Fermenter run %d %d %d\n", cpu, temp0, temp1);

	double feedback = round(temp1 / 1000);
	if(mCoolerControl)
	{
		printf("Fermenter Ctrl %0.2f -> %0.2f\n", feedback, mSetPoint);
		//try keep set point using temp1 (outside liquid)
		if(feedback > (mSetPoint + 1))
		{
			//we can only enable the cooler after it was off for the rest time
			if(mCoolerDisableTime)
			{
				uint32_t disbaledDelta = HAL_GetTick() - mCoolerDisableTime;
				if(disbaledDelta > COOLER_REST_TIME)
				{
					mCoolerControl(true);
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
				mCoolerControl(true);
			}
		}

		if(feedback < (mSetPoint - 1))
		{
			mCoolerControl(false);
			if(mCoolerDisableTime)
				mCoolerDisableTime = HAL_GetTick();
		}
	}
}


/*
 * base_omni4.cpp
 *
 *  Created on: Apr 18, 2021
 *      Author: Owner
 */
/*#include <omni4_gyro_smd.h>
#define INT16_MAX_VALUE 32767
*/
/*Omni4::MotorSpeed Omni4::calcMotorSpeed(float (&v)[RECIEVE_DATA_NUM], const Params& params){
	static Omni4::MotorSpeed retval={0};
	static float convMat[4][3] = {
			{ 0.5*SQRT2, 0.5*SQRT2, params.dtoc},
			{ 0.5*SQRT2, -0.5*SQRT2, params.dtoc},
			{ -0.5*SQRT2, -0.5*SQRT2, params.dtoc},
			{ -0.5*SQRT2, 0.5*SQRT2, params.dtoc}
	};

	//calculate each speed of rotation
	for(int i=0; i<MOTOR_NUM; i++){
		for(int j=0; j<RECIEVE_DATA_NUM; j++){
			retval[i] += convMat[i][j]*v[j];
		}
	}

	return retval;
}*/

/*Omni4::Encoder Omni4::getEncoder(Omni4::Params& pa){
	static Encoder enc;
	static uint16_t enc_buf[MOTOR_NUM]={0};

	enc_buf[0] = __HAL_TIM_GET_COUNTER(pa.enc_handler1); //get the value of the counter
	__HAL_TIM_SetCounter(pa.enc_handler1, 0); //reset the value
*/
	/*if(enc_buf[0] > INT16_MAX_VALUE) enc.e[0] = (int16_t)enc_buf[0];
	else enc.e[0] = (int16_t)enc_buf[0];*/
/*	enc.e[0] = (int16_t)enc_buf[0];

	return enc;
}

void Omni4::init(Omni4::Params& pa){
	HAL_TIM_Encoder_Start(pa.enc_handler1, TIM_CHANNEL_ALL); //start encoder

}

*/


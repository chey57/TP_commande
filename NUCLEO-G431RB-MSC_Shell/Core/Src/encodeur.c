/**
 * @file : encodeur.c
 *
 *  Created on: Nov 28, 2022
 *      Author: cheye
 */

#include "main.h"
#include "encodeur.h"
#include "math.h"
#include "stm32g4xx_it.h"

extern uint32_t counter;
float vitesse[1]={0};
extern TIM_HandleTypeDef htim4;



/**
  * @brief donne la vitesse de rotation du moteur
  * @param None
  * @retval None
  */
void vitesse_de_rotation(){

	if(htim4.Instance->CNT != 0){

		int variable =0;

	}

	vitesse[0] = (1/4096.0)*(htim4.Instance->CNT - 32767.0) /(0.1/60);
	htim4.Instance->CNT=32767.0;



}

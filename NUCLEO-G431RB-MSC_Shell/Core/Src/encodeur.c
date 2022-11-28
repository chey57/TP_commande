/*
 * encodeur.c
 *
 *  Created on: Nov 28, 2022
 *      Author: cheye
 */

#include "main.h"

extern uint32_t counter;
extern uint16_t vitesse;
extern TIM_HandleTypeDef htim4;

void vitesse_de_rotation(){

	vitesse = (1/2048)*htim4.Instance->CNT/(0,1/60);
	htim4.Instance->CNT=0;

}

/*
 * encodeur.c
 *
 *  Created on: Nov 28, 2022
 *      Author: cheye
 */

#include "main.h"
#include "encodeur.h"
#include "math.h"
#include "stm32g4xx_it.h"

extern uint32_t counter;
uint16_t vitesse = 0;
extern TIM_HandleTypeDef htim4;



void vitesse_de_rotation(){

	vitesse = (1/2048)*htim4.Instance->CNT /(0.1/60);
	htim4.Instance->CNT=0;

}

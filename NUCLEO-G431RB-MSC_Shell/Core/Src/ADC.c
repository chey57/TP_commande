/*
 * ADC.c
 *
 *  Created on: Dec 5, 2022
 *      Author: cheye
 */

#include "ADC.h"
#include "main.h"



float lecture_courant(uint16_t ADC_Buffer_courant[ADC_BUF_SIZE]){
	int sortie_ADC_numerique =0;
	for(int i=0;i<20;i++){
		sortie_ADC_numerique = sortie_ADC_numerique + (int)(ADC_Buffer_courant[i]);
	}
	int moyenne_sortie_ADC_numerique = sortie_ADC_numerique/20;
	float tension_sortie_hacheur = moyenne_sortie_ADC_numerique * 3.3 / 4095;
	float courant = (tension_sortie_hacheur -2.53)*12;
	return (courant);
}

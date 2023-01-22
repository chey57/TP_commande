/**
 * @file : ADC.c
 *
 *  Created on: Dec 5, 2022
 *      Author: cheye
 */

#include "ADC.h"
#include "main.h"


/**
 * @brief Fonction qui permet de lire le courant en sortie du moteur
 * @note  La fonction calcule la valeur moyenne du courant de sortie du moteur à partir de la tension en sortie du hacheur
 * @param  ADC_Buffer_courant : Buffer qui permet de stocker les valeurs stockées dans le DMA après le passage dans l'ADC
 * @retval float courant : courant de sortie du moteur
 */


float lecture_courant(uint16_t* ADC_Buffer_courant){
	int sortie_ADC_numerique =0;
	for(int i=0;i<20;i++){
		sortie_ADC_numerique = sortie_ADC_numerique + (int)(ADC_Buffer_courant[i]);
	}
	int moyenne_sortie_ADC_numerique = sortie_ADC_numerique/20;
	float tension_sortie_hacheur = moyenne_sortie_ADC_numerique * 3.3 / 4095;
	float courant = (tension_sortie_hacheur -2.53)*12;
	return (courant);
}

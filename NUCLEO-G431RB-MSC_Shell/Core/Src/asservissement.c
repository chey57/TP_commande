/*
 * asservissement.c
 *
 *  Created on: Dec 5, 2022
 *      Author: cheye
 */



#include "ADC.h"
#include "main.h"

#define ADC_BUF_SIZE 20
#define K 1
#define I 1
#define Te 0.1


float commande_courant(int courant_mA, uint16_t ADC_Buffer_courant[ADC_BUF_SIZE],float epsilon_courant_n_1[1],float alpha2_n_1[1]){

	// recupération des valeurs de courant
	float courant_A = courant_mA/1000.0;
	float courant_mesure = lecture_courant(ADC_Buffer_courant);

	//Calcul espilon n
	float epsilon = courant_A-courant_mesure;

	//Calcul alpha1
	float alpha1 = K*epsilon;

	//Calcul de alpha2
	float alpha2=alpha2_n_1[0]+I*Te/2.0*(epsilon+epsilon_courant_n_1[0]);

	//Calcul de alpha
	float alpha = alpha1+alpha2;

	//stockage des valeurs epsilon_courant_n_1 et alpha2_n_1
	epsilon_courant_n_1[0] = epsilon;
	alpha2_n_1[0] = alpha2;

	//prise en compte de l'anti winned-up

	if(alpha>1){

		alpha2_n_1[0] = 1;
		return alpha=1;
	}
	else if (alpha<0){

		alpha2_n_1[0] = 0;
		return alpha=0;

	}
	else {
		return alpha;
	}



}


/*
 * asservissement.c
 *
 *  Created on: Dec 5, 2022
 *      Author: cheye
 */



#include "ADC.h"
#include "main.h"

#define ADC_BUF_SIZE 20
#define K 0
#define I 0.00118870172705228
#define Te 0.0000625

/**
 * @brief fonction callback du bouton bleu
 * @note  La fonction est appelee lorsque le programme détecte l'entrée d'une consigne de courant de l'utilisateur sur le shell
 * @param  courant_mA : consigne de courant
 * mesure_courant : courant mesuré avec une fréquence de 16kHz
 * epsilon_courant_n_1[1] : écart entre la consigne et la mesure de l'itération précédente
 * alpha2_n_1[1] : sortie de l'intégrateur de l'itération précédente
 * @retval float alpha : rapport cyclique à appliquer pour effectuer l'asservissement
 */

float commande_courant(int courant_mA, float mesure_courant,float epsilon_courant_n_1, float alpha2_n_1){

	// recupération des valeurs de courant
	float courant_A = courant_mA/1000.0;
	//float courant_mesure = lecture_courant(ADC_Buffer_courant);
	float courant_mesure = mesure_courant;

	//Calcul espilon n
	float epsilon = courant_A-courant_mesure;

	//Calcul alpha1
	float alpha1 = K*epsilon;

	//Calcul de alpha2
	float alpha2=alpha2_n_1+I*Te/2.0*(epsilon+epsilon_courant_n_1);

	//Calcul de alpha
	float alpha = alpha1+alpha2;

	//stockage des valeurs epsilon_courant_n_1 et alpha2_n_1
	epsilon_courant_n_1 = epsilon;
	alpha2_n_1 = alpha2;

	//prise en compte de l'anti winned-up

	if(alpha>1){

		alpha2_n_1 = 0;
		return alpha=1;
	}
	else if (alpha<0){

		alpha2_n_1 = 0;
		return alpha=0;

	}
	else {
		return alpha;
	}



}


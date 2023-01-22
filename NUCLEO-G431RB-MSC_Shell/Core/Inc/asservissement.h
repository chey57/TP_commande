/**
 * @file : asservissement.h
 *
 *  Created on: Dec 5, 2022
 *      Author: cheye
 */

#ifndef INC_ASSERVISSEMENT_H_
#define INC_ASSERVISSEMENT_H_

float commande_courant(int courant_mA, float mesure_courant,float epsilon_courant_n_1,float alpha2_n_1);

#endif /* INC_ASSERVISSEMENT_H_ */

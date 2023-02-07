/*
 * espdatalogger.h
 *
 *  Created on: 5 Feb 2023
 *      Author: bvick
 */

#ifndef INC_ESPDATALOGGER_H_
#define INC_ESPDATALOGGER_H_

#include <stdio.h>

void bufclr (char *buf);
void ESP_Init (char *SSID, char *PASSWD);
void ESP_Send_Data (char *APIkey, int Field_num, int value);
void ESP_Send_Multi (char *APIkey, int numberoffileds, int value[]);

#endif /* INC_ESPDATALOGGER_H_ */

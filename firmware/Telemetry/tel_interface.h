/*
 * tel_interface.h
 *
 *  Created on: Feb 2, 2025
 *      Author: dkiovics
 */

#ifndef TEL_INTERFACE_H_
#define TEL_INTERFACE_H_

uint8_t tel_initEsp(void);

void tel_sendDataToEsp(void);

void tel_sendTextToEsp(const char* text);

#endif /* TEL_INTERFACE_H_ */

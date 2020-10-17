/*
 * dfu.h
 *
 *  Created on: Oct 17, 2020
 *      Author: ksstms
 */

#ifndef DFU_DFU_H_
#define DFU_DFU_H_

void enterDfuMode();
void rebootIntoDfu(const char magicWord[4]);
void setMagicWord(const char magicWord[4]);
uint8_t checkMagicWord(const char magicWord[4]);

#endif /* DFU_DFU_H_ */

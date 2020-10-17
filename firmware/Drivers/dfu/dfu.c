/*
 * dfu.c
 *
 *  Created on: Oct 17, 2020
 *      Author: ksstms
 *
 * Use this library to access the STM32F103's built-in Device Firmware Updater.
 */

#include "stm32f1xx_hal.h"

#define DFU_BASE_ADDRESS  0x1ffff000    // refer to PM0075
#define DFU_STACK_POINTER 0xfc010020    // SP found at DFU_BASE_ADDRESS

/**
 * Set the stack pointer and jump to the Device Firmware Updater's address
 * This function should be called only before any device initialization!
 */
void enterDfuMode() {
    void (*bootDfu)(void);
    bootDfu = (void (*)(void))(*((uint32_t*)(DFU_BASE_ADDRESS + 4)));
    __set_MSP(DFU_STACK_POINTER);
    bootDfu();
}

/**
 * Disable write protection of backup registers
 * See Backup registers section in RM0008
 */
void disableBackupWriteProtection() {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
}

/**
 * Write a 4-byte string to backup registers DR1-DR4
 * @param s word to write
 */
void setMagicWord(const char magicWord[4]) {
    disableBackupWriteProtection();

    BKP->DR1 = magicWord[0];
    BKP->DR2 = magicWord[1];
    BKP->DR3 = magicWord[2];
    BKP->DR4 = magicWord[3];
}

/**
 * Check if backup registers DR1-DR4 match a 4-byte string
 * @param magicWord to check
 * @return 0 if DRx != magicWord[x-1]
 */
uint8_t checkMagicWord(const char magicWord[4]) {
    return (BKP->DR1 == magicWord[0] &&
            BKP->DR2 == magicWord[1] &&
            BKP->DR3 == magicWord[2] &&
            BKP->DR4 == magicWord[3]   );
}

/**
 * Set the magic word and reset the processor
 */
void rebootIntoDfu(const char magicWord[4]) {
    setMagicWord(magicWord);
    HAL_NVIC_SystemReset();
}

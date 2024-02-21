/*
* Upload software for the robot
* Only a single FTDI device (robot) should be connected during upload
* CBUS3 - RESET (active high)
* CBUS2 - BOOT0 (active high)
*/


#include <iostream>
#include <windows.h>
#include "ftd2xx.h"
#include <stdexcept>
#include <chrono>
#include <thread>
#include <string>



int main(int argc, char* argv[]) {                          //COM port     Baud rate      BIN file path
                                                            //                                                  OR
                                                            //Baud rate      BIN file path          <--------- use default (first FTDI device) COM port number
    if (argc != 4 && argc != 3) {
        std::cout << "Num of arguments: %d\n" << argc << "Error: Upload params should be: COM_PORT BAUD_RATE BIN_FILE_PATH   or   BAUD_RATE BIN_FILE_PATH\n" << std::endl;
        return -1;
    }
    
    FT_STATUS ftStatus;
    FT_HANDLE ftHandle;

    std::cout << "TanfolyamRobot loader V2.1\n" << std::endl;
    std::cout << "WARNING: Make sure only one FTDI device is plugged in!\n" << std::endl;

    ftStatus = FT_Open(0, &ftHandle);
    if (ftStatus != FT_OK) {
        std::cout << "FT232 device not found!\n" << std::endl;
        return 1;
    }
    else {
        std::cout << "Connection with FT232 successfull\n" << std::endl;
    }

    try {
        if (FT_SetBitMode(ftHandle, 0xCC, 0x20) != FT_OK)
            throw std::runtime_error("Failed to reset device");
        std::cout << "Device reset successfull\n" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        if (FT_SetBitMode(ftHandle, 0xC4, 0x20) != FT_OK)
            throw std::runtime_error("Failed to init device");
        std::cout << "Device in bootloader mode\n" << std::endl;

        LONG comPortNumber = 0;
        if (argc == 3) {
            if ((ftStatus = FT_GetComPortNumber(ftHandle, &comPortNumber)) == FT_OK) {
                if (comPortNumber == -1) {
                    throw std::runtime_error("Failed to detect device COM port number");
                }
                else {
                    std::cout << "Detected device COM port number is " << comPortNumber << std::endl;
                }
            }
            else {
                std::cout << "FT_GetComPortNumber failed with error " << ftStatus << std::endl;
                return 1;
            }
        }

        FT_Close(ftHandle);

        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        std::string cmd;
        if(argc == 4)
            cmd = std::string("stm32flash") + " -b " + argv[2] + " -w " + argv[3] + " -v -g 0x0 " + argv[1];
        else
            cmd = std::string("stm32flash") + " -b " + argv[1] + " -w " + argv[2] + " -v -g 0x0 " + "COM" + std::to_string(comPortNumber);
        system(cmd.c_str());

        if (FT_Open(0, &ftHandle) != FT_OK || FT_SetBitMode(ftHandle, 0xC8, 0x20) != FT_OK)
            throw std::runtime_error("Failed to reset device");
        std::cout << "Device reset successfull\n" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (FT_SetBitMode(ftHandle, 0x00, 0x20) != FT_OK)
            throw std::runtime_error("Failed to start device");
        std::cout << "Device init successfull\n" << std::endl;
    }
    catch (std::runtime_error e) {
        std::cout << e.what() << std::endl;
    }

    FT_Close(ftHandle);

    return 0;
}

/*
* Upload software for the robot
* Only a single FTDI device (robot) should be connected during upload
* CBUS3 - RESET (active high)
* CBUS2 - BOOT0 (active high)
*/


#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <string>
#include <filesystem>
#include <regex>

#include "ftd2xx.h"


// COM port     Baud rate      BIN file path
// OR
// Baud rate      BIN file path          <--------- use default (first FTDI device) COM port number
int main(int argc, char *argv[]) {
    if (argc != 4 && argc != 3) {
        std::cout << "Num of arguments: " << argc
            << "\nError: Upload params should be: COM_PORT BAUD_RATE BIN_FILE_PATH   or   BAUD_RATE BIN_FILE_PATH\n"
            << std::endl;
        return -1;
    }

    FT_STATUS ftStatus;
    FT_HANDLE ftHandle;

    std::cout << "TanfolyamRobot loader V3.0\n" << std::endl;
    std::cout << "WARNING: Make sure only one FTDI device is plugged in!\n" << std::endl;

#ifndef _WIN32
    system("sudo modprobe -r ftdi_sio");
#endif

    ftStatus = FT_Open(0, &ftHandle);
    if (ftStatus != FT_OK) {
        std::cout << "Cannot open FT232, error code: " << ftStatus << "\n" << std::endl;
        return 1;
    } else {
        std::cout << "Connection with FT232 successful\n" << std::endl;
    }

    bool success = true;
    LONG comPortNumber = 0;

    try {
        ftStatus = FT_SetBitMode(ftHandle, 0xCC, 0x20);
        if (ftStatus != FT_OK)
            throw std::runtime_error("Failed to reset STM32, FTDI error code: " + std::to_string(ftStatus));
        std::cout << "STM32 reset successful\n" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        ftStatus = FT_SetBitMode(ftHandle, 0xC4, 0x20);
        if (ftStatus != FT_OK)
            throw std::runtime_error("Failed to init STM32, FTDI error code: " + std::to_string(ftStatus));
        std::cout << "STM32 is in bootloader mode\n" << std::endl;

#ifdef _WIN32
        if (argc == 3) {
            ftStatus = FT_GetComPortNumber(ftHandle, &comPortNumber);
            if (ftStatus == FT_OK) {
                if (comPortNumber == -1) {
                    throw std::runtime_error("Failed to detect device COM port number");
                } else {
                    std::cout << "Detected device COM port number is " << comPortNumber << "\n" << std::endl;
                }
            } else {
                throw std::runtime_error("FT_GetComPortNumber failed, error code: " + std::to_string(ftStatus));
            }
        }
#endif
    }
    catch (std::runtime_error e) {
        success = false;
        std::cout << e.what() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ftStatus = FT_SetBitMode(ftHandle, 0x00, 0x20);
    if (ftStatus != FT_OK)
    {
        std::cout << "Failed to restore default GPIO state, error code: " << ftStatus << "\n" << std::endl;
    }

    ftStatus = FT_Close(ftHandle);
    if (ftStatus != FT_OK) {
        std::cout << "Cannot close FT232, error code: " << ftStatus << "\n" << std::endl;
        return 1;
    }

    if (!success)
        return 1;

#ifndef _WIN32
    system("sudo modprobe ftdi_sio");
    system("sleep 1");

    std::string highestTtyUSB;
    int highestNumber = -1;
    std::regex ttyUSBPattern("/dev/ttyUSB([0-9]+)");

    for (const auto &entry : std::filesystem::directory_iterator("/dev")) {
        std::smatch match;
        std::string path = entry.path().string();
        if (std::regex_search(path, match, ttyUSBPattern)) {
            int number = std::stoi(match[1].str());
            if (number > highestNumber) {
                highestNumber = number;
                highestTtyUSB = path;
            }
        }
    }

    if (highestNumber == -1) {
        std::cout << "No /dev/ttyUSB* device found, using 0" << std::endl;
        comPortNumber = 0;
    } else {
        comPortNumber = highestNumber;
        std::cout << "Detected highest /dev/ttyUSB* device: " << highestTtyUSB << "\n" << std::endl;
    }
#endif

    std::string cmd;
    if (argc == 4)
#ifdef _WIN32
        cmd = std::string("stm32flash") + " -b " + argv[2] + " -w " + argv[3] + " -v -g 0x0 " + argv[1];
#else
        cmd = std::string("./stm32flash") + " -b " + argv[2] + " -w " + argv[3] + " -v -g 0x0 " + argv[1];
#endif
    else
#ifdef _WIN32
        cmd = std::string("stm32flash") + " -b " + argv[1] + " -w " + argv[2] + " -v -g 0x0 " + "COM" + std::to_string(comPortNumber);
#else
        cmd = std::string("./stm32flash") + " -b " + argv[1] + " -w " + argv[2] + " -v -g 0x0 " + "/dev/ttyUSB" + std::to_string(comPortNumber);
#endif
    system(cmd.c_str());

    return 0;
}

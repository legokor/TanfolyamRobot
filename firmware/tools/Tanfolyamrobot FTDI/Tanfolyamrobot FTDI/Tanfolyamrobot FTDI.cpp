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



int main(int argc, char* argv[]) {                          //COM port     Baud rate      BIN file path

    if (argc != 4) {
        std::cout << "Num of arguments: %d\n" << argc << "Error: Upload params should be: COM_PORT BAUD_RATE BIN_FILE_PATH\n" << std::endl;
        return -1;
    }
    
    FT_STATUS ft_status;
    FT_HANDLE ft_handle;
    ft_status = FT_Open(0, &ft_handle);
    if (ft_status != FT_OK) {
        std::cout << "FT232 device not found!\n" << std::endl;
        return -1;
    }
    else {
        std::cout << "Connection with FT232 successfull\n" << std::endl;
    }

    try {
        if (FT_SetBitMode(ft_handle, 0xCC, 0x20) != FT_OK)
            throw std::runtime_error("Failed to reset device");
        std::cout << "Device reset successfull\n" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        if (FT_SetBitMode(ft_handle, 0xC4, 0x20) != FT_OK)
            throw std::runtime_error("Failed to init device");
        std::cout << "Device in bootloader mode\n" << std::endl;

        FT_Close(ft_handle);

        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        std::string cmd = std::string("stm32flash") + " -b " + argv[2] + " -w " + argv[3] + " -v -g 0x0 " + argv[1];
        system(cmd.c_str());

        if (FT_Open(0, &ft_handle) != FT_OK || FT_SetBitMode(ft_handle, 0xC8, 0x20) != FT_OK)
            throw std::runtime_error("Failed to reset device");
        std::cout << "Device reset successfull\n" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (FT_SetBitMode(ft_handle, 0x00, 0x20) != FT_OK)
            throw std::runtime_error("Failed to start device");
        std::cout << "Device init successfull\n" << std::endl;
    }
    catch (std::runtime_error e) {
        std::cout << e.what() << std::endl;
    }

    FT_Close(ft_handle);

    return 0;
}

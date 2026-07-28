#include <iostream>
#include "libCodeTest.h"
#include <stdio.h>

#if defined(__x86_64__) || defined(_M_X64) 
    #define SOFTWARE_VERSION "1.0.0"
    #define LIB_PATH "./lib/x64/libVDMocapSDK_mHandPro.so"
#elif defined(__aarch64__) 
   // #define SOFTWARE_VERSION "1.0.0a" 
   // #define LIB_PATH "./lib/arm64/libVDMocapSDK_mHandProArm64.so"
    #define SOFTWARE_VERSION "1.0.b" 
    #define LIB_PATH "./lib/kylin_v10_arm64/libVDMocapSDK_mHandProArm64.so"
#else 
    #define LIB_PATH ""
    return 0;
#endif


// ./lib/libVDMocapSDK_mHandPro.so

int main() 
{
    if(LIB_PATH == "")
    {
        std::cout<<"Unknown architecture!\n"<<std::endl;
        return 0;
    }

    libCodeTest libUsing(LIB_PATH);

    if (!libUsing.handle) 
	{
        std::cout<<"lib is not exist!\n"<<std::endl;
        return 0;
	}

    char userInput;

    std::cout << "Please enter the keyboard for control(press 'h' get help):" << std::endl;
    while (true) 
    {
        std::cin >> userInput;  // input command
        if (userInput == 'q') 
        {
            std::cout << "exit application..." << std::endl;
            break;
        } 
        else if(userInput == 'h')
        {
            std::cout << "software version : " << SOFTWARE_VERSION << std::endl;

            std::cout << "h : get help" << std::endl;
            std::cout << "v : get sdk version" << std::endl;
            std::cout << "q : exit application" << std::endl;
            std::cout << "c : connect device" << std::endl;
            std::cout << "d : disconnect device" << std::endl;
            std::cout << "m : start magnetic calibration" << std::endl;
            std::cout << "p : start postrue calibration" << std::endl;
            std::cout << "a : auto upload data" << std::endl;
            std::cout << "1 : get fps of device" << std::endl;
            std::cout << "2 : get quaternion of device" << std::endl;
            std::cout << "3 : get Postition of device" << std::endl;

            std::cout << "\nPlease enter the keyboard for control(press 'h' get help):" << std::endl;
        }
        else if(userInput == 'v')
        {
            libUsing.showVersionAction();
            std::cout << "\nPlease enter the keyboard for control(press 'h' get help):" << std::endl;
        }
        else if(userInput == 'c')
        {
            libUsing.connectAction();
            std::cout << "\nPlease enter the keyboard for control(press 'h' get help):" << std::endl;
        }
        else if(userInput == 'd')
        {
            libUsing.disconnectAction();
            std::cout << "\nPlease enter the keyboard for control(press 'h' get help):" << std::endl;
        }
        else if(userInput == 'm')
        {
            libUsing.MagCorrectAction();
            std::cout << "\nPlease enter the keyboard for control(press 'h' get help):" << std::endl;
        }
        else if(userInput == 'p')
        {
            libUsing.PosCorrectAction();
            std::cout << "\nPlease enter the keyboard for control(press 'h' get help):" << std::endl;
        }
        else if(userInput == 'a')
        {
            libUsing.autoGetDataAction();
        }
        else if(userInput == '1')
        {
            libUsing.autoGetFPSAction();
        }
        else if(userInput == '2')
        {
            libUsing.autoGetQuaternionAction();
        }
        else if(userInput == '3')
        {
            libUsing.autoGetPostitionAction();
        }
        else 
        {
            std::cout << "The button you pressed was :" << userInput << std::endl;
            std::cout << "This command is unsupport!" << std::endl;
            std::cout << "\nPlease enter the keyboard for control(press 'h' get help):" << std::endl;
        }
    }

    return 0;
}


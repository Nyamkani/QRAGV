/**
  ******************************************************************************
  * @file           : ecan.hpp
  * @brief          : Main Interface header file of E-CAN Interface
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024  AGV TEAM, DEPARTMENT OF ROBOT CONTROL, STUDIO3S Ltd.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @Target          : JETSON NANO (Linux nano 4.9.253-tegra #1)
  * 
  * @Changes		     : v1.00 : first release
  * 
  * @Comments
  * 
  */

#ifndef ECAN_H_
#define ECAN_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <string>

// #include <linux/can.h>
// #include <linux/can/raw.h>

#ifdef __cplusplus
}
#endif

#define ECAN_DATA_FRAME_LENGTH   14U
#define ECAN_DATA_FRAME_TYPE_NUMBER   0U
#define ECAN_DATA_FRAME_ID_NUMBER   1U
#define ECAN_DATA_FRAME_LENGTH_NUMBER   5U
#define ECAN_DATA_FRAME_DATA_NUMBER   6U





/*ECAN DATA TYPE*/
enum ECANDataFrameType
{
    tSTDData = 0x04,
    tSTDRemote = 0x05,
    tEXTData = 0x06,
    tEXTRemote = 0x07,

    tErr = 0xff,

};



enum CANIdType
{
    tNMT = 0,
    tNMTErr = 1,
    tBootUp = 2,
    tSync = 3,
    tEmergency = 4,
    tTimeStamp = 5,
    tWPDO = 6,
    tRPDO = 7,
    tWSDO = 8,
    tRSDO = 9,
};

/* Input data */
typedef struct 
{
    int id;
    int data_length;
    char data[8];
}CANDataStructure;



class ECANInterface
{
    private:
        std::string addr_;
        int sockfd;
        int port_ = 0;

        int send_frame_type_ = ECANDataFrameType::tSTDData;


    public:

    private:

        int Send(CANDataStructure can_data);
        int Recv(CANDataStructure& can_data);

    public:
        ECANInterface();
        ECANInterface(char* addr, int port);
        ECANInterface(char* addr, int port, int send_frame_type);
        virtual ~ECANInterface();

        int Initialize();
        int Write(const CANDataStructure can_data);
        int Read();


};



#endif  // ECAN_H_

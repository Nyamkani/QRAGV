/**
  ******************************************************************************
  * @file           : pgv100_define.hpp
  * @brief          : Macro hpp file of PGV100 Position Sensor
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



/*USER UART CONFIGURATION*/
#define USE_UART_PORT                       UART_PORT_1

#define READ_TIMEOUT_MICRO                      100000

#define MAX_FILTER_CNT                          5

#define DEBUG_PRINT     0

/**
 * @brief PNF Position Sensor response data length
 * 
 */
enum PNFResponseDataLength
{
	pPGV100Color = 2,
	pPGV100Dir = 3,
	pPGV100Pos = 21,

	// kPCV80Pos = 9,
};

/**
 * @brief PNF Request Command Telegram
 * 
 */
enum PNFPGV100RequestTelegram
{
	//--------------------------------------------------PGV100 Commands
	//Write Comm. cmd
	cPGV100DirRightRequest = 0xE4,                   //for Reqeusting  changing  right direction
	cPGV100DirLeftRequest = 0xE8,                    //for Reqeusting  changing  left  direction
	cPGV100DirStraightRequest = 0xEC,                //for Reqeusting  changing  straight  direction

	cPGV100ColorRedRequest = 0x90,                              //for Reqeusting  changing  RED direction
	cPGV100ColorGreenRequest = 0x88,                            //for Reqeusting  changing  GREEN direction
	cPGV100ColorBlueRequest = 0xC4,                             //for Reqeusting  changing  BLUE direction

	cPGV100PosRequest = 0xC8,                         //for Reqeusting messages    from head to receive POSITON
};



enum PNFPosUnit
{
	pDecimeter = 1,
	pMilimeter = 10,
	pMeter = 10000,
};


struct PosSensorDataStruct
{
    bool is_detected = false;
    bool is_tagged = false;

    /*unique value*/
    size_t tag_code = 0;
    size_t error_code = 0;
    size_t pgv100_dir = 0;
    size_t pgv100_color = 0;
    int xpos = 0;
    int ypos = 0;
    int angle = 0;
};

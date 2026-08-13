/**
  ******************************************************************************
  * @file           : isv2motor_define.hpp
  * @brief          : Macro hpp file of isv2motor Control
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


#ifndef KSSBOT_HARDWARE__ISV2MOTOR_DEFINE_HPP_
#define KSSBOT_HARDWARE__ISV2MOTOR_DEFINE_HPP_



/* Debug */
#define SEND_CAN_DPRINT       0
#define RECV_CAN_DPRINT       0

#define STATUS_MOTOR_DPRINT     0

#define LEFT_MOTOR_NODE         1
#define RIGHT_MOTOR_NODE        2


enum Isv2MotorCommand
{
  cSetMoveVel = 0x01,
  cSetMoveAcc = 0x02,
  cSetMoveDec = 0x03,
  cSetMaxMoveVel = 0x04,

  cSetMoveStop = 0x10,
};

enum CANIDType
{
  sRSDO = 0x580,
  sWSDO = 0x600,

};

enum Isv2MotorModeOperation
{
  sPosMode = 1,
  sVelMode = 3,
  sTorqueMode = 4,
  sHormingMode = 6,
};

enum Isv2MotorControlWord
{
  sPowerOn = 0x06,
  sMotorStart = 0x07,
  sServoOn = 0x0f,
  sAlarmReset = 0x80
};

enum Isv2MotorDir
{
  sCW = -1,
  sUndefined = 0,
  sCCW = 1,
};


struct Isv2MotorCommandStruct
{
  int id;
  int command;
  double data;
};

struct Isv2MotorStruct
{
  /*id index*/
  int node_id = 0;

  /*software motor direction*/
  int is_motor_reversed_ = 0;

  /*command queue*/
  std::vector<Isv2MotorCommandStruct> command_queue;

  /*status word - 6041h*/
  bool is_servo_ready = false; //servo ready
  bool is_servo_on = false; //servo running
  bool is_error = false;    //fault
  bool is_inpos = false;    //arrived at pos & servo running
  bool is_qstop = false;    //quick stop

  /*Current Motor values*/
  int current_motor_dir = Isv2MotorDir::sUndefined;
  int current_motor_acc = 0;
  int current_motor_dec = 0;
  int current_motor_max_speed = 0;
  int current_motor_speed = 0;
  int current_motor_pos = 0;

  /*target Motor values*/
  int motor_dest = 0;
  int motor_speed = 0;
  
  int status = 0;


  /*constructor*/
  Isv2MotorStruct(int id)
  {
    this->node_id = id;
  }

  Isv2MotorStruct(int id, int motor_reversed)
  {
    this->node_id = id;

    this->is_motor_reversed_ = motor_reversed;
  }

  Isv2MotorStruct(){}

};


struct MotorDataStruct
{
  bool motor_move_ = false;

  int left_target_pos = 0;
  int right_target_pos = 0;

  int left_current_motor_speed = 0;
  int right_current_motor_speed = 0;

  int left_current_motor_pos = 0;
  int right_current_motor_pos = 0;

  bool is_motor_stopped = true;

  int status = 0;
};




#endif  // KSSBOT_HARDWARE__ISV2MOTOR_DEFINE_HPP_

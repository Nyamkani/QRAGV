/**
  ******************************************************************************
  * @file           : isv2motor_command.cpp
  * @brief          : Define command cpp file of isv2motor Control
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


#include "isv2motor/isv2motor.hpp"
#include "isv2motor/isv2motor_define.hpp"




/*-------------------NMT---------------------*/

//(int id_type, int id, int data_length, int index, int sub_index, int data)

void CANIdInstantReset()
{
  MCP2515_CANsend(MakingSendCANDataStruct(CANIdType::tNMT, 0, 8, 0x81, 0, 0));

  return;
}

void CANIdInstantResetComm(int id)
{
  MCP2515_CANsend(MakingSendCANDataStruct(CANIdType::tNMT, 0, 8, 0x82, id, 0));

  return;
}

void CANIdInstantRegister(int id)
{
  MCP2515_CANsend(MakingSendCANDataStruct(CANIdType::tNMT, 0, 8, 0x01, id, 0));

  return;
}

void CANIdSInstantetPreOpMode(int id)
{
  MCP2515_CANsend(MakingSendCANDataStruct(CANIdType::tNMT, 0x80, 8, 0x01, id, 0));

  return;
}




/*-------------------SDO---------------------*/
// void isv2motor::EnqueueSendCANData(const CAN_data_struct send_data)
// {
//   this->send_cmd_queue_.push_back(send_data);

//   return;
// }



/*-------------------Params---------------------*/
/*instant proccess*/
void isv2motor::CANIdInstantSetControlWord(const int id, const int status)
{
  CAN_data_struct send_data_buf = MakingSendCANDataStruct(CANIdType::tWSDO, 
                                                      id, 
                                                      8,
                                                      0x6040,
                                                      0,
                                                      status
                                                      );

  this->CANIdInstantProcess(send_data_buf);

  return;
}

void isv2motor::CANIdInstantSetModeOperation(const int id, const int mode)
{
  CAN_data_struct send_data_buf = MakingSendCANDataStruct(CANIdType::tWSDO, 
                                                      id, 
                                                      8,
                                                      0x6060,
                                                      0,
                                                      mode
                                                      );

  this->CANIdInstantProcess(send_data_buf);

  return;
}

void isv2motor::CANIdInstantSetAccel(const int id, const int value)
{
  CAN_data_struct send_data_buf = MakingSendCANDataStruct(CANIdType::tWSDO, 
                                                      id, 
                                                      8,
                                                      0x6083,
                                                      0,
                                                      value
                                                      );

  this->CANIdInstantProcess(send_data_buf);

  return;
}

void isv2motor::CANIdInstantSetDecel(const int id, const int value)
{
  CAN_data_struct send_data_buf = MakingSendCANDataStruct(CANIdType::tWSDO, 
                                                      id, 
                                                      8,
                                                      0x6084,
                                                      0,
                                                      value
                                                      );

  this->CANIdInstantProcess(send_data_buf);

  return;
}

void isv2motor::CANIdInstantMoveStop(const int id)
{
  CAN_data_struct send_data_buf = MakingSendCANDataStruct(CANIdType::tWSDO, 
                                                      id, 
                                                      8,
                                                      0x60ff,
                                                      0,
                                                      0
                                                      );

  this->CANIdInstantProcess(send_data_buf);

  return;
}


// void isv2motor::RPDOParsing(const CAN_data_struct recved_data)
// {
//   int node_id = recved_data.id;


/*-------------------SDO---------------------*/
// void isv2motor::CANIdSetPolarity(const int id, const int value)
// {
//   CAN_data_struct send_data = MakingSendCANDataStruct(CANIdType::tWSDO, 
//                                                       id, 
//                                                       8,
//                                                       0x607E,
//                                                       0,
//                                                       value
//                                                       );

//   this->EnqueueSendCANData(send_data);

//   return;
// }

// void isv2motor::CANIdReadPolarity(const int id)
// {
//   CAN_data_struct send_data = MakingSendCANDataStruct(CANIdType::tRSDO, 
//                                                       id, 
//                                                       8,
//                                                       0x607E,
//                                                       0,
//                                                       0
//                                                       );

//   this->EnqueueSendCANData(send_data);

//   return;
// }

// void isv2motor::CANIdSetVel(const int id, const int value)
// {
//   int value_ = value;

//     Isv2MotorStruct* target_motor = GetIsv2MotorStruct(id);

//     value_ *= target_motor->is_motor_reversed_;

//     CAN_data_struct send_data = MakingSendCANDataStruct(CANIdType::tWSDO, 
//                                                         id, 
//                                                         8,
//                                                         0x60ff,
//                                                         0,
//                                                         value_
//                                                         );

//     this->EnqueueSendCANData(send_data);

//   return;
// }

void isv2motor::CANIdInstantSetVel(const int id, const int value)
{
  int value_ = value;

  Isv2MotorStruct* target_motor = GetIsv2MotorStruct(id);

  value_ *= target_motor->is_motor_reversed_;

  CAN_data_struct send_data_buf = MakingSendCANDataStruct(CANIdType::tWSDO, 
                                                        id, 
                                                        8,
                                                        0x60ff,
                                                        0,
                                                        value_
                                                        );

  this->CANIdInstantProcess(send_data_buf);

  return;
}




int isv2motor::CANIdInstantProcess(const CAN_data_struct send_data)
{
  CAN_data_struct send_data_buf = send_data;

  int ritr = 0, witr = 0;

  int result = -1;

  while(witr++ < 3)
  {
    MCP2515_CANsend(send_data_buf);

    usleep(1000);

    while(ritr++ < 3)
    {
      /*Read data and check error*/
      CAN_data_struct recv_data_buf = {0,};

      MCP2515_CANRecv(&recv_data_buf);

      if((send_data_buf.id - CANIDType::sWSDO) == (recv_data_buf.id - CANIDType::sRSDO) &&
          send_data_buf.data[1] == recv_data_buf.data[1] &&
          send_data_buf.data[2] == recv_data_buf.data[2])
      {
        result = 0;

        return result;
      }
      else
      {
        this->recv_data_queue_.push_back(recv_data_buf);
      }

      usleep(1000);
    }

    usleep(1000);
  }

  return result;
}

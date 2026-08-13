/**
  ******************************************************************************
  * @file           : isv2motor.cpp
  * @brief          : Main Interface cpp file of isv2motor Control
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

static bool hw_init_ = false;


isv2motor::isv2motor(){}


isv2motor::isv2motor(double motor_acc, double motor_dec)
{
  this->motor_acc_ = motor_acc;
  this->motor_dec_ = motor_dec;
}

isv2motor::isv2motor(double motor_acc, double motor_dec, double max_motor_vel)
{
  this->motor_acc_ = motor_acc;
  this->motor_dec_ = motor_dec;
  this->max_motor_vel_ = max_motor_vel;
}

isv2motor::isv2motor(DrivingController* controller)
{
  this->controller_ = controller;

  if((this->controller_))
  {
    this->motor_acc_ = this->controller_->max_acc_;
    this->motor_dec_ = this->controller_->max_dec_;
    this->max_motor_vel_ = this->controller_->max_vel_;

    this->wheel_radius_ = this->controller_->wheel_radius_;
    this->reducer_rate_ = this->controller_->reducer_rate_;
    this->motor_encoder_increment = this->controller_->encoder_ticks_;
  }
}

isv2motor::isv2motor(DrivingController* controller, int node_id, int motor_dir)
{
  this->controller_ = controller;

  if((this->controller_))
  {
    this->motor_acc_ = this->controller_->max_acc_;
    this->motor_dec_ = this->controller_->max_dec_;
    this->max_motor_vel_ = this->controller_->max_vel_;

    this->wheel_radius_ = this->controller_->wheel_radius_;
    this->reducer_rate_ = this->controller_->reducer_rate_;
    this->motor_encoder_increment = this->controller_->encoder_ticks_;

  }

  this->RegisterMotor(node_id, motor_dir);  //right- mini sCCW  big - sCW

}




isv2motor::~isv2motor()
{
  for(auto& motor : this->motor_data_)
    this->CANIdInstantMoveStop(motor->node_id);

   this->DeleteAllMotor();

    MCP2515_CANDeinit();
}




kssbot_hardware::IReturnType isv2motor::Init()
{
  if(this->is_hw_init_)
    return kssbot_hardware::IReturnType::kReturnOk;

  if(this->HardwareInitiailize() != kssbot_hardware::IReturnType::kReturnOk)
    return kssbot_hardware::IReturnType::kReturnError;

  this->is_hw_init_ = true;

  this->state_ = kssbot_hardware::LifeCycleState::kInit;

  if(this->is_init_)
    return kssbot_hardware::IReturnType::kReturnOk;

  if(this->SystemInitiailize() != kssbot_hardware::IReturnType::kReturnOk)
    return kssbot_hardware::IReturnType::kReturnError;

  this->is_init_ = true;

  int status = 0;

  int cnt = 0;

  while(status < 0)
  {
    status = this->Read();

    usleep(10*1000);

    if(cnt++ > 100)
      break;
  }

  this->state_ = kssbot_hardware::LifeCycleState::kRun;

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::StartRun()
{
  this->is_run_ = true;

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::StopRun()
{
  this->is_stop_ = true;

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::Write()
{
  for(auto& motor: this->motor_data_)
  {
    if((motor->command_queue.empty()))
      continue;

    int id = motor->node_id;
    int cmd = motor->command_queue.front().command;
    double data = motor->command_queue.front().data;

    switch(cmd)
    {
      case Isv2MotorCommand::cSetMoveVel:
      {
        data = ChangeMMsToRPM(data, this->wheel_radius_, this->motor_encoder_increment, this->reducer_rate_);

        this->CANIdInstantSetVel(id, data);

        break;
      }

      case Isv2MotorCommand::cSetMoveAcc:
      {
        data = ChangeMMsToRPM(data, this->wheel_radius_, this->motor_encoder_increment, this->reducer_rate_);

        this->CANIdInstantSetAccel(id, data);

        break;
      }

      case Isv2MotorCommand::cSetMoveDec:
      {
        data = ChangeMMsToRPM(data, this->wheel_radius_, this->motor_encoder_increment, this->reducer_rate_);

        this->CANIdInstantSetDecel(id, data);

        break;
      }

      case Isv2MotorCommand::cSetMaxMoveVel:
      {
        // this->CANIdInstantSetAccel(id, data);

        break;
      }

      case Isv2MotorCommand::cSetMoveStop:
      {
        this->CANIdInstantMoveStop(id);

        break;
      }


      default: break;
    }

    motor->command_queue.erase(motor->command_queue.begin());

    // usleep(1);
  }



  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::Read()
{
/*Read SDO Auto Reload*/
  // if(this->send_cmd_queue_.empty())
  // {
  //   for (auto& node : motor_data_)
  //     this->CANIdReadPolarity(node->node_id);
  // }

  /*Send data and check error*/
  // if(!(this->recv_cmd_queue_.empty()) && this->send_flag_ == 0)
  // {
  //   CAN_data_struct send_data_buf = this->recv_cmd_queue_.front();
    
  //   #if SEND_CAN_DPRINT
  //     printf("send data : id = %x, index = %x ", send_data_buf.id, (send_data_buf.data[2]<< 8) + send_data_buf.data[1] );

  //     for(int i = 4; i<8; i++)
  //       printf("%x ", send_data_buf.data[i]);

  //     printf("\r\n");
  //   #endif

  //   if(MCP2515_CANsend(send_data_buf) < 0)
  //     return kssbot_hardware::IReturnType::kReturnError;

  //   if(this->last_send_id_ == send_data_buf.id)
  //   {
  //     if(this->send_repeat_cnt_++ > 3)
  //     {
  //       this->send_repeat_cnt_ = 0;

  //       this->send_cmd_queue_.clear();

  //       for(auto& node : this->motor_data_)
  //         this->CANIdInstantMoveStop(node->node_id);
  //     }
  //   }
  //   else
  //   {
  //     this->send_repeat_cnt_ = 0;
  //   }

  // }

  /*Read data and check error*/
  // CAN_data_struct recv_data_buf = {0,};


  // if(MCP2515_CANRecv(&recv_data_buf) < 0)
  //   return kssbot_hardware::IReturnType::kReturnError;

  // #if RECV_CAN_DPRINT
  //   printf("recved data : id = %x, index = %x ", recv_data_buf.id, (recv_data_buf.data[2]<< 8) + recv_data_buf.data[1] );

  //   for(int i = 0; i<8; i++)
  //     printf("%x ", recv_data_buf.data[i]);

  //   printf("\r\n");
  // #endif

  // this->recv_data_queue_.push_back(recv_data_buf);

  // if(this->recv_data_queue_.empty())
  // {
  //     for(int i = 1; i <3; i++)
  //   {

  //     this->CANIdReadEncoder(i);
  //   }
  // }


  // if(this->recv_data_queue_.empty())
  // {
  //   this->CANIdReadEncoder(this->motor_data_.front()->node_id);
  // }


  while(!(this->recv_data_queue_.empty()))
  {
    /*---------------------------Read data Parsing*/
    /*SDO Check*/
    // if((!(this->recv_cmd_queue_.empty()) &&
    //     this->recv_cmd_queue_.front().id - CANIDType::sWSDO) == (recv_data_buf.id - CANIDType::sRSDO) &&
    //     this->recv_cmd_queue_.front().data[1] == recv_data_buf.data[1] &&
    //     this->recv_cmd_queue_.front().data[2] == recv_data_buf.data[2])

    CAN_data_struct recv_data_buf = recv_data_queue_.front();
    int node_id = (recv_data_buf.id  & 0xf);


    if(recv_data_buf.id - node_id == CANIDType::sRSDO)
    {
      // int status = (recv_data_buf.data[2]<< 8) + recv_data_buf.data[1];

      // /*Polarity*/
      // if(status == 0x607E && (recv_data_buf.data[0] > 0x40 && recv_data_buf.data[0] < 0x50 ))
      // {
      //   int dir = Isv2MotorDir::sCCW;
      //   int node_id = recv_data_buf.id - 0x580;

      //   if(recv_data_buf.data[4] == 64)
      //     dir = Isv2MotorDir::sCW;

      //   Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

      //   if(!(target_motor)) 
      //     return kssbot_hardware::IReturnType::kReturnError;

      //   target_motor->current_motor_dir = target_motor->is_motor_reversed_ * dir;
      // }

              /*Object A = Status Word*/
      int status = (recv_data_buf.data[2]<< 8) + recv_data_buf.data[1];

      Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

      if(!(target_motor)) 
        break;


      if(status == 0x6064)  //id = pos. encoder
      {
          int act_pos = 0;

          for (int i = 4; i < 8; i++)
            act_pos |= (recv_data_buf.data[i] << (8*(i-4)));

          target_motor->current_motor_pos = target_motor->is_motor_reversed_ * act_pos;

      }
      this->recv_data_queue_.erase(this->recv_data_queue_.begin());
    }
    else
    {

      /*RPDO Check*/
      // int node_id = (recv_data_buf.id  & 0xf);
      int RPDO_id = recv_data_queue_.front().id - node_id;

      // printf("current read id = %d, RPDO id = %d\r\n", node_id, RPDO_id);

      switch(RPDO_id)
      {
        case 0x180 : 
        {
          Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

          if(!(target_motor)) 
            break;

          /*Object A = Status Word*/
          int status = (recv_data_buf.data[1]<< 8) + recv_data_buf.data[0];

          target_motor->is_servo_ready = (status >> 0) & (1);
          target_motor->is_servo_on = (status >> 2) & (1);
          target_motor->is_error = (status >> 3) & (1);
          target_motor->is_qstop = (status >> 5) & (1);
          target_motor->is_inpos = (status >> 10) & (1);

          break;
        }

        case 0x280:
        {
          Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

          if(!(target_motor)) 
            break;

          /*Object A = Status Word*/
          int status = (recv_data_buf.data[1]<< 8) + recv_data_buf.data[0];

          target_motor->is_servo_ready = (status >> 0) & (1);
          target_motor->is_servo_on = (status >> 2) & (1);
          target_motor->is_error = (status >> 3) & (1);
          target_motor->is_qstop = (status >> 5) & (1);
          target_motor->is_inpos = (status >> 10) & (1);

          /*Object B - Actual Pos*/
          int act_pos = 0;

          for (int i = 2; i < 6; i++)
            act_pos |= (recv_data_buf.data[i] << (8*(i-2)));

          target_motor->current_motor_pos = target_motor->is_motor_reversed_ * act_pos;

          // target_motor->current_motor_pos = ((target_motor->is_motor_reversed_ * act_pos/(2*M_PI*40))*this->motor_encoder_increment*this->reducer_rate_);

          /*Object C - Display mode*/
          // int mode = 0;

          // for (int i = 6; i < 8; i++)
          //   mode |= (recv_data_buf.data[i] << (8*i));

          break;
        }

        case 0x380:
        {
          Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

          if(!(target_motor)) 
            break;

          /*Object A - Actual Pos*/
          int act_pos = 0;

          for (int i = 0; i < 4; i++)
            act_pos |= (recv_data_buf.data[i] << (8*i));

          target_motor->current_motor_pos = target_motor->is_motor_reversed_ * act_pos;

          /*Object B - Actual Vel*/
          int act_vel = 0;

          for (int i = 4; i < 8; i++)
            act_vel |= (recv_data_buf.data[i] << (8*(i-4)));

          target_motor->current_motor_speed = target_motor->is_motor_reversed_* act_vel;

          if(target_motor->current_motor_speed >= 0)
            target_motor->current_motor_dir = 1;
          else
            target_motor->current_motor_dir = -1;

          break;
        }

        case 0x480:
        {
          Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

          if(!(target_motor)) 
            break;

          /*Object A - Actual Pos*/
          int act_pos = 0;

          for (int i = 0; i < 4; i++)
            act_pos |= (recv_data_buf.data[i] << (8*i));

          target_motor->current_motor_pos = target_motor->is_motor_reversed_ * act_pos;

          /*Object B - DIO*/
          // int act_pos = 0;

          // for (int i = 0; i < 4; i++)
          //   act_pos |= (recv_data_buf.data[i] << (8*i));

          // target_motor->current_motor_pos = act_pos;


          break;
        }

        default : break;
      }

      #if STATUS_MOTOR_DPRINT

        for(auto& node : this->motor_data_)
          printf("Node id : %d, Current Speed : %d, Current Pos : %d, inpos : %d, dir : %d \r\n", node->node_id, 
                                                                          node->current_motor_speed, 
                                                                          node->current_motor_pos,
                                                                          node->is_inpos, 
                                                                          node->current_motor_dir                                                                                                              
                                                                          );

      #endif

      this->recv_data_queue_.erase(this->recv_data_queue_.begin());
    }
  }

  // if(this->send_flag_++ > 5)
  // {
  //   this->send_flag_ = 0;

  //   // printf("current pos - Left : %d  Right : %d \r\n", motor_data_.at(0)->current_motor_pos, motor_data_.at(1)->current_motor_pos);

  // }
   

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::Drive()
{
  const kssbot_hardware::LifeCycleState now_state = this->state_;

  switch(now_state)
  {
    case kssbot_hardware::LifeCycleState::kHwInit:
    {
      if(this->HardwareInitiailize() == kssbot_hardware::IReturnType::kReturnError)
        this->state_ = kssbot_hardware::LifeCycleState::kError;

      else
        this->state_ = kssbot_hardware::LifeCycleState::kInit;

      break;
    }

    case kssbot_hardware::LifeCycleState::kInit:
    {
      if(this->SystemInitiailize() == kssbot_hardware::IReturnType::kReturnError)
        this->state_ = kssbot_hardware::LifeCycleState::kError;
      
      else
        this->state_ = kssbot_hardware::LifeCycleState::kRun;

      break;
    }

    case kssbot_hardware::LifeCycleState::kRun:
    {
      if(this->SystemOperation() == kssbot_hardware::IReturnType::kReturnError)
        this->state_ = kssbot_hardware::LifeCycleState::kError;
      
      break;


    }

    case kssbot_hardware::LifeCycleState::kStop:
    {

      break;
    }

    case kssbot_hardware::LifeCycleState::kError:
    {

      printf("Error\r\n");

      break;
    }

    default: 
    {
      this->state_ = kssbot_hardware::LifeCycleState::kError;

      break;
    }


  }




  return kssbot_hardware::IReturnType::kReturnOk;
}


kssbot_hardware::IReturnType isv2motor::HardwareInitiailize()
{
  if(hw_init_ == false)
  {
    if(MCP2515_CANinit()!= kssbot_hardware::IReturnType::kReturnOk) 
      return kssbot_hardware::IReturnType::kReturnError;

    CANIdInstantReset();


    hw_init_ = true;
  }

  this->is_hw_init_ = true;

  this->state_ = kssbot_hardware::LifeCycleState::kInit;

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::SystemInitiailize()
{

  /*CAN id Register*/
  this->DeleteAllMotor();
 
  if(this->motor_data_.empty())
  {
    this->DeleteAllMotor();

    this->RegisterMotor(LEFT_MOTOR_NODE, Isv2MotorDir::sCCW);  //mini sCW  big - sCCW
    
    this->RegisterMotor(RIGHT_MOTOR_NODE, Isv2MotorDir::sCW);  //mini sCCW  big - sCW


  }


  for (auto& node : motor_data_)
    CANIdInstantRegister(node->node_id);

  usleep(10000);

  //reset alarm
  for (auto& node : motor_data_)
    this->CANIdInstantSetControlWord(node->node_id, Isv2MotorControlWord::sAlarmReset);

  usleep(10000);

  //move to ready to switch on status
  for (auto& node : motor_data_)
    this->CANIdInstantSetControlWord(node->node_id, Isv2MotorControlWord::sPowerOn);

  usleep(10000);


  //move to switch on status
  for (auto& node : motor_data_)
    this->CANIdInstantSetControlWord(node->node_id, Isv2MotorControlWord::sMotorStart);

  usleep(10000);


  //move to enable status
  for (auto& node : motor_data_)
    this->CANIdInstantSetControlWord(node->node_id, Isv2MotorControlWord::sServoOn);

  usleep(10000);


  //vel mode change
  for (auto& node : motor_data_)
    this->CANIdInstantSetModeOperation(node->node_id, Isv2MotorModeOperation::sVelMode);

  usleep(10000);


  //accel  change
  for (auto& node : motor_data_)
    this->CANIdInstantSetAccel(node->node_id, ChangeMMsToRPM(this->motor_acc_, this->wheel_radius_, this->motor_encoder_increment, this->reducer_rate_)); //10000* rpm/60  ex.) 2m/s^2 = 2000mm/s^2 -> 2000/*40*2*pi = 7.95 = 8 

  usleep(10000);


  //decel  change
  for (auto& node : motor_data_)
    this->CANIdInstantSetDecel(node->node_id, ChangeMMsToRPM(this->motor_dec_, this->wheel_radius_, this->motor_encoder_increment, this->reducer_rate_));

  usleep(10000);

  // for (auto& node : motor_data_)
  //   this->CANIdReadPolarity(node->node_id);

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::SystemOperation()
{
  /*Read SDO Auto Reload*/
  // if(this->send_cmd_queue_.empty())
  // {
  //   for (auto& node : motor_data_)
  //     this->CANIdReadPolarity(node->node_id);
  // }

  /*Send data and check error*/
  // if(!(this->send_cmd_queue_.empty()) && this->send_flag_ == 0)
  // {
  //   CAN_data_struct send_data_buf = this->send_cmd_queue_.front();
    
  //   #if SEND_CAN_DPRINT
  //     printf("send data : id = %x, index = %x ", send_data_buf.id, (send_data_buf.data[2]<< 8) + send_data_buf.data[1] );

  //     for(int i = 4; i<8; i++)
  //       printf("%x ", send_data_buf.data[i]);

  //     printf("\r\n");
  //   #endif

  //   if(MCP2515_CANsend(send_data_buf) < 0)
  //     return kssbot_hardware::IReturnType::kReturnError;

  //   if(this->last_send_id_ == send_data_buf.id)
  //   {
  //     if(this->send_repeat_cnt_++ > 3)
  //     {
  //       this->send_repeat_cnt_ = 0;

  //       this->send_cmd_queue_.clear();

  //       for(auto& node : this->motor_data_)
  //         this->CANIdInstantMoveStop(node->node_id);
  //     }
  //   }
  //   else
  //   {
  //     this->send_repeat_cnt_ = 0;
  //   }

  // }

  // /*Read data and check error*/
  // CAN_data_struct recv_data_buf = {0,};


  // if(MCP2515_CANRecv(&recv_data_buf) < 0)
  //   return kssbot_hardware::IReturnType::kReturnError;

  //   #if RECV_CAN_DPRINT
  //     printf("recved data : id = %x, index = %x ", recv_data_buf.id, (recv_data_buf.data[2]<< 8) + recv_data_buf.data[1] );

  //     for(int i = 0; i<8; i++)
  //       printf("%x ", recv_data_buf.data[i]);

  //     printf("\r\n");
  //   #endif

  // /*---------------------------Read data Parsing*/
  // /*SDO Check*/
  // if((this->send_cmd_queue_.front().id - CANIDType::sWSDO) == (recv_data_buf.id - CANIDType::sRSDO) &&
  //     this->send_cmd_queue_.front().data[1] == recv_data_buf.data[1] &&
  //     this->send_cmd_queue_.front().data[2] == recv_data_buf.data[2])
  // {
  //   int status = (recv_data_buf.data[2]<< 8) + recv_data_buf.data[1];

  //   // /*Polarity*/
  //   // if(status == 0x607E && (recv_data_buf.data[0] > 0x40 && recv_data_buf.data[0] < 0x50 ))
  //   // {
  //   //   int dir = Isv2MotorDir::sCCW;
  //   //   int node_id = recv_data_buf.id - 0x580;

  //   //   if(recv_data_buf.data[4] == 64)
  //   //     dir = Isv2MotorDir::sCW;

  //   //   Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

  //   //   if(!(target_motor)) 
  //   //     return kssbot_hardware::IReturnType::kReturnError;

  //   //   target_motor->current_motor_dir = target_motor->is_motor_reversed_ * dir;
  //   // }



  //   this->send_cmd_queue_.erase(this->send_cmd_queue_.begin());
  // }

  // /*RPDO Check*/
  // int node_id = (recv_data_buf.id  & 0xf);
  // int RPDO_id = recv_data_buf.id - node_id;

  // switch(RPDO_id)
  // {
  //   case 0x180 : 
  //   {
  //     Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

  //     if(!(target_motor)) 
  //       break;

  //     /*Object A = Status Word*/
  //     int status = (recv_data_buf.data[1]<< 8) + recv_data_buf.data[0];

  //     target_motor->is_servo_ready = (status >> 0) & (1);
  //     target_motor->is_servo_on = (status >> 2) & (1);
  //     target_motor->is_error = (status >> 3) & (1);
  //     target_motor->is_qstop = (status >> 5) & (1);
  //     target_motor->is_inpos = (status >> 10) & (1);

  //     break;
  //   }

  //   case 0x280:
  //   {
  //     Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

  //     if(!(target_motor)) 
  //       break;

  //     /*Object A = Status Word*/
  //     int status = (recv_data_buf.data[1]<< 8) + recv_data_buf.data[0];

  //     target_motor->is_servo_ready = (status >> 0) & (1);
  //     target_motor->is_servo_on = (status >> 2) & (1);
  //     target_motor->is_error = (status >> 3) & (1);
  //     target_motor->is_qstop = (status >> 5) & (1);
  //     target_motor->is_inpos = (status >> 10) & (1);

  //     /*Object B - Actual Pos*/
  //     int act_pos = 0;

  //     for (int i = 2; i < 6; i++)
  //       act_pos |= (recv_data_buf.data[i] << (8*(i-2)));

  //     // target_motor->current_motor_pos = target_motor->is_motor_reversed_ * act_pos;

  //     target_motor->current_motor_pos = ((target_motor->is_motor_reversed_ * act_pos/(2*M_PI*40))*this->motor_encoder_increment*this->reducer_rate_);

  //     /*Object C - Display mode*/
  //     // int mode = 0;

  //     // for (int i = 6; i < 8; i++)
  //     //   mode |= (recv_data_buf.data[i] << (8*i));

  //     break;
  //   }

  //   case 0x380:
  //   {
  //     Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

  //     if(!(target_motor)) 
  //       break;

  //     /*Object A - Actual Pos*/
  //     int act_pos = 0;

  //     for (int i = 0; i < 4; i++)
  //       act_pos |= (recv_data_buf.data[i] << (8*i));

  //     target_motor->current_motor_pos = target_motor->is_motor_reversed_ * act_pos;

  //     /*Object B - Actual Vel*/
  //     int act_vel = 0;

  //     for (int i = 4; i < 8; i++)
  //       act_vel |= (recv_data_buf.data[i] << (8*(i-4)));

  //     target_motor->current_motor_speed = -1* target_motor->is_motor_reversed_* act_vel;

  //     if(target_motor->current_motor_speed >= 0)
  //       target_motor->current_motor_dir = 1;
  //     else
  //       target_motor->current_motor_dir = -1;

  //     break;
  //   }

  //   case 0x480:
  //   {
  //     Isv2MotorStruct* target_motor = GetIsv2MotorStruct(node_id);

  //     if(!(target_motor)) 
  //       break;

  //     /*Object A - Actual Pos*/
  //     int act_pos = 0;

  //     for (int i = 0; i < 4; i++)
  //       act_pos |= (recv_data_buf.data[i] << (8*i));

  //     target_motor->current_motor_pos = target_motor->is_motor_reversed_ * act_pos;

  //     /*Object B - DIO*/
  //     // int act_pos = 0;

  //     // for (int i = 0; i < 4; i++)
  //     //   act_pos |= (recv_data_buf.data[i] << (8*i));

  //     // target_motor->current_motor_pos = act_pos;


  //     break;
  //   }

  //   default : break;
  // }

  //   #if STATUS_MOTOR_DPRINT

  //     for(auto& node : this->motor_data_)
  //       printf("Node id : %d, Current Speed : %d, Current Pos : %d, inpos : %d, dir : %d \r\n", node->node_id, 
  //                                                                       node->current_motor_speed, 
  //                                                                       node->current_motor_pos,
  //                                                                       node->is_inpos, 
  //                                                                       node->current_motor_dir                                                                                                              
  //                                                                       );

  //   #endif



  // if(this->send_flag_++ > 5)
  // {
  //   this->send_flag_ = 0;

  //   // printf("current pos - Left : %d  Right : %d \r\n", motor_data_.at(0)->current_motor_pos, motor_data_.at(1)->current_motor_pos);

  // }
   

  return kssbot_hardware::IReturnType::kReturnOk;
}


//--------------------------------------------------------------------------------

Isv2MotorStruct* isv2motor::GetIsv2MotorStruct(int node)
{
    Isv2MotorStruct* target_motor =  nullptr;

    for (auto& target: motor_data_)
    {
      if(target->node_id == node)
      {
        target_motor = target;

        break;
      }
    }

  return target_motor;
}


//--------------------------------------------------------------------------------

kssbot_hardware::IReturnType isv2motor::RegisterMotor(int node)
{
  Isv2MotorStruct* register_motor = new Isv2MotorStruct(node);

  this->motor_data_.push_back(register_motor);

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::RegisterMotor(int node, int motor_dir)
{
  Isv2MotorStruct* register_motor = new Isv2MotorStruct(node, motor_dir);

  this->motor_data_.push_back(register_motor);

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::DeleteMotor(int node)
{

  int itr = 0;

  for(auto& data  : this->motor_data_)
  {
    if (data->node_id == node)
    {
      this->motor_data_.erase(this->motor_data_.begin() + itr);

      break;
    }
    itr++;
  }

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType isv2motor::DeleteAllMotor()
{
  while(!(this->motor_data_.empty()))
  {
    this->motor_data_.erase(this->motor_data_.begin());
  }
  
  return kssbot_hardware::IReturnType::kReturnOk;
}


//         target_d.left_vel = (((target_d.left_vel)/(2*M_PI*40))*10000*2) * -1;


//--------------------------------------------------------------------------------return types
std::vector<Isv2MotorStruct*> isv2motor::GetMotorData()
{
  return this->motor_data_;
}


kssbot_hardware::LifeCycleState isv2motor::GetMotorStatus()
{
  return this->state_;
}

//--------------------------------------------------------------------------------calculate 



//--------------------------------------------------------------------------------calculate 

double ChangeRPMToMMs(double rpm, double radius, size_t motor_increment, size_t motor_reducer_rate)
{
  double mms = (rpm/(motor_increment*motor_reducer_rate))*(2*M_PI*radius);
  
  return mms;
}

double ChangeMMsToRPM(double mms, double radius, size_t motor_increment, size_t motor_reducer_rate)
{
  double rpm = (((mms)/(2*M_PI*radius))*motor_increment*motor_reducer_rate);

  return rpm;
}







/**
  ******************************************************************************
  * @file           : driving_controller.cpp
  * @brief          : Main Interface cpp file of QRAGV driving_controller
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
  * @Comments        : this contains pgv100 pos. sensor and isv2 motor interfaces 
  * 
  */


#include "driving_controller/driving_controller.hpp"


#define MAX_MODULE_COUNT              10

#define PGV100_DRIVE_TICK             10
#define PGV100_LOAD_DATA_TICK         3

#define ISV2MOTOR_DRIVE_TICK          1
#define ISV2MOTOR_LOAD_DATA_TICK      5



void PostionSensorWorker(const void* arg)
{
  DrivingController* contoller = (DrivingController*)arg;

  bool quit_sig_ = false;

  int doing_tick  = 0;
  bool drive_tick_on = false;
  bool data_load_on = false;

  pgv100 pgv100_(0, 1); //id , unit

  pgv100_.Init();

  printf("test pgv100 thread start !\r\n");

  auto prev_time = std::chrono::steady_clock::now();

  while(!(quit_sig_))
  {
    auto mili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - prev_time);

    /*clocks  */
    if(mili.count() >= 10) //10 milisceondss
    {
      if(doing_tick++ >= MAX_MODULE_COUNT)
        doing_tick = 0;

      if(MAX_MODULE_COUNT%PGV100_DRIVE_TICK == 0)
        drive_tick_on = true;

      if(MAX_MODULE_COUNT%PGV100_LOAD_DATA_TICK == 0)
        data_load_on = true;


      prev_time = std::chrono::steady_clock::now();
    }

    /* Load pgv100 data and set controller data*/
    if(data_load_on)     
    {
      /*transaction data between to controller and pos sensor*/
      contoller->TransactionPosData(pgv100_.GetPGV100DataStructure(), quit_sig_);

      data_load_on = false;
    }
    /* Main  pgv100 drive*/
    else if(drive_tick_on)     
    {
      pgv100_.Drive();

      drive_tick_on = false;
    }

    usleep(1000);
  }

  printf("test pgv100 thread end !\r\n");

  return;
}

void DrivingMotorWorker(const void* arg)
{
  DrivingController* contoller = (DrivingController*)arg;

  bool quit_sig_ = false;

  int doing_tick  = 0;
  bool drive_tick_on = false;
  bool data_load_on = false;

  isv2motor isv2motor_(contoller->max_acc_, contoller->max_dec_, contoller->max_vel_);

  isv2motor_.Init();

  printf("test isv2motor thread start !\r\n");

  auto prev_time = std::chrono::steady_clock::now();

  while(!(quit_sig_))
  {
    auto mili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - prev_time);

    /*clocks  */
    if(mili.count() >= 10) //10 milisceondss
    {
      if(doing_tick++ >= 10)
        doing_tick = 0;

      drive_tick_on = true;

      if(MAX_MODULE_COUNT%ISV2MOTOR_LOAD_DATA_TICK == 0)
        data_load_on = true;

      prev_time = std::chrono::steady_clock::now();
    }

    /* Load isv2motor data and set controller data*/
    /* Set isv2motor command and load controller data*/
    if(data_load_on)
    {

      /*Get data and set controller*/
      /*send data to controller*/
      /*get quit signal from controller*/
      contoller->TransactionMotorData(isv2motor_, quit_sig_);

      data_load_on = false;
    }
    /* Main  isv2motor drive*/
    else if(drive_tick_on)     
    {
      /*if velocity set queue is set -> do setvelinstannt*/
      isv2motor_.Write();

      drive_tick_on = false;
    }

   /*OR queue is empty -> read or set parmas*/
    isv2motor_.Read();

    usleep(1000);
  }

  printf("test isv2motor thread end !\r\n");

  return;
}



/*----------------------------------------------------*/



DrivingController::DrivingController()
{
  
}


DrivingController::DrivingController(double max_acc, double max_dec, double max_vel)
{
  this->max_acc_ = max_acc;
  this->max_dec_ = max_dec;
  this->max_vel_ = max_vel;
}




DrivingController::~DrivingController()
{
  this->SetQuitSignal(true);

  if(!(this->module_workers_.empty()))
  {
    for(auto& worker : this->module_workers_)
    {
        /*send quit signal to exit while loop*/

        worker.join();
    }
  }

}







kssbot_hardware::IReturnType DrivingController::Init()
{
  /*1. Regsiter all motors and sensors*/
  if(!(this->module_workers_.empty()))
  {
    for(auto& worker : this->module_workers_)
    {
        /*send quit signal to exit while loop*/

        worker.join();
    }
  }

  this->module_workers_.push_back(std::thread(PostionSensorWorker, this));
  this->module_workers_.push_back(std::thread(DrivingMotorWorker, this));


  /*2. Resgister all info buffer data*/
  Isv2MotorStruct left_motor_data(1), right_motor_data(2);

  this->motor_infos_.push_back(left_motor_data); //left
  this->motor_infos_.push_back(right_motor_data); //right

   
  return kssbot_hardware::IReturnType::kReturnOk;
}


kssbot_hardware::IReturnType DrivingController::Drive()
{ 

  /*Make sure all modules have no errors*/


  // while(!(quit_sig))
  // {

    /* this function do : */

    /*1. Write motor vel*/
    if(!(this->is_moving_) && !(this->target_pos_queue_.empty()))
    {
      this->current_target_pos_ = target_pos_queue_.front();

      /*Make vel. profile here - TestScurveProfileGenerator*/

      this->is_moving_ = true;
    }


    if(this->is_moving_)
    {




      /*worker - TestScurveProfileWorker  send - vel to other thread*/
    }




    /*2. Get data from motor and sensors */





    /*3 . Make Velocity profile using 1, 2 */



  this->DataMutexLock();

  if(this->is_detected_)
    printf("PGV100 has Detected the Line. Xpos = %d, Ypos = %d, Angle = %d\r\n", this->x_pos_, this->y_pos_, this->angle_);
  else if (this->is_tagged_)
    printf("PGV100 has Detected the tag. Xpos = %d, Ypos = %d, Angle = %d\r\n", this->x_pos_, this->y_pos_, this->angle_);
  else 
    printf("PGV100 has no Detected anything. Xpos = %d, Ypos = %d, Angle = %d\r\n", this->x_pos_, this->y_pos_, this->angle_);


  this->DataMutexUnLock();

  // sleep(1);

  // }

  /*thread end*/
  
  // for(auto& worker : this->module_workers_)
  //   worker.join();


  return kssbot_hardware::IReturnType::kReturnOk;
}






/*handler */
bool DrivingController::GetQuitSignal()
{
  this->DataMutexLock();

  bool quit_sig = this->quit_sig_;

  this->DataMutexUnLock();

  return quit_sig;
}

bool DrivingController::SetQuitSignal(bool sig)
{
  this->DataMutexLock();

  this->quit_sig_ = sig;

  this->DataMutexUnLock();

  return sig;
}


/*mutex*/
void DrivingController::DataMutexLock()
{
  this->controller_data_mutex_.lock();

  return;
}

void DrivingController::DataMutexUnLock()
{
  this->controller_data_mutex_.unlock();

  return;

}



/*module transaction data*/
void DrivingController::TransactionPosData(PosSensorDataStruct data, bool& quit_sig)
{

  this->DataMutexLock();

  /*send data to controller*/
  this->is_detected_ = data.is_detected;
  this->is_tagged_ = data.is_tagged;

  /*unique value*/
  this->tag_code_ = data.tag_code;
  this->error_code_ = data.error_code;
  this->pgv100_dir_ = data.pgv100_dir;
  this->pgv100_color_ = data.pgv100_color;
  this->x_pos_ = data.xpos;
  this->y_pos_ = data.ypos;
  this->angle_ = data.angle;

  /*Get data from controller*/
  quit_sig = this->quit_sig_;

  this->DataMutexUnLock();

  return;
}


/*module transaction data*/
void DrivingController::TransactionMotorData(isv2motor& motor_interface, bool& quit_sig)
{
  std::vector<Isv2MotorCommandStruct> local_command_;

  std::vector<Isv2MotorStruct*> motor_infos = motor_interface.GetMotorData();

  this->DataMutexLock();

  /*send command to module*/
  local_command_ = this->motor_command_struct_;

  this->motor_command_struct_.clear();

  /*send data to controller*/
  for(auto& motor: motor_infos)
  {
    if(motor->node_id == 1)
    {
      this->left_current_motor_speed_= motor->current_motor_speed;
      this->left_current_motor_pos_ = motor->current_motor_pos;
    }
    else if(motor->node_id == 2)
    {
      this->right_current_motor_speed_= motor->current_motor_speed;
      this->right_current_motor_pos_ = motor->current_motor_pos;
    }

  }

  /*Get data from controller*/
  quit_sig = this->quit_sig_;

  this->DataMutexUnLock();

  if(!(local_command_.empty()))
  {
    int id = local_command_.front().id;

    for(auto& motor : motor_infos)
    {
      if(motor->node_id == id)
        motor->command_queue.push_back(local_command_.front());
    }

    local_command_.erase(local_command_.begin());
  }

  return;
}

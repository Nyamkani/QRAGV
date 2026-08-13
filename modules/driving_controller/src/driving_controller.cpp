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


#define MAX_MODULE_COUNT              1000

#define PGV100_DRIVE_TICK             100
#define PGV100_LOAD_DATA_TICK         10

#define ISV2MOTOR_DRIVE_TICK          5
#define ISV2MOTOR_LOAD_DATA_TICK      1



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
    if(mili.count() >= 1) //1 milisceondss
    {
      if(doing_tick++ >= MAX_MODULE_COUNT)
        doing_tick = 0;

      if(doing_tick%PGV100_DRIVE_TICK == 0)
        drive_tick_on = true;

      // if(doing_tick%PGV100_LOAD_DATA_TICK == 0)
      //   data_load_on = true;


      prev_time = std::chrono::steady_clock::now();
    }

    /* Load pgv100 data and set controller data*/
    if(data_load_on)     
    {
      /*transaction data between to controller and pos sensor*/
      // if(pgv100_.IsLineDetected() || pgv100_.IsTagged())
      contoller->TransactionPosData(pgv100_.GetPGV100DataStructure(), quit_sig_);

      data_load_on = false;
    }
    /* Main  pgv100 drive*/
    else if(drive_tick_on)     
    {
      pgv100_.Drive();

      data_load_on = true;

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
  bool write_tick_on = false;
  bool read_tick_on = false;

  bool data_load_on = false;


  isv2motor isv2motor_(contoller);

  isv2motor_.Init();

  printf("test isv2motor thread start !\r\n");

  auto prev_time = std::chrono::steady_clock::now();

  while(!(quit_sig_))
  {
    auto mili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - prev_time);

    /*clocks  */
    if(mili.count() >= 1) //1 milisceondss
    {
      if(doing_tick++ >= MAX_MODULE_COUNT)
        doing_tick = 0;

      // write_tick_on = true;

      // if(doing_tick%10 == 0)
      // {
        write_tick_on = true;

      // }

      read_tick_on = true;

      // if(doing_tick%10 == 0)
      //   test_tick_on = true;

      if(doing_tick%ISV2MOTOR_LOAD_DATA_TICK == 0)
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
    else if(write_tick_on)     /* Main  isv2motor drive*/
    {
      /*if velocity set queue is set -> do setvelinstannt*/
      isv2motor_.Write();

      write_tick_on = false;
    }
    else if(read_tick_on)     /* Main  isv2motor drive*/
    {
      /*if velocity set queue is set -> do setvelinstannt*/
      isv2motor_.Read();

      read_tick_on = false;
    }
    else
    {
      /*watchdog temp.hardcoding*/
      for(int i = 1; i <3; i++)
      {

        isv2motor_.CANIdReadEncoder(i);
      }
    }


    usleep(1);
    // else
    // {
    /*OR queue is empty -> read or set parmas*/
      // isv2motor_.Read();
    // }


  }

  printf("test isv2motor thread end !\r\n");

  return;
}

void JoypadWorker(const void* arg)
{
  DrivingController* contoller = (DrivingController*)arg;

  const char *device = "/dev/input/js0";
  int js;
  struct js_event event;
  struct axis_state axes[3] = {0};
  size_t axis;


  int left_vel = 0;
  int right_vel = 0;

  bool input_occur = false;
  bool quit_sig_ = false;

  js = open(device, O_RDONLY);

  if (js == -1)
      perror("Could not open joystick");

  printf("test joypad thread start !\r\n");

  auto prev_time = std::chrono::steady_clock::now();

  while(!(quit_sig_))
  {
    auto mili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - prev_time);

    /*clocks  */
    if(mili.count() >= 10) //10 milisceondss
    {
      Isv2MotorCommandStruct left_data, right_data;

      left_data.id = LEFT_MOTOR_NODE;
      right_data.id = RIGHT_MOTOR_NODE;

      /* This loop will exit if the controller is unplugged. */
      while (read_event(js, &event) == 0 && !(quit_sig_))
      {
        switch (event.type)
        {
          case JS_EVENT_BUTTON:
              printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");

              if(event.number ==1 && event.value ==1)
              {
                left_vel = 0;
                right_vel = 0;
                left_data.command = cSetMoveStop;
                right_data.command = cSetMoveStop;
              }

              else if(event.number ==2 && event.value ==1)
              {
                contoller->SetQuitSignal(true);
              }

              

              break;
          case JS_EVENT_AXIS:
              axis = get_axis_state(&event, axes);
              if (axis < 3)
              {
                printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);

                left_data.command = cSetMoveVel;
                right_data.command = cSetMoveVel;

                if(axes[axis].y < 0)
                {
                  left_vel = 500;
                  right_vel = 500;

                  input_occur = true;
                }
                else if(axes[axis].y > 0)
                {
                  left_vel = -500;
                  right_vel = -500;

                  input_occur = true;          
                }
                else if (axes[axis].x < 0)  //left turn
                {
                  left_vel = -500;
                  right_vel = 500;

                  input_occur = true;
                }
                else if (axes[axis].x > 0)  //right turn
                {
                  left_vel = 500;
                  right_vel = -500;

                  input_occur = true;
                }
                else 
                {
                  left_vel = 0;
                  right_vel = 0;

                  input_occur = true;
                }
              }
              break;
          default:
              /* Ignore init events. */
                  left_vel = 0;
                  right_vel = 0;

                  input_occur = true;
              break;
        }
          
        fflush(stdout);



        break;
      }

      if(input_occur)
      {

        if(!(contoller->motor_command_struct_.empty()))
          contoller->motor_command_struct_.clear();

        left_data.data = left_vel;
        right_data.data = right_vel;

        contoller->DataMutexLock();

        contoller->motor_command_struct_.push_back(left_data);
        contoller->motor_command_struct_.push_back(right_data);

        contoller->DataMutexUnLock();
      }

      quit_sig_ = contoller->GetQuitSignal();

      prev_time = std::chrono::steady_clock::now();

      input_occur = false;
    }

    usleep(10);
  }



    close(js);
//     return 0;




  printf("test joypad thread end !\r\n");

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

  if(this->now_follow_left_profiler_ != nullptr)
    delete this->now_follow_left_profiler_;

  if(this->now_follow_right_profiler_ != nullptr)
    delete this->now_follow_right_profiler_;

  if(!(this->module_workers_.empty()))
  {
    for(auto& worker : this->module_workers_)
    {
        /*send quit signal to exit while loop*/

        worker.join();
    }
  }

  for(auto& motor : this->motor_infos_)
    delete motor;
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


  this->wheel_radius_ = 100.0f; //40 mm , 100mm
  this->wheel_between_length_ = 680.0f; //400mm   ,680mm
  this->point_between_aux_ = 0.0f; //200mm
  this->encoder_ticks_ = 10000.0f;   //10000 ticks per rev
  this->reducer_rate_ =  50.0f;   //2 , 50




  this->module_workers_.push_back(std::thread(PostionSensorWorker, this));
  this->module_workers_.push_back(std::thread(DrivingMotorWorker, this));
  this->module_workers_.push_back(std::thread(JoypadWorker, this));

  // /*test*/
  // this->left_motor_ = new isv2motor(this, 2, -1); //right
  // this->right_motor_ = new isv2motor(this, 1, 1); //left



  /*2. Resgister all info buffer data*/
  if(!(this->motor_infos_.empty()))
  {
    for(auto& motor_info_pointer : this->motor_infos_)
    {
      delete motor_info_pointer;
  
      motor_info_pointer = nullptr;
    }
    this->motor_infos_.clear();
  }

  Isv2MotorStruct* left_motor_data = nullptr, *right_motor_data = nullptr;

  left_motor_data = new Isv2MotorStruct(LEFT_MOTOR_ID);
  right_motor_data = new Isv2MotorStruct(RIGHT_MOTOR_ID);

  this->motor_infos_.push_back(left_motor_data); //left
  this->motor_infos_.push_back(right_motor_data); //right



  /*3. wait for modules are ready(run)*/
  bool is_motor_run = false;
  bool is_pos_sensor_run = false;

  int cnt = 0;

  while(cnt++ < 10)
  {
    this->DataMutexLock();

    if(this->pos_sensor_data_buffer_.status == kssbot_hardware::LifeCycleState::kRun)
      is_pos_sensor_run = true;

    if(this->motor_data_renew_) //sensor data
    {
      this->current_pos_ = (double)this->pos_sensor_data_buffer_.xpos/10;
      this->current_left_pos_ = this->current_pos_;
      this->current_right_pos_=  this->current_pos_;
    }

    if(this->motor_infos_.at(0)->status == kssbot_hardware::LifeCycleState::kRun &&
        this->motor_infos_.at(1)->status == kssbot_hardware::LifeCycleState::kRun)
      is_motor_run = true;

    this->DataMutexUnLock();


    if(is_pos_sensor_run && is_motor_run)
      break;

    sleep(1);
  }

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType DrivingController::Drive()
{ 
  this->TransactionDataFromMoudules();

  this->CalculateOdometry();

  if(!(this->is_moving_) && !(this->target_pos_queue_.empty()))
    {
      this->current_target_pos_ = this->target_pos_queue_.front().x_pos;

      /*Make vel. profile here - TestScurveProfileGenerator*/

      if(this->now_follow_left_profiler_ != nullptr)
      {
        delete this->now_follow_left_profiler_;

        this->now_follow_left_profiler_ = nullptr;
      }

      this->now_follow_left_profiler_ = new ScurveProfiler(500, this->max_acc_, this->max_vel_); //jerk, acc, vel

      this->now_follow_left_profiler_->SetTargetPosition(this->current_target_pos_);

      // this->now_follow_left_profiler_->SetPresentPosition(this->current_left_pos_);
      this->first_left_encoder_val =  this->left_current_motor_encoder_;

      if(this->now_follow_right_profiler_ != nullptr)
      {
        delete this->now_follow_right_profiler_;

        this->now_follow_right_profiler_ = nullptr;
      }

      this->now_follow_right_profiler_ = new ScurveProfiler(500, this->max_acc_, this->max_vel_); //jerk, acc, vel

      this->now_follow_right_profiler_->SetTargetPosition(this->current_target_pos_);


      // this->now_follow_right_profiler_->SetPresentPosition(this->current_right_pos_);
      this->first_right_encoder_val =  this->right_current_motor_encoder_;


      // printf("prev encoder = %d\r\n",this->left_prev_motor_pos_ );

      this->is_moving_ = true;
    }



    if(this->is_moving_)
    {
      /*test*/
      Isv2MotorCommandStruct left_data, right_data;

      left_data.id = LEFT_MOTOR_NODE;
      right_data.id = RIGHT_MOTOR_NODE;
      // left_data.command = cSetMoveStop;
      // right_data.command = cSetMoveStop;



      // this->now_follow_left_profiler_->SetPresentPosition(this->current_left_pos_);

      // this->now_follow_left_profiler_->SetPresentSpeed(this->left_motor_s);

      this->now_follow_left_profiler_->Drive();

      double left_vel_data_ = this->now_follow_left_profiler_->GetCalculatedVel();

      /*test*/
      left_data.command = Isv2MotorCommand::cSetMoveVel;
      left_data.data = left_vel_data_;


      // this->now_follow_right_profiler_->SetPresentPosition(this->current_right_pos_);

      // this->now_follow_right_profiler_->SetPresentSpeed(this->right_motor_s);

      this->now_follow_right_profiler_->Drive();

      double right_vel_data_ = this->now_follow_right_profiler_->GetCalculatedVel();

      // double right_vel_data_ = left_vel_data_;

      right_data.command = Isv2MotorCommand::cSetMoveVel;
      right_data.data = right_vel_data_;


      int left_status = this->now_follow_left_profiler_->GetStatusProfiler();

      int right_status = this->now_follow_right_profiler_->GetStatusProfiler();

      /*2. Transcation data and command*/

      this->DataMutexLock();

      /*motor*/
      this->motor_command_struct_.clear();

      this->motor_command_struct_.push_back(left_data);
      this->motor_command_struct_.push_back(right_data);



      this->DataMutexUnLock();


      if(left_status >= 3 && right_status >= 3)
      {
        this->is_moving_ = false;

        delete this->now_follow_left_profiler_;

        this->now_follow_left_profiler_ = nullptr;

        delete this->now_follow_right_profiler_;

        this->now_follow_right_profiler_ = nullptr;

        // if(abs(this->current_target_pos_ - x_pos_) <= 2)
          this->target_pos_queue_.erase(this->target_pos_queue_.begin());


        printf("left encoder delta = %d, right encoder delta = %d,\r\n",
                          this->left_current_motor_encoder_ - this->first_left_encoder_val ,
                          this->right_current_motor_encoder_ - this->first_right_encoder_val );

        // printf("final left status = %d, final right status = %d, final left Pos data  = %f, final right Pos data  = %f\r\n", left_status, right_status,  this->current_left_pos_,   this->current_right_pos_);
      }
      /*worker - TestScurveProfileWorker  send - vel to other thread*/
    }   
    //  else
    // {
    //     /*test*/
    //     Isv2MotorCommandStruct left_data, right_data;

    //     left_data.id = LEFT_MOTOR_NODE;
    //     right_data.id = RIGHT_MOTOR_NODE;
    //     left_data.command = cSetMoveStop;
    //     right_data.command = cSetMoveStop;


    //   this->DataMutexLock();

    //   /*motor*/

    //   if(this->motor_command_struct_.empty())
    //   {
    //   this->motor_command_struct_.push_back(left_data);
    //   this->motor_command_struct_.push_back(right_data);

    //   }

    //   this->DataMutexUnLock();

    // }




 
  return kssbot_hardware::IReturnType::kReturnOk;
}





kssbot_hardware::IReturnType DrivingController::Drive2()
{

  this->TransactionDataFromMoudules();

  this->CalculateOdometry();

  this->ControlVelocity();

  return kssbot_hardware::IReturnType::kReturnOk;
}



kssbot_hardware::IReturnType DrivingController::Drive3()
{

  this->TransactionDataFromMoudules();

  // this->CalculateOdometry();

  // if(this->test_sig)

  this->TestQRControlVelocity();


  //   this->test_sig = false;
  // }


  // if(this->target_pos_queue_.empty())
  // {

  //     TargetPosStruct target1 = {3600, 0, 0, 5};

  //     this->target_pos_queue_.push_back(target1);

  //     TargetPosStruct target2 = {-3600, 0, 0, 1};

  //     this->target_pos_queue_.push_back(target2);

  // }

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
  // this->is_detected_ = data.is_detected;
  // this->is_tagged_ = data.is_tagged;

  // /*unique value*/
  // this->tag_code_ = data.tag_code;
  // this->error_code_ = data.error_code;
  // this->pgv100_dir_ = data.pgv100_dir;
  // this->pgv100_color_ = data.pgv100_color;
  // this->x_pos_ = data.xpos;
  // this->y_pos_ = data.ypos;
  // this->angle_ = data.angle;

  /*data sync signal*/
  // if(data.is_detected || data.is_tagged)
  this->pos_data_renew_ = true;

  /*send data to controller*/
  this->pos_sensor_data_buffer_ = data;

  /*Get data from controller*/
  quit_sig = this->quit_sig_;

  this->DataMutexUnLock();
  
  return;
}

/*module transaction data*/
void DrivingController::TransactionMotorData(isv2motor& motor_interface, bool& quit_sig)
{
  std::vector<Isv2MotorCommandStruct> local_command_;

  for(auto& motor: motor_interface.GetMotorData())
    motor->status = motor_interface.GetMotorStatus();

  this->DataMutexLock();

  /*data sync signal*/
  this->motor_data_renew_ = true;

  /*send command to module*/
  local_command_ = this->motor_command_struct_;

  this->motor_command_struct_.clear();

  /*send data to controller*/
  for(auto& motor: motor_interface.GetMotorData())
  {
    int id = motor->node_id;

    for(auto& target_motor : this->motor_infos_)
    {
      if(target_motor->node_id == id)
          *target_motor = *motor;
    }
  }
  /*Get data from controller*/
  quit_sig = this->quit_sig_;

  this->DataMutexUnLock();


  /*Prevent delayed command velocity*/
  for(auto& motor : motor_interface.GetMotorData())
  {
    if(!(motor->command_queue.empty()))
      motor->command_queue.clear();
  }



  while(!(local_command_.empty()))
  {
    for(auto& motor : motor_interface.GetMotorData())
    {
      int id = local_command_.front().id;

      if(motor->node_id == id)
      {
        motor->command_queue.push_back(local_command_.front());

        break;
      }
    }
    local_command_.erase(local_command_.begin());
  }

  return;
}






/*v2-AMR with line tape*/
/*------------------------------------------------------------------odometer*/
void DrivingController::TransactionDataFromMoudules()
{
    /*2. Transcation data and command*/
    int left_pos_data = 0, right_pos_data = 0;
    int left_vel = 0, right_vel = 0;
    bool is_motor_data_renew = false;

    // int current_pos_data = 0;

    bool is_pos_data_new = false;
    PosSensorDataStruct pos_data_buf;


    this->DataMutexLock();

    /*Send command motor*/
    // this->motor_command_struct_.push_back(left_data);
    // this->motor_command_struct_.push_back(right_data);

    if(this->motor_data_renew_)
    {
      left_vel = this->motor_infos_.at(0)->current_motor_speed;
      right_vel= this->motor_infos_.at(1)->current_motor_speed;

      left_pos_data = this->motor_infos_.at(0)->current_motor_pos;
      right_pos_data = this->motor_infos_.at(1)->current_motor_pos;

      is_motor_data_renew = true;

      this->motor_data_renew_ = false;
    }

    if(this->pos_data_renew_)
    {
      pos_data_buf = this->pos_sensor_data_buffer_;

      is_pos_data_new = true;

      this->pos_data_renew_ = false;
    }

    this->DataMutexUnLock();

    if(is_motor_data_renew)
    {
      this->left_current_motor_encoder_ = left_pos_data;
      this->right_current_motor_encoder_ = right_pos_data;

      this->left_current_motor_speed_encoder = left_vel;
      this->right_current_motor_speed_encoder = right_vel;

      // this->left_current_motor_encoder_ = this->left_motor_->motor_data_.front()->current_motor_pos;
      // this->right_current_motor_encoder_ = this->right_motor_->motor_data_.front()->current_motor_pos;

      // this->left_current_motor_speed_ = this->left_motor_->motor_data_.front()->current_motor_speed;
      // this->right_current_motor_speed_ = this->right_motor_->motor_data_.front()->current_motor_speed;

      this->CalculateOdometry();
      // this->test_sig = true;
    }

    if(is_pos_data_new)
    {
      this->is_detected_ = pos_data_buf.is_detected;
      this->is_tagged_ = pos_data_buf.is_tagged;

      /*line tape detected*/
      if(this->is_detected_ && !(this->is_tagged_))
      {
        this->is_detected_ = pos_data_buf.is_detected;

        this->x_pos_ = (double)pos_data_buf.xpos/10;
        this->y_pos_ = 1* ((double)pos_data_buf.ypos/10);

        this->angle_ = -1*((double)pos_data_buf.angle/10) + this->angle_offset_; //270 - small 90 - big
        this->angle_ = (M_PI/180 * this->angle_);
      }
      /*QR code detected*/
      else if(this->is_tagged_)
      {
        this->is_tagged_ = pos_data_buf.is_tagged;

        this->x_qrpos_ = (double)pos_data_buf.xpos/10;
        this->y_qrpos_ = 1* ((double)pos_data_buf.ypos/10);

        this->tag_code_  = pos_data_buf.tag_code;

        this->angle_ = -1*((double)pos_data_buf.angle/10) + this->angle_offset_; //270 - small 90 - big
        this->angle_ = (M_PI/180 * this->angle_);

      }
      else 
      {
        this->is_detected_ = false;
        this->is_tagged_ = false;
      }
      // printf("xpos = %f, ypos = %f, angle = %f\r\n", this->x_pos_, this->y_pos_, this->angle_);
    }


  return;
}


void DrivingController::CalculateOdometry()
{
  if(this->left_prev_motor_encoder_ == 0 || this->right_prev_motor_encoder_ == 0)
  {
    this->left_prev_motor_encoder_ = this->left_current_motor_encoder_;
    this->right_prev_motor_encoder_ = this->right_current_motor_encoder_;

    return;
  }


  int left_moved_encoder = this->left_current_motor_encoder_ - this->left_prev_motor_encoder_;
  int right_moved_encoder = this->right_current_motor_encoder_ - this->right_prev_motor_encoder_;
  int aux_moved_encoder = 0; //for test

  double mm_for_ticks = ((2.0 * M_PI * this->wheel_radius_)/(this->encoder_ticks_* this->reducer_rate_));

  /*moved distance (mm)*/

  // int left_moved_dist = left_moved_encoder*mm_for_ticks;
  // int right_moved_dist = right_moved_encoder*mm_for_ticks;

  // this->left_moved_dist_ += left_moved_dist;
  // this->right_moved_dist_ += right_moved_dist;

  // printf("left moved dist = %f, right moved dist = e%f\r\n", left_moved_dist_, right_moved_dist_);

  double dx = mm_for_ticks *((left_moved_encoder + right_moved_encoder)/2.0);
  double dy = mm_for_ticks *(aux_moved_encoder - (right_moved_encoder - left_moved_encoder) * this->point_between_aux_/this->wheel_between_length_);
  double dtheta = mm_for_ticks *((right_moved_encoder - left_moved_encoder)/this->wheel_between_length_);



  
  /*renew position*/
  double theta =  this->angle_ + (dtheta/2.0);
  this->x_pos_ += dx*cos(theta) - dy *sin(theta);
  this->y_pos_ += dx*sin(theta) + dy *cos(theta);
  this->angle_ += dtheta; 

  /*angle limit*/
  // this->angle_ = this->angle_ % (2.0 * M_PI);
  if(this->angle_ > M_PI) this->angle_ -= 2.0 * M_PI;
  if(this->angle_ < -M_PI) this->angle_ += 2.0 * M_PI;

  /*to see ease*/
  this->angle_degree_ = this->angle_ * 180/M_PI;



  this->left_prev_motor_encoder_ = this->left_current_motor_encoder_;
  this->right_prev_motor_encoder_ = this->right_current_motor_encoder_;

  this->left_current_motor_speed_ = this->left_current_motor_speed_encoder*mm_for_ticks;
  this->right_current_motor_speed_ = this->right_current_motor_speed_encoder*mm_for_ticks;

// this->current_left_pos_ = ((double)left_moved_encoder/(this->encoder_ticks_ * this->reducer_rate_))*(2*M_PI *this->wheel_radius_) + this->current_left_pos_;
    // this->current_right_pos_ = ((double)right_moved_encoder/(this->encoder_ticks_ * this->reducer_rate_))*(2*M_PI *this->wheel_radius_) + this->current_left_pos_;




  return;
}


void DrivingController::ControlVelocity()
{
  if(this->target_pos_queue_.empty())
  {
    // Isv2MotorCommandStruct left_data, right_data;

    // left_data.id = LEFT_MOTOR_NODE;
    // right_data.id = RIGHT_MOTOR_NODE;
    // left_data.command = cSetMoveStop;
    // right_data.command = cSetMoveStop;
    // left_data.data = 0.0f;
    // right_data.data = 0.0f;


    // /*5. set data to motor*/
    // this->DataMutexLock();

    // /*Send command motor*/
    // this->motor_command_struct_.push_back(left_data);
    // this->motor_command_struct_.push_back(right_data);

    // this->quit_sig_ = true;

    // this->DataMutexUnLock();

    // return;
  }
  else
  {
    TargetPosStruct target = this->target_pos_queue_.front();

    double allow_limit_pos = 2;
    double allow_limit_angle = 1.0f;


    double target_pos_x = target.x_pos;
    double target_pos_y = target.y_pos;
    double target_angle = target.angle * M_PI/180;


    #if TEST_CAL_VEL_DEFINE  == 1

      /*test function*/
      /*combine below 1,2 and 3 */
      this->CalculateTargetVelocity(target_pos_x, target_pos_y, target_angle);

    #else

      /*1. match the direciton to target position with only rotation*/
      if(this->task_ == 0)
      {
        this->CalculateDirAngle(target_pos_x, target_pos_y);

        double dx = target_pos_x - this->x_pos_;
        double dy = target_pos_y - this->y_pos_;

        double target_theta = atan2(dy, dx);

        if(abs(target_theta - this->angle_) <= (allow_limit_angle*M_PI/180))
        {
          Isv2MotorCommandStruct left_data, right_data;

          left_data.id = LEFT_MOTOR_NODE;
          right_data.id = RIGHT_MOTOR_NODE;
          left_data.command = cSetMoveStop;
          right_data.command = cSetMoveStop;
          left_data.data = 0.0f;
          right_data.data = 0.0f;

          /*. set data to motor*/
          this->DataMutexLock();

          /*Send command motor*/
          this->motor_command_struct_.push_back(left_data);
          this->motor_command_struct_.push_back(right_data);

          this->DataMutexUnLock();

          this->task_  = 1;
        }
      }
      /*2. Get vel to Target velocity with following mini target position*/
      else if(this->task_ == 1)
      {
        this->CalculateTargetVel(target_pos_x, target_pos_y);

        if(abs(target_pos_x - this->x_pos_) <= allow_limit_pos && 
            abs(target_pos_y - this->y_pos_) <= allow_limit_pos &&
          this->task_ == 1 )
        {
          Isv2MotorCommandStruct left_data, right_data;

          left_data.id = LEFT_MOTOR_NODE;
          right_data.id = RIGHT_MOTOR_NODE;
          left_data.command = cSetMoveStop;
          right_data.command = cSetMoveStop;
          left_data.data = 0.0f;
          right_data.data = 0.0f;

          /*. set data to motor*/
          this->DataMutexLock();

          /*Send command motor*/
          this->motor_command_struct_.clear();

          this->motor_command_struct_.push_back(left_data);
          this->motor_command_struct_.push_back(right_data);

          this->DataMutexUnLock();

          this->task_  = 2;
        }
      }
      /*3. making pose to desire target pose*/
      else if(this->task_ == 2)
      {
        this->CalculateTargetPose( target_angle);

        if(abs(target_angle - this->angle_) <= (allow_limit_angle*M_PI/180) && this->task_ == 2 )
          this->task_ = 10;
      }



    #endif
  
    if( this->task_  == 10)
    {
      // if(abs(target_pos_x - this->x_pos_) <= allow_limit_pos && 
      //     abs(target_pos_y - this->y_pos_) <= allow_limit_pos  &&
      //     abs(target_angle - this->angle_) <= (allow_limit_angle*M_PI/180))
      // {
        Isv2MotorCommandStruct left_data, right_data;

        left_data.id = LEFT_MOTOR_NODE;
        right_data.id = RIGHT_MOTOR_NODE;
        left_data.command = cSetMoveStop;
        right_data.command = cSetMoveStop;
        left_data.data = 0.0f;
        right_data.data = 0.0f;


        /*5. set data to motor*/
        this->DataMutexLock();

        /*Send command motor*/
        this->motor_command_struct_.clear();
        
        this->motor_command_struct_.push_back(left_data);
        this->motor_command_struct_.push_back(right_data);

        this->DataMutexUnLock();

        
        this->target_pos_queue_.erase(this->target_pos_queue_.begin());

        printf("Arrived at target pos : %f, %f, %f is completed\r\n", target.x_pos, target.y_pos, target.angle);
      
        this->task_ = 0;
      }
        // else
      // {
      //   this->task_  = 1;

      // }
    // }
  }
  return;
}


void DrivingController::CalculateTargetVelocity(double target_pos_x, double target_pos_y, double target_angle)
{
  /*1. Get deltas*/
  double theta_modify = 0.4;
  double vel_modify = 0.4;

  double dx = target_pos_x - this->x_pos_;
  double dy = target_pos_y - this->y_pos_;

  double target_theta = atan2(dy, dx);



  // double dtheta =  target_angle - this->angle_;
  double dtheta =  target_theta - this->angle_;

  int dir_pox_x = 1;
  int dir_pox_y = 1;

  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;

  // if(dx < 0)
  //   dir_pox_x = -1;


  // if(dy < 0)
  //   dir_pox_y = -1;



  /*2. compute values*/
  double vel = vel_modify *sqrt(pow(dx, 2) + pow(dy, 2));
  dtheta *= theta_modify;

  /*3. compute each wheel theta*/
  const double s = this->wheel_between_length_;
  const double r = this->wheel_radius_;

  double left_theta  = (s/(pow(r, 2))) *((r/s)*vel - (r/2)*dtheta);
  double right_theta  = (s/(pow(r, 2))) *((r/s)*vel + (r/2)*dtheta);

  /*4 get wheel velue*/

  double left_vel = r* left_theta;
  double right_vel = r* right_theta;

  // if(left_vel<= 10)
  //   left_vel= 10;

  // if(right_vel<= 10)
  //   right_vel= 10;

  if(left_vel >= this->max_vel_)
   left_vel = this->max_vel_;

  if(right_vel >= this->max_vel_)
   right_vel = this->max_vel_;

  left_vel = left_vel * dir_pox_x * dir_pox_y;
  right_vel = right_vel* dir_pox_x * dir_pox_y;


  Isv2MotorCommandStruct left_data, right_data;

  left_data.id = LEFT_MOTOR_NODE;
  right_data.id = RIGHT_MOTOR_NODE;
  // left_data.command = cSetMoveStop;
  // right_data.command = cSetMoveStop;
  left_data.command = cSetMoveVel;
  right_data.command = cSetMoveVel;
  left_data.data = left_vel;
  right_data.data = right_vel;



  /*5. set data to motor*/
  this->DataMutexLock();

  /*Send command motor*/
  this->motor_command_struct_.clear();

  this->motor_command_struct_.push_back(left_data);
  this->motor_command_struct_.push_back(right_data);


  this->DataMutexUnLock();

  return;
}

void DrivingController::CalculateDirAngle(double target_pos_x, double target_pos_y)
{
  /*1. Get deltas*/
  double theta_modify = 0.8;

  double dx = target_pos_x - this->x_pos_;
  double dy = target_pos_y - this->y_pos_;

  double target_theta = atan2(dy, dx);

  double dtheta =  target_theta - this->angle_;

  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;


  /*2. compute values*/
  dtheta *= theta_modify;

  /*3. compute each wheel theta*/
  const double s = this->wheel_between_length_;
  const double r = this->wheel_radius_;

  double left_theta  = (s/(pow(r, 2))) *(-1 *(r/2)*dtheta);
  double right_theta  = (s/(pow(r, 2))) *((r/2)*dtheta);

  /*4 get wheel velue*/

  double left_vel = r* left_theta;
  double right_vel = r* right_theta;


  if(left_vel >= this->max_vel_)
   left_vel = this->max_vel_;

  if(right_vel >= this->max_vel_)
   right_vel = this->max_vel_;

  left_vel = left_vel;
  right_vel = right_vel;

  Isv2MotorCommandStruct left_data, right_data;

  left_data.id = LEFT_MOTOR_NODE;
  right_data.id = RIGHT_MOTOR_NODE;

  left_data.command = cSetMoveVel;
  right_data.command = cSetMoveVel;
  left_data.data = left_vel;
  right_data.data = right_vel;


  /*5. set data to motor*/
  this->DataMutexLock();

  /*Send command motor*/
  this->motor_command_struct_.clear();

  this->motor_command_struct_.push_back(left_data);
  this->motor_command_struct_.push_back(right_data);

  this->DataMutexUnLock();

  return;
}

void DrivingController::CalculateTargetVel(double target_pos_x, double target_pos_y)
{

  /*1. Get deltas*/
  double theta_modify = 1.5;
  double vel_modify = 0.5;


  double dx = target_pos_x - this->x_pos_;
  double dy = target_pos_y - this->y_pos_;

  double target_theta = atan2(dy, dx);

  double dtheta =  target_theta - this->angle_;

  int dir_pox_x = 1;
  int dir_pox_y = 1;

  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;

  /*2. compute values*/
  double vel = sqrt(pow(dx, 2) + pow(dy, 2));

  if(vel >= 1000)
    vel = 1000;

  vel *= vel_modify;
  dtheta *= theta_modify;

  /*3. compute each wheel theta*/
  const double s = this->wheel_between_length_;
  const double r = this->wheel_radius_;

  double left_theta  = (s/(pow(r, 2))) *((r/s)*vel - (r/2)*dtheta);
  double right_theta  = (s/(pow(r, 2))) *((r/s)*vel + (r/2)*dtheta);

  /*4 get wheel velue*/
  double left_vel = r* left_theta;
  double right_vel = r* right_theta;

  if(left_vel >= this->max_vel_)
   left_vel = this->max_vel_;

  if(right_vel >= this->max_vel_)
   right_vel = this->max_vel_;

  left_vel = left_vel * dir_pox_x * dir_pox_y;
  right_vel = right_vel* dir_pox_x * dir_pox_y;


  Isv2MotorCommandStruct left_data, right_data;

  left_data.id = LEFT_MOTOR_NODE;
  right_data.id = RIGHT_MOTOR_NODE;

  left_data.command = cSetMoveVel;
  right_data.command = cSetMoveVel;
  left_data.data = left_vel;
  right_data.data = right_vel;



  /*5. set data to motor*/
  this->DataMutexLock();

  /*Send command motor*/
  this->motor_command_struct_.clear();

  this->motor_command_struct_.push_back(left_data);
  this->motor_command_struct_.push_back(right_data);

  this->DataMutexUnLock();

  return;
}

void DrivingController::CalculateTargetPose(double target_angle)
{
  /*1. Get deltas*/
  double theta_modify = 0.8;

  double dtheta =  target_angle - this->angle_;

  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;

  /*2. compute values*/
  dtheta *= theta_modify;

  /*3. compute each wheel theta*/
  const double s = this->wheel_between_length_;
  const double r = this->wheel_radius_;

  double left_theta  = (s/(pow(r, 2))) *(-1 *(r/2)*dtheta);
  double right_theta  = (s/(pow(r, 2))) *((r/2)*dtheta);

  /*4 get wheel velue*/
  double left_vel = r* left_theta;
  double right_vel = r* right_theta;

  if(left_vel >= this->max_vel_)
   left_vel = this->max_vel_;

  if(right_vel >= this->max_vel_)
   right_vel = this->max_vel_;

  left_vel = left_vel;
  right_vel = right_vel;


  Isv2MotorCommandStruct left_data, right_data;

  left_data.id = LEFT_MOTOR_NODE;
  right_data.id = RIGHT_MOTOR_NODE;

  left_data.command = cSetMoveVel;
  right_data.command = cSetMoveVel;
  left_data.data = left_vel;
  right_data.data = right_vel;


  /*5. set data to motor*/
  this->DataMutexLock();

  /*Send command motor*/
  this->motor_command_struct_.clear();

  this->motor_command_struct_.push_back(left_data);
  this->motor_command_struct_.push_back(right_data);

  this->DataMutexUnLock();

  return;
}



/*v3 - SCURVE WITH DATA MATRIX TYPE*/
/*------------------------------------------------------------------QR*/


void DrivingController::TestQRMoveDrive()
{
  if(!(this->now_follow_left_profiler_))
  {
    this->now_follow_left_profiler_ = new ScurveProfiler(500, this->max_acc_, this->max_vel_); //jerk, acc, vel

    this->now_follow_left_profiler_->SetTargetPosition(this->current_target_pos_);

    this->now_follow_left_profiler_->SetPresentPosition(this->current_left_pos_);
  }
  else
  {
    this->now_follow_left_profiler_->SetPresentPosition(this->current_left_pos_);

    this->now_follow_left_profiler_->SetPresentSpeed(this->left_motor_s);

    this->now_follow_left_profiler_->Drive();
  }

  if(!(this->now_follow_right_profiler_))
  {
    this->now_follow_left_profiler_ = new ScurveProfiler(500, this->max_acc_, this->max_vel_); //jerk, acc, vel

    this->now_follow_right_profiler_->SetTargetPosition(this->current_target_pos_);

    this->now_follow_right_profiler_->SetPresentPosition(this->current_right_pos_);
  }
  else
  {
    this->now_follow_left_profiler_->SetPresentPosition(this->current_left_pos_);

    this->now_follow_left_profiler_->SetPresentSpeed(this->left_motor_s);

    this->now_follow_right_profiler_->Drive();
  }

  return;
}

void DrivingController::TestQRDirAngle(double dest_pose)
{
  /*1. set data to controller*/
  /*for calculate total distance*/
  double wheel_seperate = this->wheel_between_length_;
  // double current_left_encoder = this->left_current_motor_encoder_;
  // double current_right_encoder = this->right_current_motor_encoder_;

  /*2. */


  return;
}

void DrivingController::TestQRMoveOnTag(double target_angle)
{
  /*1. Get deltas*/
  double theta_modify = 0.2;
  double vel_modify = 0.2;
  int vel_dir = 1;

  double dx = 1* this->x_qrpos_;
  double dy = 1* this->y_qrpos_;

  double target_theta = atan2(dy, dx);

  double dtheta =  (target_theta);


  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;

  if(abs(dtheta) <= M_PI/2)
    vel_dir = -1;

  /*2. compute values*/
  double vel = sqrt(pow(dx, 2) + pow(dy, 2));

  vel *= (vel_modify * vel_dir);
  dtheta *= theta_modify *vel_dir;

  /*3. compute each wheel theta*/
  const double s = this->wheel_between_length_;
  const double r = this->wheel_radius_;

  double left_theta  = (s/(pow(r, 2))) *((r/s)*vel - (r/2)*dtheta);
  double right_theta  = (s/(pow(r, 2))) *((r/s)*vel + (r/2)*dtheta);


  // if(vel_dir == 1)
  // {
   left_theta  = (s/(pow(r, 2))) *((r/s)*vel - (r/2)*dtheta);
   right_theta  = (s/(pow(r, 2))) *((r/s)*vel + (r/2)*dtheta);
  // }
  // else
  // {
  //  left_theta  = (s/(pow(r, 2))) *((r/s)*vel);
  //  right_theta  = (s/(pow(r, 2))) *((r/s)*vel);
  // }



  /*4 get wheel velue*/
  double left_vel = r* left_theta;
  double right_vel = r* right_theta;

  if(left_vel >= this->max_vel_)
   left_vel = this->max_vel_;

  if(right_vel >= this->max_vel_)
   right_vel = this->max_vel_;

  // left_vel = left_vel ;
  // right_vel = right_vel ;

  Isv2MotorCommandStruct left_data, right_data;

  left_data.id = LEFT_MOTOR_NODE;
  right_data.id = RIGHT_MOTOR_NODE;

  left_data.command = cSetMoveVel;
  right_data.command = cSetMoveVel;
  left_data.data = left_vel;
  right_data.data = right_vel;



  /*5. set data to motor*/
  this->DataMutexLock();

  /*Send command motor*/
  this->motor_command_struct_.clear();


  this->motor_command_struct_.push_back(left_data);
  this->motor_command_struct_.push_back(right_data);


  this->DataMutexUnLock();
}



void DrivingController::TestQRTagCalculateDirAngle()
{
  /*1. Get deltas*/
  double theta_modify = 0.2;

  double dx = this->x_qrpos_;
  double dy = this->y_qrpos_;

  double target_theta = atan2(dy, dx);

  double dtheta =  target_theta - this->angle_;

  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;


  /*2. compute values*/
  dtheta *= theta_modify;

  /*3. compute each wheel theta*/
  const double s = this->wheel_between_length_;
  const double r = this->wheel_radius_;

  double left_theta  = (s/(pow(r, 2))) *(-1 *(r/2)*dtheta);
  double right_theta  = (s/(pow(r, 2))) *((r/2)*dtheta);

  /*4 get wheel velue*/

  double left_vel = r* left_theta;
  double right_vel = r* right_theta;


  if(left_vel >= this->max_vel_)
   left_vel = this->max_vel_;

  if(right_vel >= this->max_vel_)
   right_vel = this->max_vel_;

  left_vel = left_vel;
  right_vel = right_vel;

  Isv2MotorCommandStruct left_data, right_data;

  left_data.id = LEFT_MOTOR_NODE;
  right_data.id = RIGHT_MOTOR_NODE;

  left_data.command = cSetMoveVel;
  right_data.command = cSetMoveVel;
  left_data.data = left_vel;
  right_data.data = right_vel;


  /*5. set data to motor*/
  this->DataMutexLock();

  /*Send command motor*/
  this->motor_command_struct_.clear();

  this->motor_command_struct_.push_back(left_data);
  this->motor_command_struct_.push_back(right_data);

  this->DataMutexUnLock();

  return;
}

void DrivingController::TestQRTagMoveToOrigin(double target_pos_x, double target_pos_y)
{
  /*1. Get deltas*/
  double theta_modify = 1.0;
  double vel_modify = 1.0;

  double dx = this->x_qrpos_;
  double dy = this->y_qrpos_;

  int move_reverse = 1;

  /*2. compute values*/
  double target_theta = atan2(target_pos_y, target_pos_x);
  double current_theta = atan2(dy, dx);

  double dtheta =  target_theta - current_theta;

  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;

  /*at this point we can know how the vel is reverse or not*/
  if(abs(dtheta) > M_PI/2)
    move_reverse = -1;

  double vel = sqrt(pow(dx, 2) + pow(dy, 2));

  vel *= (vel_modify * move_reverse);
  dtheta *= theta_modify *move_reverse;

  /*3. compute each wheel theta*/
  const double s = this->wheel_between_length_;
  const double r = this->wheel_radius_;

  double left_theta  = (s/(pow(r, 2))) *((r/s)*vel /*- (r/2)*dtheta*/);
  double right_theta  = (s/(pow(r, 2))) *((r/s)*vel /*+ (r/2)*dtheta*/);

  /*4 get wheel velue*/
  double left_vel = r* left_theta;
  double right_vel = r* right_theta;

  // if(left_vel >= this->max_vel_)
  //  left_vel = this->max_vel_;

  // if(right_vel >= this->max_vel_)
  //  right_vel = this->max_vel_;

  if(left_vel >= this->max_vel_)
   left_vel = this->max_vel_;

  if(right_vel >= this->max_vel_)
   right_vel = this->max_vel_;



  left_vel = left_vel;
  right_vel = right_vel;

  Isv2MotorCommandStruct left_data, right_data;

  left_data.id = LEFT_MOTOR_NODE;
  right_data.id = RIGHT_MOTOR_NODE;

  left_data.command = cSetMoveVel;
  right_data.command = cSetMoveVel;
  left_data.data = left_vel;
  right_data.data = right_vel;


  /*5. set data to motor*/
  this->DataMutexLock();

  /*Send command motor*/
  this->motor_command_struct_.clear();

  this->motor_command_struct_.push_back(left_data);
  this->motor_command_struct_.push_back(right_data);

  this->DataMutexUnLock();

  return;
}




void DrivingController::TestQRCalculateTargetPose(double target_angle)
{
  /*1. Get deltas*/
  double theta_modify = 1.0;

  double dtheta =  target_angle - this->angle_;

  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;

  /*2. compute values*/
  dtheta *= theta_modify;

  /*3. compute each wheel theta*/
  const double s = this->wheel_between_length_;
  const double r = this->wheel_radius_;

  double left_theta  = (s/(pow(r, 2))) *(-1 *(r/2)*dtheta);
  double right_theta  = (s/(pow(r, 2))) *((r/2)*dtheta);

  /*4 get wheel velue*/
  double left_round_dist = r* left_theta;
  double right_round_dist = r* right_theta;


  /*init scurve*/
  if(!(this->is_moving_))
  {
    /*Set initial odometery due to calculate moving distance*/

    if(this->now_follow_left_profiler_)
      delete this->now_follow_left_profiler_;

    if(this->now_follow_right_profiler_)
      delete this->now_follow_right_profiler_;


    /*left*/
    this->now_follow_left_profiler_ = new ScurveProfiler(this->max_acc_, this->max_acc_, this->max_vel_); //jerk, acc, vel

    this->now_follow_left_profiler_->SetTargetPosition(left_round_dist);

    this->now_follow_left_profiler_->SetPresentPosition(0);

    // this->now_follow_left_profiler_->SetPresentPosition(this->current_left_pos_);

    // this->now_follow_left_profiler_->SetPresentPosition(this->left_moved_dist_);


    /*right*/
    this->now_follow_right_profiler_ = new ScurveProfiler(this->max_acc_, this->max_acc_, this->max_vel_); //jerk, acc, vel

    this->now_follow_right_profiler_->SetTargetPosition(right_round_dist);

    this->now_follow_right_profiler_->SetPresentPosition(0);

    // this->now_follow_right_profiler_->SetPresentPosition(this->current_right_pos_);

    // this->now_follow_right_profiler_->SetPresentPosition(this->right_moved_dist_);

    this->is_moving_ = true;
  }

  double left_vel_data = 0.0f;
  double right_vel_data = 0.0f;

  if((this->is_moving_))
  {
    // this->now_follow_left_profiler_->SetPresentPosition(this->current_left_pos_);

    // this->now_follow_left_profiler_->SetPresentSpeed(this->left_motor_s);

    // this->now_follow_left_profiler_->SetPresentPosition(this->left_moved_dist_);

    this->now_follow_left_profiler_->Drive();

    double left_vel_data_ = this->now_follow_left_profiler_->GetCalculatedVel();

    left_vel_data = left_vel_data_;

    // this->now_follow_right_profiler_->SetPresentPosition(this->current_right_pos_);

    // this->now_follow_right_profiler_->SetPresentSpeed(this->right_motor_s);

    // this->now_follow_right_profiler_->SetPresentPosition(this->right_moved_dist_);

    this->now_follow_right_profiler_->Drive();

    double right_vel_data_ = this->now_follow_right_profiler_->GetCalculatedVel();

    right_vel_data = right_vel_data_;

    int left_status = this->now_follow_left_profiler_->GetStatusProfiler();

    int right_status = this->now_follow_right_profiler_->GetStatusProfiler();


    if(left_status >= 3 && right_status >= 3)
    {
      this->is_moving_ = false;

      delete this->now_follow_left_profiler_;

      this->now_follow_left_profiler_ = nullptr;

      delete this->now_follow_right_profiler_;

      this->now_follow_right_profiler_ = nullptr;

      // test_bool = true;

      // printf("final left status = %d, final right status = %d, final left Pos data  = %f, final right Pos data  = %f\r\n", left_status, right_status,  this->current_left_pos_,   this->current_right_pos_);
    }

  }

  // if(left_vel_data >= this->max_vel_)
  //  left_vel_data = this->max_vel_;

  // if(right_vel_data >= this->max_vel_)
  //  right_vel_data = this->max_vel_;


  Isv2MotorCommandStruct left_data, right_data;

  left_data.id = LEFT_MOTOR_NODE;
  right_data.id = RIGHT_MOTOR_NODE;

  left_data.command = cSetMoveVel;
  right_data.command = cSetMoveVel;
  left_data.data = left_vel_data;
  right_data.data = right_vel_data;


  /*5. set data to motor*/
  this->DataMutexLock();

  /*Send command motor*/
  this->motor_command_struct_.clear();

  this->motor_command_struct_.push_back(left_data);
  this->motor_command_struct_.push_back(right_data);

  this->DataMutexUnLock();

  return;
}


double dtheta_last = 0.0f;

void DrivingController::TestQRCalculateTargetVel(double target_pos_x, double target_pos_y)
{

  /*1. Get deltas*/
  double theta_modify = 1.0 ;
  double thata_deriv_mod = 1.0;
  // double vel_modify = 0.5 ;

  double abs_target_thata = atan2(target_pos_y, target_pos_x);

  double present_x_pos = this->x_pos_;
  double present_y_pos = this->y_pos_;

  if(this->is_tagged_)
  {
    if((int)target_pos_y == 0 && target_pos_x != 0) // abs_target_theta = 0 ,180 or -180 degrees
    {
      this->y_pos_ = this->y_qrpos_;

      present_y_pos = this->y_pos_;

      printf("X Direction Moving : Tagged = %d, y_pos = %f, angle = %f\r\n", this->tag_code_, this->y_pos_, this->angle_);
    }
    else if(target_pos_y != 0 && (int)target_pos_x == 0) //abs_target_theta = 90 or-90 degrees   
    {
      this->x_pos_ = this->x_qrpos_;

      present_x_pos = this->x_pos_;

      printf("Y Direction Moving : Tagged = %d, x_pos = %f, angle = %f\r\n", this->tag_code_, this->x_pos_, this->angle_);

    }
    else
    {
      this->y_pos_ = this->y_qrpos_;
      this->x_pos_ = this->x_qrpos_;
      present_x_pos = this->x_pos_;
      present_y_pos = this->y_pos_;
    }
  }


  double target_dist = 0.0f;

  double dx = target_pos_x - present_x_pos;
  double dy = target_pos_y - present_y_pos;

  double current_pos = 0.0f;

  if((int)target_pos_y == 0 && target_pos_x != 0) //abs_target_theta = 0 ,180 or -180 degrees
  {
    present_x_pos = target_pos_x/abs(target_pos_x) *sqrt(pow(this->wheel_between_length_ *0.5, 2)- pow(present_y_pos, 2));

    current_pos = this->x_pos_;

    dx = present_x_pos;

    target_dist = target_pos_x;

    // dy = target_pos_y - present_y_pos;

  }
  else if(target_pos_y != 0 && (int)target_pos_x == 0) //abs_target_theta = 90 or-90 degrees
  {
    // this->x_pos_ = this->x_qrpos_;

    present_y_pos = target_pos_y/abs(target_pos_y) *sqrt(pow(this->wheel_between_length_ *0.5, 2)- pow(present_x_pos, 2));

    current_pos = this->y_pos_;
    // dx = target_pos_x - present_x_pos;
    dy = present_y_pos;

    target_dist = target_pos_y;

  }


  double target_theta = atan2(dy, dx);

  double dtheta = (target_theta - this->angle_);

  if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
  else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;


    /*2. compute values*/


  /*init scurve*/
  if(!(this->is_moving_))
  {
    /*2. compute values*/
    // if(!(this->is_tagged_))
    // {
      dx = target_pos_x;
      dy = target_pos_y; 
    // }

    double vel = sqrt(pow(dx, 2) + pow(dy, 2));

    /*Set initial odometery due to calculate moving distance*/
    if(this->now_follow_left_profiler_)
      delete this->now_follow_left_profiler_;

    if(this->now_follow_right_profiler_)
      delete this->now_follow_right_profiler_;
      
    /*left*/
    this->now_follow_left_profiler_ = new ScurveProfiler(this->max_acc_, this->max_acc_, this->max_vel_); //jerk, acc, vel

    this->now_follow_left_profiler_->SetTargetPosition(vel);

    // this->now_follow_left_profiler_->SetPresentPosition(0);

    this->now_follow_left_profiler_->SetPresentSpeed(this->left_current_motor_speed_);

    this->now_follow_left_profiler_->SetPresentPosition(current_pos);

    printf("Current Pos = %f\r\n", current_pos);
  
    /*right*/
    this->now_follow_right_profiler_ = new ScurveProfiler(this->max_acc_, this->max_acc_, this->max_vel_); //jerk, acc, vel

    this->now_follow_right_profiler_->SetTargetPosition(vel);

    // this->now_follow_right_profiler_->SetPresentPosition(0);


    this->now_follow_right_profiler_->SetPresentSpeed(this->right_current_motor_speed_);

    this->now_follow_right_profiler_->SetPresentPosition(current_pos);

    this->is_moving_ = true;
  }

  /*output*/
  double left_vel_data = 0.0f;
  double right_vel_data = 0.0f;

  if((this->is_moving_))
  {
    // this->now_follow_left_profiler_->SetPresentPosition(this->current_left_pos_);

    this->now_follow_left_profiler_->SetPresentSpeed(this->left_current_motor_speed_);

    this->now_follow_left_profiler_->SetPresentPosition(current_pos);

    this->now_follow_left_profiler_->Drive();

    double left_vel_data_ = this->now_follow_left_profiler_->GetCalculatedVel();

    left_vel_data = left_vel_data_;

    // this->now_follow_right_profiler_->SetPresentPosition(this->current_right_pos_);

    this->now_follow_right_profiler_->SetPresentSpeed(this->right_current_motor_speed_);
    
    this->now_follow_right_profiler_->SetPresentPosition(current_pos);

    this->now_follow_right_profiler_->Drive();

    double right_vel_data_ = this->now_follow_right_profiler_->GetCalculatedVel();

    // right_vel_data = left_vel_data;

    right_vel_data = right_vel_data_;

    int left_status = this->now_follow_left_profiler_->GetStatusProfiler();

    int right_status = this->now_follow_right_profiler_->GetStatusProfiler();




    if(left_status >= 3 && right_status >= 3)
    {
      this->is_moving_ = false;

      delete this->now_follow_left_profiler_;

      this->now_follow_left_profiler_ = nullptr;

      delete this->now_follow_right_profiler_;

      this->now_follow_right_profiler_ = nullptr;

      // test_bool = true;

      // printf("final left status = %d, final right status = %d, final left Pos data  = %f, final right Pos data  = %f\r\n", left_status, right_status,  this->current_left_pos_,   this->current_right_pos_);
    
      return;
    }


    // vel = vel_data;
    double dtheta_mod = (dtheta - dtheta_last)* 0.01;

    dtheta_last = dtheta;

    dtheta = (dtheta * theta_modify) + (dtheta_mod *thata_deriv_mod);



    // dtheta = (dtheta * theta_modify) ;
    if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
    else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;

    // if(abs(dtheta <= 0.0174533/2)) //0.5 degree
    //   dtheta = 0;


    // double degree_val = 2* M_PI/180;

    // if(abs(dtheta) > degree_val)
    //   dtheta = (dtheta/abs(dtheta))* degree_val;

    /*3. compute each wheel theta*/
    const double s = this->wheel_between_length_;
    const double r = this->wheel_radius_;


    // if(angle_rev == 1)
    // {
   double left_theta =  (s/(pow(r, 2))) *((r/s)*left_vel_data - (r/2)*dtheta);
   double right_theta =  (s/(pow(r, 2))) *((r/s)*right_vel_data + (r/2)*dtheta);
    // double left_theta =  (s/(pow(r, 2))) *((r/s)*left_vel_data - left_vel_data*dtheta);
    // double right_theta =  (s/(pow(r, 2))) *((r/s)*right_vel_data + left_vel_data*dtheta);
    // }
    // else
    // {
    //   left_theta =  (s/(pow(r, 2))) *((r/s)*left_vel_data);
    //   right_theta =  (s/(pow(r, 2))) *((r/s)*right_vel_data);
    // }
  

       

    // double left_theta  = (s/(pow(r, 2))) *( -1* (r/2)*dtheta);
    // double right_theta  = (s/(pow(r, 2))) *((r/2)*dtheta);

    /*4 get wheel velue*/
    double left_vel = r* left_theta ;//+ left_vel_data_;
    double right_vel = r* right_theta;//  + right_vel_data_;





    // if(left_vel >= this->max_vel_)
    // left_vel = this->max_vel_;

    // if(right_vel >= this->max_vel_)
    // right_vel = this->max_vel_;


    Isv2MotorCommandStruct left_data, right_data;

    left_data.id = LEFT_MOTOR_NODE;
    right_data.id = RIGHT_MOTOR_NODE;

    left_data.command = cSetMoveVel;
    right_data.command = cSetMoveVel;
    left_data.data = left_vel;
    right_data.data = right_vel;



    /*5. set data to motor*/
    this->DataMutexLock();

    /*Send command motor*/
    this->motor_command_struct_.clear();

    this->motor_command_struct_.push_back(left_data);
    this->motor_command_struct_.push_back(right_data);


    this->DataMutexUnLock();





    /*worker - TestScurveProfileWorker  send - vel to other thread*/
  }


  return;
}


/*.total drive*/
void DrivingController::TestQRControlVelocity()
{
  if(this->target_pos_queue_.empty())
  {
    // Isv2MotorCommandStruct left_data, right_data;

    // left_data.id = LEFT_MOTOR_NODE;
    // right_data.id = RIGHT_MOTOR_NODE;
    // left_data.command = cSetMoveStop;
    // right_data.command = cSetMoveStop;
    // left_data.data = 0.0f;
    // right_data.data = 0.0f;


    // /*5. set data to motor*/
    // this->DataMutexLock();

    // /*Send command motor*/
    // this->motor_command_struct_.push_back(left_data);
    // this->motor_command_struct_.push_back(right_data);

    // this->quit_sig_ = true;

    // this->DataMutexUnLock();

    // return;
  }
  else
  {
    TargetPosStruct target = this->target_pos_queue_.front();

    double allow_limit_pos = 2;
    double allow_limit_angle = 1.5f;


    double target_pos_x = target.x_pos;
    double target_pos_y = target.y_pos;
    double target_angle = target.angle * M_PI/180;
    size_t target_qr_code = target.qr_code_;


    #if TEST_CAL_VEL_DEFINE  == 1

      /*test function*/
      /*combine below 1,2 and 3 */
      this->CalculateTargetVelocity(target_pos_x, target_pos_y, target_angle);

    #else
      if(this->task_ == 0) //rot to 0.0 point qr tag angle
      {
        // double dx = this->x_qrpos_;
        // double dy = this->y_qrpos_;

        // double target_theta = atan2(dy, dx);


        // this->TestQRTagCalculateDirAngle();

        // if(abs(target_theta - this->angle_) <= (allow_limit_angle*M_PI/180) && this->task_ == 0 )
        // {        
        //   Isv2MotorCommandStruct left_data, right_data;

        //   left_data.id = LEFT_MOTOR_NODE;
        //   right_data.id = RIGHT_MOTOR_NODE;
        //   left_data.command = cSetMoveStop;
        //   right_data.command = cSetMoveStop;
        //   left_data.data = 0.0f;
        //   right_data.data = 0.0f;

        //   /*. set data to motor*/
        //   this->DataMutexLock();

        //   /*Send command motor*/
        //   this->motor_command_struct_.clear();

        //   this->motor_command_struct_.push_back(left_data);
        //   this->motor_command_struct_.push_back(right_data);

        //   this->DataMutexUnLock();

        //   if(this->left_current_motor_speed_ <= 10 && this->right_current_motor_speed_ <= 10)
        //     this->task_  = 1;
        //  }
        this->x_pos_ = 0.0f;
        this->y_pos_ = 0.0f;

        // this->left_moved_dist_ = 0.0f;
        // this->right_moved_dist_ = 0.0f;

        this->task_  = 1;
      }
      else if(this->task_ == 1)  // move to 0. 0 point
      {
        // this->TestQRTagMoveToOrigin(target_pos_x, target_pos_y);

        // if(abs(this->x_qrpos_) <= allow_limit_pos 
        //     && abs(this->y_qrpos_) <= allow_limit_pos)
        // {
        //   Isv2MotorCommandStruct left_data, right_data;

        //   left_data.id = LEFT_MOTOR_NODE;
        //   right_data.id = RIGHT_MOTOR_NODE;
        //   left_data.command = cSetMoveStop;
        //   right_data.command = cSetMoveStop;
        //   left_data.data = 0.0f;
        //   right_data.data = 0.0f;

        //   /*. set data to motor*/
        //   this->DataMutexLock();

        //   /*Send command motor*/
        //   this->motor_command_struct_.clear();

        //   this->motor_command_struct_.push_back(left_data);
        //   this->motor_command_struct_.push_back(right_data);

        //   this->DataMutexUnLock();
          
        //   if(this->left_current_motor_speed_ <= 10 && this->right_current_motor_speed_ <= 10)
        //     this->task_  = 2;
        // }

        if(this->is_tagged_)
          this->task_  = 2;
      }
      /*2. Get vel to Target velocity until reach the qr-code destination*/
      else if(this->task_ == 2)
      {
        double target_theta = atan2(target_pos_y, target_pos_x);

        double dtheta = target_theta - this->angle_;

        if (dtheta > M_PI) dtheta -= 2.0 *M_PI;
        else if (dtheta < -M_PI) dtheta += 2.0 *M_PI;


        if(abs(dtheta) <= (allow_limit_angle*M_PI/180) && this->task_ == 2)
        {
          Isv2MotorCommandStruct left_data, right_data;

          left_data.id = LEFT_MOTOR_NODE;
          right_data.id = RIGHT_MOTOR_NODE;
          left_data.command = cSetMoveStop;
          right_data.command = cSetMoveStop;
          left_data.data = 0.0f;
          right_data.data = 0.0f;

          /*. set data to motor*/
          this->DataMutexLock();

          /*Send command motor*/
          this->motor_command_struct_.clear();

          this->motor_command_struct_.push_back(left_data);
          this->motor_command_struct_.push_back(right_data);

          this->DataMutexUnLock();

          if(abs(this->left_current_motor_speed_) <= 10 && abs(this->right_current_motor_speed_) <= 10)
            this->task_  = 3;
        }
        else
        {
            this->TestQRCalculateTargetPose(target_theta);
        }
          
        // }
      }
      /*3. making destination coordinate precisely*/
      else if(this->task_ == 3)
      {


        if(this->is_tagged_ && this->tag_code_ == target_qr_code && this->task_ == 3)
        {
          Isv2MotorCommandStruct left_data, right_data;

          left_data.id = LEFT_MOTOR_NODE;
          right_data.id = RIGHT_MOTOR_NODE;
          left_data.command = cSetMoveStop;
          right_data.command = cSetMoveStop;
          left_data.data = 0.0f;
          right_data.data = 0.0f;

          /*. set data to motor*/
          this->DataMutexLock();

          /*Send command motor*/
          this->motor_command_struct_.clear();

          this->motor_command_struct_.push_back(left_data);
          this->motor_command_struct_.push_back(right_data);

          this->DataMutexUnLock();

          if(abs(this->left_current_motor_speed_) <= 10 && abs(this->right_current_motor_speed_) <= 10)
            this->task_  = 4;
        }
        else
        {
          this->TestQRCalculateTargetVel(target_pos_x, target_pos_y);
        }



      }
      else if(this->task_ == 4)
      {
        // double dx = this->x_qrpos_;
        // double dy = this->y_qrpos_;

        // double target_theta = atan2(dy, dx);

        // this->TestQRTagCalculateDirAngle();

        // if(abs(target_theta - this->angle_) <= (allow_limit_angle*M_PI/180) && this->task_ == 4 )
        // {        
        //   Isv2MotorCommandStruct left_data, right_data;

        //   left_data.id = LEFT_MOTOR_NODE;
        //   right_data.id = RIGHT_MOTOR_NODE;
        //   left_data.command = cSetMoveStop;
        //   right_data.command = cSetMoveStop;
        //   left_data.data = 0.0f;
        //   right_data.data = 0.0f;

        //   /*. set data to motor*/
        //   this->DataMutexLock();

        //   /*Send command motor*/
        //   this->motor_command_struct_.clear();

        //   this->motor_command_struct_.push_back(left_data);
        //   this->motor_command_struct_.push_back(right_data);

        //   this->DataMutexUnLock();

        //   if(this->left_current_motor_speed_ <= 10 && this->right_current_motor_speed_ <= 10)
        //     this->task_  = 5;
        //  }

          double move_dist = 0.0f;

          double qr_pos_angle = atan2(this->y_qrpos_, this->x_qrpos_);

          if(this->is_tagged_)
          {
            if((int)target_pos_y == 0 && target_pos_x != 0) //abs_target_theta = 0 ,180 or -180 degrees
            {
              // this->y_pos_ = this->y_qrpos_;
              if(abs(this->angle_) > M_PI/2 && this->x_qrpos_ > 0)
                move_dist = abs(this->x_qrpos_);
              else if(abs(this->angle_) < M_PI/2 && this->x_qrpos_ < 0)
                move_dist = abs(this->x_qrpos_);
              else
                move_dist = allow_limit_pos;

              // present_x_pos = sqrt(pow(this->wheel_between_length_ *0.5, 2)- pow(present_y_pos, 2));

              // printf("Tagged = %d, y_pos = %f, angle = %f\r\n", this->tag_code_, this->y_pos_, this->angle_);
            }
            else if(target_pos_y != 0 && (int)target_pos_x == 0) //abs_target_theta = 90 or -90 degrees
            {
              // this->x_pos_ = this->x_qrpos_;
              if(qr_pos_angle > 0 && this->y_qrpos_ > 0)
                move_dist = abs(this->y_qrpos_);
              else if(qr_pos_angle < 0 && this->y_qrpos_ < 0)
                move_dist = abs(this->y_qrpos_);
              else
                move_dist = allow_limit_pos;

              // present_y_pos = sqrt(pow(this->wheel_between_length_ *0.5, 2)- pow(present_x_pos, 2));;
            }
            else
            {
              if(abs(this->x_qrpos_)<= allow_limit_pos || abs(this->y_qrpos_)<= allow_limit_pos)
                move_dist = allow_limit_pos;
              else
                move_dist = sqrt(pow(this->x_qrpos_, 2) + (pow(this->y_qrpos_, 2)))/sqrt(2);
            }


            if(move_dist > allow_limit_pos)
            {
              Isv2MotorCommandStruct left_data, right_data;

              left_data.id = LEFT_MOTOR_NODE;
              right_data.id = RIGHT_MOTOR_NODE;
              left_data.command = cSetMoveVel;
              right_data.command = cSetMoveVel;
              left_data.data = 50.0f;
              right_data.data = 50.0f;

              /*. set data to motor*/
              this->DataMutexLock();

              /*Send command motor*/
              this->motor_command_struct_.clear();

              this->motor_command_struct_.push_back(left_data);
              this->motor_command_struct_.push_back(right_data);

              this->DataMutexUnLock();
            }
            else
            {
              Isv2MotorCommandStruct left_data, right_data;

              left_data.id = LEFT_MOTOR_NODE;
              right_data.id = RIGHT_MOTOR_NODE;
              left_data.command = cSetMoveStop;
              right_data.command = cSetMoveStop;
              left_data.data = 0.0f;
              right_data.data = 0.0f;

              /*. set data to motor*/
              this->DataMutexLock();

              /*Send command motor*/
              this->motor_command_struct_.clear();

              this->motor_command_struct_.push_back(left_data);
              this->motor_command_struct_.push_back(right_data);

              this->DataMutexUnLock();

              this->task_  = 5;
            }



          }
          else
          {
            this->task_  = 5;
          }
      
        }
        else if(this->task_== 5) 
        {
          // this->TestQRTagMoveToOrigin(target_pos_x, target_pos_y);

          // if(abs(this->x_qrpos_) <= allow_limit_pos 
          //     && abs(this->y_qrpos_) <= allow_limit_pos)
          // {
          //   Isv2MotorCommandStruct left_data, right_data;

          //   left_data.id = LEFT_MOTOR_NODE;
          //   right_data.id = RIGHT_MOTOR_NODE;
          //   left_data.command = cSetMoveStop;
          //   right_data.command = cSetMoveStop;
          //   left_data.data = 0.0f;
          //   right_data.data = 0.0f;

          //   /*. set data to motor*/
          //   this->DataMutexLock();

          //   /*Send command motor*/
          //   this->motor_command_struct_.clear();

          //   this->motor_command_struct_.push_back(left_data);
          //   this->motor_command_struct_.push_back(right_data);

          //   this->DataMutexUnLock();
            
          //   if(this->left_current_motor_speed_ <= 10 && this->right_current_motor_speed_ <= 10)
          //     this->task_  = 6;
          // }

            this->task_  = 6;

        }
        else if (this->task_ == 6)
        {
          if(abs(target_angle - this->angle_) <= (allow_limit_angle*M_PI/180) && this->task_ == 6)
            {
            Isv2MotorCommandStruct left_data, right_data;

            left_data.id = LEFT_MOTOR_NODE;
            right_data.id = RIGHT_MOTOR_NODE;
            left_data.command = cSetMoveStop;
            right_data.command = cSetMoveStop;
            left_data.data = 0.0f;
            right_data.data = 0.0f;

            /*. set data to motor*/
            this->DataMutexLock();

            /*Send command motor*/
            this->motor_command_struct_.clear();

            this->motor_command_struct_.push_back(left_data);
            this->motor_command_struct_.push_back(right_data);

            this->DataMutexUnLock();
            
            if(this->left_current_motor_speed_ <= 10 && this->right_current_motor_speed_ <= 10)
              this->task_  = 10;
          }
          else
          {
            this->TestQRCalculateTargetPose(target_angle);
          }



      }



    #endif
  
    if( this->task_  == 10)
    {
      // if(abs(target_pos_x - this->x_pos_) <= allow_limit_pos && 
      //     abs(target_pos_y - this->y_pos_) <= allow_limit_pos  &&
      //     abs(target_angle - this->angle_) <= (allow_limit_angle*M_PI/180))
      // {
        Isv2MotorCommandStruct left_data, right_data;

        left_data.id = LEFT_MOTOR_NODE;
        right_data.id = RIGHT_MOTOR_NODE;
        left_data.command = cSetMoveStop;
        right_data.command = cSetMoveStop;
        left_data.data = 0.0f;
        right_data.data = 0.0f;


        /*5. set data to motor*/
        this->DataMutexLock();

        /*Send command motor*/
        this->motor_command_struct_.clear();

        this->motor_command_struct_.push_back(left_data);
        this->motor_command_struct_.push_back(right_data);

        this->DataMutexUnLock();

        
        this->target_pos_queue_.erase(this->target_pos_queue_.begin());

        printf("Arrived at target pos : %f, %f, %f is completed\r\n", target.x_pos, target.y_pos, target.angle);
      
        this->task_ = 0;
      }
        // else
      // {
      //   this->task_  = 1;

      // }
    // }
  }
  return;
}

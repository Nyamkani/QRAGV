/**
  ******************************************************************************
  * @file           : driving_controller.hpp
  * @brief          : Main Interface hpp file of QRAGV driving_controller 
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


#ifndef DRIVING_CONTROLLER_HPP_
#define DRIVING_CONTROLLER_HPP_

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#include <cmath>
#include <vector>
#include <chrono>
#include <string>
#include <iostream>
#include <mutex>
#include <thread>
#include <cmath>


#include <driving_controller/driving_controller_define.hpp>
#include <driving_controller/motor_controllers.hpp>
#include <pgv100/pgv100.hpp>
#include <isv2motor/isv2motor.hpp>
#include <isv2motor/isv2motor_define.hpp>



#define DRIVING_CONTROLLER_DEBUG_ON  1







class DrivingController
{
  private:
    /*Current position data*/
    int x_pos_;
    int y_pos_;
    int angle_;
    size_t pgv100_dir_;
    size_t pgv100_color_;
    size_t tag_code_;
    bool is_tagged_= false;
    bool is_detected_ = false;

    /*Current motor data*/
    int left_current_motor_speed_ = 0;
    int right_current_motor_speed_ = 0;

    int left_current_motor_pos_ = 0;
    int right_current_motor_pos_ = 0;

    bool is_motor_stopped_ = true;


    /*System status*/
    bool is_moving_ = false;


    size_t error_code_;


    /* Command queue*/
    std::vector<size_t> target_pos_queue_;
    size_t current_target_pos_;
    std::vector<Isv2MotorCommandStruct> motor_command_struct_;


    /*Buffer Data structure*/
    MotorDataStruct motor_data_buffer_;
    PosSensorDataStruct pos_sensor_data_buffer_;


    /*program stop signal*/
    bool quit_sig_ = false;

    /*driving buffers*/
    std::vector<Isv2MotorStruct> motor_infos_;

    /*threads*/
    std::vector<std::thread> module_workers_;

    /*mutex*/
    std::mutex controller_data_mutex_;

  public:
    bool max_acc_= 0;
    bool max_dec_ = 0;
    bool max_vel_ = 0;


  private:

  public:
    DrivingController();
    DrivingController(double max_acc, double max_dec, double max_vel);
    virtual ~DrivingController();
    
    kssbot_hardware::IReturnType Init();

    kssbot_hardware::IReturnType Drive();


    /*handler */
    bool GetQuitSignal();
    bool SetQuitSignal(bool sig);

    /*mutex*/
    void DataMutexLock();
    void DataMutexUnLock();

    /*Set data*/
    void TransactionPosData(PosSensorDataStruct data, bool& quit_sig);
    void TransactionMotorData(isv2motor& motor_interface, bool& quit_sig);
};

// int TestScurveProfileGenerator(
//                                     int target_pos, 
//                                     bool& moving_start, 
//                                     double initial_speed, 
//                                     clock_t start_time,
//                                     double& target_speed);

void PostionSensorWorker(const void* arg);
void DrivingMotorWorker(const void* arg);

#endif  //DRIVING_CONTROLLER_HPP_



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

#include <driving_controller/joypad.h>

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


#define TEST_CAL_VEL_DEFINE    0



struct TargetPosStruct
{
  double x_pos;
  double y_pos;
  double angle;
  size_t qr_code_;

};


class isv2motor;


class DrivingController
{
  private:
    /*Current position data*/

    size_t pgv100_dir_;
    size_t pgv100_color_;

    /*odometer*/
    int left_motor_s = 0;
    int right_motor_s = 0;

    int left_prev_motor_encoder_ = 0;
    int left_current_motor_encoder_ = 0;

    int right_prev_motor_encoder_ = 0;
    int right_current_motor_encoder_ = 0;

    bool is_motor_stopped_ = true;

    /*Scurve profiler*/
    bool is_moving_ = false;
    ScurveProfiler* now_follow_left_profiler_ = nullptr;
    ScurveProfiler* now_follow_right_profiler_ = nullptr;

    size_t error_code_;

    /*test*/
    size_t first_left_encoder_val = 0;
    size_t first_right_encoder_val = 0;
    // size_t last_encoder_val = 0;


    /* Command queue*/
    double current_target_pos_;


    /*Buffer Data structure*/
    /*driving buffers*/
    std::vector<Isv2MotorStruct*> motor_infos_;
    PosSensorDataStruct pos_sensor_data_buffer_;


    /*program stop signal*/
    bool quit_sig_ = false;

    /*data sync signal*/
    bool motor_data_renew_ = false;
    bool pos_data_renew_ = false;

    /*threads*/
    std::vector<std::thread> module_workers_;

    /*mutex*/
    std::mutex controller_data_mutex_;



    bool test_sig = false;

  public:
    int left_current_motor_speed_encoder = 0;
    int right_current_motor_speed_encoder = 0;
    int left_current_motor_speed_ = 0;
    int right_current_motor_speed_ = 0;

    /*tape*/
    bool is_detected_ = false;
    double x_pos_;    //fwd
    double y_pos_;    //str

    /*QR*/
    bool is_tagged_= false;
    double x_qrpos_;
    double y_qrpos_;
    size_t tag_code_;
    
    double angle_offset_ = 89;

    /*vel target follower task*/
    int task_ = 0;

    double angle_ ;   //theta
    double angle_degree_;

    double max_acc_= 0;
    double max_dec_ = 0;
    double max_vel_ = 0;

    double current_left_pos_;
    double current_right_pos_;
    double current_pos_;

    // double left_moved_dist_ = 0.0f;
    // double right_moved_dist_ = 0.0f;

    /*Current motor data*/
    double wheel_radius_ = 0.0f; //40 mm
    double wheel_between_length_ = 0.0f; //400mm
    double point_between_aux_ = 0.0f; //200mm
    double encoder_ticks_ = 0.0f;   //10000 ticks per rev
    double reducer_rate_ =  0.0f;   //2

    isv2motor* left_motor_ = nullptr;
    isv2motor* right_motor_ = nullptr;


    std::vector<Isv2MotorCommandStruct> motor_command_struct_;
    std::vector<TargetPosStruct> target_pos_queue_;
  private:

  public:
    DrivingController();
    DrivingController(double max_acc, double max_dec, double max_vel);
    virtual ~DrivingController();
    
    kssbot_hardware::IReturnType Init();

    kssbot_hardware::IReturnType Drive();
    kssbot_hardware::IReturnType Drive2();  //v2
    kssbot_hardware::IReturnType Drive3();  //v3


    /*handler */
    bool GetQuitSignal();
    bool SetQuitSignal(bool sig);

    /*mutex*/
    void DataMutexLock();
    void DataMutexUnLock();

    /*Set data*/
    void TransactionPosData(PosSensorDataStruct data, bool& quit_sig);
    void TransactionMotorData(isv2motor& motor_interface, bool& quit_sig);



/*v2 - AMR with line tape*/
/*------------------------------------------------------------------odometer*/

  void TransactionDataFromMoudules();
  void CalculateOdometry();
  void CalculateTargetVelocity(double target_pos_x, double target_pos_y, double angle);
  void ControlVelocity();

  void CalculateDirAngle(double target_pos_x, double target_pos_y);
  void CalculateTargetVel(double target_pos_x, double target_pos_y);
  void CalculateTargetPose(double target_angle);




/*v3 - SCURVE WITH DATA MATRIX TYPE*/
/*------------------------------------------------------------------QR*/
void TestQRMoveDrive();
void TestQRDirAngle(double dest_pose);
void TestQRMoveOnTag(double target_angle);
void TestQRControlVelocity();

void TestQRCalculateTargetPose(double target_angle);
void TestQRCalculateTargetVel(double target_pos_x, double target_pos_y);

void TestQRTagCalculateDirAngle();
void TestQRTagMoveToOrigin(double target_pos_x, double target_pos_y);

/*------------------------------------------------------------------Joypad*/
  int JoypadInitialize();


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



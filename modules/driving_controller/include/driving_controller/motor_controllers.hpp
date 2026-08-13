/**
  ******************************************************************************
  * @file           : motor_controllers.hpp
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

#ifndef MOTOR_CONTROLLERS_HPP
#define MOTOR_CONTROLLERS_HPP

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


#define MAX_MOVING_ITR        0
#define MAX_ALLOW_ERROR       0.5f


enum ProfilerStatus
{
  sInit = 0,
  sRun = 1,
  sEnd = 2,
  sFinish = 3,

  sFail = 10,
  sError = 100,
};


struct testcurvestruct
{
  double phase_time[7];

};

struct ScurveCalculatedParams
{
  double acc;
  double vel;
  double pos;
};

class ScurveProfiler
{
  private:
    int status_ = ProfilerStatus::sInit;
    int moving_itr = 0;

    int phase_ = 0;
    bool start_sig = false;

    bool check_test= false;

    double jerk_;
    double max_acc_;
    double max_dec_;
    double max_speed_;


    /*Common values*/
    double origin_pos_;
    double target_pos_;
    double present_pos_;
    double moving_distance_;
    int moving_dir_;
    double current_speed_;
  

    /*five s curve*/
    double tacc_;
    double tdec_;
    double tvel_;
    double ja_;
    double jd_;

    double vs_;
    double ve_;

    // double tja = 0;
    // double tjd = 0;

    double tv_ = 0.0f;


    double p1_ = 0.0f;
    double p2_ = 0.0f;
    double p3_ = 0.0f;
    double p4_ = 0.0f;
    double p5_ = 0.0f;









    /*output*/
    double desired_speed_ = 0.0f;
    double desired_pos_ = 0.0f;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point delay_time_;
    double time_offset_ = 0.0f;

    double phase_time_[7];
    ScurveCalculatedParams phase_time_params_[7];

  

    int stop_cnt_ = 0;

  public:

  private:
    int ScurveProfileGenerator();
    int ScurveProfileWorker();


  public:
    ScurveProfiler();
    ScurveProfiler(double jerk, double max_acc, double max_speed);
    ~ScurveProfiler();


    void SetPresentSpeed(double speed);
    void SetOriginPosition(double origin_pos);
    void SetTargetPosition(double target_pos);
    void SetPresentPosition(double present_pos);
    void SetPhase(int phase);

    int Drive();


    double GetCalculatedVel();
    int GetStatusProfiler();



};



int TestScurveProfileGenerator( int present_pos,
                                int target_pos, 
                                double& target_speed);

int TestScurveProfileWorker(testcurvestruct tests,
                                    double initial_speed, 
                                    std::chrono::system_clock::time_point start_time,
                                    double& target_speed,
                                    double& target_pos);



/*---------------------------------------------------------------------------------------------------------------*/





#endif  // MOTOR_CONTROLLERS_HPP




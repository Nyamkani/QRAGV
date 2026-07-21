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
    double jerk_;
    double max_acc_;
    double max_speed_;

    int target_pos_;
    int present_pos_;
    int moving_distance_;
    int moving_dir_;

    std::chrono::steady_clock::time_point start_time_;

    double phase_time_[7];
    ScurveCalculatedParams phase_time_params_[7];

  public:

  private:

  public:
    ScurveProfiler();
    ScurveProfiler(double jerk, double max_acc, double max_speed);
    ~ScurveProfiler();

    void SetTargetPostion(int target_pos);
    void SetPresentPostion(int present_pos);

    int ScurveProfileGenerator();
    int ScurveProfileWorker();

};



int TestScurveProfileGenerator( int present_pos,
                                int target_pos, 
                                double& target_speed);

int TestScurveProfileWorker(testcurvestruct tests,
                                    double initial_speed, 
                                    std::chrono::system_clock::time_point start_time,
                                    double& target_speed,
                                    double& target_pos);



#endif  // MOTOR_CONTROLLERS_HPP

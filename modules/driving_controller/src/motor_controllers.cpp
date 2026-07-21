
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


#include <driving_controller/motor_controllers.hpp>


ScurveProfiler::ScurveProfiler(){}

ScurveProfiler::ScurveProfiler(double jerk, double max_acc, double max_speed)
{
  this->jerk_ = jerk;
  this->max_acc_ = max_acc;
  this->max_speed_ = max_speed;
}

ScurveProfiler::~ScurveProfiler(){}


void ScurveProfiler::SetTargetPostion(int target_pos)
{
  this->target_pos_ = target_pos;

  return;
}

void ScurveProfiler::SetPresentPostion(int present_pos)
{
  this->present_pos_ = present_pos;

  return;
}


/**
 * @brief S-Curve Profile Generator 
 * Ref. by https://www.researchgate.net/publication/348383330_Mathematics_for_Real-Time_S-Curve_Profile_Generator
 * 
 */

int ScurveProfiler::ScurveProfileGenerator()
{
 /*differ position*/
  this->moving_distance_ = this->target_pos_ - this->present_pos_;
  this->moving_dir_ = 1;

  /*Get moving direction*/
  if( this->moving_distance_ < 0)
    this->moving_dir_ = -1;

  /*Make distance value positive */
   this->moving_distance_ *= this->moving_dir_;

  /*s-curve status values(speed, space limitation values) Equation 46 ,48 ,54 or 61*/
  double va = (pow(this->max_acc_, 2)/this->jerk_);
  double sa = ((2*pow(this->max_acc_, 3))/pow(this->jerk_, 2));
  double sv = 0.0f;

  if((this->max_speed_ * this->jerk_) < pow(this->max_acc_, 2))
    sv = 2* this->max_speed_ * sqrt(this->max_speed_/this->jerk_);
  else 
    sv = this->max_speed_ *((this->max_speed_/this->max_acc_) + (this->max_acc_/this->jerk_));

  /*Specific time*/
  double tj = 0.0f;
  double ta = 0.0f;
  double tv = 0.0f;

  /*Possible movement profiles*/  
  int profile_type = 0;

  if(this->max_speed_ < va)   //the accellation curve is triangular
  {
    if(this->moving_distance_ > sa)
    {
      profile_type = 1;  //positive and negative parts of the accellation are not continuous - (type A), C-1
    }
    else
    {
      if(this->moving_distance_ > sv)
        profile_type = 1; //positive and negative parts of the accellation are not continuous - type A, (C-1)
      else
        profile_type = 2;  //positive and negative parts of the accellation are continuous - Type B, (C-2)
    }
  }
  else
  {
    if(this->moving_distance_ < sa)
    {
      profile_type = 2;  //positive and negative parts of the accellation are continuous - (Type B), C-2
    }
    else
    {
      if(this->moving_distance_ > sv)
        profile_type = 3; //positive and negative parts of the accellation are not continuous - D-1
      else
        profile_type = 4;  //positive and negative parts of the accellation are continuous - D-2
      
    }
  }

  /*Calculate specific time*/
  switch (profile_type)
  {
    case 1 :
    {
      //72,75,76
      tj = sqrt(this->max_speed_/this->jerk_);

      ta = tj;

      tv = this->moving_distance_/this->max_speed_;

      break;
    }

    case 2 :
    {
      //66,67,68
      tj = cbrt((this->moving_distance_/(2*this->jerk_)));

      ta = tj;

      tv = 2*tj;

      break;
    }

    case 3 :
    {
      //77,78,79
      tj = this->max_acc_/this->jerk_;

      ta = this->max_speed_/this->max_acc_;

      tv = this->moving_distance_/this->max_speed_;

      break;
    }

    case 4 :
    {
      //80.88.89
      tj = this->max_acc_/this->max_acc_;

      ta = (1/2) * ((sqrt((4*this->moving_distance_*pow(this->jerk_, 2)+ pow(this->max_acc_, 3))/(this->max_acc_*pow(this->jerk_, 2)))) - (this->max_acc_/this->jerk_));

      tv = ta + tj;

      break;
    }

    default : printf("Error : Invaile profile type\r\n"); break;
  }

  // printf("va : %f, sa : %f, sv : %f\r\n", va, sa, sv);
  // printf("profile type is : %d, tj : %f, ta : %f, tv : %f\r\n", profile_type, tj, ta, tv);


  /*Calcuate phase time equation 33~39*/
  this->phase_time_[0] = tj;
  this->phase_time_[1] = ta;
  this->phase_time_[2] = tj + ta;
  this->phase_time_[3] = tv;
  this->phase_time_[4] = tj + tv;
  this->phase_time_[5] = tv + ta;
  this->phase_time_[6] = tv + ta + tj;


  // printf("va = %f, sa = %f, sv = %f\r\n", va, sa, sv);
  // printf("Profile type : %d, tj = %f, ta = %f, tv = %f\r\n", profile_type, tj, ta, tv);
  // printf("start time = %ld, t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n",start_time,  t1,t2,t3,t4,t5,t6,t7);


  /*phase 1*/
  this->phase_time_params_[0].acc = this->jerk_ * this->phase_time_[0];
  this->phase_time_params_[0].vel = (this->jerk_ * pow(this->phase_time_[0], 2) / 2);
  this->phase_time_params_[0].pos = this->jerk_ * pow(this->phase_time_[0], 3) / 6;

  /*phase 2*/
  this->phase_time_params_[1].acc = this->phase_time_params_[0].acc;
  this->phase_time_params_[1].vel = this->phase_time_params_[0].vel + this->phase_time_params_[0].acc *(this->phase_time_[1] - this->phase_time_[0]);
  this->phase_time_params_[1].pos = this->phase_time_params_[0].pos + (this->phase_time_params_[0].vel *(this->phase_time_[1] - this->phase_time_[0]))
                                    + this->phase_time_params_[0].acc *(pow(this->phase_time_[1] - this->phase_time_[0], 2)/2);

  /*phase 3*/
  this->phase_time_params_[2].acc = 0.0f;
  this->phase_time_params_[2].vel = this->phase_time_params_[1].vel + this->phase_time_params_[1].acc *(this->phase_time_[2] - this->phase_time_[1])
                                    - this->jerk_*((pow(this->phase_time_[2] - this->phase_time_[1] , 2)) /2);
  this->phase_time_params_[2].pos = this->phase_time_params_[1].pos + this->phase_time_params_[1].vel *(this->phase_time_[2] - this->phase_time_[1])
                                    + this->phase_time_params_[1].acc *((pow((this->phase_time_[2] - this->phase_time_[1]), 2))/2)
                                    - this->jerk_*(pow((this->phase_time_[2] - this->phase_time_[1]), 3)/6);

  /*phase 4*/
  this->phase_time_params_[3].acc = this->phase_time_params_[2].acc;
  this->phase_time_params_[3].vel = this->phase_time_params_[2].vel;
  this->phase_time_params_[3].pos = this->phase_time_params_[2].pos + this->phase_time_params_[2].vel *(this->phase_time_[3] - this->phase_time_[2]); 

  /*phase 5*/
  this->phase_time_params_[4].acc = -1 * this->phase_time_params_[0].acc;
  this->phase_time_params_[4].vel = this->phase_time_params_[3].vel - this->jerk_ *((pow(this->phase_time_[4] - this->phase_time_[3] , 2)) /2);
  this->phase_time_params_[4].pos = this->phase_time_params_[3].pos + this->phase_time_params_[3].vel *(this->phase_time_[4] - this->phase_time_[3])
                                    - this->jerk_ *(pow((this->phase_time_[4] - this->phase_time_[3]), 3)/6);

  /*phase 6*/
  this->phase_time_params_[5].acc = this->phase_time_params_[4].acc;
  this->phase_time_params_[5].vel = this->phase_time_params_[4].vel - this->max_acc_ *(this->phase_time_[5] - this->phase_time_[4]);
  this->phase_time_params_[5].pos = this->phase_time_params_[4].pos + this->phase_time_params_[4].vel *(this->phase_time_[5] - this->phase_time_[4])
                                    + this->phase_time_params_[4].acc *(pow((this->phase_time_[5] - this->phase_time_[4]), 2)/2);

  /*Phase 7*/
  // p.7 is only calculated immediatly

  this->start_time_ = std::chrono::steady_clock::now();

  return 0;
}


int ScurveProfiler::ScurveProfileWorker()
{
  /*output*/
  double cal_acc = 0.0f;
  double cal_speed = 0.0f;
  double cal_pos = 0.0f;

  /*Calcuate phase time equation 33~39*/
  //double now_time = ((double)clock()/1000) - start_time_;
  auto duration_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - this->start_time_);

  double past_time = duration_time.count();


  double t1 = this->phase_time_[0];
  double t2 = this->phase_time_[1];
  double t3 = this->phase_time_[2];
  double t4 = this->phase_time_[3];
  double t5 = this->phase_time_[4];
  double t6 = this->phase_time_[5];
  double t7 = this->phase_time_[6];

  
  // printf("va = %f, sa = %f, sv = %f\r\n", va, sa, sv);
  // printf("Profile type : %d, tj = %f, ta = %f, tv = %f\r\n", profile_type, tj, ta, tv);
  // printf("start time = %ld, t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n",start_time,  t1,t2,t3,t4,t5,t6,t7);

  /*Phase 1.*/
  if(past_time <= t1)
  {
      cal_speed = this->jerk_ * pow(past_time, 2) / 2;
      cal_pos = this->jerk_ * pow(past_time, 3) / 6;
  }
  /*Phase 2*/
  else if(past_time <= t2)
  {   
    double a1 = jerk * t1;
    double v1 = (jerk * pow(t1, 2) / 2);
    double p1 = jerk * pow(t1, 3) / 6;

    cal_speed = this->phase_time_params_[0].vel + this->phase_time_params_[0].acc*(past_time - t1);
    cal_pos = p1 + v1*(past_time - t1) + a1*(pow((past_time - t1), 2)/2);
  }
  /*Phase 3*/
  else if (now_time <= t3)
  {
      phase = 3;

      // jerk *= -1;

      double a1 = jerk * t1;
      double v1 = (jerk * pow(t1, 2) / 2);
      double p1 = jerk * pow(t1, 3) / 6;
      double v2 = v1 + a1 *(t2 - t1);
      double a2 = a1;
      double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);

      cal_speed = v2 + a2*(now_time - t2) - jerk* ((pow((now_time - t2) , 2)) /2);
      cal_pos = p2 + v2*(now_time - t2) + a2*((pow((now_time - t2), 2))/2) - jerk*(pow((now_time - t2), 3)/6);
  }
  /*Phase 4*/
  else if (now_time <= t4)
  {
      phase = 4;

      // jerk = 0;

      double a1 = jerk * t1;
      double v1 = (jerk * pow(t1, 2) / 2);
      double p1 = jerk * pow(t1, 3) / 6;
      double a2 = a1;
      double v2 = v1 + a1 *(t2 - t1);
      double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);
      double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 2) /2));
      double p3 = p2 + v2*(t3 - t2) + a2*((pow((t3 - t2), 2))/2) - jerk*(pow((t3 - t2), 3)/6);

      cal_speed = v3;
      cal_pos = p3 + v3*(now_time - t3);
  }
  /*Phase 5*/
  else if(now_time <= t5)
  {
      phase = 5;

      // jerk *= -1;

      double a1 = jerk * t1;
      double v1 = (jerk * pow(t1, 2) / 2);
      double p1 = jerk * pow(t1, 3) / 6;
      double a2 = a1;
      double v2 = v1 + a1 *(t2 - t1);
      double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);
      double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 2)) /2);
      double p3 = p2 + v2*(t3 - t2) + a2*((pow((t3 - t2), 2))/2) - jerk*(pow((t3 - t2), 3)/6);
      double v4 = v3;
      double p4 = p3 + v3*(t4 - t3);

      cal_speed = v4 - jerk *((pow(now_time - t4 , 2)) /2);
      cal_pos = p4 + v4 *(now_time - t4)-jerk*(pow((now_time - t4), 3)/6);
  }
  /*Phase 6*/
  else if (now_time <= t6)
  {
      phase = 6;

      // jerk = 0;

      double a1 = jerk * t1;
      double v1 = (jerk * pow(t1, 2) / 2);
      double p1 = jerk * pow(t1, 3) / 6;
      double a2 = a1;
      double v2 = v1 + a1 *(t2 - t1);
      double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);
      double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 2)) /2);
      double p3 = p2 + v2*(t3 - t2) + a2*((pow((t3 - t2), 2))/2) - jerk*(pow((t3 - t2), 3)/6);
      double v4 = v3;
      double p4 = p3 + v3*(t4 - t3);
      double a5 = -1 * a1;
      double v5 = v4 - jerk *((pow(t5 - t4 , 2)) /2);
      double p5 = p4 + v4 *(t5 - t4)-jerk*(pow((t5 - t4), 3)/6);

      cal_speed = v5 - max_acc*(now_time - t5);
      cal_pos = p5 + v5 * (now_time - t5) + a5*(pow((now_time - t5), 2)/2);

  }
  /*Phase 7**/
  else if(now_time <= t7)
  {
      phase = 7;

      double a1 = jerk * t1;
      double v1 = (jerk * pow(t1, 2) / 2);
      double p1 = jerk * pow(t1, 3) / 6;
      double a2 = a1;
      double v2 = v1 + a1 *(t2 - t1);
      double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);
      double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 2)) /2);
      double p3 = p2 + v2*(t3 - t2) + a2*((pow((t3 - t2), 2))/2) - jerk*(pow((t3 - t2), 3)/6);
      double v4 = v3;
      double p4 = p3 + v3*(t4 - t3);
      double a5 = -1 * a1;
      double v5 = v4 - jerk *((pow(t5 - t4 , 2)) /2);
      double p5 = p4 + v4 *(t5 - t4)-jerk*(pow((t5 - t4), 3)/6);
      double a6 = -1* a1;
      double v6 = v5 - max_acc*(t6 - t5);
      double p6 = p5 + v5 * (t6 - t5) + a5*(pow((t6 - t5), 2)/2);

      cal_speed = v6 + a6*(now_time - t6)+ jerk *(pow((now_time - t6), 2)/2);
      cal_pos = p6 + v6*(now_time - t6) + a6*(pow((now_time - t6), 2)/2) + jerk*(pow((now_time - t6), 3)/6);
  }
  else
  {
      cal_speed = 0;

  // printf("t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n", t1,t2,t3,t4,t5,t6,t7);


  }

  if (cal_speed > max_speed)
      cal_speed = max_speed;

  target_speed = cal_speed;
  target_pos = cal_pos;



  //printf("start time : %f, Now time : %f, Phase : %d, Speed : %f\r\n",start_time_, now_time, phase, target_speed);
  //  printf("Now time : %f, Phase : %d, Speed : %f\r\n", now_time, phase, target_speed);

  return 0;
}







/*given data*/
static double sjerk = 0.0f;
static double smax_acc = 0.0f;
static double smax_speed = 0.0f;

void TestScurveProfileParams(double jerk, double max_acc, double max_speed)
{
  sjerk = jerk;
  smax_acc = max_acc;
  smax_speed = max_speed;

  return;
}





/**
 * @brief S-Curve Profile Generator 
 * Ref. by https://www.researchgate.net/publication/348383330_Mathematics_for_Real-Time_S-Curve_Profile_Generator
 * 
 */

int TestScurveProfileGenerator( int present_pos,
                                int target_pos, 
                                double& target_speed)
{
  /*differ position*/
  int moving_distance = target_pos - present_pos;
  int moving_dir = 1;

  /*Get moving direction*/
  if(moving_distance < 0)
    moving_dir = -1;

  /*Make distance postion */
  moving_distance *= moving_dir;

  /*s-curve status values(speed, space limitation values) 46 ,48 ,54 or 61*/
  double va = (pow(smax_acc, 2)/sjerk);
  double sa = ((2*pow(smax_acc, 3))/pow(sjerk, 2));
  double sv = 0.0f;

  if((smax_speed * sjerk) < pow(smax_acc, 2))
    sv = 2* smax_speed * sqrt(smax_speed/sjerk);
  else 
    sv = smax_speed *((smax_speed/smax_acc) + (smax_acc/sjerk));

  /*Specific time*/
  double tj = 0.0f;
  double ta = 0.0f;
  double tv = 0.0f;

  /*Possible movement profiles*/  
  int profile_type = 0;

  if(smax_speed < va)   //the accellation curve is triangular
  {
    if(moving_distance > sa)
    {
      profile_type = 1;  //positive and negative parts of the accellation are not continuous - (type A), C-1
    }
    else
    {
      if(moving_distance > sv)
        profile_type = 1; //positive and negative parts of the accellation are not continuous - type A, (C-1)
      else
        profile_type = 2;  //positive and negative parts of the accellation are continuous - Type B, (C-2)
    }
  }
  else
  {
    if(moving_distance < sa)
    {
      profile_type = 2;  //positive and negative parts of the accellation are continuous - (Type B), C-2
    }
    else
    {
      if(moving_distance > sv)
        profile_type = 3; //positive and negative parts of the accellation are not continuous - D-1
      else
        profile_type = 4;  //positive and negative parts of the accellation are continuous - D-2
      
    }
  }

  /*Calculate specific time*/
  switch (profile_type)
  {
    case 1 :
    {
      //72,75,76
      tj = sqrt(smax_speed/sjerk);

      ta = tj;

      tv = moving_distance/smax_speed;

      break;
    }

    case 2 :
    {
      //66,67,68
      tj = cbrt((moving_distance/(2*sjerk)));

      ta = tj;

      tv = 2*tj;

      break;
    }

    case 3 :
    {
      //77,78,79
      tj = smax_acc/sjerk;

      ta = smax_speed/smax_acc;

      tv = moving_distance/smax_speed;

      break;
    }

    case 4 :
    {
      //80.88.89
      tj = smax_acc/sjerk;

      ta = (1/2) * ((sqrt((4*moving_distance*pow(sjerk, 2)+ pow(smax_acc, 3))/(smax_acc*pow(sjerk, 2)))) - (smax_acc/sjerk));

      tv = ta + tj;

      break;
    }

    default : printf("Error : Invaile profile type\r\n"); break;
  }

  printf("va : %f, sa : %f, sv : %f\r\n", va, sa, sv);
  printf("profile type is : %d, tj : %f, ta : %f, tv : %f\r\n", profile_type, tj, ta, tv);




    /*Calcuate phase time 33~39*/
    double now_time = (double)clock()/CLOCKS_PER_SEC;

    double t1 = tj;
    double t2 = ta;
    double t3 = tj + ta;
    double t4 = tv;
    double t5 = tj + tv;
    double t6 = tv + ta;
    double t7 = tv + ta + tj;

    
    // printf("va = %f, sa = %f, sv = %f\r\n", va, sa, sv);
    // printf("Profile type : %d, tj = %f, ta = %f, tv = %f\r\n", profile_type, tj, ta, tv);
    // printf("start time = %ld, t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n",start_time,  t1,t2,t3,t4,t5,t6,t7);


    return 0;
}


int TestScurveProfileWorker(testcurvestruct tests,
                                    double initial_speed, 
                                    std::chrono::system_clock::time_point start_time,
                                    double& target_speed,
                                    double& target_pos)
{

  /*given data*/
  double jerk = 500.0f;
  double max_acc = 1000.0f;
  double max_speed = 2000.0f;

  /*output*/
  double cal_acc = 10.0f;
  double cal_speed = 10.0f;
  double cal_pos = 5.0f;


    /*Calcuate phase time 33~39*/
    // clock_t now_time = clock() - start_time;
    //double now_time = ((double)clock()/1000) - start_time_;
    std::chrono::duration<double> now_time_ = std::chrono::system_clock::now() - start_time;

    double now_time = now_time_.count();


    double t1 = tests.phase_time[0];
    double t2 = tests.phase_time[1];
    double t3 = tests.phase_time[2];
    double t4 = tests.phase_time[3];
    double t5 = tests.phase_time[4];
    double t6 = tests.phase_time[5];
    double t7 = tests.phase_time[6];

    
    // printf("va = %f, sa = %f, sv = %f\r\n", va, sa, sv);
    // printf("Profile type : %d, tj = %f, ta = %f, tv = %f\r\n", profile_type, tj, ta, tv);
    // printf("start time = %ld, t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n",start_time,  t1,t2,t3,t4,t5,t6,t7);

    int phase = 0;

    /*Phase 1.*/
    if(now_time <= t1)
    {
        phase = 1;

        cal_speed = jerk * pow(now_time, 2) / 2;
        cal_pos = jerk * pow(now_time, 3) / 6;
    }
    /*Phase 2*/
    else if(now_time <= t2)
    {   
        phase = 2;

        // jerk = 0;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double p1 = jerk * pow(t1, 3) / 6;

        cal_speed = v1 + a1*(now_time - t1);
        cal_pos = p1 + v1*(now_time - t1) + a1*(pow((now_time - t1), 2)/2);
    }
    /*Phase 3*/
    else if (now_time <= t3)
    {
        phase = 3;

        // jerk *= -1;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double p1 = jerk * pow(t1, 3) / 6;
        double v2 = v1 + a1 *(t2 - t1);
        double a2 = a1;
        double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);

        cal_speed = v2 + a2*(now_time - t2) - jerk* ((pow((now_time - t2) , 2)) /2);
        cal_pos = p2 + v2*(now_time - t2) + a2*((pow((now_time - t2), 2))/2) - jerk*(pow((now_time - t2), 3)/6);
    }
    /*Phase 4*/
    else if (now_time <= t4)
    {
        phase = 4;

        // jerk = 0;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double p1 = jerk * pow(t1, 3) / 6;
        double a2 = a1;
        double v2 = v1 + a1 *(t2 - t1);
        double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);
        double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 2) /2));
        double p3 = p2 + v2*(t3 - t2) + a2*((pow((t3 - t2), 2))/2) - jerk*(pow((t3 - t2), 3)/6);

        cal_speed = v3;
        cal_pos = p3 + v3*(now_time - t3);
    }
    /*Phase 5*/
    else if(now_time <= t5)
    {
        phase = 5;

        // jerk *= -1;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double p1 = jerk * pow(t1, 3) / 6;
        double a2 = a1;
        double v2 = v1 + a1 *(t2 - t1);
        double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);
        double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 2)) /2);
        double p3 = p2 + v2*(t3 - t2) + a2*((pow((t3 - t2), 2))/2) - jerk*(pow((t3 - t2), 3)/6);
        double v4 = v3;
        double p4 = p3 + v3*(t4 - t3);

        cal_speed = v4 - jerk *((pow(now_time - t4 , 2)) /2);
        cal_pos = p4 + v4 *(now_time - t4)-jerk*(pow((now_time - t4), 3)/6);
    }
    /*Phase 6*/
    else if (now_time <= t6)
    {
        phase = 6;

        // jerk = 0;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double p1 = jerk * pow(t1, 3) / 6;
        double a2 = a1;
        double v2 = v1 + a1 *(t2 - t1);
        double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);
        double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 2)) /2);
        double p3 = p2 + v2*(t3 - t2) + a2*((pow((t3 - t2), 2))/2) - jerk*(pow((t3 - t2), 3)/6);
        double v4 = v3;
        double p4 = p3 + v3*(t4 - t3);
        double a5 = -1 * a1;
        double v5 = v4 - jerk *((pow(t5 - t4 , 2)) /2);
        double p5 = p4 + v4 *(t5 - t4)-jerk*(pow((t5 - t4), 3)/6);

        cal_speed = v5 - max_acc*(now_time - t5);
        cal_pos = p5 + v5 * (now_time - t5) + a5*(pow((now_time - t5), 2)/2);

    }
    /*Phase 7**/
    else if(now_time <= t7)
    {
        phase = 7;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double p1 = jerk * pow(t1, 3) / 6;
        double a2 = a1;
        double v2 = v1 + a1 *(t2 - t1);
        double p2 = p1 + v1*(t2 - t1) + a1*(pow(t2 - t1, 2)/2);
        double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 2)) /2);
        double p3 = p2 + v2*(t3 - t2) + a2*((pow((t3 - t2), 2))/2) - jerk*(pow((t3 - t2), 3)/6);
        double v4 = v3;
        double p4 = p3 + v3*(t4 - t3);
        double a5 = -1 * a1;
        double v5 = v4 - jerk *((pow(t5 - t4 , 2)) /2);
        double p5 = p4 + v4 *(t5 - t4)-jerk*(pow((t5 - t4), 3)/6);
        double a6 = -1* a1;
        double v6 = v5 - max_acc*(t6 - t5);
        double p6 = p5 + v5 * (t6 - t5) + a5*(pow((t6 - t5), 2)/2);

        cal_speed = v6 + a6*(now_time - t6)+ jerk *(pow((now_time - t6), 2)/2);
        cal_pos = p6 + v6*(now_time - t6) + a6*(pow((now_time - t6), 2)/2) + jerk*(pow((now_time - t6), 3)/6);
    }
    else
    {
        cal_speed = 0;

    // printf("t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n", t1,t2,t3,t4,t5,t6,t7);


    }

    if (cal_speed > max_speed)
        cal_speed = max_speed;

    target_speed = cal_speed;
    target_pos = cal_pos;



  //printf("start time : %f, Now time : %f, Phase : %d, Speed : %f\r\n",start_time_, now_time, phase, target_speed);
//  printf("Now time : %f, Phase : %d, Speed : %f\r\n", now_time, phase, target_speed);

    return 0;
}

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


void ScurveProfiler::SetPresentSpeed(double speed)
{
  this->current_speed_ = speed;

  return;
}

void ScurveProfiler::SetTargetPosition(double target_pos)
{
  this->target_pos_ = target_pos;

  return;
}

void ScurveProfiler::SetPresentPosition(double present_pos)
{
  this->present_pos_ = present_pos;

  return;
}

void ScurveProfiler::SetOriginPosition(double origin_pos)
{
  this->origin_pos_ = origin_pos;

  return;
}







/**
 * @brief S-Curve Profile Generator 
 * Ref. by https://www.researchgate.net/publication/348383330_Mathematics_for_Real-Time_S-Curve_Profile_Generator
 * 
 */

// int ScurveProfiler::ScurveProfileGenerator()
// {
//   /*differ position*/
//   double target_pos = 0.0f;

//   /*Get moving direction*/
//   this->moving_dir_ = 1;

//   if(this->target_pos_ - this->present_pos_ <= 0)
//     this->moving_dir_ = -1;

//   target_pos = this->target_pos_;

//   /*Make distance value positive */
//   this->moving_distance_ = this->moving_dir_*(target_pos - this->present_pos_);



//   printf("Target_pos = %lf, presetn_pos = %lf\r\n", this->target_pos_,this->present_pos_);
//   printf("moving dist = %lf, dir = %d\r\n", this->moving_distance_,this->moving_dir_);

//   /*Make distance value positive */
//   //  this->moving_distance_ *= this->moving_dir_;

//     // this->moving_distance_ -=20;   //stop offset

//   /*s-curve status values(speed, space limitation values) Equation 46 ,48 ,54 or 61*/
//   double va = (pow(this->max_acc_, 2)/this->jerk_); //max speed of movement
//   // double va = ((this->max_speed_ - this->current_speed_)/this->jerk_);
//   double sa = ((2*pow(this->max_acc_, 3))/pow(this->jerk_, 2)); 
//   // double sa = ((2*pow(this->max_acc_, 3))/pow(this->jerk_, 2)); // 
//   double sv = 0.0f;

//   if((this->max_speed_ * this->jerk_) < pow(this->max_acc_, 2))
//     sv = 2* this->max_speed_ * sqrt(this->max_speed_/this->jerk_);
//   else 
//     sv = this->max_speed_ *((this->max_speed_/this->max_acc_) + (this->max_acc_/this->jerk_));

//   /*Specific time*/
//   double tj = 0.0f;
//   double ta = 0.0f;
//   double tv = 0.0f;

//   /*Possible movement profiles*/  
//   int profile_type = 0;

//   if(this->max_speed_ < va)   //the accellation curve is triangular
//   {
//     if(this->moving_distance_ > sa)
//     {
//       profile_type = 1;  //positive and negative parts of the accellation are not continuous - (type A), C-1
//     }
//     else
//     {
//       if(this->moving_distance_ > sv)
//         profile_type = 1; //positive and negative parts of the accellation are not continuous - type A, (C-1)
//       else
//         profile_type = 2;  //positive and negative parts of the accellation are continuous - Type B, (C-2)
//     }
//   }
//   else
//   {
//     if(this->moving_distance_ < sa)
//     {
//       profile_type = 2;  //positive and negative parts of the accellation are continuous - (Type B), C-2
//     }
//     else
//     {
//       if(this->moving_distance_ > sv)
//         profile_type = 3; //positive and negative parts of the accellation are not continuous - D-1
//       else
//         profile_type = 4;  //positive and negative parts of the accellation are continuous - D-2
      
//     }
//   }

//   /*Calculate specific time*/
//   switch (profile_type)
//   {
//     case 1 :
//     {
//       //72,75,76
//       tj = sqrt(this->max_speed_/this->jerk_);

//       ta = tj;

//       tv = this->moving_distance_/this->max_speed_;

//       break;
//     }

//     case 2 :
//     {
//       //66,67,68
//       tj = cbrt((this->moving_distance_/(2*this->jerk_)));

//       ta = tj;

//       tv = 2*tj;

//       break;
//     }

//     case 3 :
//     {
//       //77,78,79
//       tj = this->max_acc_/this->jerk_;

//       ta = this->max_speed_/this->max_acc_;

//       tv = this->moving_distance_/this->max_speed_;

//       break;
//     }

//     case 4 :
//     {
//       //80.88.89
//       tj = this->max_acc_/this->jerk_;

//      //ta = (1/2) * ((sqrt((4*this->moving_distance_*pow(this->jerk_, 2)+ pow(this->max_acc_, 3))/(this->max_acc_*pow(this->jerk_, 2)))) - (this->max_acc_/this->jerk_));

//       double x1 = (sqrt((4*this->moving_distance_*pow(this->jerk_, 2)+ pow(this->max_acc_, 3))/(this->max_acc_*pow(this->jerk_, 2))));

//       ta = 0.5* (x1 - (this->max_acc_/this->jerk_));

//       tv = ta + tj;

//       break;
//     }

//     default : printf("Error : Invaile profile type\r\n"); break;
//   }

//   // printf("va : %f, sa : %f, sv : %f\r\n", va, sa, sv);
//   printf("profile type is : %d, tj : %f, ta : %f, tv : %f\r\n", profile_type, tj, ta, tv);


//   /*Calcuate phase time equation 33~39*/
//   this->phase_time_[0] = tj;
//   this->phase_time_[1] = ta;
//   this->phase_time_[2] = tj + ta;
//   this->phase_time_[3] = tv;
//   this->phase_time_[4] = tj + tv;
//   this->phase_time_[5] = tv + ta;
//   this->phase_time_[6] = tv + ta + tj;


//   // printf("va = %f, sa = %f, sv = %f\r\n", va, sa, sv);
//   // printf("Profile type : %d, tj = %f, ta = %f, tv = %f\r\n", profile_type, tj, ta, tv);
//   // printf("start time = %ld, t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n",start_time,  t1,t2,t3,t4,t5,t6,t7);


//   /*phase 1*/
//   this->phase_time_params_[0].acc = this->jerk_ * this->phase_time_[0];
//   this->phase_time_params_[0].vel = (this->jerk_ * pow(this->phase_time_[0], 2) / 2);
//   this->phase_time_params_[0].pos = (this->jerk_ * pow(this->phase_time_[0], 3) / 6);

//   /*phase 2*/
//   this->phase_time_params_[1].acc = this->phase_time_params_[0].acc;
//   this->phase_time_params_[1].vel = this->phase_time_params_[0].vel + this->phase_time_params_[0].acc *(this->phase_time_[1] - this->phase_time_[0]);
//   this->phase_time_params_[1].pos = this->phase_time_params_[0].pos + (this->phase_time_params_[0].vel *(this->phase_time_[1] - this->phase_time_[0]))
//                                     + this->phase_time_params_[0].acc *(pow(this->phase_time_[1] - this->phase_time_[0], 2)/2);

//   /*phase 3*/
//   this->phase_time_params_[2].acc = 0.0f;
//   this->phase_time_params_[2].vel = this->phase_time_params_[1].vel + this->phase_time_params_[1].acc *(this->phase_time_[2] - this->phase_time_[1])
//                                     - this->jerk_*((pow(this->phase_time_[2] - this->phase_time_[1] , 2)) /2);
//   this->phase_time_params_[2].pos = this->phase_time_params_[1].pos + this->phase_time_params_[1].vel *(this->phase_time_[2] - this->phase_time_[1])
//                                     + this->phase_time_params_[1].acc *((pow((this->phase_time_[2] - this->phase_time_[1]), 2))/2)
//                                     - this->jerk_*(pow((this->phase_time_[2] - this->phase_time_[1]), 3)/6);

//   /*phase 4*/
//   this->phase_time_params_[3].acc = this->phase_time_params_[2].acc;
//   this->phase_time_params_[3].vel = this->phase_time_params_[2].vel;
//   this->phase_time_params_[3].pos = this->phase_time_params_[2].pos + this->phase_time_params_[2].vel *(this->phase_time_[3] - this->phase_time_[2]); 

//   /*phase 5*/
//   this->phase_time_params_[4].acc = -1 * this->phase_time_params_[0].acc;
//   this->phase_time_params_[4].vel = this->phase_time_params_[3].vel - this->jerk_ *((pow(this->phase_time_[4] - this->phase_time_[3] , 2)) /2);
//   this->phase_time_params_[4].pos = this->phase_time_params_[3].pos + this->phase_time_params_[3].vel *(this->phase_time_[4] - this->phase_time_[3])
//                                     - this->jerk_ *(pow((this->phase_time_[4] - this->phase_time_[3]), 3)/6);

//   /*phase 6*/
//   this->phase_time_params_[5].acc = this->phase_time_params_[4].acc;
//   this->phase_time_params_[5].vel = this->phase_time_params_[4].vel - this->max_acc_ *(this->phase_time_[5] - this->phase_time_[4]);
//   this->phase_time_params_[5].pos = this->phase_time_params_[4].pos + this->phase_time_params_[4].vel *(this->phase_time_[5] - this->phase_time_[4])
//                                     + this->phase_time_params_[4].acc *(pow((this->phase_time_[5] - this->phase_time_[4]), 2)/2);

//   /*Phase 7*/
//   // p.7 is only calculated immediatly

//   this->phase_time_params_[6].acc = 0;
//   this->phase_time_params_[6].vel = 0;
//   this->phase_time_params_[6].pos = this->phase_time_params_[5].pos + this->phase_time_params_[5].vel*(this->phase_time_[6] - this->phase_time_[5])+this->phase_time_params_[5].acc*(pow((this->phase_time_[6] - this->phase_time_[5]),2))/2+this->jerk_*(pow((this->phase_time_[6] - this->phase_time_[5]),3)/6);


//   printf("p7 pos = %f\r\n", this->phase_time_params_[6].pos);

//   this->start_time_ = std::chrono::steady_clock::now();

//   return 0;
// }


// int ScurveProfiler::ScurveProfileWorker()
// {
//   /*output*/
//   // double cal_acc = 0.0f;
//   double cal_speed = 0.0f;
//   double cal_pos = 0.0f;

//   /*Calcuate phase time equation 33~39*/
//   //double now_time = ((double)clock()/1000) - start_time_;



//   double t1 = this->phase_time_[0];
//   double t2 = this->phase_time_[1];
//   double t3 = this->phase_time_[2];
//   double t4 = this->phase_time_[3];
//   double t5 = this->phase_time_[4];
//   double t6 = this->phase_time_[5];
//   double t7 = this->phase_time_[6];


//   // if(this->moving_itr == 0)
//   // {



//     // printf("va = %f, sa = %f, sv = %f\r\n", va, sa, sv);
//     // printf("Profile type : %d, tj = %f, ta = %f, tv = %f\r\n", profile_type, tj, ta, tv);
//     // printf("start time = %ld, t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n",start_time,  t1,t2,t3,t4,t5,t6,t7);




//     // if(past_time >= t7  || this->present_pos_ >= this->target_pos_)
//     // {
//     //   cal_pos = 0;

//     //   phase = 8;


//     // // printf("t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n", t1,t2,t3,t4,t5,t6,t7);
//     // }

//     // if(!(this->start_sig))
//     // {
//     //   this->start_time_ = std::chrono::steady_clock::now();

//     //   this->start_sig = true;
//     // }

//   // auto duration_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->start_time_);

//   std::chrono::duration<double> duration_time =  std::chrono::steady_clock::now() - this->start_time_;

//   double past_time = duration_time.count();

//     int phase = 0;

//     /*Phase 1.*/
//     if(past_time <= t1 )
//     {
//       phase = 1;

//       cal_speed = (this->jerk_ * pow(past_time, 2) / 2);
//       cal_pos = (this->jerk_ * pow(past_time, 3) / 6);
//     }
//     /*Phase 2*/
//     else if(past_time <= t2)
//     {   
//       phase = 2;

//       double a1 = this->phase_time_params_[0].acc;
//       double v1 = this->phase_time_params_[0].vel;
//       double p1 = this->phase_time_params_[0].pos;

//       cal_speed = v1 + a1 *(past_time - t1);
//       cal_pos = p1 + v1*(past_time - t1) + a1*(pow((past_time - t1), 2)/2);
//     }
//     /*Phase 3*/
//     else if (past_time <= t3 )
//     {
//       phase = 3;

//       double a2 = this->phase_time_params_[1].acc;
//       double v2 = this->phase_time_params_[1].vel;
//       double p2 = this->phase_time_params_[1].pos;

//       cal_speed = v2 + a2 *(past_time - t2) - this->jerk_* ((pow((past_time - t2) , 2)) /2);
//       cal_pos = p2 + v2 *(past_time - t2) + a2 *((pow((past_time - t2), 2))/2) - this->jerk_ *(pow((past_time - t2), 3)/6);
//     }
//     /*Phase 4*/
//     else if (past_time <= t4 )
//     {
//       phase = 4;

//       // double a3 = this->phase_time_params_[2].acc;
//       double v3 = this->phase_time_params_[2].vel;
//       double p3 = this->phase_time_params_[2].pos;

//       cal_speed = v3;
//       cal_pos = p3 + v3 *(past_time - t3);
//     }
//     /*Phase 5*/
//     else if(past_time <= t5 )
//     {
//       phase = 5;

//       // double a4 = this->phase_time_params_[3].acc;
//       double v4 = this->phase_time_params_[3].vel;
//       double p4 = this->phase_time_params_[3].pos;

//       // if(!(check_test))
//       // {
//       //   if(abs((abs(this->present_pos_) - cal_pos))/(abs(this->present_pos_))*100 >= 10)
//       //   {
//       //     this->status_ = ProfilerStatus::sInit;

//       //     check_test = true;
//       //   }
//       // }

//       cal_speed = v4 - this->jerk_ *((pow(past_time - t4 , 2)) /2);
//       cal_pos = p4 + v4 *(past_time - t4)- this->jerk_ *(pow((past_time - t4), 3)/6);
//     }
//     /*Phase 6*/
//     else if (past_time <= t6 )
//     {
//       phase = 6;

//       double a5 = this->phase_time_params_[4].acc;
//       double v5 = this->phase_time_params_[4].vel;
//       double p5 = this->phase_time_params_[4].pos;

//       cal_speed = v5 - this->max_acc_ *(past_time - t5);
//       cal_pos = p5 + v5 * (past_time - t5) + a5*(pow((past_time - t5), 2)/2);
//     }
//     /*Phase 7**/
//     else if(past_time <= t7 )
//     {
//       phase = 7;

//       double a6 = this->phase_time_params_[5].acc;
//       double v6 = this->phase_time_params_[5].vel;
//       double p6 = this->phase_time_params_[5].pos;

//       cal_speed = v6 + a6 *(past_time - t6) + this->jerk_ *(pow((past_time - t6), 2)/2);
//       cal_pos = p6 + v6 *(past_time - t6) + a6 *(pow((past_time - t6), 2)/2) + this->jerk_ *(pow((past_time - t6), 3)/6);

//       // cal_speed = 0;
//     }
//     else
//     {

//       cal_pos = 0;
      
//       cal_speed = 0.0f;

//       phase = 8;

//     }




//   this->desired_speed_ = this->moving_dir_ * cal_speed;
//   this->desired_pos_ = cal_pos;


//   return phase;
// }


int ScurveProfiler::ScurveProfileGenerator()
{
  /*differ position*/
  double target_pos = 0.0f;

  /*Get moving direction*/
  this->moving_dir_ = 1;

  if(this->target_pos_ - this->present_pos_ <= 0)
    this->moving_dir_ = -1;

  target_pos = this->target_pos_;

  /*Make distance value positive */
  this->moving_distance_ = this->moving_dir_*(target_pos - this->present_pos_);


  printf("Target_pos = %lf, presetn_pos = %lf\r\n", this->target_pos_,this->present_pos_);
  printf("moving dist = %lf, dir = %d\r\n", this->moving_distance_,this->moving_dir_);

  /*Make distance value positive */
  //  this->moving_distance_ *= this->moving_dir_;

    // this->moving_distance_ -=20;   //stop offset

  //test
  this->vs_ = this->current_speed_;
  this->tacc_ = this->max_acc_;
  this->tdec_ = this->max_acc_;
  this->tvel_ = this->max_speed_;

  /*1. Check S-Curve has uniform velocity distance*/
  double A = this->tacc_;
  double D = this->tdec_;
  double Vs = this->vs_;
  double Ve = this->ve_;
  double V = this->tvel_;
  double L = this->moving_distance_;

  double tja = (V - Vs)/A;
  double tjd = (V - Ve)/D;
  double La = ((V + Vs)*tja)/2;
  double Ld = ((V + Ve)*tjd)/2;
  double Lt = La + Ld;

  int root = 0;

  double Ja = 0;
  double Jd = 0;

  double tv = 0;
  double tvel = 0;

  if (Lt <= L)  // S-Curve reached Max speed with uniform velocity distance or reached Max speed but No uniform velocity distance
    root = 1;
  else        // S-Curve has uniform velocity distance but no reached max speed;
    root = 2;  

  /*2. Get Parameters using given data*/
  if (root == 1)
  {
    double Lv = (L - (Lt));
    tv = Lv/V;
  }
  else 
  {
    tv = 0;
    tvel = sqrt((((2*A*D*L)+(D*(pow(Vs,2)) + A*(pow(Ve,2))))/(A+D)));



    tja = (tvel-Vs)/A;
    tjd = (tvel-Ve)/D;

    this->tvel_ = tvel;
  }

  if (tja == 0)
    Ja = 0;
  else
    Ja = 4*A/tja;

  if (tjd == 0)
    Jd = 0;
  else
    Jd = 4*D/tjd;




  double t1 = tja/2;
  double t2 = 2*tja/2;
  double t3 = (2*tja/2) + tv;
  double t4 = (2*tja/2) + tv + tjd/2;
  double t5 = (2*tja/2) + tv + (2*tjd/2);

  double a0 = 0;
  double v0 = Vs;
  double p0 = 0;

  double a1 = a0 + Ja*t1;
  double v1 = v0 + Ja*(pow(t1,2))/2;
  double p1 = (v0*t1) + Ja*(pow(t1,3))/6;

  double a2 = a1 + -Ja*(t2-t1);
  double v2 = v1 + a1*(t2-t1) -Ja*(pow((t2-t1),2))/2;
  double p2 = p1 + v1*(t2-t1) + a1*(pow((t2-t1),2)/2) + -Ja*((pow((t2-t1),3)/6));

  double a3 = 0;
  double v3 = v2;
  double p3 = p2 + v2*(t3-t2);

  double a4 = a3 + -Jd*(t4-t3);
  double v4 = v3 + a3*(t4-t3) + -Jd*(pow((t4-t3),2)/2);
  double p4 = p3 + v3*(t4-t3) + -Jd*(pow((t4-t3),3)/6);

  double a5 = a4 + Jd*(t5-t4);
  double v5 = v4 + a4*(t5-t4) + (1/2)*(Jd)*(pow((t5-t4),2));   
  double p5 = p4 + v4*(t5-t4) + a4*(pow((t5-t4),2)/2) + Jd*((pow((t5-t4),3)/6));


  double total_time = t1+t2+t3+t4+t5;

  // printf("va : %f, sa : %f, sv : %f\r\n", va, sa, sv);
  printf("profile type is : %d, total_time : %f, t1 : %f, t2 : %f, t3 : %f, t4 : %f, t5 : %f\r\n", root, total_time, t1, t2, t3, t4, t5);


  /*Calcuate phase time equation 33~39*/
  this->phase_time_[0] = t1;
  this->phase_time_[1] = t2;
  this->phase_time_[2] = t3;
  this->phase_time_[3] = t4;
  this->phase_time_[4] = t5;



  // printf("va = %f, sa = %f, sv = %f\r\n", va, sa, sv);
  // printf("Profile type : %d, tj = %f, ta = %f, tv = %f\r\n", profile_type, tj, ta, tv);
  // printf("start time = %ld, t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n",start_time,  t1,t2,t3,t4,t5,t6,t7);
  this->tv_ = tv;
  this->ja_ = Ja;
  this->jd_ = Jd;



  this->phase_time_params_[1].acc = a0;
  this->phase_time_params_[1].vel = v0;
  this->phase_time_params_[1].pos = p0;

  /*phase 1*/
  this->phase_time_params_[1].acc = a1;
  this->phase_time_params_[1].vel = v1;
  this->phase_time_params_[1].pos = p1;

  /*phase 2*/
  this->phase_time_params_[2].acc = a2;
  this->phase_time_params_[2].vel = v2;
  this->phase_time_params_[2].pos = p2;
                                
  /*phase 3*/
  this->phase_time_params_[3].acc = a3;
  this->phase_time_params_[3].vel = v3;
  this->phase_time_params_[3].pos = p3;

  /*phase 4*/
  this->phase_time_params_[4].acc = a4;
  this->phase_time_params_[4].vel = v4;
  this->phase_time_params_[4].pos = p4;

  /*phase 5*/
  this->phase_time_params_[5].acc = a5;
  this->phase_time_params_[5].vel = v5;
  this->phase_time_params_[5].pos = p5;
  /*Phase 7*/
  // p.7 is only calculated immediatly

  this->phase_time_params_[6].acc = 0;
  this->phase_time_params_[6].vel = 0;
  this->phase_time_params_[6].pos = p5;

  printf("p5 pos = %f\r\n", p5);

  this->start_time_ = std::chrono::steady_clock::now();

  return 0;
}


int ScurveProfiler::ScurveProfileWorker()
{
  /*output*/
  // double cal_acc = 0.0f;
  double cal_speed = 0.0f;
  double cal_pos = 0.0f;

  /*Calcuate phase time equation 33~39*/
  //double now_time = ((double)clock()/1000) - start_time_;



  double t1 = this->phase_time_[0];
  double t2 = this->phase_time_[1];
  double t3 = this->phase_time_[2];
  double t4 = this->phase_time_[3];
  double t5 = this->phase_time_[4];
  // double t6 = this->phase_time_[5];
  // double t7 = this->phase_time_[6];


  std::chrono::duration<double> duration_time =  std::chrono::steady_clock::now() - this->start_time_;

  double past_time = duration_time.count();

    int phase = 0;

    /*Phase 1.*/
    if(past_time >= 0 && past_time < t1)
    {
      phase = 1;

      double J = this->ja_;
      double a0 = this->phase_time_params_[0].acc;
      double v0 = this->phase_time_params_[0].vel;
      double p0 = this->phase_time_params_[0].pos;


      cal_speed = v0 + J*pow(past_time, 2)/2;
      cal_pos = (v0*past_time) + J*(pow(past_time,3))/6;
    }
    /*Phase 2*/
    else if(past_time >= t1 && past_time < t2)
    {   
      phase = 2;

      double J = -1*this->ja_;
      double a1 = this->phase_time_params_[1].acc;
      double v1 = this->phase_time_params_[1].vel;
      double p1 = this->phase_time_params_[1].pos;

      cal_speed = v1 + a1*(past_time - t1) + J*(pow((past_time - t1),2))/2;
      cal_pos = p1 + v1*(past_time - t1) + a1*(pow((past_time-t1),2)/2) + J*((pow((past_time - t1),3)/6));
    }
    /*Phase 3*/
    else if (past_time >= t2 && past_time < t3 )
    {
      phase = 3;

      // double a2 = this->phase_time_params_[2].acc;
      double v2 = this->phase_time_params_[2].vel;
      double p2 = this->phase_time_params_[2].pos;

      cal_speed = v2;
      cal_pos = p2 + v2*(past_time - t2);
    }
    /*Phase 4*/
    else if (past_time >= t3 && past_time < t4)
    {

      phase = 4;

      double J = -1*this->ja_;
      double a3 = this->phase_time_params_[3].acc;
      double v3 = this->phase_time_params_[3].vel;
      double p3 = this->phase_time_params_[3].pos;

      // if(abs(this->present_pos_ - p3) >= (this->moving_distance_*0.1))
      // {
      //   this->status_ =ProfilerStatus::sInit;

      //   this->max_speed_ = this->current_speed_;

      //   return phase;
      // }


      cal_speed = v3 + a3*(past_time-t3) + J*(pow((past_time-t3),2)/2);
      cal_pos = p3 + v3*(past_time-t3) + J*(pow((past_time-t3),3)/6);

    }
    /*Phase 5*/
    else if(past_time >= t4 && past_time < t5 )
    {
      phase = 5;

      double J = this->ja_;
      double a4 = this->phase_time_params_[4].acc;
      double v4 = this->phase_time_params_[4].vel;
      double p4 = this->phase_time_params_[4].pos;

      cal_speed = v4 + a4*(past_time-t4) + (1/2)*(J)*(pow((past_time-t4),2));   
      cal_pos = p4 + v4*(past_time-t4) + a4*(pow((past_time-t4),2)/2) + J*((pow((past_time-t4),3)/6));

    }
    else
    {

      cal_pos = 0;
      
      cal_speed = 0.0f;

      phase = 8;

    }




  this->desired_speed_ = this->moving_dir_ * cal_speed;
  this->desired_pos_ = cal_pos;


  return phase;
}






int ScurveProfiler::Drive()
{
  int status = this->status_;

  switch(status)
  {
    case ProfilerStatus::sInit: //init
    {
      int result = this->ScurveProfileGenerator();

      if(result >= 0)
        this->status_ =ProfilerStatus::sRun;
      else
        this->status_ =ProfilerStatus::sError;

      break;
    }

    case ProfilerStatus::sRun: //Run
    {
      int result = this->ScurveProfileWorker();

      if(result >= 8)
        this->status_ =ProfilerStatus::sEnd;
      else if (result <0)
        this->status_ =ProfilerStatus::sError;

      break;
    }

    case ProfilerStatus::sEnd: //End
    {
      this->desired_speed_ = 0;

      this->status_ =ProfilerStatus::sFinish;

      // if(abs(this->target_pos_ - present_pos_) <= MAX_ALLOW_ERROR )
      //   this->status_ =ProfilerStatus::sFinish;
      // else if((this->moving_itr)++ < MAX_MOVING_ITR)
      //   this->status_ =ProfilerStatus::sInit; 
      // else
      //   this->status_ =ProfilerStatus::sError;

      break;
    }

    case ProfilerStatus::sFinish: //finish
    {

      break;
    }

    case ProfilerStatus::sError: //error
    {


      break;
    }


    default : /*error*/ break;
  }





  return this->status_;
}






double ScurveProfiler::GetCalculatedVel()
{
  return this->desired_speed_;
}


int ScurveProfiler::GetStatusProfiler()
{
  return this->status_;
}



/*---------------------------------------------------------------------------------------------------------------*/
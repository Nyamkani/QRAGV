
/**
  ******************************************************************************
  * @file           : driving_controller.cpp
  * @brief          : Main Interface cpp file of QRAGV driving_controller for test
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

#include "driving_controller/driving_controller.hpp"



/**
 * @brief S-Curve Profile Generator 
 * Ref. by https://www.researchgate.net/publication/348383330_Mathematics_for_Real-Time_S-Curve_Profile_Generator
 * 
 */

// int TestScurveProfileGenerator(
//                                     int target_pos, 
//                                     bool& moving_start, 
//                                     double initial_speed, 
//                                     clock_t start_time,
//                                     double& target_speed)
// {
//     // int current_pos = motor_info.current_motor_pos;
//     // int current_speed = motor_info.current_motor_speed;

//   double start_time_ = (double)start_time/CLOCKS_PER_SEC;


//   /*given data*/
//   double jerk = 2000.0f;
//   double max_acc = 2000.0f;
//   double max_speed = 1000.0f;

//   /*output*/
//   double cal_acc = 0.0f;
//   double cal_speed = 0.0f;
//   double cal_pos = 0.0f;

//   /*s-curve status values(speed, space limitation values) 46 ,48 ,54 or 61*/
//   double va = (pow(max_acc, 2)/jerk);
//   double sa = ((2*pow(max_acc, 3))/pow(jerk, 2));
//   double sv = 0.0f;

//   if(max_speed * jerk < pow(max_acc, 2))
//     sv = 2* max_speed * sqrt(max_speed/jerk);
//   else 
//     sv = max_speed *((max_speed/max_acc) + (max_acc/jerk));



//   /*Specific time*/
//   double tj = 0.0f;
//   double ta = 0.0f;
//   double tv = 0.0f;


//   /*Possible movement profiles*/  
//   int profile_type = 0;

//   if(max_speed < va)   //the accellation curve is triangular
//   {
//     if(target_pos > sa)
//     {
//       profile_type = 1;  //positive and negative parts of the accellation are not continuous - (type A), C-1
//     }
//     else
//     {
//       if(target_pos > sv)
//         profile_type = 1; //positive and negative parts of the accellation are not continuous - type A, (C-1)
//       else
//         profile_type = 2;  //positive and negative parts of the accellation are continuous - Type B, (C-2)
//     }
//   }
//   else
//   {
//     if(target_pos < sa)
//     {
//       profile_type = 2;  //positive and negative parts of the accellation are continuous - (Type B), C-2
//     }
//     else
//     {
//       if(target_pos > sv)
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
//       tj = sqrt(max_speed/jerk);

//       ta = tj;

//       tv = target_pos/max_speed;

//       break;
//     }

//     case 2 :
//     {
//       //66,67,68
//       tj = cbrt((target_pos/(2*jerk)));

//       ta = tj;

//       tv = 2*tj;

//       break;
//     }

//     case 3 :
//     {
//       //77,78,79
//       tj = max_acc/jerk;

//       ta = max_speed/max_acc;

//       tv = target_pos/max_speed;

//       break;
//     }

//     case 4 :
//     {
//       //80.88.89
//       tj = max_acc/jerk;

//       ta = (1/2) * ((sqrt((4*target_pos*pow(jerk, 2)+pow(max_acc, 3))/(max_acc*pow(jerk, 2)))) - (max_acc/jerk));

//       tv = ta + tj;

//       break;
//     }

//     default : printf("Error : Invaile profile type\r\n"); break;
//   }





//     /*Calcuate phase time 33~39*/
//     double now_time = (double)clock()/CLOCKS_PER_SEC;

//     double t1 = start_time_ + tj;
//     double t2 = start_time_ + ta;
//     double t3 = start_time_ + tj + ta;
//     double t4 = start_time_ + tv;
//     double t5 = start_time_ + tj + tv;
//     double t6 = start_time_ + tv + ta;
//     double t7 = start_time_ + tv + ta + tj;

    
//     // printf("va = %f, sa = %f, sv = %f\r\n", va, sa, sv);
//     // printf("Profile type : %d, tj = %f, ta = %f, tv = %f\r\n", profile_type, tj, ta, tv);
//     // printf("start time = %ld, t1 = %f, t2 = %f, t3 = %f, t4 = %f, t5 = %f, t6 = %f, t7 = %f\r\n",start_time,  t1,t2,t3,t4,t5,t6,t7);

//     int phase = 0;

//     /*Phase 1.*/
//     if(now_time < t1)
//     {
//         phase = 1;

//         cal_speed = jerk * pow(now_time, 2) / 2;
//         //cal_pos = jerk * pow(now_time, 3) / 6;
//     }
//     /*Phase 2*/
//     else if(now_time < t2)
//     {   
//         phase = 2;

//         double a1 = jerk * t1;
//         double v1 = (jerk * pow(t1, 2) / 2);

//         cal_speed = v1 + a1*(now_time - t1);
//         //cal_pos = jerk * (pow(t1, 3) / 6) + (jerk * pow(t1, 2) / 2)  *(now_time - t1) + (jerk*t1)*(pow((now_time - t1), 2)/6);
//     }
//     /*Phase 3*/
//     else if (now_time < t3)
//     {
//         phase = 3;

//         double a1 = jerk * t1;
//         double v1 = (jerk * pow(t1, 2) / 2);
//         double v2 = v1 + a1 *(t2 - t1);
//         double a2 = a1;

//         cal_speed = v2 + a2*(now_time - t2) - jerk* ((pow(now_time - t2 , 3)) /2);
//     }
//     /*Phase 4*/
//     else if (now_time < t4)
//     {
//         phase = 4;

//         double a1 = jerk * t1;
//         double v1 = (jerk * pow(t1, 2) / 2);
//         double v2 = v1 + a1 *(t2 - t1);
//         double a2 = a1;
//         double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 3)) /2);

//         cal_speed = v3;
//     }
//     /*Phase 5*/
//     else if(now_time < t5)
//     {
//         phase = 5;

//         double a1 = jerk * t1;
//         double v1 = (jerk * pow(t1, 2) / 2);
//         double v2 = v1 + a1 *(t2 - t1);
//         double a2 = a1;
//         double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 3)) /2);
//         double v4 = v3;

//         cal_speed = v4 - jerk *((pow(now_time - t4 , 3)) /2);
//     }
//     /*Phase 6*/
//     else if (now_time < t6)
//     {
//         phase = 6;

//         double a1 = jerk * t1;
//         double v1 = (jerk * pow(t1, 2) / 2);
//         double v2 = v1 + a1 *(t2 - t1);
//         double a2 = a1;
//         double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 3)) /2);
//         double v4 = v3;
//         double v5 = v4 - jerk *((pow(t5 - t4 , 3)) /2);

//         cal_speed = v5 - max_acc*(now_time - t5);

//     }
//     /*Phase 7**/
//     else if(now_time < t7);
//     {
//         phase = 7;

//         // double a1 = jerk * t1;
//         // double v1 = (jerk * pow(t1, 2) / 2);
//         // double v2 = v1 + a1 *(t2 - t1);
//         // double a2 = a1;
//         // double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 3)) /2);
//         // double v4 = v3;
//         // double v5 = v4 - jerk *((pow(t5 - t4 , 3)) /2);
//         // double v6 = v5 - max_acc*(t6 - t5);
//         // double a6 = 0;

//         cal_speed = 0;

//         moving_start = false;
//     }

//     if (cal_speed > max_speed)
//         cal_speed = max_speed;

//     target_speed = cal_speed;

//   printf("Now time : %f, Phase : %d, Speed : %f\r\n", now_time, phase, target_speed);


//     return 0;
// }

struct testcurvestruct
{
  double phase_time[7];

};



bool test_end = true;


/**
 * @brief S-Curve Profile Generator 
 * Ref. by https://www.researchgate.net/publication/348383330_Mathematics_for_Real-Time_S-Curve_Profile_Generator
 * 
 */

int TestScurveProfileGenerator(testcurvestruct& tests,
                                    int target_pos, 
                                    bool& moving_start, 
                                    double initial_speed, 
                                    double& target_speed)
{
    // int current_pos = motor_info.current_motor_pos;
    // int current_speed = motor_info.current_motor_speed;



  /*given data*/
  double jerk = 2000.0f;
  double max_acc = 2000.0f;
  double max_speed = 4000.0f;

  /*s-curve status values(speed, space limitation values) 46 ,48 ,54 or 61*/
  double va = (pow(max_acc, 2)/jerk);
  double sa = ((2*pow(max_acc, 3))/pow(jerk, 2));
  double sv = 0.0f;

  if(max_speed * jerk < pow(max_acc, 2))
    sv = 2* max_speed * sqrt(max_speed/jerk);
  else 
    sv = max_speed *((max_speed/max_acc) + (max_acc/jerk));



  /*Specific time*/
  double tj = 0.0f;
  double ta = 0.0f;
  double tv = 0.0f;


  /*Possible movement profiles*/  
  int profile_type = 0;

  if(max_speed < va)   //the accellation curve is triangular
  {
    if(target_pos > sa)
    {
      profile_type = 1;  //positive and negative parts of the accellation are not continuous - (type A), C-1
    }
    else
    {
      if(target_pos > sv)
        profile_type = 1; //positive and negative parts of the accellation are not continuous - type A, (C-1)
      else
        profile_type = 2;  //positive and negative parts of the accellation are continuous - Type B, (C-2)
    }
  }
  else
  {
    if(target_pos < sa)
    {
      profile_type = 2;  //positive and negative parts of the accellation are continuous - (Type B), C-2
    }
    else
    {
      if(target_pos > sv)
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
      tj = sqrt(max_speed/jerk);

      ta = tj;

      tv = target_pos/max_speed;

      break;
    }

    case 2 :
    {
      //66,67,68
      tj = cbrt((target_pos/(2*jerk)));

      ta = tj;

      tv = 2*tj;

      break;
    }

    case 3 :
    {
      //77,78,79
      tj = max_acc/jerk;

      ta = max_speed/max_acc;

      tv = target_pos/max_speed;

      break;
    }

    case 4 :
    {
      //80.88.89
      tj = max_acc/jerk;

      ta = (1/2) * ((sqrt((4*target_pos*pow(jerk, 2)+pow(max_acc, 3))/(max_acc*pow(jerk, 2)))) - (max_acc/jerk));

      tv = ta + tj;

      break;
    }

    default : printf("Error : Invaile profile type\r\n"); break;
  }





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

    tests.phase_time[0] = t1;
    tests.phase_time[1] = t2;
    tests.phase_time[2] = t3;
    tests.phase_time[3] = t4;
    tests.phase_time[4] = t5;
    tests.phase_time[5] = t6;
    tests.phase_time[6] = t7;



    return 0;
}


int TestScurveProfileWorker(testcurvestruct tests,
                                    double initial_speed, 
                                    clock_t start_time,
                                    double& target_speed)
{
    // int current_pos = motor_info.current_motor_pos;
    // int current_speed = motor_info.current_motor_speed;

  double start_time_ = (double)start_time/1000;


  /*given data*/
  double jerk = 2000.0f;
  double max_acc = 2000.0f;
  double max_speed = 4000.0f;

  /*output*/
  double cal_acc = 0.0f;
  double cal_speed = 0.0f;
  double cal_pos = 0.0f;


    /*Calcuate phase time 33~39*/
    // clock_t now_time = clock() - start_time;
    double now_time = ((double)clock()/1000) - start_time_;

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
    if(now_time < t1)
    {
        phase = 1;

        cal_speed = jerk * pow(now_time, 2) / 2;
        //cal_pos = jerk * pow(now_time, 3) / 6;
    }
    /*Phase 2*/
    else if(now_time < t2)
    {   
        phase = 2;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);

        cal_speed = v1 + a1*(now_time - t1);
        //cal_pos = jerk * (pow(t1, 3) / 6) + (jerk * pow(t1, 2) / 2)  *(now_time - t1) + (jerk*t1)*(pow((now_time - t1), 2)/6);
    }
    /*Phase 3*/
    else if (now_time < t3)
    {
        phase = 3;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double v2 = v1 + a1 *(t2 - t1);
        double a2 = a1;

        cal_speed = v2 + a2*(now_time - t2) - jerk* ((pow(now_time - t2 , 3)) /2);
    }
    /*Phase 4*/
    else if (now_time < t4)
    {
        phase = 4;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double v2 = v1 + a1 *(t2 - t1);
        double a2 = a1;
        double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 3)) /2);

        cal_speed = v3;
    }
    /*Phase 5*/
    else if(now_time < t5)
    {
        phase = 5;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double v2 = v1 + a1 *(t2 - t1);
        double a2 = a1;
        double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 3)) /2);
        double v4 = v3;

        cal_speed = v4 - jerk *((pow(now_time - t4 , 3)) /2);
    }
    /*Phase 6*/
    else if (now_time < t6)
    {
        phase = 6;

        double a1 = jerk * t1;
        double v1 = (jerk * pow(t1, 2) / 2);
        double v2 = v1 + a1 *(t2 - t1);
        double a2 = a1;
        double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 3)) /2);
        double v4 = v3;
        double v5 = v4 - jerk *((pow(t5 - t4 , 3)) /2);

        cal_speed = v5 - max_acc*(now_time - t5);

    }
    /*Phase 7**/
    else 
    {
        phase = 7;

        // double a1 = jerk * t1;
        // double v1 = (jerk * pow(t1, 2) / 2);
        // double v2 = v1 + a1 *(t2 - t1);
        // double a2 = a1;
        // double v3 = v2 + a2*(t3 - t2) - jerk* ((pow(t3 - t2 , 3)) /2);
        // double v4 = v3;
        // double v5 = v4 - jerk *((pow(t5 - t4 , 3)) /2);
        // double v6 = v5 - max_acc*(t6 - t5);
        // double a6 = 0;

        cal_speed = 0;

        test_end = false;

    }

    if (cal_speed > max_speed)
        cal_speed = max_speed;

    target_speed = cal_speed;

  printf("start time : %f, Now time : %f, Phase : %d, Speed : %f\r\n",start_time_, now_time, phase, target_speed);


    return 0;
}






// int main()
// {
//   /*1. init pgv100 and isv2 motor */
//     Isv2MotorStruct left_motor_info_;
//     Isv2MotorStruct right_motor_info_;
//   /*2. making thread and command, data queue*/


//   /*3.Wait for typing command to controlling each modules*/

//   /* Read datas*/


//   double target_speed = 0.0f;

//   bool moving_start = true;

//   testcurvestruct tests_;

//   TestScurveProfileGenerator(tests_, 30000, moving_start, 50, target_speed);

//   clock_t start_time = clock();

//   while(test_end)
//   {
//     TestScurveProfileWorker(tests_, 0.0f, start_time, target_speed);

//     usleep(100000);
//   }

//   for(int i = 0; i<7; i++)
//     printf("%f ", tests_.phase_time[i]);

//   printf("\r\n");

//   return 0;
// }

// int main()
// {



//   return 0;
// }

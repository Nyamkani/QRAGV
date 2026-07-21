
#include "main/main.hpp"

QRAGV qragv_;


int main(int argc, char* argv[])
{
    qragv_.Initialize();

    while(1)
    {
        qragv_.Drive();

        usleep(5); 

    }




    return 0;
}













// void controller2(target_data& target, int target_pos, std::mutex& m)
// {

//   usleep(1000000);

//     printf("controller2 Thread Created!\r\n");

//   testcurvestruct tests_;

//   double target_speed = 0.0f;
//   double target_pos_ = 0.0f;

//  bool moving_start = true;

//     int dir_ = 1;

//   if(target_pos < 0)
//   {
//     dir_ = -1;

//     target_pos *=-1;
//   }




//   TestScurveProfileGenerator(tests_, target_pos, moving_start, 50, target_speed);


//   std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

//   auto prev_time = std::chrono::system_clock::now();

//   while(test_end)
//   {
//         target_data target_;

//         auto mili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - prev_time);

//         if(mili.count() >= 10) //10 milisceonds
//         {

//           TestScurveProfileWorker(tests_, 0.0f, start_time, target_speed, target_pos_);

//           target_.left_vel = target_speed* dir_;
//           target_.right_vel = target_speed* dir_;
//           target_.target_left_pos = target_pos_*dir_;
//           target_.target_right_pos = target_pos_*dir_;


//             m.lock();

//             target = target_;

//             m.unlock();

//            prev_time = std::chrono::system_clock::now();

//         }
//         usleep(100);

//   }
//             m.lock();

//             target.quit_sig = true;

//             m.unlock();



//     printf("controller2 Thread closed!\r\n");

//   return;
// }


// void motor_worker(target_data& target, std::mutex& m)
// {
//     isv2motor test;

//     test.Init();

//     bool quit_sig_ = false;
//     // int last_left_vel, last_right_vel;


//     printf("Motor Thread Created!\r\n");

//     usleep(1000);

//   Isv2MotorStruct* left_data = test.GetIsv2MotorStruct(1);
//   Isv2MotorStruct* right_data = test.GetIsv2MotorStruct(2);

//   for(int i = 0; i<100; i++)
//         test.Drive();

//   int left_encoder[2];
//     int right_encoder[2];
//   int left_velocity;
//     int right_velocity;

//   double left_conterr = 0;
//   double right_conterr = 0;

//   double left_prev_err = 0;
//   double right_prev_err = 0;

//   left_encoder[0] = left_data->current_motor_pos;
//   right_encoder[0] = right_data->current_motor_pos;


//   left_velocity = left_data->current_motor_speed;
//   right_velocity = right_data->current_motor_speed;



//   double kp = 1.5f;
//   double ki = 1.0f;
//   double kd = 1.0f;



//   printf("intial left encoder : %d, right_encoder = %d\r\n", left_data->current_motor_pos, right_data->current_motor_pos);


//   auto prev_time = std::chrono::system_clock::now();

//   int tick = 0;

//   target_data target_;

//     while(!(quit_sig_))
//     {
//       auto mili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - prev_time);

//       if(mili.count() >= 10) //10 milisceondss
//       {
//         m.lock();

//         target_ = target;

//         m.unlock();

//         target_data target_d = target_;

//         target_d.left_vel = (((target_d.left_vel)/(2*M_PI*40))*10000*2) * -1;

//         target_d.right_vel = (((target_d.right_vel)/(2*M_PI*40))*10000*2)* -1;

//         test.CANIdInstantSetVel(1, (int)target_d.left_vel);  //4m.s -> 160000
//         test.CANIdInstantSetVel(2, (int)target_d.right_vel);

//         // left_conterr = 0;
//         // right_conterr = 0;


//         // left_prev_err = 0;
//         // right_prev_err = 0;


//       if(target_.quit_sig)
//       {
//           test.CANIdInstantSetVel(1, 0);
//           test.CANIdInstantSetVel(2, 0);
//       }
//       quit_sig_ = target_.quit_sig;


//         prev_time = std::chrono::system_clock::now();
//       }

//       test.Drive();

//       usleep(100);

//     }


//   printf("Wait for motor stop\r\n");

//   while(1)
//   {
//     if(left_data->current_motor_speed <= 100 && left_data->is_inpos == true &&
//       right_data->current_motor_speed <= 100 && right_data->is_inpos == true 
//     )
//     {
//       break;
//     }

  

//    test.Drive();

//     usleep(1000);


//   }



//   printf("end left encoder : %d, right_encoder = %d\r\n", left_data->current_motor_pos, right_data->current_motor_pos);


//   left_encoder[1] = left_data->current_motor_pos;
//   right_encoder[1] = right_data->current_motor_pos;

//   double left_pos = ((double)(left_encoder[1] - left_encoder[0])/10000) *(2*M_PI*40)/2 * -1;
//   double right_pos = ((double)(right_encoder[1] - right_encoder[0])/10000) *(2*M_PI*40)/2 * -1;

//   printf(" left move pos : %f, right_move pos = %f\r\n", left_pos, right_pos);



//     printf("Motor Thread Closed!\r\n");

//     return;
// }


//           else if(tick %2 ==0)
//           {
//             target_data target_d = target_;

// /*


//             // left_velocity = (left_data->current_motor_speed/(10000 *(2*M_PI*40/2))) * -1;
//             // right_velocity = (right_data->current_motor_speed/(10000 *(2*M_PI*40/2))) * -1;

//             // double left_vel_err = target_d.left_vel - left_velocity;
//             // double right_vel_err = target_d.right_vel - right_velocity;
            
//             // printf("left_vel_err = %0.f, right_bel_err=  %0.f\r\n", left_vel_err, right_vel_err);


//             // left_conterr += left_vel_err;
//             // right_conterr += right_vel_err;

//             // double left_err_gap = left_vel_err - left_prev_err;
//             // double right_err_gap = right_vel_err - right_prev_err;

//             // double left_u = kp*left_vel_err + ki*(left_conterr) *(0.003) + kd* left_err_gap/(0.003);
//             // double right_u = kp*right_vel_err + ki*(right_conterr) *(0.003) + kd* right_err_gap/(0.003);

//             // left_prev_err = left_vel_err;
//             // right_prev_err = right_vel_err;
// */


//             left_encoder[1] = left_data->current_motor_pos;
//             right_encoder[1] = right_data->current_motor_pos;


//               double left_pos = ((double)(left_encoder[1] - left_encoder[0])/10000) *(2*M_PI*40)/2 * -1;
//               double right_pos = ((double)(right_encoder[1] - right_encoder[0])/10000) *(2*M_PI*40)/2 * -1;

//               double left_pos_err = target_d.target_left_pos - left_pos;
//               double right_pos_err = target_d.target_right_pos - right_pos;


//           left_conterr += left_pos_err;
//           right_conterr += right_pos_err;

//           double left_err_gap = left_pos_err - left_prev_err;
//           double right_err_gap = right_pos_err - right_prev_err;


//               // double left_u = kp*left_pos_err + ki*(left_pos_err) *(0.003) + kd* left_pos_err / (0.003);
//               // double right_u = kp*right_pos_err + ki*(right_pos_err) *(0.003) + kd* right_pos_err / (0.003);
//               double left_u = kp*left_pos_err + ki*(left_conterr) *(0.003) + kd* left_err_gap / (0.003);
//               double right_u = kp*right_pos_err + ki*(right_conterr) *(0.003) + kd* right_err_gap / (0.003);


//               double left_vel = left_u;//+ target_d.left_vel;
//               double right_vel = right_u;//+ target_d.right_vel;


//               //anti wind-up
//               if(left_vel >= 2000.0f)
//                 left_vel = 2000.0f;

//               if(right_vel >= 2000.0f)
//                 right_vel = 2000.0f;

//               target_d.left_vel = (((left_vel)/(2*M_PI*40))*10000*2) * -1;

//               target_d.right_vel = (((right_vel)/(2*M_PI*40))*10000*2)* -1;

//               //printf("motor left = %f, right = %f\r\n", target_.left_vel, target_.right_vel);


//         left_prev_err = left_pos_err;
//         right_prev_err = right_pos_err;

//               test.CANIdInstantSetVel(1, (int)target_d.left_vel);  //4m.s -> 160000
//               test.CANIdInstantSetVel(2, (int)target_d.right_vel);

            

         



//           //     if(tick++ >= 10)
//           //       tick = 0;

//         }
          


// int main()
// {
//     isv2motor test;

//     test.Init();


//   testcurvestruct tests_;

//   double target_speed = 0.0f;

//  bool moving_start = true;


//       printf("Test program start!\r\n");

//   TestScurveProfileGenerator(tests_, 50000, moving_start, 50, target_speed);

//   Isv2MotorStruct* left_data = test.GetIsv2MotorStruct(1);
//   Isv2MotorStruct* right_data = test.GetIsv2MotorStruct(2);

//   for(int i = 0; i<10; i++)
//         test.Drive();

//   printf("intial left encoder : %d, right_encoder = %d\r\n", left_data->current_motor_pos, right_data->current_motor_pos);


//   std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();



//   while(test_end)
//   {

//     TestScurveProfileWorker(tests_, 0.0f, start_time, target_speed);

//     test.CANIdInstantSetVel(1, (int)(target_speed/251.32736)*10000*2);  //4m.s -> 160000 251.32736
//     test.CANIdInstantSetVel(2, (int)(target_speed/251.32736)*10000*2);

//     test.Drive();


//     usleep(100000);

//   }
//   printf("finish left encoder : %d, right_encoder = %d\r\n", left_data->current_motor_pos, right_data->current_motor_pos);

//     test.CANIdInstantSetVel(1, 0);  //4m.s -> 160000
//     test.CANIdInstantSetVel(2, 0);



//     printf("Test program Susccessfully finished!!\r\n");

//   return 0;
// }





// int main(int argc, char* argv[])
// {
//     // target_data target_;
    
//     // std::mutex m;

//     // std::vector<std::thread> workers;

//     // int target_pos = 0;

//     // if(argc < 2)
//     //  target_pos = 1000;
//     //  else 
//     // target_pos = std::atoi(argv[1]);

//     // printf("Test program start!\r\n");

//     // // workers.push_back(std::thread(controller, std::ref(target_), std::ref(m)));
//     // workers.push_back(std::thread(controller2, std::ref(target_), std::ref(target_pos), std::ref(m)));
//     // workers.push_back(std::thread(motor_worker, std::ref(target_), std::ref(m)));

   
//     // for(auto& worker : workers)
//     //     worker.join();

//     // printf("Test program Susccessfully finished!!\r\n");

//     return 0;
// }





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

//   // clock_t start_time = clock();
//   std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

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


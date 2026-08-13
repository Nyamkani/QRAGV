




#include "main/main.hpp"

#include "isv2motor/isv2motor.hpp"
#include <thread>
#include <mutex>

#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <signal.h>


// int test_sig = 0;

static bool quit_sig = false;

static void stophandler(int sig)
{   
    quit_sig = true;

    printf("Signal occur! program will be quit\r\n!");

    return;
}


int target_pos_x = 1000;
int target_pos_y = 0;
int target_angle = 0;
int target_qr = 1;

/*driving controller test*/
int main(int argc, char **argv)
{
    signal(SIGINT, stophandler);

    printf("test program start !\r\n");

    DrivingController* test1 = nullptr;

    test1 = new DrivingController(500, 500, 1000);

    test1->Init();

    int target_pos_x = 0;
    int target_pos_y = 0;
    int target_angle = 0;
    int target_qr = 1;

    /*test*/
    // TargetPosStruct target = {(double)target_pos_x, (double)target_pos_y, (double)target_angle, target_qr};

    //     test1->target_pos_queue_.push_back(target);


    if(argc == 5)
    {
        target_pos_x = atoi(argv[1]);
        target_pos_y = atoi(argv[2]);
        target_angle = atoi(argv[3]);
        target_qr = atoi(argv[4]);


        printf("Got target Pos x = %d, Pos y = %d, angle = %d, qr_code = %d\r\n", target_pos_x, target_pos_y, target_angle, target_qr);


        /*test*/
        TargetPosStruct target = {(double)target_pos_x, (double)target_pos_y, (double)target_angle, target_qr};

        test1->target_pos_queue_.push_back(target);

    }
    else if (argc == 2)
    {
        TargetPosStruct target1 = {1800, 0, -45, 3};

        test1->target_pos_queue_.push_back(target1);

        TargetPosStruct target2 = {900, -900, 90, 27};

        test1->target_pos_queue_.push_back(target2);

        TargetPosStruct target3 = {0, 900, 0, 4};

        test1->target_pos_queue_.push_back(target3);

        TargetPosStruct target4 = {1800, 0, -180, 6};

        test1->target_pos_queue_.push_back(target4);

        TargetPosStruct target5 = {-4500, 0, 0, 1};

        test1->target_pos_queue_.push_back(target5);

    }
    else
    {
        printf("Invalid No of parameters no of input is = %d\r\n", argc -1);
        printf("test program will check sensors !\r\n");

    }

    sleep(2);

    auto prev_time = std::chrono::steady_clock::now();

    int cnt = 0;
    bool tg_ = false;

    while(!(quit_sig))
    {
        auto mili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - prev_time);

        if(mili.count() >= 10)
        {
            test1->Drive3();

            prev_time = std::chrono::steady_clock::now();

            if(cnt++ >= 10)
                tg_= true;
        }

        if(tg_)
        {
        //    printf("Current left Pos data  = %f, Current right Pos data  = %f\r\n",  test1->current_left_pos_,   test1->current_right_pos_);

            // if(test1->is_detected_ && !(test1->is_tagged_))
                printf("Odometry Test: xpos = %f, ypos = %f, angle_rad = %f angle_degree = %f left vel = %d, right vel = %d task = %d\r\n", 
                            test1->x_pos_, test1->y_pos_, test1->angle_, test1->angle_degree_, test1->left_current_motor_speed_, test1->right_current_motor_speed_, test1->task_);
            if(test1->is_tagged_)
                printf("QRTag Test: x qrpos = %f, y qrpos = %f, tag code = %ld, angle_rad = %f angle_degree = %f \r\n", 
                            test1->x_qrpos_, test1->y_qrpos_, test1->tag_code_, test1->angle_, test1->angle_degree_);

                // printf("Task = %d\r\n", test1->task_);

            tg_= false;

            cnt = 0;

        }
    
        // if(test_sig == 1)
        // {
        //     TargetPosStruct target1 = {900, 0, 0, 2};

        //     test1->target_pos_queue_.push_back(target1);

        // }

        usleep(1);
    }

  delete test1;







  printf("test program end !\r\n");
}
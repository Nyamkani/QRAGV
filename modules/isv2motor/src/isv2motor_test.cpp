/**
  ******************************************************************************
  * @file           : isv2motor_test.cpp
  * @brief          : Test running cpp file of isv2motor Control
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


#include "isv2motor/isv2motor.hpp"
#include <thread>
#include <mutex>

#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}




struct target_data
{
    double left_vel = 0.0f;
    double right_vel = 0.0f;
    bool quit_sig = false;
};



void controller(target_data& target, std::mutex& m)
{
    clock_t prev_time = clock();

    bool quit_sig_ = false;

    target_data target_;
    
     const char *device;
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0,};
    size_t axis;


    printf("Controller Thread Created!\r\n");

    device = "/dev/input/js0";

    js = open(device, O_RDONLY);

    if (js == -1)
        perror("Could not open joystick");


    while((!(quit_sig_)) && read_event(js, &event) == 0)
    {
        /*Controller*/
        switch (event.type)
        {
            case JS_EVENT_BUTTON:
            {
                printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
                    
                if(event.number ==1 && event.value ==1)
                {
                    //TestRobotStopMove();
                    target_.left_vel = 0;
                    target_.right_vel = 0;
                }
                    
                else if(event.number ==9 && event.value ==1)
                {
                    target_.left_vel = 0;
                    target_.right_vel = 0;
                    target_.quit_sig = true;
                }

                break;
            }

            case JS_EVENT_AXIS:
                axis = get_axis_state(&event, axes);
                if (axis < 3)
                {
                  printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);

                  if(axes[axis].y < 0)
                  {
                    // TestRobotForwardMove();
                    target_.left_vel = 1* (axes[axis].y/32767);
                    target_.right_vel = 1* (axes[axis].y/32767);
                  }
                  else if(axes[axis].y > 0)
                  {
                    // TestRobotBackwardMove();
                    target_.left_vel = 1* (axes[axis].y/32767);
                    target_.right_vel = 1* (axes[axis].y/32767);

                  }
                  else if (axes[axis].x < 0)  //left turn
                  {
                    // TestRobotLeftTurn();
                    target_.left_vel = -1* (axes[axis].x/32767);
                    target_.right_vel = 1* (axes[axis].x/32767);
                  }
                  else if (axes[axis].x > 0)  //right turn
                  {
                    // TestRobotRightTurn();
                    target_.left_vel = -1* (axes[axis].x/32767);
                    target_.right_vel = 1* (axes[axis].x/32767);
                  }
                  else 
                  {
                    // TestRobotStopMove();
                    target_.left_vel = 0;
                    target_.right_vel = 0;
                  }

                }
                    
                break;
            default:
                /* Ignore init events. */
                break;
        }

        fflush(stdout);


        if(clock() - prev_time > 10000) //milisceonds
        {
            m.lock();

            target = target_;

            //printf("controller left = %d, right = %d\r\n", target.left_vel, target.right_vel);

            m.unlock();

            quit_sig_ = target_.quit_sig;

            prev_time = clock();
        }

        usleep(100);

    }

    close(js);

    printf("Controller Thread Closed!\r\n");

    return;
}




void motor_worker(target_data& target, std::mutex& m)
{
    isv2motor test;

    test.Init();

    clock_t prev_time = clock();

    bool quit_sig_ = false;

    // int last_left_vel, last_right_vel;


    printf("Motor Thread Created!\r\n");

    while(!(quit_sig_))
    {
        if(clock() - prev_time > 33000) //milisceonds
        {
            target_data target_;

            m.lock();

            target_ = target;

            m.unlock();



        //    printf("motor left = %f, right = %f\r\n", target_.left_vel, target_.right_vel);

            test.CANIdInstantSetVel(1, target_.left_vel *  100000);
            test.CANIdInstantSetVel(2, target_.right_vel *  100000 );


            // test.CANIdSetVel(1, target_.left_vel *  100000);
            // test.CANIdSetVel(2, target_.right_vel *  100000 );


            if(target_.quit_sig)
            {
                test.CANIdInstantSetVel(1, 0);
                test.CANIdInstantSetVel(2, 0);
            }

            quit_sig_ = target_.quit_sig;

            prev_time = clock();
        }



        test.Drive();

        usleep(100);

    }

    printf("Motor Thread Closed!\r\n");

    return;
}




int main()
{
    target_data target_;
    
    std::mutex m;

    std::vector<std::thread> workers;

    printf("Test program start!\r\n");

    workers.push_back(std::thread(controller, std::ref(target_), std::ref(m)));
    workers.push_back(std::thread(motor_worker, std::ref(target_), std::ref(m)));

   
    for(auto& worker : workers)
        worker.join();

    printf("Test program Susccessfully finished!!\r\n");

    return 0;
}





    // test.Init();
    
    // for(int i = 0; i <300; i++)
    // {

    //     test.Drive();

    //     usleep(100);
    // }

    // test.CANIdSetVel(1, 100000);
    // test.CANIdSetVel(2, 100000);
    // for(int i = 0; i <20; i++)
    // {

    //     test.Drive();

    //     usleep(100);
    // }

    // sleep(20);

    // test.CANIdSetVel(1, 0);
    // test.CANIdSetVel(2, 0);
    // for(int i = 0; i <50; i++)
    // {

    //     test.Drive();

    //     usleep(100);
    // }


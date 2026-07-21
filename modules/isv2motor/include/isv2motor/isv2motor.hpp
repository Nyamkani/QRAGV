/**
  ******************************************************************************
  * @file           : isv2motor.hpp
  * @brief          : Main Interface hpp file of isv2motor Control
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


#ifndef KSSBOT_HARDWARE__ISV2MOTOR_HPP_
#define KSSBOT_HARDWARE__ISV2MOTOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "isv2motor/jetson_can.h"

#ifdef __cplusplus
}
#endif

#include <cmath>
#include <vector>
#include <chrono>
#include <string>
#include <iostream>
#include <mutex>

#include "isv2motor/isv2motor_define.hpp"
#include <hw_interface/hw_interface.hpp>



/**
 * @brief Main Application Class for using isv2motor(control) 
 * 
 */
class isv2motor : public kssbot_hardware::actuator_interface
{
  private:
    /*system data*/
    std::vector<Isv2MotorStruct*> motor_data_;

    /*buffer*/
    std::vector<Isv2MotorCommandStruct> send_cmd_queue_;
    std::vector<CAN_data_struct> recv_cmd_queue_;

    std::vector<CAN_data_struct> recv_data_queue_;


    /*Comm. timer and values*/
    /*send-recv check */ 
    int send_flag_ = 0;
    int last_send_id_ = 0;
    int send_repeat_cnt_ = 0;

    double wheel_radius_ = 40.0;
    size_t reducer_rate_ = 2;
    size_t motor_encoder_increment = 10000;


    /*init data*/
    double motor_acc_ = 0.0f;
    double motor_dec_ = 0.0f;
    double max_motor_vel_ = 0.0f;

  public:

  private:
    kssbot_hardware::IReturnType HardwareInitiailize();
    kssbot_hardware::IReturnType SystemInitiailize();
    kssbot_hardware::IReturnType SystemOperation();




  public:
    /**
     * @brief Construct a new isv2motor object
     * 
     */
    isv2motor();

    /**
     * @brief Construct a new isv2motor object
     * 
     */
    isv2motor(double motor_acc, double motor_dec);

    /**
     * @brief Construct a new isv2motor object
     * 
     */
    isv2motor(double motor_acc, double motor_dec, double max_motor_vel);



    /**
     * @brief Destroy the isv2motor object
     * 
     */
    virtual ~isv2motor();

    /**
     * @brief isv2motor Initialize
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType Init() override;

    /**
     * @brief isv2motor Start Operation(start)
       * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType StartRun() override;

    /**
     * @brief isv2motor Stop Operation(stop)
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType StopRun() override;

    /**
     * @brief isv2motor write data
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType Write() override;

    /**
     * @brief isv2motor read data
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType Read() override;

    /**
     * @brief isv2motor main operation function
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType Drive() override;



    Isv2MotorStruct* GetIsv2MotorStruct(int node);


    //--------------------------------------------------------------------------------
    kssbot_hardware::IReturnType RegisterMotor(int node);

    kssbot_hardware::IReturnType RegisterMotor(int node, int motor_dir);

    kssbot_hardware::IReturnType DeleteMotor(int node);

    kssbot_hardware::IReturnType DeleteAllMotor();


    //--------------------------------------------------------------------------------params
    void EnqueueSendCANData(const CAN_data_struct send_data);


    void CANIdInstantSetControlWord(const int id, const int status);

    void CANIdInstantSetModeOperation(const int id, const int mode);

    void CANIdInstantSetAccel(const int id, const int value);

    void CANIdInstantSetDecel(const int id, const int value);

    void CANIdInstantMoveStop(const int id);




    void CANIdSetPolarity(const int id, const int value);

    void CANIdReadPolarity(const int id);

    void CANIdSetVel(const int id, const int value);

    void CANIdMoveStop(const int id);

    void CANIdInstantSetVel(const int id, const int value);

    //--------------------------------------------------------------------------------return types

    std::vector<Isv2MotorStruct*> GetMotorData();

    /*instant process*/
    int CANIdInstantProcess(const CAN_data_struct send_data);

};


/*-------------------NMT---------------------*/
void CANIdInstantReset();
void CANIdInstantResetComm(int id);
void CANIdInstantRegister(int id);
void CANIdSInstantetPreOpMode(int id);
/*-------------------PDO---------------------*/



/*calculate */
double ChangeRPMToMMs(double rpm, double radius, size_t motor_increment, size_t motor_reducer_rate);
double ChangeMMsToRPM(double mms, double radius, size_t motor_increment, size_t motor_reducer_rate);




#endif  // KSSBOT_HARDWARE__PGV100_HPP_



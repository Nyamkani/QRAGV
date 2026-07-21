/**
  ******************************************************************************
  * @file           : pgv100.hpp
  * @brief          : Main Interface hpp file of PGV100 Position Sensor
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


#ifndef KSSBOT_HARDWARE__PGV100_HPP_
#define KSSBOT_HARDWARE__PGV100_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <pgv100/jetson_uart.h>


#ifdef __cplusplus
}
#endif

#include <cmath>
#include <vector>
#include <chrono>
#include <string>
#include <iostream>
#include <mutex>


#include "pgv100/pgv100_define.hpp"
#include <hw_interface/hw_interface.hpp>




/**
 * @brief Main Application Class for using pgv100 sensor
 * 
 */
class pgv100 : public kssbot_hardware::sensor_interface
{
  private:
    /*system param*/
    size_t pos_unit_;
    size_t sesnor_id_;

    /*Unique status*/
    bool is_detected_ = false;
    bool is_tagged_ = false;

    /*unique value*/
    size_t tag_code_= 0;
    size_t error_code_ = 0;
    size_t pgv100_dir_ = 0;
    size_t pgv100_color_ = 0;
    int xpos_= 0;
    int ypos_ = 0;
    int angle_ = 0;



    /*buffer*/
    std::string recv_stream_;

    std::vector<unsigned char> req_tele_queue_;

  public:

  private:
    kssbot_hardware::IReturnType HardwareInitiailize();
    kssbot_hardware::IReturnType SystemInitiailize();
    kssbot_hardware::IReturnType ResponseTelegramParse();
    kssbot_hardware::IReturnType SystemOperation();

    size_t GetResponseDataLength(const size_t request_telegram);
    void EnqueueRequestTelegram(size_t request_telegram);


  public:
    /**
     * @brief Construct a new pgv100 object
     * 
     */
    pgv100();

    /**
     * @brief Construct a new pgv100 object
     * 
     */
    pgv100(size_t pos_unit);

    /**
     * @brief Construct a new pgv100 object
     * 
     */
    pgv100(size_t sesnor_id, size_t pos_unit);

    /**
     * @brief Destroy the pgv100 object
     * 
     */
    virtual ~pgv100();

    /**
     * @brief pgv100 Initialize
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType Init() override;

    /**
     * @brief pgv100 Start Operation(start)
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType StartRun() override;

    /**
     * @brief pgv100 Stop Operation(stop)
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType StopRun() override;

    /**
     * @brief pgv100 write data
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType Write() override;

    /**
     * @brief pgv100 read data
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType Read() override;

    /**
     * @brief pgv100 main operation function
     * 
     * @return kssbot_hardware::IReturnType 
     */
    kssbot_hardware::IReturnType Drive() override;

    int GetXPos() const {return this->xpos_;} 
    int GetYPos() const {return this->ypos_;} 
    int GetAngle() const {return this->angle_;} 
    size_t GetPgv100Dir() const {return this->pgv100_dir_;} 
    size_t GetPgv100Color() const {return this->pgv100_color_;} 
    size_t GetErrorCode() const {return this->error_code_;} 
    size_t GetTagCode() const {return this->tag_code_;} 

    bool IsLineDetected() const {return this->is_detected_;} 
    bool IsTagged() const {return this->is_tagged_;} 

    PosSensorDataStruct GetPGV100DataStructure();





};


#endif  // KSSBOT_HARDWARE__PGV100_HPP_
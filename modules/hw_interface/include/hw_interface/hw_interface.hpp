/**
  ******************************************************************************
  * @file           : hw_interface.hpp
  * @brief          : Standard interface for Hardware control
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


#ifndef KSSBOT_HARDWARE__HW_INTERFACE_HPP_
#define KSSBOT_HARDWARE__HW_INTERFACE_HPP_


namespace kssbot_hardware
{
  enum LifeCycleState
  {
    kHwInit = 0,
    kInit = 1,
    kRun = 2,
    kStop = 3,
    kError = 4,
  };


  enum IReturnType
  {
    kReturnOk = 0,
    kReturnAlarm = 10,
    kReturnError = 20,
  };



  /**
   * @brief Interface for using sensor
   * 
   */
  class sensor_interface
  {
      protected:

        /*sensor status field*/
        bool is_hw_init_ = false;
        bool is_init_ = false;
        bool is_run_ = false;
        bool is_stop_ = false;
        bool is_error_ = false;        

        /*sensor lifecycle state*/
        kssbot_hardware::LifeCycleState state_ = kssbot_hardware::LifeCycleState::kHwInit;

        /*sensor filtering*/
        size_t filter_cnt_ = 0;
        size_t sensor_filter_val_ = 0;

      public:



      protected:
          /**
           * @brief Construct a new sensor interface object
           * 
           */
          sensor_interface(){};

          /**
           * @brief Destroy the sensor interface object
           * 
           */
          virtual ~sensor_interface(){};

          /**
           * @brief Initialize sensor function interfce
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType Init(){return kssbot_hardware::IReturnType::kReturnOk;};      

          /**
           * @brief Start Operation Sensor function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType StartRun(){return kssbot_hardware::IReturnType::kReturnOk;};   

          /**
           * @brief Stop Operation Sensor function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType StopRun(){return kssbot_hardware::IReturnType::kReturnOk;};   

          /**
           * @brief Write Data Sensor function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType Write(){return kssbot_hardware::IReturnType::kReturnOk;};   

          /**
           * @brief Read Data Sensor function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType Read(){return kssbot_hardware::IReturnType::kReturnOk;};    

          /**
           * @brief Main Operation Sensor Function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType Drive(){return kssbot_hardware::IReturnType::kReturnOk;};    


          /**
           * @brief Get Hardware initialize status
           * 
           * @return true 
           * @return false 
           */
          bool is_hw_init() const {return this->is_hw_init_;} 

          /**
           * @brief Get System Initialize status
           * 
           * @return true 
           * @return false 
           */
          bool is_init() const {return this->is_init_;}

          /**
           * @brief Get System running status
           * 
           * @return true 
           * @return false 
           */
          bool is_run() const {return this->is_run_;}

          /**
           * @brief Get System stop status
           * 
           * @return true 
           * @return false 
           */
          bool is_stop() const {return this->is_stop_;} 

          /**
           * @brief Get System error status
           * 
           * @return true 
           * @return false 
           */
          bool is_error() const {return this->is_hw_init_;}

      private:

  };



  /**
   * @brief Interface for using actuator
   * 
   */
  class actuator_interface
  {
      protected:
        bool is_hw_init_ = false;
        bool is_init_ = false;
        bool is_run_ = false;
        bool is_stop_ = false;
        bool is_error_ = false;        


        /*sensor lifecycle state*/
        kssbot_hardware::LifeCycleState state_ = kssbot_hardware::LifeCycleState::kHwInit;

      public:

 protected:
          /**
           * @brief Construct a new sensor interface object
           * 
           */
          actuator_interface(){};

          /**
           * @brief Destroy the sensor interface object
           * 
           */
          virtual ~actuator_interface(){};

          /**
           * @brief Initialize sensor function interfce
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType Init(){return kssbot_hardware::IReturnType::kReturnOk;};      

          /**
           * @brief Start Operation Sensor function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType StartRun(){return kssbot_hardware::IReturnType::kReturnOk;};   

          /**
           * @brief Stop Operation Sensor function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType StopRun(){return kssbot_hardware::IReturnType::kReturnOk;};   

          /**
           * @brief Write Data Sensor function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType Write(){return kssbot_hardware::IReturnType::kReturnOk;};   

          /**
           * @brief Read Data Sensor function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType Read(){return kssbot_hardware::IReturnType::kReturnOk;};    

          /**
           * @brief Main Operation Sensor Function interface
           * 
           * @return kssbot_hardware::IReturnType 
           */
          virtual kssbot_hardware::IReturnType Drive(){return kssbot_hardware::IReturnType::kReturnOk;};    


          /**
           * @brief Get Hardware initialize status
           * 
           * @return true 
           * @return false 
           */
          bool is_hw_init() const {return this->is_hw_init_;} 

          /**
           * @brief Get System Initialize status
           * 
           * @return true 
           * @return false 
           */
          bool is_init() const {return this->is_init_;}

          /**
           * @brief Get System running status
           * 
           * @return true 
           * @return false 
           */
          bool is_run() const {return this->is_run_;}

          /**
           * @brief Get System stop status
           * 
           * @return true 
           * @return false 
           */
          bool is_stop() const {return this->is_stop_;} 

          /**
           * @brief Get System error status
           * 
           * @return true 
           * @return false 
           */
          bool is_error() const {return this->is_hw_init_;}

      private:

  };








}

#endif  // KSSBOT_HARDWARE__HW_INTERFACE_HPP_
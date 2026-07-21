/**
  ******************************************************************************
  * @file           : qragv.hpp
  * @brief          : Main Interface hpp file of qragv drive
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

#ifndef QRAGV_MAIN_HPP_
#define QRAGV_MAIN_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<unistd.h>

#ifdef __cplusplus
}
#endif

#include <cmath>
#include <vector>
#include <chrono>
#include <string>
#include <iostream>
#include <mutex>

#include <qragv/qragv_server.hpp>
#include <qragv/qragv_state.hpp>
#include <qragv/qragv_define.hpp>

#include <driving_controller/driving_controller.hpp>


#define QUEUE_DEFAULT_NUM 5
#define BUF_MAX_LEN 4096
#define SERVER_PORT 127




/* for reference class */
class QRAGVState;

class QRAGV
{
    private:
		/*FSM STATE*/
		QRAGVState* state_= nullptr;

        /*FSM Conctrol*/
        std::vector<int>sfm_events_;

        /*Modules*/
        /*make each moudules at each threads*/
        //1. eth server
        //2. driving controller


        /*Command Queue*/


    public:
        /*Moudule threads*/
        std::vector<std::thread> workers_;
        bool quit_sig_ = false;


        /*Command Queue*/


    private:
        /*qragv_fsm Conctrol*/
        void FSMEventCheck();
        void StartInitState();
        void FSMEventRegister();
        void FSMStateBehavior();
        void FSMTransitionState(int state);
        void FSMEventAdd(int event);

    public:
        QRAGV();
        virtual ~QRAGV();


        void Drive();

        int Initialize();

};

#endif  // QRAGV_MAIN_HPP_



/**
  ******************************************************************************
  * @file           : qragv_define.hpp
  * @brief          : Interface hpp file of qragv define
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

#ifndef QRAGV_DEFINE_HPP_
#define QRAGV_DEFINE_HPP_


#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#include <cmath>
#include <vector>
#include <chrono>
#include <string>
#include <iostream>
#include <mutex>

enum FSMEventNumbering
{
	FSM_EVENT_NONE = 0,

	// SFM_EVENT_IS_IPC_INIT = 1,
	// SFM_EVENT_IS_ETH_INIT = 2,
	// SFM_EVENT_IS_FATFS_INIT = 3,
	// SFM_EVENT_IS_ALL_INIT = 10,

	// SFM_EVENT_IS_IPC_RUN = 11,
	// SFM_EVENT_IS_ETH_RUN = 12,
	// SFM_EVENT_IS_FATFS_RUN = 13,
	// SFM_EVENT_IS_ALL_RUN = 20,

	// SFM_EVENT_IS_IPC_ERROR = 21,
	// SFM_EVENT_IS_ETH_ERROR = 22,
	// SFM_EVENT_IS_FATFS_ERROR = 23,

	// SFM_EVENT_IS_CONNECTED = 30,
	// SFM_EVENT_IS_DISCONNECTED = 31,
};

struct QRAGVCommandStruct
{
	uint64_t transactionid_;
	uint16_t cmd_;

	std::vector<int> data_;
};



#endif  // QRAGV_DEFINE_HPP_


/**
  ******************************************************************************
  * @file           : qragv_data.hpp
  * @brief          : Interface hpp file of qragv data
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024  AGV TEAM, DEPARTMENT OF ROBOT CONTROL, STUDIO 3S Ltd.
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



struct QRAGVCommandStruct
{
	uint64_t transactionid_;
	uint16_t cmd_;

	std::vector<int> data_;
};



#endif  // qragv_data


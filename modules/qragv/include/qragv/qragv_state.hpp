/**
  ******************************************************************************
  * @file           : qragv_state.hpp
  * @brief          : Interface hpp file of qragv state
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

#ifndef QRAGV_STATE_HPP_
#define QRAGV_STATE_HPP_

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



#include <qragv/qragv.hpp>

// const uint32_t odom_save_time =  6000000;   //10m
#define ODOM_SAVE_TIME      60*60*1000; 	//1h
#define BAKCUP_SAVE_TIME    12*60*60*1000; 	//12h
#define RESET_M7_TIME       1*1000        	//1s
/* for reference class */
class QRAGV;

/* Condition Interfaces*/
class QRAGVState
{
	private:


	public:


	private:

	public:
		// qragv_state(){};
		// virtual ~qragv_state(){};

		// /* Common Moudule Event */
		// virtual bool IsIPCInit(SHTM7Main& SHTM7Main_);
		// virtual bool IsETHInit(SHTM7Main& SHTM7Main_);
		// virtual bool IsFatFsInit(SHTM7Main& SHTM7Main_);
		// virtual bool IsAllInit(SHTM7Main& SHTM7Main_);

		// virtual bool IsIPCRun(SHTM7Main& SHTM7Main_);
		// virtual bool IsETHRun(SHTM7Main& SHTM7Main_);
		// virtual bool IsFatFsRun(SHTM7Main& SHTM7Main_);
		// virtual bool IsAllRun(SHTM7Main& SHTM7Main_);

		// virtual bool IsIPCError(SHTM7Main& SHTM7Main_);
		// virtual bool IsETHError(SHTM7Main& SHTM7Main_);
		// virtual bool IsFatFsError(SHTM7Main& SHTM7Main_);

		// virtual bool IsConnected(SHTM7Main& SHTM7Main); 
		// virtual bool IsDisconnected(SHTM7Main& SHTM7Main); 

		// virtual bool IsPaused(){return false;};
		// virtual bool IsReset(){return false;};
		// virtual bool IsResume(){return false;};

		/* Specify State Event */
		virtual void StateBehavior(QRAGV& QRAGV_){return ;};


		/* SFM Control Fucntion */
		virtual QRAGVState* TransitionState(QRAGV& QRAGV_, int state){return nullptr;};
		virtual void EnterState(QRAGV& QRAGV_){return ;};
		virtual void ExitState(QRAGV& QRAGV_){return ;};
};


class QRAGVInitState : public QRAGVState
{
	private:
		/* init sequence*/

	public:


	private:

	public:
		QRAGVInitState(){};
		virtual ~QRAGVInitState(){};
		virtual QRAGVState* TransitionState(QRAGV& QRAGV_, int state);
		virtual void EnterState(QRAGV& QRAGV_);
		virtual void ExitState(QRAGV& QRAGV_);
		virtual void StateBehavior(QRAGV& QRAGV_);
};

class QRAGVPreRunState : public QRAGVState
{
	// private:

	// public:


	// private:

	// public:
	// 	SHTPreRunState(){};
	// 	virtual ~SHTPreRunState(){};
	// 	virtual SHTM7State* TransitionState(SHTM7Main& SHTM7Main_, int state);
	// 	virtual void EnterState(SHTM7Main& SHTM7Main_);
	// 	virtual void ExitState(SHTM7Main& SHTM7Main_);
	// 	virtual void StateBehavior(SHTM7Main& SHTM7Main_);
};

class QRAGVRunState : public QRAGVState
{
	// private:
	// 	uint32_t odom_save_timer;
	// 	uint32_t backup_save_timer;

	// 	const uint32_t odom_save_time = ODOM_SAVE_TIME; 	 //60m = 1000ms*(60*60)
	// 	const uint32_t backup_save_time = BAKCUP_SAVE_TIME; //12*60*60*1000;   //12h = 12*1000ms*(60*60)
	// public:	


	// private:

	// public:
	// 	SHTRunState(){};
	// 	virtual ~SHTRunState(){};
	// 	virtual SHTM7State* TransitionState(SHTM7Main& SHTM7Main_, int state);
	// 	virtual void EnterState(SHTM7Main& SHTM7Main_);
	// 	virtual void ExitState(SHTM7Main& SHTM7Main_);
	// 	virtual void StateBehavior(SHTM7Main& SHTM7Main_);
};

class QRAGVPauseState : public QRAGVState
{
	// private:

	// public:


	// private:

	// public:
	// 	SHTPauseState(){};
	// 	virtual ~SHTPauseState(){};
	// 	virtual bool IsInitDone();
	// 	virtual bool IsConnected();
	// 	virtual bool IsFatFsError();
	// 	virtual bool IsIPCError();
	// 	virtual bool IsETHError();
	// 	virtual bool IsPaused();
	// 	virtual bool IsReset();
	// 	virtual bool IsResume();
};

class QRAGVErrorState : public QRAGVState
{
	// private:

	// public:


	// private:

	// 	uint32_t reset_timer_;

	// 	const uint32_t reset_time_ = RESET_M7_TIME; 

	// 	bool reset_flag_ = false;

	// public:
	// 	SHTErrorState(){};
	// 	virtual ~SHTErrorState(){};
	// 	virtual SHTM7State* TransitionState(SHTM7Main& SHTM7Main_, int state);
	// 	virtual void EnterState(SHTM7Main& SHTM7Main_);
	// 	virtual void ExitState(SHTM7Main& SHTM7Main_);
	// 	virtual void StateBehavior(SHTM7Main& SHTM7Main_);
};

#endif  // QRAGV_STATE_HPP_



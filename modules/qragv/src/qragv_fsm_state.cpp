#include <qragv/qragv_state.hpp>



/* Common Moudule State */
// bool SHTM7State::IsIPCInit(SHTM7Main& SHTM7Main_){return SHTM7Main_.OpenAMP_M7_->IsInit();}
// bool SHTM7State::IsETHInit(SHTM7Main& SHTM7Main_){return SHTM7Main_.TcpRtos_->IsInit();}
// bool SHTM7State::IsFatFsInit(SHTM7Main& SHTM7Main_){return SHTM7Main_.FatFs_->IsInit();}
// bool SHTM7State::IsAllInit(SHTM7Main& SHTM7Main_){return SHTM7Main_.OpenAMP_M7_->IsInit() & SHTM7Main_.TcpRtos_->IsInit() & SHTM7Main_.FatFs_->IsInit();}

// bool SHTM7State::IsIPCRun(SHTM7Main& SHTM7Main_){return SHTM7Main_.OpenAMP_M7_->IsRun();}
// bool SHTM7State::IsETHRun(SHTM7Main& SHTM7Main_){return SHTM7Main_.TcpRtos_->IsRun();}
// bool SHTM7State::IsFatFsRun(SHTM7Main& SHTM7Main_){return SHTM7Main_.FatFs_->IsRun();}
// bool SHTM7State::IsAllRun(SHTM7Main& SHTM7Main_){return SHTM7Main_.OpenAMP_M7_->IsRun() && SHTM7Main_.TcpRtos_->IsRun() && SHTM7Main_.FatFs_->IsRun();}

// bool SHTM7State::IsIPCError(SHTM7Main& SHTM7Main_){return SHTM7Main_.OpenAMP_M7_->IsError();}
// bool SHTM7State::IsETHError(SHTM7Main& SHTM7Main_){return SHTM7Main_.TcpRtos_->IsError();}
// bool SHTM7State::IsFatFsError(SHTM7Main& SHTM7Main_){return SHTM7Main_.FatFs_->IsError();}

// bool SHTM7State::IsConnected(SHTM7Main& SHTM7Main_){return SHTM7Main_.TcpRtos_->IsConnected();}
// bool SHTM7State::IsDisconnected(SHTM7Main& SHTM7Main_){return !(SHTM7Main_.TcpRtos_->IsConnected());}


/********************************** Init Status **********************************/
QRAGVState* QRAGVInitState::TransitionState(QRAGV& QRAGV_, int state) 
{
    const int state_ = state;

    switch(state_)
    {
        // case SFMEventNumbering::SFM_EVENT_IS_ALL_RUN:
        //     return new QRAGVPreRunState(); break;

        // case SFMEventNumbering::SFM_EVENT_IS_IPC_ERROR:
        // case SFMEventNumbering::SFM_EVENT_IS_ETH_ERROR:
        // case SFMEventNumbering::SFM_EVENT_IS_FATFS_ERROR:
        //     return new QRAGVErrorState(); break;

        default: break;
    }

    return nullptr;
}

void QRAGVInitState::EnterState(QRAGV& QRAGV_) 
{
    //do something by states
    //run something like that
    /*make all moudule thread*/
    
    /*1. if threads are not empty -> delete all*/
    if(!(QRAGV_.workers_.empty()))
    {
        /*call all threads be out the loops*/
        /*be sure this must use mutex*/
        QRAGV_.quit_sig_ = true;
        
        /*delete all threads*/
        for(auto& worker : QRAGV_.workers_)
            worker.join();
    }

    QRAGV_.quit_sig_ = false;

    /*2. Make all  moudule threads*/


    return;
}

void QRAGVInitState::ExitState(QRAGV& QRAGV_) 
{
    //do something by states
    //stop something like that

    
    //Initiaiize done

    return ;
}

void QRAGVInitState::StateBehavior(QRAGV& QRAGV_)
{
    

    return;
}



// /********************************** Prerun Status **********************************/
// SHTM7State* SHTPreRunState::TransitionState(SHTM7Main& SHTM7Main_, int state) 
// {
//     const int state_ = state;

//     switch(state_)
//     {
//         case SFMEventNumbering::SFM_EVENT_IS_CONNECTED:
//             return new SHTRunState(); break;

//         case SFMEventNumbering::SFM_EVENT_IS_IPC_ERROR:
//         case SFMEventNumbering::SFM_EVENT_IS_ETH_ERROR:
//         case SFMEventNumbering::SFM_EVENT_IS_FATFS_ERROR:
//             return new SHTErrorState(); break;

//         default: break;
//     }

//     return nullptr;
// }

// void SHTPreRunState::EnterState(SHTM7Main& SHTM7Main_) 
// {
//     //do something by states
//     //run something like that
// 	Dprintf("Shuttle Prerun Start!\r\n");

// 	if(SHTM7Main_.data_sema_ && osSemaphoreWait(*(SHTM7Main_.data_sema_), osWaitForever) == osOK)
// 	{
// 		WriteDataToMainData(SHTM7Main_.Dst_, RobotDataId::ETH_IS_CONNECTED, SHTM7Main_.TcpRtos_->IsConnected());

//         WriteDataToMainData(SHTM7Main_.Dst_, RobotDataId::INIT_STATUS, RobotResult::RESULT_SUCCESS);

// 		osSemaphoreRelease(*(SHTM7Main_.data_sema_));
// 	}

//     return ;
// }

// void SHTPreRunState::ExitState(SHTM7Main& SHTM7Main_) 
// {
// 	Dprintf("Shuttle Prerun Done!\r\n");

//     //do something by states
//     //stop something like that
//     return ;
// }

// void SHTPreRunState::StateBehavior(SHTM7Main& SHTM7Main_)
// {
    
//     /*if fatfs init is done -> do ipc init*/
//     /*if ipc init is done -> do eth init*/  
//     /*if eth init is done -> get ready for transition state*/  

    
//     return;
// }


// /********************************** SHTRunState Status **********************************/
// SHTM7State* SHTRunState::TransitionState(SHTM7Main& SHTM7Main_, int state) 
// {
//     const int state_ = state;

//     switch(state_)
//     {
//         case SFMEventNumbering::SFM_EVENT_IS_DISCONNECTED:
//             return new SHTPreRunState(); break;

//         case SFMEventNumbering::SFM_EVENT_IS_IPC_ERROR:
//         case SFMEventNumbering::SFM_EVENT_IS_ETH_ERROR:
//         case SFMEventNumbering::SFM_EVENT_IS_FATFS_ERROR:
//             return new SHTErrorState(); break;

//         default: break;
//     }

//     return nullptr;
// }

// void SHTRunState::EnterState(SHTM7Main& SHTM7Main_) 
// {
// 	Dprintf("Shuttle is ready to operate!\r\n");

// 	if(SHTM7Main_.data_sema_ && osSemaphoreWait(*(SHTM7Main_.data_sema_), osWaitForever) == osOK)
// 	{
// 		WriteDataToMainData(SHTM7Main_.Dst_, RobotDataId::ETH_IS_CONNECTED, SHTM7Main_.TcpRtos_->IsConnected());

// 		osSemaphoreRelease(*(SHTM7Main_.data_sema_));
// 	}

//     //do something by states
//     //run something like that
//     this->odom_save_timer = 0;
//     this->backup_save_timer = 0;

//     return ;
// }

// void SHTRunState::ExitState(SHTM7Main& SHTM7Main_) 
// {
//     //do something by states
//     //stop something like that

//     return ;
// }

// void SHTRunState::StateBehavior(SHTM7Main& SHTM7Main_)
// {
//     uint32_t now_time_ = osKernelSysTick();

//     if(now_time_ - this->odom_save_timer > this->odom_save_time)
//     {
//         /*add queue for fatfs odometry*/
//         SHTM7Main_.FatFs_->FatFsCommandWritingOdometry();

//         this->odom_save_timer = now_time_;
//     }

//     if(now_time_ - this->backup_save_timer > this->backup_save_time)
//     {
//         /*add queue for fatfs odometry*/
//         SHTM7Main_.FatFs_->FatFsCommandWritingBackup();

//         this->backup_save_timer = now_time_;
//     }


//     return;
// }



// /********************************** SHTErrorState Status **********************************/
// SHTM7State* SHTErrorState::TransitionState(SHTM7Main& SHTM7Main_, int state) 
// {
//     const int state_ = state;

//     switch(state_)
//     {
//         // case SFMEventNumbering::SFM_EVENT_IS_DISCONNECTED:
//         //     return new SHTRunState(); break;

//         // case SFMEventNumbering::SFM_EVENT_IS_IPC_ERROR:
//         // case SFMEventNumbering::SFM_EVENT_IS_ETH_ERROR:
//         // case SFMEventNumbering::SFM_EVENT_IS_FATFS_ERROR:
//         //     return new SHTErrorState(); break;

//         default: break;
//     }

//     return nullptr;
// }

// void SHTErrorState::EnterState(SHTM7Main& SHTM7Main_) 
// {
//     //do something by states
//     //run something like that

// 		Dprintf("Shuttle M7 Error Occur!!\n");

// 	if(SHTM7Main_.data_sema_ && osSemaphoreWait(*(SHTM7Main_.data_sema_), osWaitForever) == osOK)
// 	{
//         if(SHTM7Main_.TcpRtos_->IsError())
// 		    WriteDataToMainData(SHTM7Main_.Dst_, RobotDataId::ETH_ERROR, SHTM7Main_.TcpRtos_->IsError());

//         if(SHTM7Main_.OpenAMP_M7_->IsError())
//         {
//             WriteDataToMainData(SHTM7Main_.Dst_, RobotDataId::OPENAMP_ERROR, SHTM7Main_.OpenAMP_M7_->IsError());

//             WriteDataToMainData(SHTM7Main_.Dst_, RobotDataId::ERROR_CODE, 10002);
//         }
		    
//         if(SHTM7Main_.FatFs_->IsError())
//         {
//             WriteDataToMainData(SHTM7Main_.Dst_, RobotDataId::FATFS_ERROR, SHTM7Main_.FatFs_->IsError());

//             WriteDataToMainData(SHTM7Main_.Dst_, RobotDataId::ERROR_CODE, 10002);
//         }
		 
// 		osSemaphoreRelease(*(SHTM7Main_.data_sema_));
// 	}

//     // SHTM7Main_.FatFs_->WritingShuttleErrorReport();    

//     return ;
// }

// void SHTErrorState::ExitState(SHTM7Main& SHTM7Main_) 
// {
//     //do something by states
//     //stop something like that
//     return ;
// }

// void SHTErrorState::StateBehavior(SHTM7Main& SHTM7Main_)
// {
//     /* Exeception behavior of m4 Dead error */
// 	if(SHTM7Main_.data_sema_ && osSemaphoreWait(*(SHTM7Main_.data_sema_), osWaitForever) == osOK)
// 	{
//         if(SHTM7Main_.OpenAMP_M7_->IsError() && !(this->reset_flag_))
                
//             this->reset_flag_ = true;

// 		osSemaphoreRelease(*(SHTM7Main_.data_sema_));
// 	}

//     if( this->reset_flag_ && osKernelSysTick() - this->reset_timer_ > this->reset_time_)
//     {
//         /* Reset System */
//         NVIC_SystemReset();
//     }

//     return;
// }
/**
  ******************************************************************************
  * @file           : pgv100.cpp
  * @brief          : Main Interface cpp file of PGV100 Position Sensor
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


#include "pgv100/pgv100.hpp"


pgv100::pgv100(){}

pgv100::pgv100(size_t sesnor_id)
{
  this->sesnor_id_ = sesnor_id;
}

pgv100::pgv100(size_t sesnor_id, size_t pos_unit)
{
  this->sesnor_id_ = sesnor_id;

  this->pos_unit_ = pos_unit;
}




pgv100::~pgv100()
{
  Uartclose(USE_UART_PORT);
}

kssbot_hardware::IReturnType pgv100::Init()
{
  if(this->is_hw_init_)
    return kssbot_hardware::IReturnType::kReturnOk;

  if(this->HardwareInitiailize() != kssbot_hardware::IReturnType::kReturnOk)
    return kssbot_hardware::IReturnType::kReturnError;

  this->is_hw_init_ = true;

  this->state_ = kssbot_hardware::LifeCycleState::kInit;

  if(this->is_init_)
    return kssbot_hardware::IReturnType::kReturnOk;

  if(this->SystemInitiailize() != kssbot_hardware::IReturnType::kReturnOk)
    return kssbot_hardware::IReturnType::kReturnError;

  this->is_init_ = true;


  int status = 0;

  int cnt = 0;

  while(status < 0)
  {
    status = this->SystemOperation();

    usleep(100*1000);

    if(cnt++ > 10)
      break;
  }


  this->state_ = kssbot_hardware::LifeCycleState::kRun;

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType pgv100::StartRun()
{
  this->is_run_ = true;

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType pgv100::StopRun()
{
  this->is_stop_ = true;

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType pgv100::Write()
{
  if(this->req_tele_queue_.empty())
    this->EnqueueRequestTelegram(PNFPGV100RequestTelegram::cPGV100PosRequest);

  const unsigned char req_tele_cmd = (this->req_tele_queue_.front()) + this->sesnor_id_;

  unsigned char write_buffer[3] = {req_tele_cmd, ((unsigned char)~(req_tele_cmd)), '\0'};

  size_t response_data_length = this->GetResponseDataLength(req_tele_cmd);

  if(response_data_length == 0)
    return kssbot_hardware::IReturnType::kReturnError;

  if(sendUart(USE_UART_PORT, 
              write_buffer, 
              sizeof(write_buffer)/sizeof(unsigned char)) != kssbot_hardware::IReturnType::kReturnOk)
    return kssbot_hardware::IReturnType::kReturnError;

  unsigned char recv_buffer[UART_MAX_BUF_SIZE] = {0,};

  if(readUartWithTimeout(USE_UART_PORT, recv_buffer, response_data_length, READ_TIMEOUT_MICRO) != kssbot_hardware::IReturnType::kReturnOk)
    return kssbot_hardware::IReturnType::kReturnError;

  this->recv_stream_.append((char*)recv_buffer, response_data_length);

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType pgv100::Read()
{



  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType pgv100::Drive()
{
  const kssbot_hardware::LifeCycleState now_state = this->state_;

  switch(now_state)
  {
    case kssbot_hardware::LifeCycleState::kHwInit:
    {
      if(this->HardwareInitiailize() == kssbot_hardware::IReturnType::kReturnError)
        this->state_ = kssbot_hardware::LifeCycleState::kError;

      else
        this->state_ = kssbot_hardware::LifeCycleState::kInit;

      break;
    }

    case kssbot_hardware::LifeCycleState::kInit:
    {
      if(this->SystemInitiailize() == kssbot_hardware::IReturnType::kReturnError)
        this->state_ = kssbot_hardware::LifeCycleState::kError;
      
      else
        this->state_ = kssbot_hardware::LifeCycleState::kRun;

      break;
    }

    case kssbot_hardware::LifeCycleState::kRun:
    {
      if(this->SystemOperation() == kssbot_hardware::IReturnType::kReturnError)
        this->state_ = kssbot_hardware::LifeCycleState::kError;
      
      else
        this->state_ = kssbot_hardware::LifeCycleState::kRun;

      break;


    }

    case kssbot_hardware::LifeCycleState::kStop:
    {

      break;
    }

    case kssbot_hardware::LifeCycleState::kError:
    {

      printf("Error\r\n");

      break;
    }

    default: 
    {
      this->state_ = kssbot_hardware::LifeCycleState::kError;

      break;
    }


  }




  return kssbot_hardware::IReturnType::kReturnOk;
}


kssbot_hardware::IReturnType pgv100::HardwareInitiailize()
{
  if(initUARTPort(USE_UART_PORT,                                //port
                  B115200,                            //baudrate
                  UART_STOP_BIT_1,                    //stopbit
                  UART_DATA_BIT_8,                    //databit
                  UART_HWFLOW_CONTROL_DISABLE,        //hwflow
                  UART_CONONICAL_MODE_DISABLE,        //canonicalmode
                  UART_PARATY_EVEN)                  //paraty)
                  )     
    return kssbot_hardware::IReturnType::kReturnError;

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType pgv100::SystemInitiailize()
{
  this->sensor_filter_val_ = MAX_FILTER_CNT;

  this->req_tele_queue_.clear();

  this->EnqueueRequestTelegram(PNFPGV100RequestTelegram::cPGV100DirStraightRequest);

  this->EnqueueRequestTelegram(PNFPGV100RequestTelegram::cPGV100PosRequest);

  return kssbot_hardware::IReturnType::kReturnOk;
}

kssbot_hardware::IReturnType pgv100::SystemOperation()
{
  if(this->filter_cnt_ >= this->sensor_filter_val_)
  { 
    filter_cnt_= 0;

    this->state_ = kssbot_hardware::LifeCycleState::kError;
  }

  if(this->Write() != kssbot_hardware::IReturnType::kReturnOk)
  {
    this->filter_cnt_++;

    return kssbot_hardware::IReturnType::kReturnAlarm;
  }

  if(this->ResponseTelegramParse() != kssbot_hardware::IReturnType::kReturnOk)
  {
    this->filter_cnt_++;

    return kssbot_hardware::IReturnType::kReturnAlarm;
  }

  this->filter_cnt_ = 0;

  this->recv_stream_.clear();

  if(!(this->req_tele_queue_.empty()))
    this->req_tele_queue_.erase(this->req_tele_queue_.begin());


  return kssbot_hardware::IReturnType::kReturnOk;
}



kssbot_hardware::IReturnType pgv100::ResponseTelegramParse()
{
  if(this->req_tele_queue_.empty())
    return kssbot_hardware::IReturnType::kReturnAlarm;

  const unsigned char req_tele_cmd = this->req_tele_queue_.front();

  size_t response_data_length = this->GetResponseDataLength(req_tele_cmd);

  /*Check Data length */
  if(this->recv_stream_.length() != response_data_length)
    return kssbot_hardware::IReturnType::kReturnError;


  /*check crc*/
  char crc_data = this->recv_stream_.back();

  this->recv_stream_.erase(this->recv_stream_.end() -1);

  char temp = 0; 
  size_t ChkSum_Data = 0;
  char even_cnt[8]={0,};

  for(uint16_t i=0; i<8; i++)
  {
    for (auto& index : this->recv_stream_)
    {
      temp = index;

      if((temp>>i)&0x01) even_cnt[i]+=1;//8bit, even

      temp = 0;
    }
    if(even_cnt[i]!=0) ChkSum_Data |= (even_cnt[i]%2) * (1<<i);
  }

  if(ChkSum_Data != (size_t)crc_data)
  {
    printf("crc error\r\n");

    return kssbot_hardware::IReturnType::kReturnError;
  }
   
  /*Parsing data */
  switch(req_tele_cmd)
  {
    case PNFPGV100RequestTelegram::cPGV100DirRightRequest:
    case PNFPGV100RequestTelegram::cPGV100DirLeftRequest:
    case PNFPGV100RequestTelegram::cPGV100DirStraightRequest:
    {
      /*Dir command parsing*/
      this->pgv100_dir_ = this->recv_stream_.at(1) & 0x03;

      break;
    }

    case PNFPGV100RequestTelegram::cPGV100ColorRedRequest:
    case PNFPGV100RequestTelegram::cPGV100ColorGreenRequest:
    case PNFPGV100RequestTelegram::cPGV100ColorBlueRequest:
    {
      /*Color select command*/
      if((this->recv_stream_.at(0) & 0x07) && (this->recv_stream_.at(1) & 0x07))
        this->pgv100_color_ = this->recv_stream_.at(1) & 0x07;

      break;
    }
    
    case PNFPGV100RequestTelegram::cPGV100PosRequest:
    {

      /*Check error status */
      if(this->recv_stream_.at(0) & 0x01)
      {
        size_t error_code = this->recv_stream_.at(5);

        error_code |= (this->recv_stream_.at(4) << 7);
        error_code |= (this->recv_stream_.at(3) << 14);
        error_code |= ((this->recv_stream_.at(2) & 0x07) << 21);

        this->error_code_ = error_code;

        this->is_error_ = true;

        return kssbot_hardware::IReturnType::kReturnError;
      }

      /*check tag*/
      if(this->recv_stream_.at(1) & 0x40)
      {
        size_t tag_code = this->recv_stream_.at(17);

        tag_code |= (this->recv_stream_.at(16) << 7);
        tag_code |= (this->recv_stream_.at(15) << 14);
        tag_code |= (this->recv_stream_.at(14) << 21);


        this->is_tagged_ = true;

        this->tag_code_ = tag_code;
      }
      else
      {
        this->is_tagged_ = false;  
      }

      /*check position lane*/
      if(this->recv_stream_.at(0) & 0x02)
      {
        this->is_detected_ = false;
      }
      else
      {
        this->is_detected_ = true;  
      }

      if(!(this->is_tagged_) && !(this->is_detected_))
        return kssbot_hardware::IReturnType::kReturnOk;

      /*posistion parsing*/
	    int angle = ((this->recv_stream_.at(11)));

	    (angle) |= ((this->recv_stream_.at(10) << 7));

      this->angle_ = angle;


      int xpos = (this->recv_stream_.at(5));

      xpos |= (this->recv_stream_.at(4) << 7);

      xpos |= (this->recv_stream_.at(3) << 14);

      xpos |= ((this->recv_stream_.at(2) & 0x07) << 21);

      if(xpos > pow(2, 24 - 1) && this->is_tagged_)  //24bits signed if tagged but lane is 24bits unsigned
        xpos = xpos - pow(2, 24); 

      this->xpos_ = xpos;



      int ypos = (this->recv_stream_.at(7));

      ypos |= (this->recv_stream_.at(6) << 7);

      if(ypos > pow(2, 14 - 1))
        ypos = ypos - pow(2, 14);   //14bits signed

      this->ypos_ = ypos;

      break;
    }
    
    default:  
    {

      /*Unknown command : delete all stream*/

      break;
    }
  }


  return kssbot_hardware::IReturnType::kReturnOk;
}

/****************************************************************** */
size_t pgv100::GetResponseDataLength(const size_t request_telegram)
{
  size_t r_data_length;

  switch(request_telegram)
  {
    case PNFPGV100RequestTelegram::cPGV100DirRightRequest:
    case PNFPGV100RequestTelegram::cPGV100DirLeftRequest:
    case PNFPGV100RequestTelegram::cPGV100DirStraightRequest:
    {
      r_data_length = PNFResponseDataLength::pPGV100Dir;

      break;
    }

    case PNFPGV100RequestTelegram::cPGV100ColorRedRequest:
    case PNFPGV100RequestTelegram::cPGV100ColorGreenRequest:
    case PNFPGV100RequestTelegram::cPGV100ColorBlueRequest:
    {
      r_data_length = PNFResponseDataLength::pPGV100Color;

      break;
    }
    
    case PNFPGV100RequestTelegram::cPGV100PosRequest:
    {
      r_data_length = PNFResponseDataLength::pPGV100Pos;

      break;
    }
    
    default: r_data_length = 0; break;
  }

  return r_data_length;
}

void pgv100::EnqueueRequestTelegram(size_t request_telegram)
{
  this->req_tele_queue_.push_back(request_telegram);

  return;
}




/*********************************************************/

PosSensorDataStruct pgv100::GetPGV100DataStructure()
{
    PosSensorDataStruct pgv100_data;

    pgv100_data.is_detected = this->IsLineDetected();
    pgv100_data.is_tagged = this->IsTagged();

    pgv100_data.tag_code = this->GetTagCode();
    pgv100_data.error_code = this->GetErrorCode();
    pgv100_data.pgv100_dir = this->GetTagCode();
    pgv100_data.pgv100_color = this->GetPgv100Color();
    pgv100_data.xpos = this->GetXPos();
    pgv100_data.ypos = this->GetYPos();
    pgv100_data.angle = this->GetAngle();

    pgv100_data.status = this->state_;

  return pgv100_data;
}
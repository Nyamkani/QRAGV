#include "isv2motor/jetson_can.h"


#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>



/*#Note: there is no function to select CAN TXn chan or RXn chan*/

/*prevent duplicate initialize */
bool is_init_ = false;

/* W/R fd*/
static int SPI_init;

/* MCP2515 VARIABLES */


/* Private */
static int initSPIdev()
{
  int Init;

  Init = gpioInitialise();
  
  if (Init < 0)
  {
    /* jetgpio initialisation failed */
    printf("Jetgpio initialisation failed. Error code:  %d\n", Init);
    
    return -1;
  }
  else
  {
    /* jetgpio initialised okay*/
    printf("Jetgpio initialisation OK. Return code:  %d\n", Init);
  }

  SPI_init = spiOpen(SPI_CHAN, 
                      SPI_FREQ, 
                      SPI_MODE, 
                      SPI_CS_DELAY, 
                      SPI_BIT_WORDS, 
                      SPI_LSB_FIRST, 
                      SPI_CS_CAHNGE);

  if (SPI_init < 0)
  {
    /* Port SPI opening failed */
    printf("Port SPI opening failed. Error code:  %d\n", SPI_init);

    return -2;
  }
  else
  {
    /* Port SPI opened  okay*/
    printf("Port SPI opened OK. Return code:  %d\n", SPI_init);
  }

  return 0;
}

static int readByte(size_t addr, char* readdata)
{
  char senddata[] = {CAN_READ, addr, 0};

  return spiXfer(SPI_init, senddata, readdata, sizeof(senddata)/sizeof(char));
}

static int writeByte(size_t addr, size_t data, char* readdata)
{
  char senddata[] = {CAN_WRITE, addr, data};

  return spiXfer(SPI_init, senddata, readdata, sizeof(senddata)/sizeof(char));
}

static int reset()
{
  char senddata = CAN_RESET, dummy = 0;

  return spiXfer(SPI_init, &senddata, &dummy, sizeof(senddata)/sizeof(char));
}

static int initMCP2515()
{ 
  reset();

  printf("Reset MCP2515\r\n");

  usleep(100*1000);

  char dummy[8] = {0,}, status[8] = {0,};

  /*control the bit timing for the CAN bus interface*/
  //Config. 1 baudrate bit([5:0], SJW[7:6])  Tq = [2*(Br + 1)]/Foc = 
  writeByte(CNF1, CAN_250Kbps, dummy);

  //Config. 2 propagation segament bit([2:0]), phase segement1([5:3])
  //number of check bit at sample point [6], config. ps2 bit[7]
  writeByte(CNF2, 0x80|PHSEG1_3TQ|PRSEG_1TQ, dummy);

  //config. 3 phase segment2[2:0], SOF[7] -> CANCTRL will control
  writeByte(CNF3, PHSEG2_3TQ, dummy);

  //set tid configuartion(High <<3 | low )
  //set TXB0
  // writeByte(TXB0SIDH, 0xB2, dummy);
  // writeByte(TXB0SIDL, 0x00, dummy);
  // writeByte(TXB0DLC,  DLC_8, dummy);    //Set DLC = 3 bytes and RTR bit*/

  //set TXB1
  // writeByte(TXB1SIDH, 0xC0, dummy);    //Set TXB0 SIDH  50
  // writeByte(TXB1SIDL, 0x00, dummy);    //Set TXB0 SIDL  00
  // writeByte(TXB1DLC,  DLC_8, dummy);    //Set DLC = 3 bytes and RTR bit*/

  //set RXB0
  // writeByte(RXB0SIDH, 0x00 ,dummy);
  // writeByte(RXB0SIDL, 0x60, dummy);
  
  writeByte(RXB0CTRL, 0x60, dummy); // Turns mask/filters off; receives any message
  // writeByte(RXB0DLC, DLC_8, dummy);

  //due to RXB0CTRL no fiter/mask below are ignore
  //rx filter
  writeByte(RXF0SIDH, 0xFF, dummy);
  writeByte(RXF0SIDL, 0xE0, dummy);

  //rx mask
  writeByte(RXM0SIDH, 0xFF, dummy);
  writeByte(RXM0SIDL, 0xE0, dummy);

  //set interrupt
  writeByte(CANINTF, 0x00, dummy);
  writeByte(CANINTE, RX0IE_ENABLED | TX1IE_ENABLED, dummy);

  //set mode 
  writeByte(CANCTRL, REQOP_NORMAL | CLKOUT_ENABLED, dummy);

  readByte(CANSTAT, status);

  if( OPMODE_NORMAL != (status[2] & 0xE0))
    writeByte(CANCTRL, REQOP_NORMAL|CLKOUT_ENABLED, dummy);

  return 0;
}


/*Public*/
int MCP2515_CANinit()
{
  if (is_init_)
    return 0;

  if(initSPIdev() < 0)
    return -1;

  if(initMCP2515() < 0)
    return -1;

  is_init_ = true;

  return 0;
}

int MCP2515_CANsend(CAN_data_struct sendcmd)
{
  char dummy[8] = {0,}, stream[8] = {0,};

  char ctrl = TXB1CTRL;
  char dlc = TXB1DLC;
  char buf = TXB1D0;
  char rtx = CAN_RTS_TXB1;

  char tid_h = (sendcmd.id >> 3) & 0xff;
  char tid_l = (sendcmd.id & 0x07) << 5;

  char data_length = sendcmd.data_length;
  char send_buffer[data_length];

  clock_t prev_time, now_time;

  //0. get data from staructure
  memcpy(send_buffer, sendcmd.data, data_length);

  //1. check the TXREQ is SET. if SET -> wait 
  //writeByte(ctrl, 0, dummy);
  
  //2. load send data's length and id
  writeByte(TXB1SIDH, tid_h, dummy);

  writeByte(TXB1SIDL, tid_l, dummy);

  writeByte(dlc, data_length, dummy);

  //3. load send data to mcp2515 buffer
  for(int i = 0; i< data_length; i++)
    writeByte(buf + i, send_buffer[i], dummy);

  //5. check the TXREQ

  readByte(ctrl, stream);

  if((stream[2] & TXREQ_SET) == TXREQ_SET)
  {
    // usleep(1);

    writeByte(ctrl, 0, dummy);
  }
   

  prev_time = clock();

  do
  {
    readByte(ctrl, stream);

    // usleep(1);

    now_time = clock();

    if(now_time - prev_time > MAX_TIME_OUT)
        return -10;
  }
  while((stream[2] & TXREQ_SET) == TXREQ_SET);



  //4. send data request
  char tempdata = rtx;

  if(spiXfer(SPI_init, &tempdata, dummy, 1)<0)
	printf("spiXfer error\r\n");

  prev_time = clock();

  do
  {
    readByte(ctrl, stream);

    // usleep(1);

    now_time = clock();

    // if(now_time - prev_time > MAX_TIME_OUT)
    //     return -11;
  }
  while((stream[2] & TXREQ_SET) == TXREQ_SET);

  #if CANDPRINT  
    printf("Post status = %x\r\n", stream[2]);   //debug
  #endif

  

  //6. check error or missing data
  readByte(ctrl, stream);

  //check lost arbitaration
  //if(stream[2] & 0x20)
  //  return -1;

  //check send msg lost
  if(stream[2] & 0x10)
    return -2;


  return 0;
}

int MCP2515_CANRecv(CAN_data_struct* recvcmd)
{
  char stream[8] = {0,}, recvdata[8] = {0,}, dummy[8] = {0,};
  char data_length_ = 0;
  char data_[2] = {0,};
  int id_ = 0;

  clock_t prev_time, now_time;

  /*#Notice : here no mask, no filter. so we skip check mask and filters*/

  //1. read interrupt flag
  prev_time = clock();

  while((stream[2] & RX0IF_SET) != RX0IF_SET)
  {
    now_time = clock();

    readByte(CANINTF, stream);
    
    if(now_time - prev_time > MAX_TIME_OUT)
      return 0; //->timeout
  }

  //2. check buffer is null
  if(recvcmd == NULL)
    return -30;

  //3. if flag up. read data length and tid
  readByte(RXB0DLC, stream);

  data_length_ = stream[2];

  if(data_length_ < 0 || data_length_> 8)
    return -21;

  readByte(RXB0SIDH, stream);
  
  data_[0] = stream[2];

  readByte(RXB0SIDL, stream);

  data_[1] = stream[2];

  id_ =  (data_[0] << 3) | (data_[1] >> 5); //get tid

  //4. read data contents
  for(int i = 0; i < data_length_; i++)
  {
    readByte(RXB0D0 + i, stream);

    recvdata[i] = stream[2];
  }

  //5. input data to structure
  recvcmd->id = id_;
  recvcmd->data_length = data_length_;
  memcpy(recvcmd->data, recvdata, data_length_);


  #if CANDPRINT
    printf("Recved data id  = %x\r\n", recvcmd->id );   //debug
    printf("Recved data length  = %d\r\n", recvcmd->data_length );   //debug

    for(int i = 0; i < 8; i++)
      printf("%x, ",*((recvcmd->data) + i));

    printf("\r\n");
  #endif
    

  //6. resetting recv interrupt
  writeByte(CANINTF, 0, dummy);
  writeByte(CANINTE, 0x01, dummy);
  
  //7. clear prev recved tid and length
  writeByte(RXB0SIDH, 0x00, dummy);
  writeByte(RXB0SIDL, 0x00, dummy);
  writeByte(RXB0DLC, 0x00, dummy);

  return 0;
}

int MCP2515_CANDeinit()
{
  if(!(is_init_))
    return 0;
  
  spiClose(SPI_init);

  gpioTerminate();

  printf("CAN and SPI Closed.\r\n");

  is_init_ = false;

  return 0;
}



CAN_data_struct MakingSendCANDataStruct(int id_type, int id, int data_length, int index, int sub_index, int data)
{
  CAN_data_struct send_data = {0,};

  switch(id_type)
  {
    case tNMT:
    {
      int node_id = 0x0000 + id;

      send_data.id = node_id;
      send_data.data_length = data_length;
      send_data.data[0] = index;
      send_data.data[1] = sub_index;

    printf("id = %x, index a= %x, index b = %x\r\n", send_data.id, (send_data.data[0]), send_data.data[1] );

      break;
    }
    
    case tWSDO:
    {
      /*id */
      int node_id = 0x600 + id;

      /*data length value calculate*/
      char data_length_val = 0x22;

      // if (data <= 0xff) //1byte
      //   data_length_val = 0x2f;
      // else if(data <= 0xffff)  //22bytes
      //   data_length_val = 0x2b;
      // else if(data <= 0xffffffff)
      //   data_length_val = 0x23;

      /*index */
      char send_index[2] = {0,};

      for(int i = 0; i < 2; i++)
        send_index[i] = (index >> (8*i)) & 0xff;

      /*data */
      char send_data_val[4] = {0,};

      for(int i = 0; i < 4; i++)
        send_data_val[i] = (data >> (8*i)) & 0xff;

      send_data.id = node_id;
      send_data.data_length = data_length;
      send_data.data[0] = data_length_val;
      send_data.data[1] = send_index[0];
      send_data.data[2] = send_index[1];
      send_data.data[3] =  (char)sub_index;
      send_data.data[4] =  send_data_val[0];
      send_data.data[5] =  send_data_val[1];
      send_data.data[6] =  send_data_val[2];
      send_data.data[7] =  send_data_val[3];


      break;
    }
    
    case tRSDO:
    {
      /*id */
      int node_id = 0x600 + id;

      /*data length value calculate*/
      char data_length_val = 0x40;

      // if (data <= 0xff) //1byte
      //   data_length_val = 0x2f;
      // else if(data <= 0xffff)  //22bytes
      //   data_length_val = 0x2b;
      // else if(data <= 0xffffffff)
      //   data_length_val = 0x23;

      /*index */
      char send_index[2] = {0,};

      for(int i = 0; i < 2; i++)
        send_index[i] = (index >> (8*i)) & 0xff;

      /*data */
      char send_data_val[4] = {0,};

      for(int i = 0; i < 4; i++)
        send_data_val[i] = (data >> (8*i)) & 0xff;

      send_data.id = node_id;
      send_data.data_length = data_length;
      send_data.data[0] = data_length_val;
      send_data.data[1] = send_index[0];
      send_data.data[2] = send_index[1];
      send_data.data[3] =  (char)sub_index;
      send_data.data[4] =  0;
      send_data.data[5] =  0;
      send_data.data[6] =  0;
      send_data.data[7] =  0;

      break;
    }

    default: break;

  }

  return send_data;
}





























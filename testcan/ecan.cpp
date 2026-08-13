#include "ecan.hpp"

ECANInterface::ECANInterface(){}

ECANInterface::ECANInterface(char* addr, int port)
{
    this->addr_.append(addr);

    this->port_ = port;

    
}

ECANInterface::ECANInterface(char* addr, int port, int send_frame_type)
{
    this->addr_.append(addr);

    this->port_ = port;

    this->send_frame_type_ = send_frame_type;
}


ECANInterface::~ECANInterface(){}



int ECANInterface::Initialize()
{
    if(this->addr_.empty() || this->port_ == 0)
        return -1;

    struct sockaddr_in addr;

	const char* server_addr = this->addr_.c_str();

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(server_addr);
	addr.sin_port = htons(this->port_);

	if(connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
	{
		perror("Connect failed.\r\n");
		return -1;
	}

    return 0;
}


int ECANInterface::Send(const CANDataStructure can_data)
{
    char data_buf[ECAN_DATA_FRAME_LENGTH];

    memset(data_buf, 0, sizeof(data_buf));

    //set data Frame type
    data_buf[ECAN_DATA_FRAME_TYPE_NUMBER] = this->send_frame_type_;

    //Set data Frame id 
    for(int i = ECAN_DATA_FRAME_ID_NUMBER; i < ECAN_DATA_FRAME_LENGTH_NUMBER; i++ )
        data_buf[i] = (((long long)can_data.id >> ((ECAN_DATA_FRAME_LENGTH_NUMBER - ECAN_DATA_FRAME_ID_NUMBER)-i) * 8) & 0xff);

    //Set data Frame length 
    data_buf[ECAN_DATA_FRAME_LENGTH_NUMBER] = (can_data.data_length);

    //Set data Frame data
    for(int i = ECAN_DATA_FRAME_DATA_NUMBER; i < ECAN_DATA_FRAME_LENGTH; i++ )
        data_buf[i] = can_data.data[i - ECAN_DATA_FRAME_DATA_NUMBER];


    //send data to connected socket
	if(send(sockfd, &data_buf,  sizeof(data_buf), 0) < 0)
	{
		perror("send");
		return -1;
	}

    return 0;    
}

int ECANInterface::Recv(CANDataStructure& can_data)
{
	char data_buf[ECAN_DATA_FRAME_LENGTH] = {0,};

    memset(data_buf, 0, sizeof(data_buf));

	if(recv(sockfd, data_buf, ECAN_DATA_FRAME_LENGTH, 0) <0)
	{
		perror("recv");
		return -1;
	}

    //Get data Frame type
    int ECAN_data_type = data_buf[ECAN_DATA_FRAME_TYPE_NUMBER];
    int id = 0;
    int data_length = 0;
    int data[8];

    memset(data, 0, sizeof(data));

    //Get data Frame id 
    for(int i = ECAN_DATA_FRAME_ID_NUMBER; i < ECAN_DATA_FRAME_LENGTH_NUMBER; i++ )
        id +=  data_buf[i] << ((((ECAN_DATA_FRAME_LENGTH_NUMBER - ECAN_DATA_FRAME_ID_NUMBER)-i) * 8) & 0xff);

    //Get data Frame length 
    data_length =  data_buf[ECAN_DATA_FRAME_LENGTH_NUMBER];

    //Get data Frame data
    for(int i = ECAN_DATA_FRAME_DATA_NUMBER; i < ECAN_DATA_FRAME_LENGTH; i++ )
        data[i - ECAN_DATA_FRAME_DATA_NUMBER] = data_buf[i];

    can_data.id = id;
    can_data.data_length = data_length;

    for(int i = 0; i<8; i++)
        can_data.data[i] = data[i];

    return 0;    
}



int ECANInterface::Write(const CANDataStructure can_data)
{
    // int cmd = can_data.data





    return 0;    
}

int ECANInterface::Read()
{
    


    return 0;    
}


CANDataStructure MakingSendCANDataStruct(int id_type, int id, int data_length, int index, int sub_index, int data)
{
  CANDataStructure send_data = {0,};

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











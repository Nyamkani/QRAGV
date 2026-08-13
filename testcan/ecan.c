
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>


#include <linux/can.h>
#include <linux/can/raw.h>

int main(int argc, char **argv)
{
	// int s; 
	// struct sockaddr_can addr;
	// struct ifreq ifr;
	// struct can_frame frame;

	// printf("CAN Sockets Demo\r\n");

	// if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
	// 	perror("Socket");
	// 	return 1;
	// }

	// strcpy(ifr.ifr_name, "vcan0" );
	// ioctl(s, SIOCGIFINDEX, &ifr);

	// memset(&addr, 0, sizeof(addr));
	// addr.can_family = AF_CAN;
	// addr.can_ifindex = ifr.ifr_ifindex;

	// if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
	// 	perror("Bind");
	// 	return 1;
	// }

	// frame.can_id = 0x555;
	// frame.can_dlc = 5;
	// sprintf(frame.data, "Hello");

	// if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
	// 	perror("Write");
	// 	return 1;
	// }

	// if (close(s) < 0) {
	// 	perror("Close");
	// 	return 1;
	// }


	struct sockaddr_in addr;
	int sockfd;
	// struct can_frame frame;


	if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("Socket");
		return -1;
	}

	char* server_addr = "192.168.0.223";

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	// inet_pton(AF_INET, server_addr, &addr.sin_addr);
	addr.sin_addr.s_addr = inet_addr(server_addr);
	addr.sin_port = htons(4001);

	if(connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
	{
		perror("Connect failed.\r\n");
		return -1;
	}
	printf("Coneected\r\n");


	// frame.can_id = 0x555;
	// frame.can_dlc = 5;
	// frame.data[0] = 0x12;
	// frame.data[1] = 0x34;s
	// sprintf(frame.data, "Hello");

	char can_data[14] = {0,};
	can_data[0] = 0x04; //data type 
	can_data[1] = 0x00; //id 1
	can_data[2] = 0x00; //id 2
	can_data[3] = 0x05; //id 3 
	can_data[4] = 0x80; //id 4
	can_data[5] = 0x08; //dlc 
	can_data[6] = 0x01; //data 1
	can_data[7] = 0x23; //data 2
	can_data[8] = 0x45; //data 3

	if(send(sockfd, &can_data,  sizeof(can_data), 0) < 0)
	{
		perror("send");
		return -1;
	}

	printf("send\r\n");

	usleep(100);

	char recvmsg[30] = {0,};

	if(recv(sockfd, recvmsg, 14, 0) <0)
	{
		perror("recv");
		return -1;
	}

	printf("recv : ");

	for(int i =0; i<sizeof(recvmsg); i++)
	{

		printf("%d ", recvmsg[i]);

	}
	printf("\r\n");

	printf("recv\r\n");

	// if (write(sockfd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
	// 	perror("Write");
	// 	return 1;
	// }

	if (close(sockfd) < 0) {
		perror("Close");
		return 1;
	}




	return 0;
}
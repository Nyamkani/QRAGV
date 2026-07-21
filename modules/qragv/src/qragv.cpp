#include <qragv/qragv.hpp>




QRAGV::QRAGV(){}

QRAGV::~QRAGV(){}


int QRAGV::Initialize()
{
    /*Make state at Initialize*/
	this->StartInitState();

}


/* Main System Drive*/
void QRAGV::Drive()
{
    this->FSMStateBehavior();

    this->FSMEventCheck();

    return;
}


/********************************threads************************** */


void ServerWorker(const void* argument)
{
    /*from upper class*/
    QRAGV* qragv = (QRAGV*)argument;
    bool quit_sig = false;

    /*socket*/
    int socket_fd;
    struct sockaddr_in host_addr;// , client_addr;
    // int recv_length;
    // char buffer[BUF_SIZE];


    if(socket_fd = socket(PF_INET,SOCK_STREAM, 0) < 0)
    {
        /*error occur*/
    }

    host_addr.sin_family = AF_INET;
    host_addr.sin_port = htons(SERVER_PORT);
    host_addr.sin_addr.s_addr = inet_addr("192.168.1.31");
    memset(&(host_addr.sin_zero),0,8);

    if(bind(socket_fd,(struct sockaddr *)&host_addr,sizeof(struct sockaddr)) < 0)
    {
        /*error occur*/ 
    }

    if(listen(socket_fd ,3) < 0)
    {
        /*error occur*/
    }


    while(!(quit_sig))
    {
        int accepted_fd = 0;
        socklen_t size = sizeof(struct sockaddr_in);

        if( accepted_fd = accept(socket_fd, (struct sockaddr *)&client_addr, &size) < 0)
        {
            close(accepted_fd);
        }

        // send(accepted_fd, "Connected", 10 , 0);

        // printf("Client Info : IP %s, Port %d\n", inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));

        // recv_length = recv(accepted_fd,&buffer,BUF_SIZE,0);

        // while(recv_length>0){
        //     printf("From Client : %s\n",buffer);
        //     recv_length=recv(accepted_fd,&buffer,BUF_SIZE,0);
        // }

    }


    return;
}

void DrivingControllerWorker(const void* argument)
{
    QRAGV* qragv = (QRAGV*)argument;

    bool quit_sig = false;

    while(!(quit_sig))
    {



        
    }

    return;
}
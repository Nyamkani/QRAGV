
#ifndef QRAGV_SERVER_HPP_
#define QRAGV_SERVER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<unistd.h>

#include<sys/types.h>
#include<sys/socket.h>
#include<sys/un.h>
#include<netinet/in.h>
#include<netinet/ip.h> 
#include <arpa/inet.h>

#ifdef __cplusplus
}
#endif

#include <vector>
#include <thread>



class qragv_server
{
    private:
        int accepted_fd_ = 0;
        bool quit_sig_ = false;
        std::vector<std::thread> workers_;
    public:


    private:


    public:
        qragv_server();
        qragv_server(int accepted_fd);
        virtual ~qragv_server();


        int Initialize();

};



#endif //QRAGV_SERVER_HPP_
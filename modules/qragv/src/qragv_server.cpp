#include <qragv/qragv_server.hpp>



void ReadServerWorker(const void* argument)
{

    return;
}


void WriteServerWorker(const void* argument)
{

    return;
}


void ParseServerWorker(const void* argument)
{

    return;
}


/************************************************************************* */



qragv_server::qragv_server(){}

qragv_server::qragv_server(int accepted_fd)
{
    this->accepted_fd_ = accepted_fd;
}


qragv_server::~qragv_server(){}


/*make 3 threads for read, write and parsing*/
int qragv_server::Initialize()
{
  if(!(this->workers_.empty()))
  {
    for(auto& worker : this->workers_)
    {
        /*send quit signal to exit while loop*/

        worker.join();
    }
  }

  this->workers_.push_back(std::thread(ReadServerWorker, this));
  this->workers_.push_back(std::thread(WriteServerWorker, this));
  this->workers_.push_back(std::thread(ParseServerWorker, this));


    return 0;
}




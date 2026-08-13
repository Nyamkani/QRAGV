#include <include/qragv/qragv.h>







qragv::qragv(){}


qragv::~qragv(){}



int qragv::Initialize()
{
    /*drving controller*/
    if(!(this->dc_))
        delete this->dc_;

    this->dc_ = new DrivingController(500, 500, 500);

    this->dc_->Init();





    return 0;
}
#include <include/driving_controller/driving_controller.hpp>








class qragv
{
    private:
        DrivingController* dc_ = nullptr;


        /*global command queue*/
        std::vector<int> agv_global_command_queue_;





    public:

    private:

    public:
        /**
         * @brief Construct a new qragv object
         * 
         */
        qragv();

        /**
         * @brief Destroy the qragv object
         * 
         */
        virtual ~qragv();

        /**
         * @brief intialize all modules -> ethernet, driving controller, and more
         * 
         * @return int 
         */
        int Initialize();

};
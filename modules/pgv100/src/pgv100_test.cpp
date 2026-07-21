/**
  ******************************************************************************
  * @file           : pgv100_test.cpp
  * @brief          : Test running cpp file of PGV100 Position Sensor
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



int main(int argc, char* argv[])
{
  int itr = 0;

  if(argc < 2)
    itr = 10;
  else 
   itr = atoi(argv[1]);

  pgv100 pgv100_test(0, 1);

  printf("Drive Will be looped %d times\r\n", itr);


    // pgv100_test.Init();

    for(int i = 0; i<itr; i++)
    {
      pgv100_test.Drive();


      if(pgv100_test.IsLineDetected())
        printf("PGV100 has Detected the Line. Xpos = %d, Ypos = %d, Angle = %d\r\n", pgv100_test.GetXPos(), pgv100_test.GetYPos(), pgv100_test.GetAngle());
      else if (pgv100_test.IsTagged())
        printf("PGV100 has Detected the tag. Xpos = %d, Ypos = %d, Angle = %d\r\n", pgv100_test.GetXPos(), pgv100_test.GetYPos(), pgv100_test.GetAngle());
      else 
        printf("PGV100 has no Detected anything. Xpos = %d, Ypos = %d, Angle = %d\r\n", pgv100_test.GetXPos(), pgv100_test.GetYPos(), pgv100_test.GetAngle());

      usleep(100*1000);
    }



    return 0;
}
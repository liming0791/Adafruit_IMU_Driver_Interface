#include "stdio.h"
#include "stdlib.h"

#include <iostream>
#include <fstream>

#include "IMUDriver.h"

using namespace std;

int main(int argc, char** argv)
{
    if(argc < 3){
        printf("need more args ");
        return 0;
    }

    float imuR[9];
    long long imuTime = 0;

    ofstream imuFile("imu.txt");

    IMUDriver imuDriver;
    imuDriver.initIMU(argv[1], atoi(argv[2]));

    imuDriver.begin();

    while(true)
    {
        imuDriver.getIMUData(imuR, imuTime);
        imuFile << imuR[0] << " " << imuR[1] << " " << imuR[2] << " " 
            << imuR[3] << " " << imuR[4] << " " << imuR[5] << " "
            << imuR[6] << " " << imuR[7] << " " << imuR[8] << endl;
        printf("data: %f %f %f %f %f %f %f %f %f\n", imuR[0], imuR[1], imuR[2], imuR[3], imuR[4], imuR[5], imuR[6], imuR[7], imuR[8]);
    }

    imuDriver.close();
    imuFile.close();

    return 0;
}

#ifndef IMUDRIVER_H
#define IMUDRIVER_H

#include "stdlib.h"
#include "stdio.h"

#include <chrono>
#include <mutex>

#include "AsyncSerial.h"

using namespace std;
class IMUDriver
{
    public:
        IMUDriver();
        ~IMUDriver();
        bool initIMU(char* port, int baud);
        void setIMUCallback(boost::function< void (float, float, float, 
                    float, float, float, long long ) >& callback);
        void getIMUData(float* IMUData, long long& IMUTime);
        void close();
        void begin();
        void stop();
        void IMUReceived(const char* data, size_t len);
        void getFloats(char* str, float* f);

    private:
        CallbackAsyncSerial serial;
        
        stringstream ssline;
        char line[200];
        int char_idx;

        float RealTimeR[9]; // IMU data with real frequence
        int i_count;

        long long base_time_stamp;
        long long imu_time_stamp;
        long long realtime_imu_time_stamp;
        long long last_imu_time_stamp;

        mutex frame_mutex;
        mutex data_mutex;
        mutex imu_mutex;

        bool firstIMU;
        bool imuInited;
        bool started;

        std::chrono::high_resolution_clock::time_point imu_base_time;
        std::chrono::high_resolution_clock::time_point imu_last_time;

        boost::function< void (float, float, float, float, float, float, long long) > imuCallback;
};

#endif

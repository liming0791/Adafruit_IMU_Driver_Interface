#include "IMUDriver.h"
#include <chrono>

using namespace std;

IMUDriver::IMUDriver()
{
    firstIMU = true;
    imuInited = false;
    started = false;

    i_count = 0;

    ssline.str(std::string());

    char_idx = 0;
}

IMUDriver::~IMUDriver()
{
}

/*
 * INIT
 */

bool IMUDriver::initIMU(char* port, int baud)
{
    printf("init imu......\n");
    if(imuInited){
        printf("IMU has inited !");
        return true;
    }
    serial.open(std::string(port), (unsigned int)baud);
    serial.setCallback(boost::bind(&IMUDriver::IMUReceived, this, _1, _2));
    imu_base_time = chrono::high_resolution_clock::now();
    imu_last_time = chrono::high_resolution_clock::now();
    imuInited = true;

    printf("init imu done\n");
    return true;
}

void IMUDriver::getFloats(char* str, float* f)
{
    int b_idx = 0, e_idx = 0, i=0;
    while (str[e_idx] != '\n') {
        e_idx++;
        while (str[e_idx] != ' ' && str[e_idx] != '\n') {
            e_idx++;
        }
        char word[50];
        memcpy(word, &str[b_idx], e_idx - b_idx);
        f[i++] = atof(word);
        b_idx = e_idx+1;

        if (i > 8) return;
    }
}

void IMUDriver::IMUReceived(const char *data, size_t len)
{
    char rawdata[500] = "";
    memcpy(rawdata, data, len*sizeof(char));
    rawdata[len] = '\0';

    //printf("\n\n===\nreceive raw data:\n%s\n===\n\n",rawdata) ;

    for (unsigned int i=0; i<len;i++)
    {
        if (data[i] == '\n')
        {
            line[char_idx++] = '\n'; 
            line[char_idx++] = '\0';

            //time it
//            i_count++;
//            if(i_count==10){
//                i_count = 0;
//                auto nowTime = chrono::high_resolution_clock::now();
//                long long dua = (long long)chrono::duration_cast<chrono::microseconds>(nowTime - imu_last_time).count();
//                printf("imu rate: %f\n", 1.f/ (dua/1000000.f/10.f));
//                imu_last_time = chrono::high_resolution_clock::now();
//            }

            // IMU data process 
            {
                lock_guard<mutex> lock(imu_mutex);

                if(firstIMU){
                    firstIMU = false;
                    imu_base_time = chrono::high_resolution_clock::now();
                }
                auto nowTime = chrono::high_resolution_clock::now();
                realtime_imu_time_stamp = (long long)chrono::duration_cast<chrono::microseconds>
                    (nowTime - imu_base_time).count();
                //printf("imu timestam: %lld\n", realtime_imu_time_stamp);
                //printf("\n===string===%s\n\n", line);
                getFloats(line, RealTimeR);
                if (imuCallback) {
                    imuCallback(RealTimeR[0], RealTimeR[1], RealTimeR[2], 
                            RealTimeR[3], RealTimeR[4], RealTimeR[5], 
                            realtime_imu_time_stamp);
                }
                //printf("\n===Receive===: %f %f %f %f %f %f %f %f %f\n\n", RealTimeR[0], RealTimeR[1], RealTimeR[2], 
                //RealTimeR[3], RealTimeR[4], RealTimeR[5], RealTimeR[6], RealTimeR[7], RealTimeR[8]);
            }

            char_idx = 0;
        } else {
            line[char_idx++] = data[i];
        }
    }
}

/*
 * SET CALLBACK
 */

void IMUDriver::setIMUCallback(boost::function< void (float, float, float, 
            float, float, float, long long) >& callback)
{
    imuCallback = callback;
}

/*
 * BEGIN
 */

void IMUDriver::begin()
{
    if(started){
        printf("already started !");
        return;
    }

    started = true;
    printf("start oni_imu\n");
}

/*
 * GETDATA
 */

//get imu data
void IMUDriver::getIMUData(float* IMUData, long long& IMUTime)
{
    if(!started){
        printf(" Oni_IMU Has not started !\n");
        memset(IMUData, 0, 9*sizeof(float));
        return;
    }

    {
        lock_guard<mutex> lock(imu_mutex);
        memcpy(IMUData, RealTimeR , 9*sizeof(float));
        IMUTime = realtime_imu_time_stamp;
    }
}

/*
 *STOP
 */

void IMUDriver::stop()
{

}

/*
 * CLOSE
 */

void IMUDriver::close()
{
    serial.close();
}

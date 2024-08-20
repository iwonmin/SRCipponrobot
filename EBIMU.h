#ifndef EBIMU_H
#define EBIMU_H
#include "mbed.h"

class EBIMU 
{
    public:
    EBIMU(PinName tx, PinName rx, int baud);
    void parse();//데이터 파싱을 위한 함수
    float getYaw();//Yaw 값을 반환하는 함수
    void initialze();// 초기화 함수

    private:
    Serial imu;//IMU와의 직렬 통신을 위한 객체
    char data[64];//수신 데이터 저장
    float roll, pitch, yaw;
    float initialYaw;//초기 yaw 값을 저장
    bool isInitialized;//초기화 여부
    float normalizeYaw(float angle);// yaw 각도 정규화 함수
    void chartodata();//수신된 데이터 처리하는 함수수
};
#endif
#include "EBIMU.h"

EBIMU::EBIMU(PinName tx, PinName rx, int baud) : imu(tx, rx, baud), initialYaw(0.0), isInitialized(false) {
    memset(data, 0, sizeof(data)); // 데이터 배열 초기화
}

// 각도를 -180도에서 180도로 변환하는 함수
float EBIMU::normalizeYaw(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}
float EBIMU::getRoll()
{
    return roll;
}

float EBIMU::getPitch()
{
    return pitch;
}
// yaw 값을 업데이트하고 반환하는 함수
float EBIMU::getYaw() {
    return yaw; // 현재 yaw 값을 반환
}

// 데이터 처리 함수
void EBIMU::chartodata() {
    char* start = strchr(data, '*');
    if (start != NULL) {
        start++;

        char* token = strtok(start, ",");
        if (token != NULL) {
            roll = atof(token);

            token = strtok(NULL, ",");
            if (token != NULL) {
                pitch = atof(token);

                token = strtok(NULL, ",");
                if (token != NULL) {
                    float rawYaw = atof(token); // 측정된 yaw 값을 가져옴

                    if (!isInitialized) {
                        // 최초 초기화 시 yaw 값을 0으로 설정
                        initialYaw = rawYaw;
                        isInitialized = true; // 초기화 완료 표시
                    }
                    yaw = normalizeYaw(rawYaw - initialYaw); // 기준 yaw 값을 초기화한 값으로 설정하고 정규화
                }
            }
        }
    }
}

// 데이터 파싱 함수
void EBIMU::parse() {
    for (int i = 0; i < 64; i++) {
        char a = imu.getc();
        if (a == 0x0A) {
            chartodata(); // 줄바꿈 문자 수신 시 데이터 처리
            memset(data, 0, sizeof(data)); // 데이터 배열 초기화
            return; // 파싱 후 함수 종료
        } else {
            data[i] = a;
        }
    }
}
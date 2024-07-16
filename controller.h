#include "mbed.h"
#include <string>
#include <stdlib.h>
#include "GP2A.h"
#include "rtos.h"
#include "MPU9250.h"
#pragma region Preprocessor
#define MAXSPEED 0.5
#define ESCAPESPEED -0.5
#define PSD_INTERVAL_us 0.1 // @@ dummy value, should be defined !!@@
#define PSD_THRESHOLD 5 // encounter distance(cm) diff, must be defined with experiment - threshold / inverval = speed
#define CIRCLE_DISTANCE 70 //cm
#define WALL_DISTANCE 70 //cm
#define TIME_90DEGTURN 50 //ms, pwm == 0.5
#define Time_10CMMOVE 20 //ms, pwm == 0.5
#define IMU_THRESHOLD 8.f
#define ESCAPE_TIME 800 //ms
#pragma endregion Preprocessor
#pragma region external
extern InterruptIn btn;
extern DigitalOut DirL;
extern DigitalOut DirR;
extern PwmOut PwmL;
extern PwmOut PwmR;
extern GP2A psdlf;
extern GP2A psdf;
extern GP2A psdrf;
extern GP2A psdlc;
extern GP2A psdrc;
extern GP2A psdlb;
extern GP2A psdb;
extern GP2A psdrb;
extern DigitalIn irfl;
extern DigitalIn irfr;
extern DigitalIn irfc;
extern DigitalIn irbc;
extern DigitalIn irbl;
extern DigitalIn irbr;
extern class Controller controller;
extern Thread Thread1;
extern Thread Thread2;
extern Serial pc;
#pragma endregion external
class Controller
{
     public:   
        enum class RoboState
    {
        //초기
        START,
        //대기
        IDLE,
        //감지
        DETECT,
        //공격
        ATTACK,
        //탈출
        ESCAPE
    };

        enum class ColorOrient
    {
        FRONT, TAN_LEFT, TAN_RIGHT, BACK, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, SAFE    
    };
        enum class Position
    {
        ClosetoLeftWall, CriticalLeftWall, ClosetoRightWall, CriticalRightWall,
        WallFront, WallBehind, ClosetoCenter, FartoCenter
    };
        enum class TiltState
    {
        FRONT, FRONT_LEFT, FRONT_RIGHT, SIDE_LEFT, SIDE_RIGHT, SAFE
    };
        bool StartFlag = false;

    //객체 생성시 실행되는 생성자 함수
    Controller();
//-------------------Get & Set methods----------------------//
    //현재 로봇의 상태 반환
    RoboState GetState();

    //로봇의 상태를 변환
    void SetState(RoboState state);
    
    //적 감지 여부 변환
    void SetEnemyState(bool enemyState);
    //적 감지 여부 반환
    bool GetEnemyState();

    //적 감지 여부 변환
    void SetAttackState(bool attackState);
    //적 감지 여부 반환
    bool GetAttackState();

    //Ir 감지 여부 변환
    void SetIrSafetyState(bool IrSafetyState);
    //Ir 감지 여부 반환
    bool GetIrSafetyState();

    //Psd 감지 여부 변환
    void SetImuSafetyState(bool ImuSafetyState);
    //Psd 감지 여부 반환
    bool GetImuSafetyState();

    //벽 감지 여부 변환
    void SetWallSafetyState(bool WallSafetyState);
    //벽 감지 여부 반환
    bool GetWallSafetyState();

    //좌측 바퀴 속도 반환
    float GetSpeedL();
    
    //우측 바퀴 속도 반환
    float GetSpeedR();
    
    // 양쪽 바퀴 같은 속도 설정
    void SetSpeed(float speed);

    //양쪽 바퀴 다른 속도 설정
    void SetSpeed(float sL, float sR);
    
    //적과의 수평거리 반환 함수
    int GetHD();

    //적과의 수평거리 변환
    void SetHD(int HD);
//--------------------State Machine methods----------------------//
    //초기상태 시 실행 함수
    void Start();

    //대기상태 시 실행 함수
    void Idle();

     //감지상태 시 실행 함수
    void Detect();

     //공격상태 시 실행 함수
    void Attack();

     //탈출상태 시 실행 함수
    void Escape();

    //주행 함수
    void Move(float sL, float sR);

    void EnemyDetect();

    //-----------------------psd--------------------//
    uint16_t PsdDistance(GP2A, uint8_t);

    void PsdRefresh();
    
    void PsdDetect();

    void PsdWallDetect();

    void PsdWallEscape();
    
    //-------------------------IR------------------------//
    Position GetPosition();

    void IrRefresh();
    
    void EnemyPushPull();

    void IrEscape(ColorOrient orient);

    // void IrEscapeWhenImuUnsafe();

    void ColorOrient();

    enum ColorOrient GetOrient();
    //----------------------적 찾기 & 위치파악 & 적 괴롭히기 전략-------------------------//
    void SetPosition();

    void EnemyFind(Position);
    
    void EnemyFind_Extended(Position);

    void LeftWallTrack();

    void RightWallTrack();

    void WallTwerk();
    /*
    void CenterSpin();
    
    void FrontWall();

    void BehindWall();
    */

    //-----------------------MPU9250-----------------------------//
    void SetupImu();
    
    void ImuRefresh();

    void ImuDetect();
    
    void ImuEscape();

    Timer Escape_Timer;

    //------------------------Tester's Choice-------------------//
    void OrientViewer(int);

    void WallViewer();

    void ImuViewer();
//--------------------Private variables--------------------------//
    private:
    //로봇 상태
    RoboState robo_state;
    //색 영역 위치
    enum ColorOrient Orient;
    //예상되는 위치
    enum Position CurrentPos;
    //Imu 상태
    enum TiltState tilt_state;
    //적 감지 여부
    volatile bool enemy = false;

    volatile bool attack = false;
    
    //적과 벌어진 거리
    int enemy_horizontal_distance;

    //위험 지역 여부
    volatile bool irSafe = true;

    volatile bool imuSafe = true;

    volatile bool wallSafe = true;

    //좌측 바퀴 속력
    float speedL;

    //우측 바퀴 속력
    float speedR;

    const float alpha_psd = 0.88f;

    uint16_t prev_distance[8]; //psdlf, psdf, psdrf, psdlc, psdrc, psdlb, psdb, psdrb

    uint16_t now_distance[8]; //psdlf, psdf, psdrf, psdlc, psdrc, psdlb, psdb, psdrb

    float filtered_distance[8]; //psdlf, psdf, psdrf, psdlc, psdrc, psdlb, psdb, psdrb

    bool detection[8]; //psdlf, psdf, psdrf, psdlc, psdrc, psdlb, psdb, psdrb

    uint16_t MinValue = 0;

    uint8_t MinIndex = 0;

    uint8_t FollowIndex = 0;

    bool ir_val[6]; //irfl, irfr, irc, irbl, irbr //미리 선언되어야 함.

    uint8_t ir_total; 

    uint16_t psd_val[8]; //psdlf, psdf, psdrf, psdlc, psdrc, psdlb, psdb, psdrb

    //벽 충돌 감지
    //벽이 두방향에서 보일때는 다 색영역인데 그냥 열거형 쓰기 ??
    bool FrontCollision;

    bool BackCollision;

    bool LeftCollision;

    bool RightCollision;

    const float alpha_imu = 0.9f;

    float gyro_angle_x, gyro_angle_y, gyro_angle_z;

    float accel_angle_x, accel_angle_y, mag_angle_z;

    Timer t; //for gyro integral;

};

//-------------------------Thread----------------------------//
void ImuThread();

void PsdThread();

void Starter();

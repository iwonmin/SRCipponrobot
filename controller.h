#include "mbed.h"
#include <string>
#include <stdlib.h>
#include "Thread.h"
#include "GP2A.h"
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
#define imu_time 50 //확정X
#pragma endregion Preprocessor
#pragma region external
extern DigitalOut DirL;
extern DigitalOut DirR;
extern PwmOut PwmL;
extern PwmOut PwmR;
extern GP2A psdf; //그냥 거리감지
extern GP2A psdb;
extern GP2A psdlf; //PA_0 -> 핀 바꿔야함 !!!!
extern GP2A psdlc;
extern GP2A psdlb;
extern GP2A psdrf;
extern GP2A psdrc;
extern GP2A psdrb;
extern DigitalIn irfl;
extern DigitalIn irfr;
extern DigitalIn irc;
extern DigitalIn irbl;
extern DigitalIn irbr;
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
    //객체 생성시 실행되는 생성자 함수
    Controller();
//-------------------Get & Set methods----------------------//
    //현재 로봇의 상태 반환
    RoboState GetState();

    //로봇의 상태를 변환
    void SetState(RoboState state);

    //적 감지 여부 반환
    bool GetEnemyState();

    //적 감지 여부 변환
    void SetEnemyState(bool enemyState);

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

    //적과의 수평거리 변환환
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

    void WallDetect();

    void Psd_Escape();
    

    //-------------------------IR------------------------//
    Position GetPosition();

    void IrRefresh();

    void IrEscape(ColorOrient orient);

    void ColorOrient();

    //----------------------적 찾기 & 위치파악-------------------------//
    void SetPosition();

    void EnemyFind(Position);
    
    void EnemyFind_Extended(Position);

    void LeftWallTrack();

    void RightWallTrack();

    void CenterSpin();
    
    void FrontWall();

    void BehindWall();

    //-----------------------MPU9250-----------------------------//(작년 코드와 same)
    void SetupImu();
    
    void ImuRefresh();

    void ImuRead();

//--------------------Private variables--------------------------//
    private:
    //로봇 상태
    RoboState robo_state;
    //색 영역 위치
    enum ColorOrient Orient;
    //예상되는 위치
    enum Position CurrentPos;
    //적 감지 여부
    bool enemy = false;

    //적과 벌어진 거리
    int enemy_horizontal_distance;

    //위험 지역 여부
    bool isSafe = true;

    //좌측 바퀴 속력
    float speedL;

    //우측 바퀴 속력
    float speedR;

    uint16_t prev_distance[8]; //psdlf, psdrf, psdlc, psdrc, psdlb, psdrb, psdf, psdb

    uint16_t now_distance[8]; //psdlf, psdrf, psdlc, psdrc, psdlb, psdrb, psdf, psdb

    float filtered_distance[8]; //psdlf, psdrf, psdlc, psdrc, psdlb, psdrb, psdf, psdb

    float alpha = 0.9f;

    bool detection[8]; //psdlf, psdrf, psdlc, psdrc, psdlb, psdrb, psdf, psdb

    bool ir_val[5]; //irfl, irfr, irc, irbl, irbr //미리 선언되어야 함.

    uint8_t ir_total; 

    uint16_t psd_val[8]; //psdlf, psdrf, psdlc, psdrc, psdlb, psdrb, psdf, psdb

    //벽 충돌 감지
    //벽이 두방향에서 보일때는 다 색영역인데 그냥 열거형 쓰기 ??
    bool FrontCollision;

    bool BackCollision;

    bool LeftCollision;

    bool RightCollision;
    //------------------imu------------------//(미완성)
    float tmp_angle_x, tmp_angle_y, tmp_angle_z;

    float filltered_angle_x, filltered_angle_y, filltered_angle_z;

    float tmp_acc_x, tmp_acc_y, tmp_acc_z;
    
    float filtered_acc_x = 0, filtered_acc_y, filtered_acc_z;

    int imu_count = 0;

    Timer t;

    float sum = 0;
    
    uint32_t sumCount = 0;

    uint64_t Now_time,Work_time,Nowm_time,Workm_time,Nowi_time,Worki_time;

    uint16_t chek_time;
};


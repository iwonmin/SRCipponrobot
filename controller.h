#include "mbed.h"
#include <string>
#include <stdlib.h>
#include "Thread.h"
#define MAXSPEED 0.5
#define ESCAPESPEED -0.5
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
//--------------------Private variables--------------------------//
    private:
    //로봇 상태
    RoboState robo_state;

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
};


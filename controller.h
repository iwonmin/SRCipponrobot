#include "mbed.h"
#include <string>
#include <stdlib.h>
#include "Thread.h"
#define MAXSPEED 0.9
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

    //현재 로봇의 상태 반환
    RoboState GetState();

    //로봇의 상태를 변환
    void SetState(RoboState state);

    //적 감지 여부 반환
    bool GetEnemyState();

    //적 감지 여부 변환
    void SetEnemyState();

    //좌측 바퀴 속도 반환
    float GetSpeedL();
    
    //우측 바퀴 속도 반환
    float GetSpeedR();
    
    // 양쪽 바퀴 같은 속도 설정
    void SetSpeed(float speed);

    //양쪽 바퀴 다른 속도 설정
    void SetSpeed(float sL, float sR);
    
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

    private:
    //로봇 상태
    RoboState robo_state;

    //적 감지 여부
    bool enemy;

    //위험 지역 여부
    bool isSafe;

    //좌측 바퀴 속력
    float speedL;

    //우측 바퀴 속력
    float speedR;
};


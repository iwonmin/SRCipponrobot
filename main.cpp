#include "mbed.h"
#include "controller.h"

//상태 디버깅 용 타이머
Timer timer;

//블루투스 통신
Serial hm10(PC_10,PC_11,115200);

//디버그용 키보드 입력
char keyInput;

int main()
{
    hm10.printf("Waiting for keyboard input\n");
    Controller controller;
    timer.start();
     while (true) {
         if(hm10.readable())
         {
             keyInput = hm10.getc();
             hm10.printf("input char = %c\n",keyInput);
         }
        switch (keyInput) 
        {
            case '1':
            controller.startFlag = true;
            break;
            case '2':
            controller.startFlag=false;
            controller.SetState(Controller::RoboState::START);
            break;
            case '3':
            controller.isSafe = false;
            break;
            case '4':
            controller.isSafe = true;
            break;
            case '5':
            controller.SetEnemyState(false);
            break;
            case '6':
            controller.SetEnemyState(true);
            break;
            case '7':
            controller.yellow = false;
            break;
            case '8':
            controller.yellow = true;
            break;
            case '9':
            controller.isClose = false;
            break;
            case '0':
            controller.isClose = true;
            break;
        }
        controller.EnemyDetect();
        switch(controller.GetState())
        {
            case Controller::RoboState::START:
            controller.Start();
            break;
            case Controller::RoboState::IDLE:
            controller.Idle();
            break;
            case Controller::RoboState::DETECT:
            controller.Detect();
            break;
            case Controller::RoboState::ATTACK:
            controller.Attack();
            break;
            case Controller::RoboState::ESCAPE:
            controller.Escape();
            break;
            case Controller::RoboState::YELLOW:
            controller.Yellow();
            break;
        }
        controller.Move(controller.GetSpeedL(),controller.GetSpeedR());
        
    }
}

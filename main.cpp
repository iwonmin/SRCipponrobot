#include "mbed.h"
#include "controller.h"
//디버그용 키보드 입력
char c;
int main()
{
    pc.printf("Serial check check");
    hm10.printf("Waiting for keyboard input\n");
    timer.start();
     while (true) {
        if(hm10.readable())
        {
            c = hm10.getc();
            hm10.printf("input char = %c\n",c);
        }else{
            hm10.printf("Serial is not readable");
        }
        switch (c) 
        {
            case '1':
            controller.SetStartFlag(true);
            break;
            case '2':
            controller.SetStartFlag(false);
            controller.SetState(Controller::RoboState::START);
            break;
            case '3':
            //controller.isSafe = false;
            break;
            case '4':
            //controller.isSafe = true;
            break;
            case '5':
            controller.SetEnemyState(false);
            break;
            case '6':
            controller.SetEnemyState(true);
            break;
            case '7':
            controller.SetYellow(false);
            break;
            case '8':
            controller.SetYellow(true);
            break;
            case '9':
            controller.SetClose(false);
            break;
            case '0':
            controller.SetClose(true);
            break;
        }
        //controller.EnemyDetect();
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

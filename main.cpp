#include "mbed.h"
#include "controller.h"

Controller controller;  
int main()
{
    //hm10.printf("Waiting for Keyboard input\n");
    timer.start();
 while (true) 
    {
        if(hm10.readable())
        {
            blInput=hm10.getc();
            hm10.printf("input char = %c\n",blInput);
        }
        switch(blInput)
        {
            case '1':
            controller.SetStartFlag(true);
            break;
            case '2':
            controller.SetStartFlag(false);
            controller.SetState(Controller::RoboState::START);
            break;
            case '3':
            controller.SetSafe(false);
            break;
            case '4':
            controller.SetSafe(true);
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
            controller.SetIsClose(false);
            break;
            case '0':
            controller.SetIsClose(true);
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

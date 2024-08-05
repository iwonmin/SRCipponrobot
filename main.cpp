#include "mbed.h"
#include "controller.h"

int main()
{
    pc.printf("hello");
    hm10.printf("Waiting for Keyboard input\n");
    timer.start();
 while (true) 
    {
        if(hm10.readable())
        {

            blInput=hm10.getc();
            hm10.printf("input char = %c\n",blInput);
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
        }
        controller.Move(controller.GetSpeedL(),controller.GetSpeedR());
        
    }
}

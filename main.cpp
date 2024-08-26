#include "mbed.h"
#include "controller.h"

int main()
{
    pc.printf("main started\n");
 while (true) {
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
        }
        controller.Move(controller.GetSpeedL(),controller.GetSpeedR());
        wait_us(10000);
    }
}

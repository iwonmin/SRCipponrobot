#include "mbed.h"
#include "controller.h"


// main() runs in its own thread in the OS
int main()
{
Controller controller;
// wait(3000);
// controller.SetEnemyState(true);
// wait(3000);
// controller.SetEnemyState(false);
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
        }
        controller.Move(controller.GetSpeedL(),controller.GetSpeedR());
    }
}

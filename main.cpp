#include "mbed.h"
#include "controller.h"

int main()
{
<<<<<<< Updated upstream
    pc.printf("main started\n");
 while (true) {
=======
    device.attach(&EnemyDetect, SerialBase::RxIrq);
    while (true) {
>>>>>>> Stashed changes
        controller.CheckStartTime();
        //controller.EnemyDetect();
        // DetectThread2();
        //ImuThread();
        //controller.ImuParse();
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
        ThisThread::sleep_until(controller.GetStartTime() + 10); //절대 시간으로 10ms 만큼 쉬기
        //ThisThread::sleep_for(20);
    }
}

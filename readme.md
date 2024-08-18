2024/08/18
Line 159 - void Controller::Escape()
    - 초기 계획은 플래그가 동시에 켜져있을 때 함수에서 선언된대로 탈출 플래그 우선순위가 Imu -> Ir -> Psd 순이었지만, Imu가 예상보다 너무 많이 튀어서 psd와 융합하였지만 psd가 작동이 안되는 관계로 Ir -> Imu -> Psd 순서로 사용, 또한 실제 구동시 ColorOrient::Front 상태가 가장 많이 뜨는데, 이 때 탈출구동이 후진(PWM -0.5, -0.5) 이므로 가장 보수적이라고 판단. 그러므로 순서는 다음과 같이 작성됨 : Ir -> Imu -> Psd
    - 추후에 우선순위 재변동할 필요 있으므로 if문 조건함수와 그에 따른 Escape 함수만 바꾸기.

Line 228 - void Controller::PsdRefresh()
    - psd 센서 작업중이므로 PsdRefresh()에서 PsdWallDetect() 비활성화 -> wallsafe 플래그 항상 true
    - psd 센서 안끼면 30~60 떠서 그냥 기능 살리기로 함 -> 벽으로 인식하는 거리 10cm로 변경

[ㅈ버그]Line 408 - void Controller::ColorOrient()
    - 로봇 밑판이 생각보다 낮아서 색영역에 앞부분만 발을 조금만 담가도 중앙 IR이 인식되버려서(검정영역 근거리 이슈) Color합이 3이라 깊게 들어간 FRONT 또는 FRONT_LEFT 로 뜨는데 구동상에 차이는 없어서 일단 둠 -> 알고리즘 크게 변경할 예정 있음

Line 725 - Imuthread()
    - psd 센서 작업중이므로 SIDE_TILT 작동 안될 것으로 보임. 필요시 비활성화
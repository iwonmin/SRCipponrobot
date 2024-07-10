gyro
    [확정]gyro로 인한 자세변화 감지(RoboState -> Escape)
    [생각중]가속도 감지로 힘싸움중에 밀려서 ir에 들어갔음에도 이동명령은 내렸지만 못움직이는데 이거를 확인할 방법??

SetSpeed
    타이머 attach 하여 함수 호출시 입력받은 속도,속도,시간으로 control?

(7.10)SEXXXXXXXXXXXXXXXXXX
1. 규정상 바뀐 사각형 색 영역에 앉아서 대기하기 = ??
2. 뒤쪽 바퀴로 벽 타고 올라가서 낮은자세잡기 = while문으로 구현했는데 흐음..
3. 중앙 색영역에 상대 넣었을때 왔다갔다 가동범위 늘리기(ir 앞쪽에서만 감지되도 바로 빼기, 기존은 3개 감지하기) = partially done
4. 더 나은 IMU Criterion
## self-balancing-robot
A PID control self balancing robot using esp32 and mpu6050.

![balancing-bot](/robot-imgs.png)

### Video demo

[video](https://www.youtube.com/shorts/ST1ROw0RA6I)

### Key Points
1. Make constant frequency main loop
2. Choose one falling side to be forward direction and write all the code logics
3. Handle the perfectly vertical dead zone and motor driver's deadzone (actual number up to when motors don't start moving)
4. Calibrate the sensor
5. Try tuning the PID constants one at a time, keeping others zero

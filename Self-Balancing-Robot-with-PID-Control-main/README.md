# Self-Balancing-Robot-with-PID-Control
**Hardware used-**<br>

- Arduino Mega<br>
- mpu6050 gyro sensor<br>
- LCD display with keypad<br>
- L298n motor driver<br>
- 12V DC motor<br>
- 12v lipo battery<br>
- Sonar sensor(optional-for obstacle avoiding)<br>

The code uses PID control parameters to balance the bot. PID stands for propostional-integral-derivative<br>
Kp-parameter for tuning proportional part<br>
Ki-parameter for tuning integral part<br>
Kd-parameter for tuning derivative part<br>
***Value of these parameters are choosen by trial and error and then fine tuned for the maximum stability.***
<br><br>
The code uses standard algorithm to smooth and filter the gyro sensor data, otherwise there will be too much jerking. One may use Kalman filter for better filtering but it may introduce complexity in the code. Arduino Mega or better microcontroller is recommended

**For the structure- 3D printed parts are used(designed in solidworks), GrabCad link for STL files will be given soon**

![IMG_20210521_075809](https://user-images.githubusercontent.com/87447678/197686570-1450aeb8-f899-4b87-ab4f-f1f76b5e7233.jpg)

# Udoo-neo-quadcopter
work in slow progress

Android application: Send commands to UDOO

UDOO(A9)
inputs: commands from android, GPS, air pressure, temperature (Bricks snap in sensors)
outputs: send data back to android (position, speed, altitude, errors)
         send Euler angles and other stuff to M4

UDOO(M4)
inputs: Euler angles / pulse lengths, mode(standby/fly) from android, 9-axis motion sensors
outputs: RC-signals to ESC (around 200hz hopefully)


Changes made in Servo library to increase update rate
servo_mqx.h:
#define DEF_SERVO_TICKS_PERIOD            (15000-2)   // default: (60000-2)
#define DEF_SERVO_uSEC_PERIOD             5000        // default: 20000

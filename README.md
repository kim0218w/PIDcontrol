# PIDcontrol
모터제어에서 PID제어를 한 코드내용
rypi/Desktop/PID/PIDcontrol/curve.py
[INFO] GPIO encoder initialized on pins 2 and 3
[WARN] No I2C encoder channels responded.
[INFO] Total encoder channels available: ['gpio']
Commands:
  ACT e/r                              -> actuator extend/retract
  M1 f/b [steps] [accel] [gamma <shape>] [accel2 <decel>] [gamma2 <shape>] [speed <deg/s>|rpm <value>] [pid? Kp Ki Kd]  -> motor1
  M2 f/b [steps] [accel] [gamma <shape>] [accel2 <decel>] [gamma2 <shape>] [speed <deg/s>|rpm <value>] [pid? Kp Ki Kd]  -> motor2
  q                                    -> quit
>> m2 b 10000 0.45 gamma=0.3 pid Kp=0.3 Ki=0 Kd=0         
[M2] steps=10000, accel_ratio=0.45, gamma=0.3, direction=1, PID=True gains=(2.0, 0.2, 0.0)
GPIO released.
Traceback (most recent call last):
  File "/home/raspberrypi/Desktop/PID/PIDcontrol/curve.py", line 1524, in <module>
    main()
  File "/home/raspberrypi/Desktop/PID/PIDcontrol/curve.py", line 1247, in main
    encoders_dict=encoders_dict
                  ^^^^^^^^^^^^^
NameError: name 'encoders_dict' is not defined

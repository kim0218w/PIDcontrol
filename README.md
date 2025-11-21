# PIDcontrol
모터제어에서 PID제어를 한 코드내용


Multiplexer detected at 0x70

Trying to find AS5600 on multiplexer channels...

✗ Channel 0: No AS5600
✗ Channel 1: No AS5600
✗ Channel 2: No AS5600
✗ Channel 3: No AS5600
✗ Channel 4: No AS5600
✗ Channel 5: No AS5600
✗ Channel 6: No AS5600
✗ Channel 7: No AS5600
Traceback (most recent call last):
  File "/home/raspberrypi/Desktop/PID/PIDcontrol/test.py", line 32, in <module>
    bus.write_byte(MUX_ADDR, 0x00)
TimeoutError: [Errno 110] Connection timed out

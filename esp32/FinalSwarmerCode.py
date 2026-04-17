from machine import Pin, PWM
import time

AIN1 = 13
AIN2 = 14
PWMA = 26
BIN1 = 12
BIN2 = 27
PWMB = 25
STBY = 33

class TB6612FNG:
    def __init__(self, in1, in2, pwm, standby, freq=5000):
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.pwm = PWM(Pin(pwm), freq=freq)
        self.stby = Pin(standby, Pin.OUT)
        self.stby.value(1) # Keep driver enabled

    def drive(self, speed):
        """Speed should be between -1023 and 1023"""
        if speed > 0:
            self.in1.value(1)
            self.in2.value(0)
        elif speed < 0:
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.stop()
            return

        self.pwm.duty(min(abs(speed), 1023))

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)

motor_left = TB6612FNG(AIN1, AIN2, PWMA, STBY)
motor_right = TB6612FNG(BIN1, BIN2, PWMB, STBY)

print("Robot starting in 3 seconds... Prepare for movement!")
time.sleep(3)

MOVE_SPEED = 800 

print("Moving Forward...")
motor_left.drive(MOVE_SPEED)
motor_right.drive(MOVE_SPEED)

try:
    while True:
        time.sleep(1) 
except KeyboardInterrupt:
    
    print("Stopping...")
    motor_left.stop()
    motor_right.stop()
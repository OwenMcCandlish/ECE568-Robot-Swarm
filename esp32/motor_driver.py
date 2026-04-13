from machine import Pin, PWM
import time

#Motor A (Right)
AIN1_PIN = 13
AIN2_PIN = 12
PWMA_PIN = 14

#Motor B (Left)
BIN1_PIN = 27
BIN2_PIN = 26
PWMB_PIN = 25

#Standby
STBY_PIN = 33

#PWM frequency: 1 kHz
PWM_FREQ = 1000

class Motor:
    """
    Represents single motor channel on TB6612FNG \n
    TB6612FNG:
        if IN1 & ~IN2, as well as PWM, it moves forward
        if ~IN1 & IN2 as well as PWM, it reverses
        if IN1 & IN2, doesn't matter for PWM, it performs short brake (immediate stop)
        if ~IN1 & ~IN2, doesn't matter for PWM, it coasts (disconnected from driver, slower rollout)
    """

    def __init__(self, in1_pin, in2_pin, pwm_pin, freq=PWM_FREQ):
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin), freq=freq, duty_u16=0)

    def run(self, speed):
        """
        Set motor speed and direction.
        Args: speed: Integer from -100 (full reverse) to 100 (full forward). 
              0 coasts to stop.
        """
        speed = max(-100, min(100, speed))  #clamp
        duty = int(abs(speed) / 100 * 65535)

        if speed > 0:
            self.in1.on()
            self.in2.off()
        elif speed < 0:
            self.in1.off()
            self.in2.on()
        else:
            self._coast()
            return

        self.pwm.duty_u16(duty)

    def brake(self):
        """Short-brake: actively resists rotation (fast stop)."""
        self.in1.on()
        self.in2.on()
        self.pwm.duty_u16(65535)

    def _coast(self):
        """Coast: let motor spin down freely (no braking)."""
        self.in1.off()
        self.in2.off()
        self.pwm.duty_u16(0)

    def stop(self):
        """Same as coast."""
        self._coast()

    def deinit(self):
        """Release PWM resource."""
        self.pwm.deinit()


class TB6612FNG:
    """
    High-level driver for the TB6612FNG Dual DC Motor Driver.
    Wiring:
        STBY -> STBY_PIN (default 33) (Motor A (Right))
        AIN1 -> AIN1_PIN (default 13) (Motor A (Right))
        AIN2 -> AIN2_PIN (default 12) (Motor A (Right))
        PWMA -> PWMA_PIN (default 14) (Motor A (Right))
        
        BIN1 -> BIN1_PIN (default 27) (Motor B (Left))
        BIN2 -> BIN2_PIN (default 26) (Motor B (Left))
        PWMB -> PWMB_PIN (default 25) (Motor B (Left))

        VM -> battery +ve
        VCC -> 3.3 V
        GND -> GND 

    Speed values are always in the range [-100, 100]:
        Positive → forward
        Negative → reverse
        Zero → coast
    """

    def __init__(
        self,
        ain1=AIN1_PIN, ain2=AIN2_PIN, pwma=PWMA_PIN,
        bin1=BIN1_PIN, bin2=BIN2_PIN, pwmb=PWMB_PIN,
        stby=STBY_PIN,
        freq=PWM_FREQ,
    ):
        self.stby = Pin(stby, Pin.OUT)
        self.motor_right = Motor(ain1, ain2, pwma, freq) #Motor A
        self.motor_left  = Motor(bin1, bin2, pwmb, freq) #Motor B
        self.wake() #Enable driver immediately

    def wake(self):
        """Bring the driver out of standby: enable outputs."""
        self.stby.on()

    def standby(self):
        """Put the driver into standby: all outputs off, low power."""
        self.stby.off()

    def forward(self, speed=80):
        """Drive both motors forward at the given speed: (0–100)."""
        self.motor_right.run(speed)
        self.motor_left.run(speed)

    def backward(self, speed=80):
        """Drive both motors backward at the given speed: (0–100)."""
        self.motor_right.run(-speed)
        self.motor_left.run(-speed)

    def turn_right(self, speed=70):
        """
        Pivot right in place: left motor forward, right motor backward.
        """
        self.motor_right.run(-speed)
        self.motor_left.run(speed)

    def turn_left(self, speed=70):
        """
        Pivot left in place: right motor forward, left motor backward.
        """
        self.motor_right.run(speed)
        self.motor_left.run(-speed)

    def arc_right(self, outer_speed=80, inner_speed=40):
        """Curve right by running the left motor faster than the right."""
        self.motor_right.run(inner_speed)
        self.motor_left.run(outer_speed)

    def arc_left(self, outer_speed=80, inner_speed=40):
        """Curve left by running the right motor faster than the left."""
        self.motor_right.run(outer_speed)
        self.motor_left.run(inner_speed)

    def set_speeds(self, right_speed, left_speed):
        """
        Independently set each motor speed. \n
        Args:
            right_speed: -100 to 100 for the right motor.
            left_speed:  -100 to 100 for the left motor.
        """
        self.motor_right.run(right_speed)
        self.motor_left.run(left_speed)

    def stop(self):
        """Coast both motors to a stop."""
        self.motor_right.stop()
        self.motor_left.stop()

    def brake(self):
        """Hard-brake both motors (immediate stop)."""
        self.motor_right.brake()
        self.motor_left.brake()

    def deinit(self):
        """Release all PWM resources & enter standby."""
        self.standby()
        self.motor_right.deinit()
        self.motor_left.deinit()


#* Quick test
if __name__ == "__main__":
    car = TB6612FNG()

    print("Forward")
    car.forward(70)
    time.sleep(2)

    print("Brake")
    car.brake()
    time.sleep(0.5)

    print("Backward")
    car.backward(70)
    time.sleep(2)

    print("Brake")
    car.brake()
    time.sleep(0.5)

    print("Turn right (pivot)")
    car.turn_right(60)
    time.sleep(1)

    print("Turn left (pivot)")
    car.turn_left(60)
    time.sleep(1)

    print("Stop")
    car.stop()
    car.deinit()

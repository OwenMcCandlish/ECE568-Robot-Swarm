from machine import Pin, time_pulse_us
import time

#Default GPIO pins
TRIG_PIN = 33
ECHO_PIN = 32
class Ultrasonic:
    """
    Wiring:
        VCC -> 5V
        GND -> GND
        TRIG -> TRIG_PIN
        ECHO -> ECHO_PIN
    """

    #Speed of sound (cm per µs)
    SOUND_SPEED_CM_US = 0.0343

    def __init__(self, trig_pin=TRIG_PIN, echo_pin=ECHO_PIN, timeout_us=30000):
        """
        params:
            trig_pin: GPIO number for the TRIG pin.
            echo_pin: GPIO number for the ECHO pin.
            timeout_us: Echo timeout in microseconds (default 30 ms ≈ ~5 m max range).
        """
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.timeout_us = timeout_us
        self.trig.off()

    def get_distance_cm(self):
        """
        Triggers a measurement and returns the distance in centimetres.
        Returns None if no echo is received within the timeout window.
        """
        self.trig.off() #Ensure trigger is low before firing
        time.sleep_us(2)

        #Send 10 µs trigger pulse
        self.trig.on()
        time.sleep_us(10)
        self.trig.off()

        #Measure how long the echo pin stays HIGH
        duration = time_pulse_us(self.echo, 1, self.timeout_us)

        if duration < 0:
            return None  #Timeout: no object detected/out of range

        return (duration * self.SOUND_SPEED_CM_US) / 2

    def get_distance_m(self):
        """Returns the distance in metres, or None on timeout."""
        cm = self.get_distance_cm()
        return cm / 100.0 if cm is not None else None


#Quick test
if __name__ == "__main__":
    sensor = Ultrasonic(trig_pin=TRIG_PIN, echo_pin=ECHO_PIN)

    print("Ultrasonic sensor running. Ctrl+C to stop.")
    while True:
        distance = sensor.get_distance_cm()
        if distance is None:
            print("No object detected (out of range)")
        else:
            print(f"Distance: {distance:.2f} cm ({distance / 100:.3f} m)")
        time.sleep(0.5)

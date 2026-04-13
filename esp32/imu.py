from machine import SoftI2C, Pin
import struct
import time

#!Default I2C pins

SDA_PIN = 22 #!Normal SDA Pin
SCL_PIN = 27 #!SCL Pin wasn't working for me, so i changed it GPIO27
I2C_FREQ = 400000 #400 kHz

STEMMA_POWER_PIN = 2

#!I2C address (0x68 when AD0 is LOW, 0x69 (nice) when AD0 is HIGH)
MPU6050_ADDR = 0x68

#!Registers
_WHO_AM_I = 0x75
_PWR_MGMT_1 = 0x6B
_ACCEL_CONFIG = 0x1C
_GYRO_CONFIG = 0x1B
_ACCEL_XOUT_H = 0x3B #First of 14 consecutive data bytes (accel + temp + gyro)

#!Accelerometer full-scale range options
ACCEL_RANGE_2G = 0x00 #±2 g -> 16384 LSB/g
ACCEL_RANGE_4G = 0x08 #±4 g -> 8192 LSB/g
ACCEL_RANGE_8G = 0x10 #±8 g -> 4096 LSB/g
ACCEL_RANGE_16G = 0x18 #±16 g -> 2048 LSB/g

#!Gyroscope full-scale range options
GYRO_RANGE_250 = 0x00 #±250 °/s -> 131.0 LSB/°/s
GYRO_RANGE_500 = 0x08 #±500 °/s -> 65.5 LSB/°/s
GYRO_RANGE_1000 = 0x10 #±1000 °/s -> 32.8 LSB/°/s
GYRO_RANGE_2000 = 0x18 #±2000 °/s -> 16.4 LSB/°/s

_ACCEL_SENS = {
    ACCEL_RANGE_2G: 16384.0,
    ACCEL_RANGE_4G: 8192.0,
    ACCEL_RANGE_8G: 4096.0,
    ACCEL_RANGE_16G: 2048.0,
}

_GYRO_SENS = {
    GYRO_RANGE_250: 131.0,
    GYRO_RANGE_500: 65.5,
    GYRO_RANGE_1000: 32.8,
    GYRO_RANGE_2000: 16.4,
}

class MPU6050:
    """
    MicroPython driver for the MPU-6050 6-DoF IMU. \n
    Wiring:
        VCC (red) -> 3.3 V
        GND (black) -> GND
        SDA (blue) -> SDA_PIN
        SCL (yellow) -> SCL_PIN
    """

    def __init__(self, i2c=None, addr=MPU6050_ADDR, accel_range=ACCEL_RANGE_2G, gyro_range=GYRO_RANGE_250):
        """
        Args:
            i2c: I2C instance, or keep empty for "None" to create one using SDA_PIN / SCL_PIN defaults.
            addr: I2C address (0x68 or 0x69 (nice)).
            accel_range: Accelerometer full-scale range constant.
            gyro_range: Gyroscope full-scale range constant.
        """
        if i2c is None:
            #Enable STEMMA QT 3.3V power rail
            stemma_pwr = Pin(STEMMA_POWER_PIN, Pin.OUT)
            stemma_pwr.on()
            time.sleep_ms(10) #Allow power rail to stabilise
            i2c = SoftI2C(sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)
        self.i2c = i2c
        self.addr = addr

        #Verify device identity
        who = self._read_byte(_WHO_AM_I)
        if who != 0x68:
            raise RuntimeError("MPU-6050 not found at 0x{:02X} (WHO_AM_I returned 0x{:02X})".format(addr, who))

        #Wake the sensor (clear sleep bit in PWR_MGMT_1)
        self._write_byte(_PWR_MGMT_1, 0x00)
        time.sleep_ms(100) #Allow sensor to stabilise

        #Apply initial range settings
        self._accel_range = None
        self._gyro_range = None
        self.set_accel_range(accel_range)
        self.set_gyro_range(gyro_range)

        #Calibration offsets (zeroed until calibrate() is called)
        self._accel_offset = (0.0, 0.0, 0.0)
        self._gyro_offset  = (0.0, 0.0, 0.0)

    #!Register helpers
    def _write_byte(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read_byte(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    #!Configuration
    def set_accel_range(self, range_val):
        """Set accelerometer full-scale range (use ACCEL_RANGE_* constants)."""
        if range_val not in _ACCEL_SENS:
            raise ValueError("Invalid accel range. Use an ACCEL_RANGE_* constant.")
        self._write_byte(_ACCEL_CONFIG, range_val)
        self._accel_range = range_val

    def set_gyro_range(self, range_val):
        """Set gyroscope full-scale range (use GYRO_RANGE_* constants)."""
        if range_val not in _GYRO_SENS:
            raise ValueError("Invalid gyro range. Use a GYRO_RANGE_* constant.")
        self._write_byte(_GYRO_CONFIG, range_val)
        self._gyro_range = range_val

    #!Calibration
    def calibrate(self, samples=200):
        """
        Place the sensor **FLAT** and **STILL** before calling this.
        Collects sample readings, averages them, then stores:
          -Accel offsets so that ax=0, ay=0, az=1.0 when flat.
          -Gyro offsets so that gx=gy=gz=0 when still.

        Args:
            samples: Number of readings to average (default 200).
        """
        print("Calibrating IMU, keep sensor flat and still...")
        ax_sum = ay_sum = az_sum = 0.0
        gx_sum = gy_sum = gz_sum = 0.0

        for _ in range(samples):
            raw = self.i2c.readfrom_mem(self.addr, _ACCEL_XOUT_H, 14)
            ax, ay, az, _, gx, gy, gz = struct.unpack('>7h', raw)
            a_s = _ACCEL_SENS[self._accel_range]
            g_s = _GYRO_SENS[self._gyro_range]
            ax_sum += ax / a_s
            ay_sum += ay / a_s
            az_sum += az / a_s
            gx_sum += gx / g_s
            gy_sum += gy / g_s
            gz_sum += gz / g_s
            time.sleep_ms(2)

        #Accel: ax/ay should be 0, az should be 1.0 (good ol' gravity)
        self._accel_offset = (
            ax_sum / samples,
            ay_sum / samples,
            az_sum / samples - 1.0
        )
        #Gyro: all axes should be 0
        self._gyro_offset = (
            gx_sum / samples,
            gy_sum / samples,
            gz_sum / samples
        )
        print("Calibration done.")
        print("Accel offsets: ax={:.4f} ay={:.4f} az={:.4f}".format(*self._accel_offset))
        print("Gyro offsets:  gx={:.4f} gy={:.4f} gz={:.4f}".format(*self._gyro_offset))

    #!Sensor reads
    def get_accel(self):
        """
        Read accelerometer data.
        Returns:
            (ax, ay, az) in g (1 g ≈ 9.81 m/s^2)
        """
        raw = self.i2c.readfrom_mem(self.addr, _ACCEL_XOUT_H, 6)
        vals = struct.unpack('>3h', raw)
        s = _ACCEL_SENS[self._accel_range]
        ox, oy, oz = self._accel_offset
        return vals[0] / s - ox, vals[1] / s - oy, vals[2] / s - oz

    def get_gyro(self):
        """
        Read gyroscope data.
        Returns:
            (gx, gy, gz) in degrees/sec
        """
        raw = self.i2c.readfrom_mem(self.addr, 0x43, 6)
        vals = struct.unpack('>3h', raw)
        s = _GYRO_SENS[self._gyro_range]
        ox, oy, oz = self._gyro_offset
        return vals[0] / s - ox, vals[1] / s - oy, vals[2] / s - oz

    def get_temperature(self):
        """
        Read the on-chip temperature sensor.
        Returns:
            Temperature in Celsius
        """
        raw = self.i2c.readfrom_mem(self.addr, 0x41, 2)
        val = struct.unpack('>h', raw)[0]
        return (val / 340.0) + 36.53

    def get_all(self):
        """
        Read all sensor data in 14-byte I2C transaction. \n
        Returns:
            { \n
                'accel': (ax, ay, az), #in g \n
                'gyro': (gx, gy, gz), #in degrees/second \n
                'temp': t #in °C \n
            }
        """
        raw = self.i2c.readfrom_mem(self.addr, _ACCEL_XOUT_H, 14)
        ax, ay, az, raw_t, gx, gy, gz = struct.unpack('>7h', raw)
        a_s = _ACCEL_SENS[self._accel_range]
        g_s = _GYRO_SENS[self._gyro_range]
        aox, aoy, aoz = self._accel_offset
        gox, goy, goz = self._gyro_offset
        return {
            'accel': (ax / a_s - aox, ay / a_s - aoy, az / a_s - aoz),
            'gyro':  (gx / g_s - gox, gy / g_s - goy, gz / g_s - goz),
            'temp':  (raw_t / 340.0) + 36.53,
        }

#?Quick test
if __name__ == "__main__":
    imu = MPU6050()
    imu.calibrate()

    print("MPU-6050 found. Reading sensor data. Press Ctrl+C to stop.\n")
    while True:
        data = imu.get_all()
        ax, ay, az = data['accel']
        gx, gy, gz = data['gyro']
        temp = data['temp']
        print("Accel (g) ax={:+.3f} ay={:+.3f} az={:+.3f}".format(ax, ay, az))
        print("Gyro (d/s) gx={:+.3f} gy={:+.3f} gz={:+.3f}".format(gx, gy, gz))
        print("Temp (C) {:.2f}\n".format(temp))
        time.sleep(0.5)


from time import sleep
from struct import unpack_from
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device

# from adafruit_register.i2c_struct import ROUnaryStruct, Struct
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_struct import UnaryStruct
# from adafruit_register.i2c_bit import RWBit

__version__ = "0.0.0-auto.0"
__repo__ = "https:#github.com/adafruit/Adafruit_CircuitPython_MLX90395.git"

_DEFAULT_ADDR = const(0x0C)  ## Can also be 0x18, depending on IC */

_STATUS_RESET = const(0x02)
_STATUS_SMMODE = const(0x20)
_STATUS_DRDY = const(0x01)
_REG_0 = const(0x0)
_REG_1 = const(0x2)
_REG_2 = const(0x4)
_REG_SM = const(0x30)
_REG_EX = const(0x80)
_REG_RT = const(0xF0)
GAIN_AMOUNT = [
    0.2,
    0.25,
    0.3333,
    0.4,
    0.5,
    0.6,
    0.75,
    1,
    0.1,
    0.125,
    0.1667,
    0.2,
    0.25,
    0.3,
    0.375,
    0.5,
]

class CV:
    """struct helper"""

    @classmethod
    def add_values(cls, value_tuples):
        "creates CV entires"
        cls.string = {}
        cls.lsb = {}

        for value_tuple in value_tuples:
            name, value, string, lsb = value_tuple
            setattr(cls, name, value)
            cls.string[value] = string
            cls.lsb[value] = lsb

    @classmethod
    def is_valid(cls, value):
        "Returns true if the given value is a member of the CV"
        return value in cls.string

class OSR(CV):
    """Options for ``oversample_rate``"""


OSR.add_values(
    (
        ("RATE_1X", 0, 1, None),
        ("RATE_2X", 1, 2, None),
        ("RATE_4X", 2, 4, None),
        ("RATE_8X", 3, 8, None),
    )
)

class MLX90395:
    """Class for interfacing with the MLX90395 3-axis magnetometer"""

    _gain = RWBits(4, _REG_0, 4, 2, False)

    _resolution = RWBits(2, _REG_2, 5, 2, False)
    _filter = RWBits(3, _REG_2, 2, 2, False)
    _osr = RWBits(2, _REG_2, 0, 2, False)
    _reg0 = UnaryStruct(_REG_0, ">H",)
    _reg2 = UnaryStruct(_REG_2, ">H")
    def __init__(self, i2c_bus, address=_DEFAULT_ADDR):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._ut_lsb = None
        self._gain_val = 0
        self._buffer = bytearray(12)

        self.reset()
        self.initialize()

    def reset(self):
        """Reset the sensor to it's power-on state"""
        print("reset here plz")
        # exitMode()
        # do once and ignore status, then once to do it?
        status = self._command(_REG_EX)
        status = self._command(_REG_EX)

        sleep(0.10)
        if self._command(_REG_RT) != _STATUS_RESET:
            raise RuntimeError("Unable to reset!")

        sleep(0.10)

    def initialize(self):
        """Configure the sensor for use"""
        self._gain_val = self.gain
        print("gain is", self._gain_val)
        if self._gain_val == 8:  # default high field gain
            print("lsb is 7.14")
            self._ut_lsb = 7.14
        else:
            print("LSB IS WAT", 2.5)
            self._ut_lsb = 2.5  # medium field gain

    @property
    def resolution(self):
        """/**
 * Get the resolution, note that its till 16 bits just offset within the 19
 * bit ADC output.
 * @return MLX90395_RES_16, MLX90395_RES_17, MLX90395_RES_18 or
 *  MLX90395_RES_19
 */"""
        return self._resolution

    @property
    def gain(self):
        return self._gain
    
    @gain.setter
    def gain(self, value):
        if not value in GAIN_AMOUNT:
            raise AttributeError("gain must be a valid value")
        self._gain = value
        self._gain_val = value

    def _command(self, command_id):

        buffer = bytearray([0x80, command_id])
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer, buffer, in_end=1)
        return buffer[0]
    @property
    def magnetic(self):
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats.
        """
        if self._command(_REG_SM | 0x0F) != _STATUS_SMMODE:
            raise RuntimeError("Unable to initiate a single reading")
        res = self._read_measurement()
        while res is None:
            sleep(0.001)
            res = self._read_measurement()

        return res

    def _read_measurement(self):
      # uint8_t tx[1] = {0x80}; // Read memory command
      # uint8_t rx[12] = {0};   // status, crc, X16, Y16, Z16, T16, V16
      # clear the buffer
      for i in range(len(self._buffer)): self._buffer[i] = 0
      self._buffer[0] = 0x80 # read memory command

      with self.i2c_device as i2c:
        i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)

      if self._buffer[0] != _STATUS_DRDY: return None

      self._buffer = bytearray([0x1, 0xDF, 0xFF, 0xED, 0xFF, 0xD6, 0x0, 0x7, 0x4, 0xC6, 0x0, 0x0])
      xi, yi, zi = unpack_from(">hhh", self._buffer, offset=2)
      print("xi", xi, "yi", yi, "zi", zi)
      # xi -19 yi -42 zi 7

      scalar = GAIN_AMOUNT[self._gain_val] * self._ut_lsb
      print("scalar:", scalar, "self._ut_lsb", self._ut_lsb, "GAIN_AMOUNT[self._gain_val]", GAIN_AMOUNT[self._gain_val], "(gain_val:", self._gain_val)
      return (xi * scalar, yi * scalar, zi * scalar)
    
    @property
    def oversample_rate(self):
        """The number of times that the measurements are re-sampled to reduce noise"""
        return self._osr

    @oversample_rate.setter
    def oversample_rate(self, value):
        if not OSR.is_valid(value):
            raise AttributeError("oversample_rate must be an OSR")
        self._osr = value
    
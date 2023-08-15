from .buttons import Button
from machine import Pin, PWM
from array import array
import time
import rp2
from rp2 import PIO

CHANNELS = const(5)
TIMEOUT = const(1024)
_DONE = const(0)
_READ_LINE = const(1)
_state = _DONE
_qtr = None

class QTRSensors:
    """A multi-channel QTR sensor reader using PIO"""
    @rp2.asm_pio(
        out_init=(PIO.OUT_HIGH,) * CHANNELS,
        autopush=True, # saves push instructions
        push_thresh=CHANNELS + 16,
        fifo_join=PIO.JOIN_RX
        )
    def counter():
        # Set OSR to 32 bits of 1s for future shifting out to intialize pindirs,
        # y, y again, and x. This requires CHANNELS + 8 + 10 + CHANNELS bits
        # (maximum of 32 bits for CHANNELS = 7).
        mov(osr, invert(null))

        # Set pindirs to 1s to enable output and start charging
        # the capacitor.
        out(pindirs, CHANNELS)

        # Charge up the capacitors for ~32us.
        # Set Y counter to 255 by pulling another 8 bits from OSR.
        out(y, 8)
        label("charge")
        jmp(y_dec, "charge")

        # Load 1023 (10 bits of 1s) into Y as a counter
        out(y, 10)

        # Initialize X (last pin state) to 1s.
        out(x, CHANNELS)

        # Set pins back to inputs by writing 0s to pindirs.
        mov(osr, null)
        out(pindirs, CHANNELS)

        # loop is 8 instructions long = 1us
        label("loop")

        # read pins into ISR
        in_(pins, CHANNELS)

        # save y in OSR
        mov(osr, y)

        # compare x to ISR
        mov(y, isr) # new value -> y
        jmp(x_not_y, "change")

        # discard the pin values and reset shift counter
        mov(isr, null)
        jmp("decrement")

        # a pin changed!
        label("change")
        mov(x, y) # save new pins

        # save and write data
        # 7 pins are in low bits of ISR
        in_(osr, 16) # time

        label("decrement")
        mov(y, osr) # restore y
        jmp(y_dec, "loop")

        # END OF PROGRAM
        label("finish")

        # Send 0xFFFFFFFF to tell the CPU we are done.
        in_(y, 32)

        wrap_target()
        nop()
        wrap()

    def __init__(self, id, pin1):
        for i in range(CHANNELS):
            Pin(pin1+i, Pin.IN, pull=None)

        p = Pin(pin1, Pin.OUT, value=1)
        for i in range(1, CHANNELS):
            Pin(pin1+i, Pin.OUT, value=1)

        self.sm = rp2.StateMachine(id, self.counter, freq=8000000, in_base=p, out_base=p)
        self.data_line = array('H', [0] * 5)

    def run(self):
        while self.sm.rx_fifo():
            self.sm.get()
        self.sm.restart()
        self.sm.active(1)

    @micropython.viper
    def read_line(self):
        last_states = uint(0x7f0000)
        data = ptr16(self.data_line)
        for i in range(5):
            data[i] = TIMEOUT

        sm = self.sm
        while True:  # TODO: TIMEOUT?
            val = uint(sm.get())
            if(val == uint(0xffffffff)):
                break
            new_zeros = last_states ^ val
            if new_zeros & 0x10000:
                data[4] = TIMEOUT - val
            if new_zeros & 0x20000:
                data[3] = TIMEOUT - val
            if new_zeros & 0x40000:
                data[2] = TIMEOUT - val
            if new_zeros & 0x80000:
                data[1] = TIMEOUT - val
            if new_zeros & 0x100000:
                data[0] = TIMEOUT - val
            last_states = val
        return self.data_line

class _IRSensors():
    def __init__(self):
        self.ir_down = Pin(26, Pin.IN)

        global _qtr
        if not _qtr:
            _qtr = QTRSensors(4, 18)
        self.qtr = _qtr

        self.reset_calibration()

class LineSensors(_IRSensors):
    def _state(self):
        # for testing
        return _state

    def reset_calibration(self):
        self.cal_min = array('H', [1025] * 5)
        self.cal_max = array('H', [0] * 5)

    def calibrate(self):
        tmp_min = array('H', [1025] * 5)
        tmp_max = array('H', [0] * 5)

        # do 10 measurements
        for trials in range(10):
            data = self.read()
            for i in range(5):
                tmp_min[i] = min(data[i], tmp_min[i])
                tmp_max[i] = max(data[i], tmp_max[i])

        # update data only if ALL data beyond one of the limits
        for i in range(5):
            self.cal_max[i] = max(tmp_min[i], self.cal_max[i])
            self.cal_min[i] = min(tmp_max[i], self.cal_min[i])

    def start_read(self):
        global _state
        self.ir_down.init(Pin.OUT, value=1)
        _state = _READ_LINE
        self.qtr.run()

    @micropython.viper
    def read(self):
        global _state
        if uint(_state) != uint(_READ_LINE):
            self.start_read()
        data = self.qtr.read_line()

        self.ir_down.init(Pin.IN)
        _state = _DONE
        return data

    @micropython.viper
    def read_calibrated(self):
        data = self.read()
        d = ptr16(data)
        cal_min = ptr16(self.cal_min)
        cal_max = ptr16(self.cal_max)
        for i in range(5):
            if cal_min[i] >= cal_max[i] or d[i] < cal_min[i]:
                d[i] = 0
            elif d[i] > cal_max[i]:
                d[i]= 1000
            else:
               d[i] = (d[i] - cal_min[i])*1000 // (cal_max[i] - cal_min[i])
        return data


DEFAULT_FREQ = const(56000)

# According to the TSSP770 datasheet, the delay between the start of the IR
# pulses and the start of the sensor output pulse could be anywhere between
# 7/(56 kHz) and 13/(56 kHz).
#
# The default pulse on time of 14/(56 kHz) = 250 us guarantees we are not
# missing output pulses by reading the sensor too soon.
#
# TODO: allow configuring?
PULSE_ON_TIME_US = const(250)

# According to the TSSP770 datasheet, the sensor output pulse duration
# could be up to 4/(56 kHz) longer than the duration of the IR pulses,
# and the sensor output pulse could start as late as 13/(56 kHz) after
# the IR pulses start.  Therefore, it is possible for the sensor output
# pulse to end up to 17/(56 kHz) after the ending of the IR pulses.
#
# So the default off time is 18/(56 kHz) = 321 us.
#
# TODO: allow configuring?
PULSE_OFF_TIME_US = const(321)

_ir_pulses = None

class IRPulses:
    def __init__(self):
        self.right_pulses_pwm_pin = Pin(16, Pin.OUT, value=0)
        self.left_pulses_pwm_pin = Pin(17, Pin.OUT, value=0)
        self.right_pulses_pwm = PWM(self.right_pulses_pwm_pin, freq=DEFAULT_FREQ, duty_ns=0)
        self.left_pulses_pwm = PWM(self.left_pulses_pwm_pin, freq=DEFAULT_FREQ, duty_ns=0)

    def set_freq(self, freq):
        self.right_pulses_pwm.freq(freq)
        # TODO: limit or reset duty cycle (brightness) if out of range?

    def set_brightnesses(self, left, right):
        self.left_pulses_pwm.duty_ns(left)
        self.right_pulses_pwm.duty_ns(right)

    def set_left_brightness(self, brightness):
        self.left_pulses_pwm.duty_ns(brightness)

    def set_right_brightness(self, brightness):
        self.right_pulses_pwm.duty_ns(brightness)

    def off(self):
        self.set_brightnesses(0, 0)

BRIGHTNESSES = [313, 1000, 2063, 3500, 5375, 7563] # TODO: allow configuring?

class ProximitySensors:
    def __init__(self):
        global _ir_pulses
        if not _ir_pulses:
            _ir_pulses = IRPulses()
        self.ir_pulses = _ir_pulses

        # TODO: allow remapping?
        self.left = Pin(23, Pin.IN)
        self.right = Pin(24, Pin.IN)
        self.front = Pin(27, Pin.IN)
        self.ir_down = Pin(26)

        self.sensors = [self.left, self.front, self.right]
        self.counts = [[0, 0], [0, 0], [0, 0]]

    def _prepare_to_read(self):
        # pull-ups on
        self.left.init(Pin.IN, Pin.PULL_UP)
        self.right.init(Pin.IN, Pin.PULL_UP)
        self.front.init(Pin.IN, Pin.PULL_UP)

        # line sensor emitters off
        self.ir_down.init(Pin.OUT, value=0)
        time.sleep_us(PULSE_OFF_TIME_US)

    def read(self):
        self._prepare_to_read()

        self.counts = [[0, 0], [0, 0], [0, 0]]

        for brightness in BRIGHTNESSES:
            self.ir_pulses.set_brightnesses(brightness, 0)
            time.sleep_us(PULSE_ON_TIME_US)
            for i, sensor in enumerate(self.sensors):
                if not sensor.value(): self.counts[i][0] += 1

            self.ir_pulses.off()
            time.sleep_us(PULSE_OFF_TIME_US)

            self.ir_pulses.set_brightnesses(0, brightness)
            time.sleep_us(PULSE_ON_TIME_US)
            for i, sensor in enumerate(self.sensors):
                if not sensor.value(): self.counts[i][1] += 1

            self.ir_pulses.off()
            time.sleep_us(PULSE_OFF_TIME_US)

    def counts_with_left_leds(self, sensor_number):
        return self.counts[sensor_number][0]

    def counts_with_right_leds(self, sensor_number):
        return self.counts[sensor_number][1]

    def left_counts_with_left_leds(self):
        return self.counts_with_left_leds(0)

    def left_counts_with_right_leds(self):
        return self.counts_with_right_leds(0)

    def front_counts_with_left_leds(self):
        return self.counts_with_left_leds(1)

    def front_counts_with_right_leds(self):
        return self.counts_with_right_leds(1)

    def right_counts_with_left_leds(self):
        return self.counts_with_left_leds(2)

    def right_counts_with_right_leds(self):
        return self.counts_with_right_leds(2)
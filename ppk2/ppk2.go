// SPDX-License-Identifier: GPL-2.0
//
// This python API is written for use with the Nordic Semiconductor's Power
// Profiler Kit II (PPK 2).
//
// The PPK2 uses Serial communication. The unofficial ppk2-api-python
// (https://github.com/IRNAS/ppk2-api-python) and the official nRF Connect Power
// Profiler (https://github.com/NordicSemiconductor/pc-nrfconnect-ppk) was used
// as a reference

package ppk2

import (
	"context"
	"fmt"
	"math"
	"strconv"
	"strings"
	"time"

	"go.bug.st/serial"
	"golang.org/x/exp/constraints"
)

const (
	NoOp               = 0x00
	TriggerSet         = 0x01
	AvgNumSet          = 0x02 // No-Firmware
	TriggerWindowSet   = 0x03
	TriggerIntervalSet = 0x04
	TriggerSingleSet   = 0x05
	AverageStart       = 0x06
	AverageStop        = 0x07
	RangeSet           = 0x08
	LcdSet             = 0x09
	TriggerStop        = 0x0a
	DeviceRunningSet   = 0x0c
	RegulatorSet       = 0x0d
	SwitchPointDown    = 0x0e
	SwitchPointUp      = 0x0f
	TriggerExtToggle   = 0x11
	SetPowerMode       = 0x11
	ResUserSet         = 0x12
	SpikeFilteringOn   = 0x15
	SpikeFilteringOff  = 0x16
	GetMetaData        = 0x19
	Reset              = 0x20
	SetUserGains       = 0x25
)

// PPK2 measurement modes
const (
	AmpereMode = 1
	SourceMode = 2
)

const (
	VddLow  = uint(800)
	VddHigh = uint(5000)
)

const (
	AdcMult = 1.8 / 163840
)

type Mask struct {
	Msk uint32
	Pos uint32
}

func (m Mask) GetMaskValue(i uint32) uint32 {
	return (i & m.Msk) >> m.Pos
}

var ( //             32bit: 0b00000000000000000000000000000000
	MeasAdc     = Mask{Msk: 0b00000000000000000011111111111111, Pos: 0}  // 14bits, pos 0
	MeasRange   = Mask{Msk: 0b00000000000000011100000000000000, Pos: 14} // 3bits, pos 14
	MeasCounter = Mask{Msk: 0b00000000111111000000000000000000, Pos: 18} // 6bits, pos 18
	MeasLogic   = Mask{Msk: 0b11111111000000000000000000000000, Pos: 24} // 8bits, pos 24
)

type modifiers struct {
	R  [5]float64
	GS [5]float64
	GI [5]float64
	O  [5]float64
	S  [5]float64
	I  [5]float64
	UG [5]float64
	IA int
}

func min[T constraints.Ordered](a, b T) T {
	if a < b {
		return a
	}
	return b
}

func max[T constraints.Ordered](a, b T) T {
	if a > b {
		return a
	}
	return b
}

type Sample struct {
	Adc   float64
	Logic uint8
	Cnt   uint8
}

type Samples []Sample

func (s Samples) Average() float64 {
	if len(s) == 0 {
		return 0
	}

	var res float64
	for i := range s {
		res += s[i].Adc
	}
	return res / float64(len(s))
}

func (s Samples) MinMax() (float64, float64) {
	if len(s) == 0 {
		return 0, 0
	}

	mi := s[0].Adc
	ma := s[0].Adc
	for i := range s[1:] {
		mi = min(s[i].Adc, mi)
		ma = max(s[i].Adc, ma)
	}
	return mi, ma
}

type PPK2 struct {
	mods                    modifiers
	currentVdd              uint
	mode                    int
	port                    serial.Port
	rollingAvg              float64
	rollingAvg4             float64
	spikeFilterAlpha        float64
	spikeFilterAlpha5       float64
	spikeFilterSamples      uint
	afterSpike              uint
	consecutiveRangeSamples uint
	prevRange               uint8
	remainder               []byte
	hw                      uint
	calibrated              bool

	running       bool
	portData      chan []byte
	readerStopped chan struct{}
	readerData    chan Samples
}

type Option func(*PPK2)

func WithOption(p string) Option {
	return func(p *PPK2) {

	}
}

func NewPPK2(port string, opts ...Option) (*PPK2, error) {
	sp, err := serial.Open(port, &serial.Mode{BaudRate: 9600})
	if err != nil {
		return nil, err
	}
	sp.SetReadTimeout(time.Millisecond)

	p := &PPK2{
		mods: modifiers{
			R:  [5]float64{1031.64, 101.65, 10.15, 0.94, 0.043},
			GS: [5]float64{1, 1, 1, 1, 1},
			GI: [5]float64{1, 1, 1, 1, 1},
			O:  [5]float64{0, 0, 0, 0, 0},
			S:  [5]float64{0, 0, 0, 0, 0},
			I:  [5]float64{0, 0, 0, 0, 0},
			UG: [5]float64{1, 1, 1, 1, 1},
		},
		rollingAvg:         math.MaxFloat64,
		rollingAvg4:        math.MaxFloat64,
		spikeFilterAlpha:   0.18,
		spikeFilterAlpha5:  0.06,
		spikeFilterSamples: 3,
		prevRange:          math.MaxInt8,
		port:               sp,
	}

	for _, opt := range opts {
		opt(p)
	}

	return p, nil
}

// StartMeasuring starts a continuous measurement
func (p *PPK2) StartMeasuring() error {
	if p.currentVdd == 0 {
		switch p.mode {
		case SourceMode:
			return fmt.Errorf("output voltage not set")
		case AmpereMode:
			return fmt.Errorf("input voltage not set")
		}
	}

	return p.WriteCmd(AverageStart)
}

// StopMeasuring stops the continuous measurement
func (p *PPK2) StopMeasuring() error {
	return p.WriteCmd(AverageStop)
}

// SetSourceVoltage inits device - based on observation only REGULATOR_SET is the command.
// The other two values correspond to the voltage level.
//
// 800mV is the lowest setting - [3,32] - the values then increase linearly
func (p *PPK2) SetSourceVoltage(mV uint) error {
	b_1, b_2 := convertSourceVoltage(mV)
	if err := p.WriteCmd(RegulatorSet, b_1, b_2); err != nil {
		return err
	}
	p.currentVdd = mV
	return nil
}

// ToggleDUTPower toggles DUT power based on parameter
func (p *PPK2) ToggleDUTPower(state bool) error {
	if state {
		return p.WriteCmd(DeviceRunningSet, TriggerSet)
	} else {
		return p.WriteCmd(DeviceRunningSet, NoOp)
	}
}

// UseAmpereMeter configures the device to use ampere meter
func (p *PPK2) UseAmpereMeter() error {
	p.mode = AmpereMode
	return p.WriteCmd(SetPowerMode, TriggerSet)
}

// UseSourceMeter configures the device to use source meter
func (p *PPK2) UseSourceMeter() error {
	p.mode = SourceMode
	return p.WriteCmd(SetPowerMode, AvgNumSet)
}

// GetRawData return all data in the serial buffer
func (p *PPK2) GetRawData() ([]byte, error) {
	b := make([]byte, 2048)
	n, err := p.port.Read(b)
	return b[:n], err
}

// GetModifiers gets modifiers from device memory
func (p *PPK2) GetModifiers() error {
	if err := p.WriteCmd(GetMetaData); err != nil {
		return err
	}

	meta := ""
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	// try to get metadata from device
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
		}

		// it appears the second reading is the metadata
		read, _ := p.GetRawData()
		meta += string(read)

		if idx := strings.Index(meta, "END"); idx != -1 {
			meta = meta[:idx-1] // trailling '\n'
			cancel()
			break
		}

		time.Sleep(time.Second / 10)
	}

	parseFloat := func(v string, d *float64) {
		f, err := strconv.ParseFloat(v, 32)
		if err == nil {
			*d = f
			return
		}
		fmt.Printf("err: %v\n", err)
	}

	for _, i := range strings.Split(meta, "\n") {
		s := strings.Split(i, ":")
		key := s[0]
		val := strings.Trim(s[1], " ")

		switch key {
		case "Calibrated":
			if i, err := strconv.Atoi(val); err == nil {
				p.calibrated = i != 0
			}
		case "R0":
			parseFloat(val, &p.mods.R[0])
		case "R1":
			parseFloat(val, &p.mods.R[1])
		case "R2":
			parseFloat(val, &p.mods.R[2])
		case "R3":
			parseFloat(val, &p.mods.R[3])
		case "R4":
			parseFloat(val, &p.mods.R[4])
		case "GS0":
			parseFloat(val, &p.mods.GS[0])
		case "GS1":
			parseFloat(val, &p.mods.GS[1])
		case "GS2":
			parseFloat(val, &p.mods.GS[2])
		case "GS3":
			parseFloat(val, &p.mods.GS[3])
		case "GS4":
			parseFloat(val, &p.mods.GS[4])
		case "GI0": //  1.0000
			parseFloat(val, &p.mods.GI[0])
		case "GI1": //  0.9481
			parseFloat(val, &p.mods.GI[1])
		case "GI2": //  0.9340
			parseFloat(val, &p.mods.GI[2])
		case "GI3": //  0.9161
			parseFloat(val, &p.mods.GI[3])
		case "GI4": //  0.9300
			parseFloat(val, &p.mods.GI[4])
		case "O0": //  197.3432
			parseFloat(val, &p.mods.O[0])
		case "O1": //  138.9471
			parseFloat(val, &p.mods.O[1])
		case "O2": //  118.1641
			parseFloat(val, &p.mods.O[2])
		case "O3": //  91.8737
			parseFloat(val, &p.mods.O[3])
		case "O4": //  158.0201
			parseFloat(val, &p.mods.O[0])
		case "VDD": //  3300
			if i, err := strconv.Atoi(val); err == nil {
				p.currentVdd = uint(i)
			}
		case "HW": //  34504
			if i, err := strconv.Atoi(val); err == nil {
				p.hw = uint(i)
			}
		case "mode": //  1
			if i, err := strconv.Atoi(val); err == nil {
				p.mode = i
			}
		case "S0": //  -0.000000251
			parseFloat(val, &p.mods.S[0])
		case "S1": //  -0.000002386
			parseFloat(val, &p.mods.S[1])
		case "S2": //  -0.000023209
			parseFloat(val, &p.mods.S[2])
		case "S3": //  -0.000240963
			parseFloat(val, &p.mods.S[3])
		case "S4": //  -0.001672392
			parseFloat(val, &p.mods.S[4])
		case "I0": //  0.000000632
			parseFloat(val, &p.mods.I[0])
		case "I1": //  0.000006073
			parseFloat(val, &p.mods.I[1])
		case "I2": //  0.000141188
			parseFloat(val, &p.mods.I[2])
		case "I3": //  0.000209487
			parseFloat(val, &p.mods.I[3])
		case "I4": //  0.002355191
			parseFloat(val, &p.mods.I[4])
		case "UG0": //  1.00
			parseFloat(val, &p.mods.UG[0])
		case "UG1": //  1.00
			parseFloat(val, &p.mods.UG[1])
		case "UG2": //  1.00
			parseFloat(val, &p.mods.UG[2])
		case "UG3": //  1.00
			parseFloat(val, &p.mods.UG[3])
		case "UG4": //  1.00
			parseFloat(val, &p.mods.UG[4])
		case "IA": //  57
			if i, err := strconv.Atoi(val); err == nil {
				p.mods.IA = i
			}
		}
	}

	return nil
}

// GetRollingAverage return two average value, filtered differently
func (p *PPK2) GetRollingAverage() (float64, float64) {
	return p.rollingAvg * math.Pow(10, 6), p.rollingAvg4 * math.Pow(10, 6)
}

// GetSamples returns the samples converted from the raw values in the provided buffer
func (p *PPK2) GetSamples(buf []byte) Samples {
	const sample_size = 4 // one analog value is 4 bytes in size
	offset := len(p.remainder)
	samples := make(Samples, 0, len(buf)/sample_size+2)
	var v uint32

	p.remainder = append(p.remainder, buf[0:min(sample_size-offset, len(buf))]...)
	if len(p.remainder) < sample_size {
		return samples
	}

	v, _ = newBytesReader(p.remainder).ReadUint32()
	a, l, c := p.handleRawData(v)
	samples = append(samples, Sample{Adc: a, Logic: l, Cnt: c})

	r := newBytesReader(buf[min(len(buf), sample_size-offset):])

	for r.Len() >= sample_size {
		v, _ = r.ReadUint32()
		a, l, c = p.handleRawData(v)
		samples = append(samples, Sample{Adc: a, Logic: l, Cnt: c})
	}

	p.remainder = make([]byte, r.Len())
	r.Read(p.remainder)

	return samples
}

// StartReader starts the reader and returns a channel which will provide the samples
func (p *PPK2) StartReader() chan Samples {
	p.readerData = make(chan Samples, 10000)
	p.portData = make(chan []byte, 10)
	p.readerStopped = make(chan struct{})

	p.StartMeasuring()
	go p.reader()

	return p.readerData
}

// StopReader stops the reader and closes the channel
func (p *PPK2) StopReader() {
	p.StopMeasuring()
	p.running = false
	<-p.readerStopped
}

func (p *PPK2) reader() {
	defer close(p.portData)

	p.running = true
	go p.converter()

	for p.running {
		d, _ := p.GetRawData()
		if len(d) > 0 {
			p.portData <- d
		}
	}
}

func (p *PPK2) converter() {
	defer func() {
		close(p.readerData)
		close(p.readerStopped)
	}()

	for {
		d := <-p.portData
		if len(d) == 0 {
			break
		}
		s := p.GetSamples(d)
		if len(s) > 0 {
			p.readerData <- s
		}
	}
}

// WriteCmd writes a command with optional data
func (p *PPK2) WriteCmd(cmd ...byte) error {
	_, err := p.port.Write(cmd)
	return err
}

// handleRawData converts the raw value to a analog value
func (p *PPK2) handleRawData(adc_value uint32) (float64, uint8, uint8) {
	current_measurement_range := uint8(min(MeasRange.GetMaskValue(adc_value), uint32(len(p.mods.R)-1)))
	adc_result := MeasAdc.GetMaskValue(adc_value) * 4
	bits := uint8(MeasLogic.GetMaskValue(adc_value))
	cnt := uint8(MeasCounter.GetMaskValue(adc_value))
	analog_value := p.getAdcResult(current_measurement_range, adc_result) * math.Pow(10, 6)
	return analog_value, bits, cnt
}

// getAdcResult converts the adc raw value and calculated a rolling average
func (p *PPK2) getAdcResult(current_range uint8, adc_value uint32) float64 {
	result_without_gain := (float64(adc_value) - p.mods.O[current_range]) * (AdcMult / p.mods.R[current_range])
	adc := p.mods.UG[current_range] * (result_without_gain*(p.mods.GS[current_range]*result_without_gain+p.mods.GI[current_range]) + (p.mods.S[current_range]*(float64(p.currentVdd)/1000) + p.mods.I[current_range]))

	prev_rolling_avg := p.rollingAvg
	prev_rolling_avg4 := p.rollingAvg4

	// spike filtering / rolling average
	if p.rollingAvg == math.MaxFloat64 {
		p.rollingAvg = adc
	} else {
		p.rollingAvg = p.spikeFilterAlpha*adc + (1-p.spikeFilterAlpha)*p.rollingAvg
	}

	if p.rollingAvg4 == math.MaxFloat64 {
		p.rollingAvg4 = adc
	} else {
		p.rollingAvg4 = p.spikeFilterAlpha5*adc + (1-p.spikeFilterAlpha5)*p.rollingAvg4
	}

	if p.prevRange == math.MaxInt8 {
		p.prevRange = current_range
	}

	if p.prevRange != current_range || p.afterSpike > 0 {
		if p.prevRange != current_range {
			p.consecutiveRangeSamples = 0
			p.afterSpike = p.spikeFilterSamples
		} else {
			p.consecutiveRangeSamples += 1
		}

		if current_range == 4 {
			if p.consecutiveRangeSamples < 2 {
				p.rollingAvg = prev_rolling_avg
				p.rollingAvg4 = prev_rolling_avg4
			}
			adc = p.rollingAvg4
		} else {
			adc = p.rollingAvg
		}

		p.afterSpike -= 1
	}

	p.prevRange = current_range
	return adc
}

// convertSourceVoltage converts the input voltage to a device command
func convertSourceVoltage(mV uint) (uint8, uint8) {
	// minimal possible mV is 800
	if mV < VddLow {
		mV = VddLow
	}

	// maximal possible mV is 5000
	if mV > VddHigh {
		mV = VddHigh
	}

	offset := uint(32)
	// get difference to baseline (the baseline is 800mV but the initial offset is 32)
	diff_to_baseline := mV - VddLow + offset
	base_b_1 := uint(3)
	base_b_2 := uint(0) // is actually 32 - compensated with above offset

	// get the number of times we have to increase the first byte of the command
	ratio := uint(diff_to_baseline / 256)
	remainder := diff_to_baseline % 256 // get the remainder for byte 2

	set_b_1 := uint8(base_b_1 + ratio)
	set_b_2 := uint8(base_b_2 + remainder)

	return set_b_1, set_b_2
}

/*
class PPK2_API():
    def __init__(self, port):

        self.ser = None
        self.ser = serial.Serial(port)
        self.ser.baudrate = 9600

        self.modifiers = {
            "Calibrated": None,
            "R": {"0": 1031.64, "1": 101.65, "2": 10.15, "3": 0.94, "4": 0.043},
            "GS": {"0": 1, "1": 1, "2": 1, "3": 1, "4": 1},
            "GI": {"0": 1, "1": 1, "2": 1, "3": 1, "4": 1},
            "O": {"0": 0, "1": 0, "2": 0, "3": 0, "4": 0},
            "S": {"0": 0, "1": 0, "2": 0, "3": 0, "4": 0},
            "I": {"0": 0, "1": 0, "2": 0, "3": 0, "4": 0},
            "UG": {"0": 1, "1": 1, "2": 1, "3": 1, "4": 1},
            "HW": None,
            "IA": None
        }

        self.vdd_low = 800
        self.vdd_high = 5000

        self.current_vdd = None

        self.adc_mult = 1.8 / 163840

        self.MEAS_ADC = self._generate_mask(14, 0)
        self.MEAS_RANGE = self._generate_mask(3, 14)
        self.MEAS_LOGIC = self._generate_mask(8, 24)

        self.mode = None

        self.rolling_avg = None
        self.rolling_avg4 = None
        self.prev_range = None
        self.consecutive_range_samples = 0

        self.spike_filter_alpha = 0.18
        self.spike_filter_alpha5 = 0.06
        self.spike_filter_samples = 3
        self.after_spike = 0

        # adc measurement buffer remainder and len of remainder
        self.remainder = {"sequence": b'', "len": 0}

    def __del__(self):
        """Destructor"""
        try:
            if self.ser:
                self.ser.close()
        except Exception as e:
            logging.error(f"An error occured while closing ppk2_api: {e}")

    def _pack_struct(self, cmd_tuple):
        """Returns packed struct"""
        return struct.pack("B" * len(cmd_tuple), *cmd_tuple)

    def _write_serial(self, cmd_tuple):
        """Writes cmd bytes to serial"""
        try:
            cmd_packed = self._pack_struct(cmd_tuple)
            self.ser.write(cmd_packed)
        except Exception as e:
            logging.error(f"An error occured when writing to serial port: {e}")

    def _twos_comp(self, val):
        """Compute the 2's complement of int32 value"""
        if (val & (1 << (32 - 1))) != 0:
            val = val - (1 << 32)  # compute negative value
        return val

    def _convert_source_voltage(self, mV):
        """Convert input voltage to device command"""
        # minimal possible mV is 800
        if mV < self.vdd_low:
            mV = self.vdd_low

        # maximal possible mV is 5000
        if mV > self.vdd_high:
            mV = self.vdd_high

        offset = 32
        # get difference to baseline (the baseline is 800mV but the initial offset is 32)
        diff_to_baseline = mV - self.vdd_low + offset
        base_b_1 = 3
        base_b_2 = 0  # is actually 32 - compensated with above offset

        # get the number of times we have to increase the first byte of the command
        ratio = int(diff_to_baseline / 256)
        remainder = diff_to_baseline % 256  # get the remainder for byte 2

        set_b_1 = base_b_1 + ratio
        set_b_2 = base_b_2 + remainder

        return set_b_1, set_b_2

    def _read_metadata(self):
        """Read metadata"""
        # try to get metadata from device
        for _ in range(0, 5):
            # it appears the second reading is the metadata
            read = self.ser.read(self.ser.in_waiting)
            time.sleep(0.1)

            # TODO add a read_until serial read function with a timeout
            if read != b'' and "END" in read.decode("utf-8"):
                return read.decode("utf-8")

    def _parse_metadata(self, metadata):
        """Parse metadata and store it to modifiers"""
        # TODO handle more robustly
        try:
            data_split = [row.split(": ") for row in metadata.split("\n")]

            for key in self.modifiers.keys():
                for data_pair in data_split:
                    if key == data_pair[0]:
                        self.modifiers[key] = data_pair[1]
                    for ind in range(0, 5):
                        if key+str(ind) == data_pair[0]:
                            if "R" in data_pair[0]:
                                # problem on some PPK2s with wrong calibration values - this doesn't fix it
                                if float(data_pair[1]) != 0:
                                    self.modifiers[key][str(ind)] = float(
                                        data_pair[1])
                            else:
                                self.modifiers[key][str(ind)] = float(
                                    data_pair[1])
            return True
        except Exception as e:
            # if exception triggers serial port is probably not correct
            return None


    def _get_masked_value(self, value, meas):
        masked_value = (value & meas["mask"]) >> meas["pos"]
        if meas["pos"] == 24:
            if masked_value == 255:
                masked_value = -1
        return masked_value

    def _handle_raw_data(self, adc_value):
        """Convert raw value to analog value"""
        try:
            current_measurement_range = min(self._get_masked_value(
                adc_value, self.MEAS_RANGE), 4)  # 5 is the number of parameters
            adc_result = self._get_masked_value(adc_value, self.MEAS_ADC) * 4
            bits = self._get_masked_value(adc_value, self.MEAS_LOGIC)
            analog_value = self.get_adc_result(
                current_measurement_range, adc_result) * 10**6
            return analog_value
        except Exception as e:
            print("Measurement outside of range!")
            return None

    @staticmethod
    def list_devices():
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if os.name == 'nt':
            devices = [port.device for port in ports if port.description.startswith("nRF Connect USB CDC ACM")]
        else:
            devices = [port.device for port in ports if port.product == 'PPK2']
        return devices

    def get_data(self):
        """Return readings of one sampling period"""
        sampling_data = self.ser.read(self.ser.in_waiting)
        return sampling_data

    def get_modifiers(self):
        """Gets and sets modifiers from device memory"""
        self._write_serial((PPK2_Command.GET_META_DATA, ))
        metadata = self._read_metadata()
        ret = self._parse_metadata(metadata)
        return ret

    def start_measuring(self):
        """Start continuous measurement"""
        if not self.current_vdd:
            if self.mode == PPK2_Modes.SOURCE_MODE:
                raise Exception("Output voltage not set!")
            if self.mode == PPK2_Modes.AMPERE_MODE:
                raise Exception("Input voltage not set!")

        self._write_serial((PPK2_Command.AVERAGE_START, ))

    def stop_measuring(self):
        """Stop continuous measurement"""
        self._write_serial((PPK2_Command.AVERAGE_STOP, ))

    def set_source_voltage(self, mV):
        """Inits device - based on observation only REGULATOR_SET is the command.
        The other two values correspond to the voltage level.

        800mV is the lowest setting - [3,32] - the values then increase linearly
        """
        b_1, b_2 = self._convert_source_voltage(mV)
        self._write_serial((PPK2_Command.REGULATOR_SET, b_1, b_2))
        self.current_vdd = mV

    def toggle_DUT_power(self, state):
        """Toggle DUT power based on parameter"""
        if state == "ON":
            self._write_serial(
                (PPK2_Command.DEVICE_RUNNING_SET, PPK2_Command.TRIGGER_SET))  # 12,1

        if state == "OFF":
            self._write_serial(
                (PPK2_Command.DEVICE_RUNNING_SET, PPK2_Command.NO_OP))  # 12,0

    def use_ampere_meter(self):
        """Configure device to use ampere meter"""
        self.mode = PPK2_Modes.AMPERE_MODE
        self._write_serial((PPK2_Command.SET_POWER_MODE,
                            PPK2_Command.TRIGGER_SET))  # 17,1

    def use_source_meter(self):
        """Configure device to use source meter"""
        self.mode = PPK2_Modes.SOURCE_MODE
        self._write_serial((PPK2_Command.SET_POWER_MODE,
                            PPK2_Command.AVG_NUM_SET))  # 17,2

    def get_adc_result(self, current_range, adc_value):
        """Get result of adc conversion"""
        current_range = str(current_range)
        result_without_gain = (adc_value - self.modifiers["O"][current_range]) * (
            self.adc_mult / self.modifiers["R"][current_range])
        adc = self.modifiers["UG"][current_range] * (result_without_gain * (self.modifiers["GS"][current_range] * result_without_gain + self.modifiers["GI"][current_range]) + (
            self.modifiers["S"][current_range] * (self.current_vdd / 1000) + self.modifiers["I"][current_range]))

        prev_rolling_avg = self.rolling_avg
        prev_rolling_avg4 = self.rolling_avg4

        # spike filtering / rolling average
        if self.rolling_avg is None:
            self.rolling_avg = adc
        else:
            self.rolling_avg = self.spike_filter_alpha * adc + (1 - self.spike_filter_alpha) * self.rolling_avg

        if self.rolling_avg4 is None:
            self.rolling_avg4 = adc
        else:
            self.rolling_avg4 = self.spike_filter_alpha5 * adc + (1 - self.spike_filter_alpha5) * self.rolling_avg4

        if self.prev_range is None:
            self.prev_range = current_range

        if self.prev_range != current_range or self.after_spike > 0:
            if self.prev_range != current_range:
                self.consecutive_range_samples = 0
                self.after_spike = self.spike_filter_samples
            else:
                self.consecutive_range_samples += 1

            if current_range == "4":
                if self.consecutive_range_samples < 2:
                    self.rolling_avg = prev_rolling_avg
                    self.rolling_avg4 = prev_rolling_avg4
                adc = self.rolling_avg4
            else:
                adc = self.rolling_avg

            self.after_spike -= 1

        self.prev_range = current_range
        return adc

    def _digital_to_analog(self, adc_value):
        """Convert discrete value to analog value"""
        return int.from_bytes(adc_value, byteorder="little", signed=False)  # convert reading to analog value

    def get_samples(self, buf):
        """
        Returns list of samples read in one sampling period.
        The number of sampled values depends on the delay between serial reads.
        Manipulation of samples is left to the user.
        See example for more info.
        """

        sample_size = 4  # one analog value is 4 bytes in size
        offset = self.remainder["len"]
        samples = []

        first_reading = (
            self.remainder["sequence"] + buf[0:sample_size-offset])[:4]
        adc_val = self._digital_to_analog(first_reading)
        measurement = self._handle_raw_data(adc_val)
        if measurement is not None:
            samples.append(measurement)

        offset = sample_size - offset

        while offset <= len(buf) - sample_size:
            next_val = buf[offset:offset + sample_size]
            offset += sample_size
            adc_val = self._digital_to_analog(next_val)
            measurement = self._handle_raw_data(adc_val)
            if measurement is not None:
                samples.append(measurement)

        self.remainder["sequence"] = buf[offset:len(buf)]
        self.remainder["len"] = len(buf)-offset

        return samples  # return list of samples, handle those lists in PPK2 API wrapper


class PPK_Fetch(multiprocessing.Process):
    '''
    Background process for polling the data in multiprocessing variant
    '''
    def __init__(self, ppk2, quit_evt, buffer_len_s=10, buffer_chunk_s=0.5):
        super().__init__()
        self._ppk2 = ppk2
        self._quit = quit_evt

        self.print_stats = False
        self._stats = (None, None)
        self._last_timestamp = 0

        self._buffer_max_len = int(buffer_len_s * 100000 * 4)    # 100k 4-byte samples per second
        self._buffer_chunk = int(buffer_chunk_s * 100000 * 4)    # put in the queue in chunks of 0.5s

        # round buffers to a whole sample
        if self._buffer_max_len % 4 != 0:
            self._buffer_max_len = (self._buffer_max_len // 4) * 4
        if self._buffer_chunk % 4 != 0:
            self._buffer_chunk = (self._buffer_chunk // 4) * 4

        self._buffer_q = multiprocessing.Queue()

    def run(self):
        s = 0
        t = time.time()
        local_buffer = b''
        while not self._quit.is_set():
            d = PPK2_API.get_data(self._ppk2)
            tm_now = time.time()
            local_buffer += d
            while len(local_buffer) >= self._buffer_chunk:
                # FIXME: check if lock might be needed when discarding old data
                self._buffer_q.put(local_buffer[:self._buffer_chunk])
                while self._buffer_q.qsize()>self._buffer_max_len/self._buffer_chunk:
                    self._buffer_q.get()
                local_buffer = local_buffer[self._buffer_chunk:]
                self._last_timestamp = tm_now
                #print(len(d), len(local_buffer), self._buffer_q.qsize())

            # calculate stats
            s += len(d)
            dt = tm_now - t
            if dt >= 0.1:
                if self.print_stats:
                    print(f"Samples: {s}, delta time: {dt}")
                self._stats = (s, dt)
                s = 0
                t = tm_now

            time.sleep(0.0001)

        # process would hang on join() if there's data in the buffer after the measurement is done
        while True:
            try:
                self._buffer_q.get(block=False)
            except queue.Empty:
                break

    def get_data(self):
        ret = b''
        count = 0
        while True:
            try:
                ret += self._buffer_q.get(timeout=0.01) # get_nowait sometimes skips a chunk for some reason
                count += 1
            except queue.Empty:
                break
        return ret


class PPK2_MP(PPK2_API):
    '''
    Multiprocessing variant of the object. The interface is the same as for the regular one except it spawns
    a background process on start_measuring()
    '''
    def __init__(self, port, buffer_seconds=10):
        '''
        port - port where PPK2 is connected
        buffer_seconds - how many seconds of data to keep in the buffer
        '''
        super().__init__(port)
        self._fetcher = None
        self._quit_evt = multiprocessing.Event()
        self._buffer_seconds = buffer_seconds

    def __del__(self):
        """Destructor"""
        PPK2_API.stop_measuring(self)
        self._quit_evt.clear()
        self._quit_evt = None
        del self._quit_evt
        if self._fetcher is not None:
            self._fetcher.join()
        self._fetcher = None
        del self._fetcher

    def start_measuring(self):
        # discard the data in the buffer
        self.stop_measuring()
        while self.get_data()!=b'':
            pass

        PPK2_API.start_measuring(self)
        self._quit_evt.clear()
        if self._fetcher is not None:
            return

        self._fetcher = PPK_Fetch(self, self._quit_evt, self._buffer_seconds)
        self._fetcher.start()

    def stop_measuring(self):
        PPK2_API.stop_measuring(self)
        self.get_data() # flush the serial buffer (to prevent unicode error on next command)
        self._quit_evt.set()
        if self._fetcher is not None:
            self._fetcher.join() # join() will block if the queue isn't empty
            self._fetcher = None

    def get_data(self):
        try:
            return self._fetcher.get_data()
        except (TypeError, AttributeError):
            return b''
*/

/*
Package sensehat provides direct I2C access to Raspberry Pi Sense HAT sensors.

Supports both Sense HAT v1 and v2 (with color sensor detection).

Sensors:
  - HTS221: Humidity and temperature
  - LPS25H: Pressure and temperature
  - LSM9DS1: Accelerometer, gyroscope, magnetometer (IMU)
  - TCS34725: Color sensor (v2 only)
*/
package sensehat

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"unsafe"
)

// I2C addresses for Sense HAT sensors
const (
	addrAccelGyro = 0x6A // LSM9DS1 Accelerometer and Gyroscope
	addrMag       = 0x1C // LSM9DS1 Magnetometer
	addrHTS221    = 0x5F // Humidity and temperature
	addrLPS25H    = 0x5C // Pressure and temperature
	addrTCS34725  = 0x29 // Color sensor (Sense HAT v2 only)
	addrLEDMatrix = 0x46 // LED matrix controller
)

// I2C ioctl commands
const (
	i2cSlave = 0x0703
)

// JoystickEvent represents a joystick press direction.
type JoystickEvent int

const (
	JoystickUp JoystickEvent = iota
	JoystickDown
	JoystickLeft
	JoystickRight
	JoystickMiddle
)

// SenseHat represents a Raspberry Pi Sense HAT device.
type SenseHat struct {
	i2cFile        *os.File
	hasColorSensor bool
	joystickFile   *os.File
	joystickEvents []JoystickEvent
	joystickMu     sync.Mutex

	// HTS221 calibration coefficients
	h0RH, h1RH       float64
	t0DegC, t1DegC   float64
	h0T0Out, h1T0Out int16
	t0Out, t1Out     int16
}

// Available returns true if a Sense HAT appears to be connected.
func Available() bool {
	// Check if I2C bus exists and HTS221 responds
	f, err := os.OpenFile("/dev/i2c-1", os.O_RDWR, 0)
	if err != nil {
		return false
	}
	defer f.Close()

	if err := setI2CAddr(f, addrHTS221); err != nil {
		return false
	}

	// Try to read WHO_AM_I register
	buf := make([]byte, 1)
	if _, err := f.Write([]byte{0x0F}); err != nil {
		return false
	}
	if _, err := f.Read(buf); err != nil {
		return false
	}
	return buf[0] == 0xBC // HTS221 WHO_AM_I value
}

// New initializes and returns a new SenseHat instance.
func New() (*SenseHat, error) {
	f, err := os.OpenFile("/dev/i2c-1", os.O_RDWR, 0)
	if err != nil {
		return nil, fmt.Errorf("failed to open I2C bus: %w", err)
	}

	hat := &SenseHat{
		i2cFile: f,
	}

	// Initialize sensors
	if err := hat.initHTS221(); err != nil {
		f.Close()
		return nil, fmt.Errorf("failed to initialize HTS221: %w", err)
	}

	if err := hat.initLPS25H(); err != nil {
		f.Close()
		return nil, fmt.Errorf("failed to initialize LPS25H: %w", err)
	}

	if err := hat.initIMU(); err != nil {
		f.Close()
		return nil, fmt.Errorf("failed to initialize IMU: %w", err)
	}

	// Try to initialize color sensor (Sense HAT v2 only)
	hat.hasColorSensor = hat.initTCS34725() == nil

	// Initialize joystick
	hat.initJoystick()

	return hat, nil
}

// Close releases all resources.
func (s *SenseHat) Close() error {
	if s.joystickFile != nil {
		s.joystickFile.Close()
	}
	return s.i2cFile.Close()
}

// HasColorSensor returns true if a color sensor is available (Sense HAT v2).
func (s *SenseHat) HasColorSensor() bool {
	return s.hasColorSensor
}

func setI2CAddr(f *os.File, addr uint8) error {
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, f.Fd(), i2cSlave, uintptr(addr))
	if errno != 0 {
		return errno
	}
	return nil
}

func (s *SenseHat) writeReg(addr uint8, reg, value byte) error {
	if err := setI2CAddr(s.i2cFile, addr); err != nil {
		return err
	}
	_, err := s.i2cFile.Write([]byte{reg, value})
	return err
}

func (s *SenseHat) readReg(addr uint8, reg byte) (byte, error) {
	if err := setI2CAddr(s.i2cFile, addr); err != nil {
		return 0, err
	}
	if _, err := s.i2cFile.Write([]byte{reg}); err != nil {
		return 0, err
	}
	buf := make([]byte, 1)
	if _, err := s.i2cFile.Read(buf); err != nil {
		return 0, err
	}
	return buf[0], nil
}

func (s *SenseHat) readRegs(addr uint8, reg byte, n int) ([]byte, error) {
	if err := setI2CAddr(s.i2cFile, addr); err != nil {
		return nil, err
	}
	// Set auto-increment bit for multi-byte reads
	if _, err := s.i2cFile.Write([]byte{reg | 0x80}); err != nil {
		return nil, err
	}
	buf := make([]byte, n)
	if _, err := s.i2cFile.Read(buf); err != nil {
		return nil, err
	}
	return buf, nil
}

// initHTS221 initializes the HTS221 humidity/temperature sensor and reads calibration data.
func (s *SenseHat) initHTS221() error {
	// Power on, BDU enabled, ODR 1Hz
	if err := s.writeReg(addrHTS221, 0x20, 0x85); err != nil {
		return err
	}

	// Read calibration data
	h0RH, _ := s.readReg(addrHTS221, 0x30)
	h1RH, _ := s.readReg(addrHTS221, 0x31)
	s.h0RH = float64(h0RH) / 2.0
	s.h1RH = float64(h1RH) / 2.0

	t0DegC, _ := s.readReg(addrHTS221, 0x32)
	t1DegC, _ := s.readReg(addrHTS221, 0x33)
	t1t0msb, _ := s.readReg(addrHTS221, 0x35)
	s.t0DegC = float64(int(t0DegC)|((int(t1t0msb)&0x03)<<8)) / 8.0
	s.t1DegC = float64(int(t1DegC)|((int(t1t0msb)&0x0C)<<6)) / 8.0

	h0T0OutL, _ := s.readReg(addrHTS221, 0x36)
	h0T0OutH, _ := s.readReg(addrHTS221, 0x37)
	s.h0T0Out = int16(h0T0OutL) | int16(h0T0OutH)<<8

	h1T0OutL, _ := s.readReg(addrHTS221, 0x3A)
	h1T0OutH, _ := s.readReg(addrHTS221, 0x3B)
	s.h1T0Out = int16(h1T0OutL) | int16(h1T0OutH)<<8

	t0OutL, _ := s.readReg(addrHTS221, 0x3C)
	t0OutH, _ := s.readReg(addrHTS221, 0x3D)
	s.t0Out = int16(t0OutL) | int16(t0OutH)<<8

	t1OutL, _ := s.readReg(addrHTS221, 0x3E)
	t1OutH, _ := s.readReg(addrHTS221, 0x3F)
	s.t1Out = int16(t1OutL) | int16(t1OutH)<<8

	return nil
}

// initLPS25H initializes the LPS25H pressure/temperature sensor.
func (s *SenseHat) initLPS25H() error {
	// Power on, BDU enabled, ODR 25Hz
	return s.writeReg(addrLPS25H, 0x20, 0xC4)
}

// initIMU initializes the LSM9DS1 IMU sensor.
func (s *SenseHat) initIMU() error {
	// Initialize accelerometer/gyroscope
	// CTRL_REG1_G: ODR 119Hz, 245 dps full scale
	if err := s.writeReg(addrAccelGyro, 0x10, 0x60); err != nil {
		return err
	}
	// CTRL_REG6_XL: ODR 119Hz, ±2g full scale
	if err := s.writeReg(addrAccelGyro, 0x20, 0x60); err != nil {
		return err
	}

	// Initialize magnetometer
	// CTRL_REG1_M: Temp compensated, high performance, ODR 20Hz
	if err := s.writeReg(addrMag, 0x20, 0x70); err != nil {
		return err
	}
	// CTRL_REG2_M: ±4 gauss full scale
	if err := s.writeReg(addrMag, 0x21, 0x00); err != nil {
		return err
	}
	// CTRL_REG3_M: Continuous conversion mode
	if err := s.writeReg(addrMag, 0x22, 0x00); err != nil {
		return err
	}

	return nil
}

// initTCS34725 initializes the TCS34725 color sensor (Sense HAT v2 only).
func (s *SenseHat) initTCS34725() error {
	// Check device ID (command bit 0x80 | register)
	id, err := s.readReg(addrTCS34725, 0x80|0x12)
	if err != nil {
		return err
	}
	if id != 0x44 && id != 0x4D { // TCS34725 or TCS3400
		return fmt.Errorf("unknown color sensor ID: 0x%02X", id)
	}

	// Enable device with AEN and PON (command bit | register)
	if err := s.writeReg(addrTCS34725, 0x80|0x00, 0x03); err != nil {
		return err
	}
	// Set integration time (64 cycles ≈ 154ms)
	if err := s.writeReg(addrTCS34725, 0x80|0x01, 0xC0); err != nil {
		return err
	}
	// Set gain (4x)
	if err := s.writeReg(addrTCS34725, 0x80|0x0F, 0x01); err != nil {
		return err
	}

	return nil
}

// initJoystick opens the joystick input device.
func (s *SenseHat) initJoystick() {
	// Try common input device paths
	paths := []string{
		"/dev/input/by-id/gpio-Raspberry-Pi-Sense-HAT-Joystick-event-kbd",
		"/dev/input/event0",
	}
	for _, path := range paths {
		if f, err := os.Open(path); err == nil {
			s.joystickFile = f
			go s.readJoystickEvents()
			break
		}
	}
}

// readJoystickEvents reads joystick events in a goroutine.
func (s *SenseHat) readJoystickEvents() {
	buf := make([]byte, 24) // input_event structure size
	for {
		n, err := s.joystickFile.Read(buf)
		if err != nil || n < 24 {
			continue
		}

		// Parse input_event: time(16), type(2), code(2), value(4)
		evType := binary.LittleEndian.Uint16(buf[16:18])
		code := binary.LittleEndian.Uint16(buf[18:20])
		value := int32(binary.LittleEndian.Uint32(buf[20:24]))

		// EV_KEY = 1, value 1 = pressed
		if evType == 1 && value == 1 {
			var event JoystickEvent
			switch code {
			case 103: // KEY_UP
				event = JoystickUp
			case 108: // KEY_DOWN
				event = JoystickDown
			case 105: // KEY_LEFT
				event = JoystickLeft
			case 106: // KEY_RIGHT
				event = JoystickRight
			case 28: // KEY_ENTER
				event = JoystickMiddle
			default:
				continue
			}

			s.joystickMu.Lock()
			s.joystickEvents = append(s.joystickEvents, event)
			s.joystickMu.Unlock()
		}
	}
}

// GetJoystickEvents returns and clears pending joystick events.
func (s *SenseHat) GetJoystickEvents() []JoystickEvent {
	s.joystickMu.Lock()
	defer s.joystickMu.Unlock()
	events := s.joystickEvents
	s.joystickEvents = nil
	return events
}

// GetTemperatureFromHumidity returns temperature in Celsius from the HTS221 sensor.
func (s *SenseHat) GetTemperatureFromHumidity() (float64, error) {
	data, err := s.readRegs(addrHTS221, 0x2A, 2)
	if err != nil {
		return 0, err
	}
	tOut := int16(data[0]) | int16(data[1])<<8

	// Linear interpolation using calibration coefficients
	temp := s.t0DegC + (s.t1DegC-s.t0DegC)*float64(tOut-s.t0Out)/float64(s.t1Out-s.t0Out)
	return temp, nil
}

// GetTemperatureFromPressure returns temperature in Celsius from the LPS25H sensor.
func (s *SenseHat) GetTemperatureFromPressure() (float64, error) {
	data, err := s.readRegs(addrLPS25H, 0x2B, 2)
	if err != nil {
		return 0, err
	}
	tOut := int16(data[0]) | int16(data[1])<<8
	return 42.5 + float64(tOut)/480.0, nil
}

// GetHumidity returns relative humidity percentage.
func (s *SenseHat) GetHumidity() (float64, error) {
	data, err := s.readRegs(addrHTS221, 0x28, 2)
	if err != nil {
		return 0, err
	}
	hOut := int16(data[0]) | int16(data[1])<<8

	// Linear interpolation using calibration coefficients
	humidity := s.h0RH + (s.h1RH-s.h0RH)*float64(hOut-s.h0T0Out)/float64(s.h1T0Out-s.h0T0Out)
	return humidity, nil
}

// GetPressure returns atmospheric pressure in millibars.
func (s *SenseHat) GetPressure() (float64, error) {
	data, err := s.readRegs(addrLPS25H, 0x28, 3)
	if err != nil {
		return 0, err
	}
	pOut := int32(data[0]) | int32(data[1])<<8 | int32(data[2])<<16
	// Convert to millibars (hPa)
	return float64(pOut) / 4096.0, nil
}

// GetAccelerometer returns accelerometer readings in Gs (x, y, z).
func (s *SenseHat) GetAccelerometer() (x, y, z float64, err error) {
	data, err := s.readRegs(addrAccelGyro, 0x28, 6)
	if err != nil {
		return 0, 0, 0, err
	}
	// ±2g scale factor: 0.061 mg/LSB
	scale := 0.061 / 1000.0
	x = float64(int16(data[0])|int16(data[1])<<8) * scale
	y = float64(int16(data[2])|int16(data[3])<<8) * scale
	z = float64(int16(data[4])|int16(data[5])<<8) * scale
	return
}

// GetGyroscope returns gyroscope readings in degrees/second (x, y, z).
func (s *SenseHat) GetGyroscope() (x, y, z float64, err error) {
	data, err := s.readRegs(addrAccelGyro, 0x18, 6)
	if err != nil {
		return 0, 0, 0, err
	}
	// 245 dps scale factor: 8.75 mdps/LSB
	scale := 8.75 / 1000.0
	x = float64(int16(data[0])|int16(data[1])<<8) * scale
	y = float64(int16(data[2])|int16(data[3])<<8) * scale
	z = float64(int16(data[4])|int16(data[5])<<8) * scale
	return
}

// GetMagnetometer returns magnetometer readings in microteslas (x, y, z).
func (s *SenseHat) GetMagnetometer() (x, y, z float64, err error) {
	data, err := s.readRegs(addrMag, 0x28, 6)
	if err != nil {
		return 0, 0, 0, err
	}
	// ±4 gauss scale factor: 0.14 mgauss/LSB = 0.014 µT/LSB
	scale := 0.014
	x = float64(int16(data[0])|int16(data[1])<<8) * scale
	y = float64(int16(data[2])|int16(data[3])<<8) * scale
	z = float64(int16(data[4])|int16(data[5])<<8) * scale
	return
}

// GetCompass returns compass heading in degrees (0-360).
func (s *SenseHat) GetCompass() (float64, error) {
	mx, my, _, err := s.GetMagnetometer()
	if err != nil {
		return 0, err
	}
	heading := math.Atan2(my, mx) * 180.0 / math.Pi
	if heading < 0 {
		heading += 360.0
	}
	return heading, nil
}

// GetOrientation returns orientation in degrees (pitch, roll, yaw).
// This is a simplified calculation using accelerometer and magnetometer.
func (s *SenseHat) GetOrientation() (pitch, roll, yaw float64, err error) {
	ax, ay, az, err := s.GetAccelerometer()
	if err != nil {
		return 0, 0, 0, err
	}
	mx, my, mz, err := s.GetMagnetometer()
	if err != nil {
		return 0, 0, 0, err
	}

	// Calculate pitch and roll from accelerometer
	pitch = math.Atan2(-ax, math.Sqrt(ay*ay+az*az)) * 180.0 / math.Pi
	roll = math.Atan2(ay, az) * 180.0 / math.Pi

	// Tilt-compensated compass heading for yaw
	pitchRad := pitch * math.Pi / 180.0
	rollRad := roll * math.Pi / 180.0

	mxComp := mx*math.Cos(pitchRad) + mz*math.Sin(pitchRad)
	myComp := mx*math.Sin(rollRad)*math.Sin(pitchRad) + my*math.Cos(rollRad) - mz*math.Sin(rollRad)*math.Cos(pitchRad)

	yaw = math.Atan2(myComp, mxComp) * 180.0 / math.Pi
	if yaw < 0 {
		yaw += 360.0
	}

	return
}

// IsIMUCalibrated returns true if the IMU is providing valid readings.
func (s *SenseHat) IsIMUCalibrated() bool {
	ax, ay, az, err := s.GetAccelerometer()
	if err != nil {
		return false
	}
	// At rest, magnitude should be approximately 1g
	mag := math.Sqrt(ax*ax + ay*ay + az*az)
	return mag > 0.8 && mag < 1.2
}

// GetColor returns color sensor readings (red, green, blue, clear).
func (s *SenseHat) GetColor() (r, g, b, c uint8, err error) {
	if !s.hasColorSensor {
		return 0, 0, 0, 0, fmt.Errorf("color sensor not available")
	}

	// Read RGBC data (command bit | auto-increment | register)
	data, err := s.readRegs(addrTCS34725, 0x80|0x20|0x14, 8)
	if err != nil {
		return 0, 0, 0, 0, err
	}

	cRaw := uint16(data[0]) | uint16(data[1])<<8
	rRaw := uint16(data[2]) | uint16(data[3])<<8
	gRaw := uint16(data[4]) | uint16(data[5])<<8
	bRaw := uint16(data[6]) | uint16(data[7])<<8

	// Normalize to 0-255
	if cRaw > 0 {
		r = uint8(min(255, int(rRaw)*255/int(cRaw)))
		g = uint8(min(255, int(gRaw)*255/int(cRaw)))
		b = uint8(min(255, int(bRaw)*255/int(cRaw)))
		c = uint8(min(255, int(cRaw)/256))
	}
	return
}

// GetCPUTemperature returns the Raspberry Pi CPU temperature in Celsius.
func GetCPUTemperature() (float64, error) {
	f, err := os.Open("/sys/class/thermal/thermal_zone0/temp")
	if err != nil {
		return 0, err
	}
	defer f.Close()

	scanner := bufio.NewScanner(f)
	if scanner.Scan() {
		if temp, err := strconv.ParseFloat(strings.TrimSpace(scanner.Text()), 64); err == nil {
			return temp / 1000.0, nil
		}
	}
	return 0, scanner.Err()
}

// ClearLEDs turns off all LEDs on the 8x8 matrix.
func (s *SenseHat) ClearLEDs() error {
	if err := setI2CAddr(s.i2cFile, addrLEDMatrix); err != nil {
		return err
	}
	// Write 193 bytes: 1 register address + 192 bytes of zeros (64 pixels × 3 bytes RGB)
	buf := make([]byte, 193)
	buf[0] = 0x00 // Register address
	_, err := s.i2cFile.Write(buf)
	return err
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// inputEvent represents a Linux input event structure for unsafe sizeof
type inputEvent struct {
	Time  [16]byte // struct timeval
	Type  uint16
	Code  uint16
	Value int32
}

func init() {
	// Verify inputEvent size matches expected 24 bytes
	_ = [1]struct{}{}[24-int(unsafe.Sizeof(inputEvent{}))]
}

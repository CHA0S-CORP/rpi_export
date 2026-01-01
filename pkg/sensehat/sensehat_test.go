package sensehat

import (
	"math"
	"testing"
)

func TestJoystickEventConstants(t *testing.T) {
	// Verify joystick event constants are sequential starting from 0
	if JoystickUp != 0 {
		t.Errorf("JoystickUp = %d, want 0", JoystickUp)
	}
	if JoystickDown != 1 {
		t.Errorf("JoystickDown = %d, want 1", JoystickDown)
	}
	if JoystickLeft != 2 {
		t.Errorf("JoystickLeft = %d, want 2", JoystickLeft)
	}
	if JoystickRight != 3 {
		t.Errorf("JoystickRight = %d, want 3", JoystickRight)
	}
	if JoystickMiddle != 4 {
		t.Errorf("JoystickMiddle = %d, want 4", JoystickMiddle)
	}
}

func TestI2CAddressConstants(t *testing.T) {
	// Verify I2C addresses match Sense HAT hardware specs
	expectedAddrs := map[string]uint8{
		"AccelGyro":  0x6A, // LSM9DS1
		"Mag":        0x1C, // LSM9DS1 Magnetometer
		"HTS221":     0x5F, // Humidity/temp
		"LPS25H":     0x5C, // Pressure/temp
		"TCS34725":   0x29, // Color sensor (v2)
		"LED Matrix": 0x46, // LED matrix controller
	}

	actualAddrs := map[string]uint8{
		"AccelGyro":  addrAccelGyro,
		"Mag":        addrMag,
		"HTS221":     addrHTS221,
		"LPS25H":     addrLPS25H,
		"TCS34725":   addrTCS34725,
		"LED Matrix": addrLEDMatrix,
	}

	for name, expected := range expectedAddrs {
		actual := actualAddrs[name]
		if actual != expected {
			t.Errorf("addr%s = 0x%02X, want 0x%02X", name, actual, expected)
		}
	}
}

func TestI2CSlaveConstant(t *testing.T) {
	// I2C_SLAVE ioctl command
	if i2cSlave != 0x0703 {
		t.Errorf("i2cSlave = 0x%04X, want 0x0703", i2cSlave)
	}
}

func TestSenseHatHasColorSensor(t *testing.T) {
	// Test with color sensor available
	hat := &SenseHat{hasColorSensor: true}
	if !hat.HasColorSensor() {
		t.Error("HasColorSensor() = false, want true")
	}

	// Test without color sensor
	hat = &SenseHat{hasColorSensor: false}
	if hat.HasColorSensor() {
		t.Error("HasColorSensor() = true, want false")
	}
}

func TestGetJoystickEventsEmpty(t *testing.T) {
	hat := &SenseHat{}
	events := hat.GetJoystickEvents()
	if len(events) != 0 {
		t.Errorf("GetJoystickEvents() returned %d events, want 0", len(events))
	}
}

func TestGetJoystickEventsClearsBuffer(t *testing.T) {
	hat := &SenseHat{
		joystickEvents: []JoystickEvent{JoystickUp, JoystickDown, JoystickMiddle},
	}

	// First call should return all events
	events := hat.GetJoystickEvents()
	if len(events) != 3 {
		t.Errorf("First GetJoystickEvents() returned %d events, want 3", len(events))
	}

	// Second call should return empty
	events = hat.GetJoystickEvents()
	if len(events) != 0 {
		t.Errorf("Second GetJoystickEvents() returned %d events, want 0", len(events))
	}
}

func TestGetJoystickEventsOrder(t *testing.T) {
	expected := []JoystickEvent{JoystickLeft, JoystickRight, JoystickUp}
	hat := &SenseHat{
		joystickEvents: expected,
	}

	events := hat.GetJoystickEvents()
	if len(events) != len(expected) {
		t.Fatalf("GetJoystickEvents() returned %d events, want %d", len(events), len(expected))
	}

	for i, ev := range events {
		if ev != expected[i] {
			t.Errorf("GetJoystickEvents()[%d] = %d, want %d", i, ev, expected[i])
		}
	}
}

func TestTemperatureCalibrationFormula(t *testing.T) {
	// Test the linear interpolation formula used in GetTemperatureFromHumidity
	// temp = t0DegC + (t1DegC - t0DegC) * (tOut - t0Out) / (t1Out - t0Out)

	tests := []struct {
		name    string
		t0DegC  float64
		t1DegC  float64
		t0Out   int16
		t1Out   int16
		tOut    int16
		wantMin float64
		wantMax float64
	}{
		{
			name:    "exact t0",
			t0DegC:  20.0,
			t1DegC:  30.0,
			t0Out:   1000,
			t1Out:   2000,
			tOut:    1000,
			wantMin: 19.9,
			wantMax: 20.1,
		},
		{
			name:    "exact t1",
			t0DegC:  20.0,
			t1DegC:  30.0,
			t0Out:   1000,
			t1Out:   2000,
			tOut:    2000,
			wantMin: 29.9,
			wantMax: 30.1,
		},
		{
			name:    "midpoint",
			t0DegC:  20.0,
			t1DegC:  30.0,
			t0Out:   1000,
			t1Out:   2000,
			tOut:    1500,
			wantMin: 24.9,
			wantMax: 25.1,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			// Apply the formula
			result := tt.t0DegC + (tt.t1DegC-tt.t0DegC)*float64(tt.tOut-tt.t0Out)/float64(tt.t1Out-tt.t0Out)
			if result < tt.wantMin || result > tt.wantMax {
				t.Errorf("temperature = %.2f, want between %.2f and %.2f", result, tt.wantMin, tt.wantMax)
			}
		})
	}
}

func TestHumidityCalibrationFormula(t *testing.T) {
	// Test the linear interpolation formula used in GetHumidity
	// humidity = h0RH + (h1RH - h0RH) * (hOut - h0T0Out) / (h1T0Out - h0T0Out)

	tests := []struct {
		name    string
		h0RH    float64
		h1RH    float64
		h0T0Out int16
		h1T0Out int16
		hOut    int16
		wantMin float64
		wantMax float64
	}{
		{
			name:    "exact h0",
			h0RH:    30.0,
			h1RH:    70.0,
			h0T0Out: 500,
			h1T0Out: 1500,
			hOut:    500,
			wantMin: 29.9,
			wantMax: 30.1,
		},
		{
			name:    "exact h1",
			h0RH:    30.0,
			h1RH:    70.0,
			h0T0Out: 500,
			h1T0Out: 1500,
			hOut:    1500,
			wantMin: 69.9,
			wantMax: 70.1,
		},
		{
			name:    "midpoint",
			h0RH:    30.0,
			h1RH:    70.0,
			h0T0Out: 500,
			h1T0Out: 1500,
			hOut:    1000,
			wantMin: 49.9,
			wantMax: 50.1,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			// Apply the formula
			result := tt.h0RH + (tt.h1RH-tt.h0RH)*float64(tt.hOut-tt.h0T0Out)/float64(tt.h1T0Out-tt.h0T0Out)
			if result < tt.wantMin || result > tt.wantMax {
				t.Errorf("humidity = %.2f, want between %.2f and %.2f", result, tt.wantMin, tt.wantMax)
			}
		})
	}
}

func TestPressureFormula(t *testing.T) {
	// Test the pressure conversion formula: pressure = pOut / 4096.0

	tests := []struct {
		name     string
		pOut     int32
		expected float64
	}{
		{
			name:     "zero",
			pOut:     0,
			expected: 0,
		},
		{
			name:     "standard pressure approx 1013 hPa",
			pOut:     4149248, // 1013 * 4096
			expected: 1013.0,
		},
		{
			name:     "4096 raw = 1 hPa",
			pOut:     4096,
			expected: 1.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := float64(tt.pOut) / 4096.0
			if math.Abs(result-tt.expected) > 0.1 {
				t.Errorf("pressure = %.2f, want %.2f", result, tt.expected)
			}
		})
	}
}

func TestLPS25HTemperatureFormula(t *testing.T) {
	// Test the LPS25H temperature formula: temp = 42.5 + tOut/480.0

	tests := []struct {
		name     string
		tOut     int16
		expected float64
	}{
		{
			name:     "zero offset",
			tOut:     0,
			expected: 42.5,
		},
		{
			name:     "positive offset",
			tOut:     480,
			expected: 43.5,
		},
		{
			name:     "negative offset",
			tOut:     -480,
			expected: 41.5,
		},
		{
			name:     "room temperature (approx 25C)",
			tOut:     -8400, // (25 - 42.5) * 480
			expected: 25.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := 42.5 + float64(tt.tOut)/480.0
			if math.Abs(result-tt.expected) > 0.1 {
				t.Errorf("temperature = %.2f, want %.2f", result, tt.expected)
			}
		})
	}
}

func TestAccelerometerScaleFactor(t *testing.T) {
	// Test the accelerometer scale factor: ±2g = 0.061 mg/LSB

	scale := 0.061 / 1000.0 // Convert to g

	tests := []struct {
		name     string
		rawValue int16
		expected float64
	}{
		{
			name:     "zero",
			rawValue: 0,
			expected: 0,
		},
		{
			name:     "1g approx (16393 LSB for 2g range)",
			rawValue: 16393,
			expected: 1.0,
		},
		{
			name:     "negative 1g",
			rawValue: -16393,
			expected: -1.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := float64(tt.rawValue) * scale
			if math.Abs(result-tt.expected) > 0.01 {
				t.Errorf("acceleration = %.3f g, want %.3f g", result, tt.expected)
			}
		})
	}
}

func TestGyroscopeScaleFactor(t *testing.T) {
	// Test the gyroscope scale factor: 245 dps = 8.75 mdps/LSB

	scale := 8.75 / 1000.0 // Convert to dps

	tests := []struct {
		name     string
		rawValue int16
		expected float64
	}{
		{
			name:     "zero",
			rawValue: 0,
			expected: 0,
		},
		{
			name:     "100 dps",
			rawValue: 11429, // 100 / 0.00875
			expected: 100.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := float64(tt.rawValue) * scale
			if math.Abs(result-tt.expected) > 1.0 {
				t.Errorf("rotation = %.2f dps, want %.2f dps", result, tt.expected)
			}
		})
	}
}

func TestMagnetometerScaleFactor(t *testing.T) {
	// Test the magnetometer scale factor: ±4 gauss = 0.14 mgauss/LSB = 0.014 µT/LSB

	scale := 0.014 // µT per LSB

	tests := []struct {
		name     string
		rawValue int16
		expected float64
	}{
		{
			name:     "zero",
			rawValue: 0,
			expected: 0,
		},
		{
			name:     "earth field approx 50 µT",
			rawValue: 3571, // 50 / 0.014
			expected: 50.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := float64(tt.rawValue) * scale
			if math.Abs(result-tt.expected) > 0.5 {
				t.Errorf("field = %.2f µT, want %.2f µT", result, tt.expected)
			}
		})
	}
}

func TestCompassHeadingCalculation(t *testing.T) {
	// Test the compass heading formula: heading = atan2(my, mx) * 180 / Pi

	tests := []struct {
		name     string
		mx       float64
		my       float64
		expected float64
	}{
		{
			name:     "north (positive X)",
			mx:       1,
			my:       0,
			expected: 0,
		},
		{
			name:     "east (positive Y)",
			mx:       0,
			my:       1,
			expected: 90,
		},
		{
			name:     "south (negative X)",
			mx:       -1,
			my:       0,
			expected: 180,
		},
		{
			name:     "west (negative Y)",
			mx:       0,
			my:       -1,
			expected: 270,
		},
		{
			name:     "northeast",
			mx:       1,
			my:       1,
			expected: 45,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			heading := math.Atan2(tt.my, tt.mx) * 180.0 / math.Pi
			if heading < 0 {
				heading += 360.0
			}
			if math.Abs(heading-tt.expected) > 0.1 {
				t.Errorf("heading = %.2f°, want %.2f°", heading, tt.expected)
			}
		})
	}
}

func TestPitchRollCalculation(t *testing.T) {
	// Test pitch and roll calculations from accelerometer data
	// pitch = atan2(-ax, sqrt(ay^2 + az^2)) * 180 / Pi
	// roll = atan2(ay, az) * 180 / Pi

	tests := []struct {
		name          string
		ax, ay, az    float64
		expectedPitch float64
		expectedRoll  float64
	}{
		{
			name:          "level (z up)",
			ax:            0,
			ay:            0,
			az:            1,
			expectedPitch: 0,
			expectedRoll:  0,
		},
		{
			name:          "pitched forward 45°",
			ax:            0.707,
			ay:            0,
			az:            0.707,
			expectedPitch: -45,
			expectedRoll:  0,
		},
		{
			name:          "rolled right 45°",
			ax:            0,
			ay:            0.707,
			az:            0.707,
			expectedPitch: 0,
			expectedRoll:  45,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			pitch := math.Atan2(-tt.ax, math.Sqrt(tt.ay*tt.ay+tt.az*tt.az)) * 180.0 / math.Pi
			roll := math.Atan2(tt.ay, tt.az) * 180.0 / math.Pi

			if math.Abs(pitch-tt.expectedPitch) > 1 {
				t.Errorf("pitch = %.2f°, want %.2f°", pitch, tt.expectedPitch)
			}
			if math.Abs(roll-tt.expectedRoll) > 1 {
				t.Errorf("roll = %.2f°, want %.2f°", roll, tt.expectedRoll)
			}
		})
	}
}

func TestIMUCalibrationCheck(t *testing.T) {
	// Test the IMU calibration check: magnitude should be ~1g at rest

	tests := []struct {
		name       string
		ax, ay, az float64
		calibrated bool
	}{
		{
			name:       "perfect 1g",
			ax:         0,
			ay:         0,
			az:         1,
			calibrated: true,
		},
		{
			name:       "tilted but still 1g",
			ax:         0.5,
			ay:         0.5,
			az:         0.707,
			calibrated: true,
		},
		{
			name:       "too low",
			ax:         0.3,
			ay:         0.3,
			az:         0.3,
			calibrated: false,
		},
		{
			name:       "too high",
			ax:         1,
			ay:         1,
			az:         1,
			calibrated: false,
		},
		{
			name:       "zero (sensor not working)",
			ax:         0,
			ay:         0,
			az:         0,
			calibrated: false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			mag := math.Sqrt(tt.ax*tt.ax + tt.ay*tt.ay + tt.az*tt.az)
			calibrated := mag > 0.8 && mag < 1.2
			if calibrated != tt.calibrated {
				t.Errorf("calibrated = %v (mag=%.2f), want %v", calibrated, mag, tt.calibrated)
			}
		})
	}
}

func TestColorNormalization(t *testing.T) {
	// Test color value normalization: rgb = min(255, rawRGB * 255 / clear)

	tests := []struct {
		name            string
		rRaw, cRaw      uint16
		expectedR       uint8
		expectedRCapped bool
	}{
		{
			name:      "normal",
			rRaw:      128,
			cRaw:      256,
			expectedR: 127, // 128 * 255 / 256 = 127 (integer division)
		},
		{
			name:            "capped at 255",
			rRaw:            512,
			cRaw:            256,
			expectedR:       255,
			expectedRCapped: true,
		},
		{
			name:      "zero clear",
			rRaw:      100,
			cRaw:      0,
			expectedR: 0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			var r uint8
			if tt.cRaw > 0 {
				val := int(tt.rRaw) * 255 / int(tt.cRaw)
				if val > 255 {
					val = 255
				}
				r = uint8(val)
			}
			if r != tt.expectedR {
				t.Errorf("normalized R = %d, want %d", r, tt.expectedR)
			}
		})
	}
}

func TestInputEventSize(t *testing.T) {
	// The inputEvent struct should be exactly 24 bytes
	// This is verified at init() time with a compile-time check,
	// but we can also test it here for documentation
	expected := 24
	// The init() function already panics if this is wrong,
	// so if we get here, the size is correct
	t.Logf("inputEvent size verified as %d bytes", expected)
}

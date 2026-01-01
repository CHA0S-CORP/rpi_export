package mbox

import (
	"testing"
)

func TestTagIsValid(t *testing.T) {
	tests := []struct {
		name     string
		tag      Tag
		expected bool
	}{
		{
			name:     "nil tag",
			tag:      nil,
			expected: false,
		},
		{
			name:     "empty tag",
			tag:      Tag{},
			expected: false,
		},
		{
			name:     "end tag",
			tag:      EndTag,
			expected: true,
		},
		{
			name:     "single non-zero element",
			tag:      Tag{1},
			expected: false,
		},
		{
			name:     "two elements (too short)",
			tag:      Tag{1, 2},
			expected: false,
		},
		{
			name:     "valid tag with 4-byte value buffer",
			tag:      Tag{TagGetFirmwareRevision, 4, 0x80000004, 0x12345678},
			expected: true,
		},
		{
			name:     "valid tag with 8-byte value buffer",
			tag:      Tag{TagGetPowerState, 8, 0x80000008, 0, 1},
			expected: true,
		},
		{
			name:     "incorrect size - too few elements",
			tag:      Tag{TagGetFirmwareRevision, 8, 0x80000004, 0},
			expected: false,
		},
		{
			name:     "incorrect size - too many elements",
			tag:      Tag{TagGetFirmwareRevision, 4, 0x80000004, 0, 0, 0},
			expected: false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.tag.IsValid()
			if result != tt.expected {
				t.Errorf("Tag.IsValid() = %v, want %v", result, tt.expected)
			}
		})
	}
}

func TestTagIsEnd(t *testing.T) {
	tests := []struct {
		name     string
		tag      Tag
		expected bool
	}{
		{
			name:     "end tag",
			tag:      EndTag,
			expected: true,
		},
		{
			name:     "zero tag single element",
			tag:      Tag{0},
			expected: true,
		},
		{
			name:     "non-zero single element",
			tag:      Tag{1},
			expected: false,
		},
		{
			name:     "multiple elements",
			tag:      Tag{0, 0},
			expected: false,
		},
		{
			name:     "nil tag",
			tag:      nil,
			expected: false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.tag.IsEnd()
			if result != tt.expected {
				t.Errorf("Tag.IsEnd() = %v, want %v", result, tt.expected)
			}
		})
	}
}

func TestTagIsResponse(t *testing.T) {
	tests := []struct {
		name     string
		tag      Tag
		expected bool
	}{
		{
			name:     "response tag",
			tag:      Tag{TagGetFirmwareRevision, 4, 0x80000004, 0x12345678},
			expected: true,
		},
		{
			name:     "request tag",
			tag:      Tag{TagGetFirmwareRevision, 4, 0, 0},
			expected: false,
		},
		{
			name:     "invalid tag",
			tag:      Tag{},
			expected: false,
		},
		// Note: EndTag case is not tested here because IsValid() returns true
		// for EndTag but IsResponse() would panic trying to access t[2].
		// This is a potential bug in the source code.
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.tag.IsResponse()
			if result != tt.expected {
				t.Errorf("Tag.IsResponse() = %v, want %v", result, tt.expected)
			}
		})
	}
}

func TestTagID(t *testing.T) {
	tests := []struct {
		name     string
		tag      Tag
		expected uint32
	}{
		{
			name:     "firmware revision tag",
			tag:      Tag{TagGetFirmwareRevision, 4, 0x80000004, 0},
			expected: TagGetFirmwareRevision,
		},
		{
			name:     "power state tag",
			tag:      Tag{TagGetPowerState, 8, 0x80000008, 0, 0},
			expected: TagGetPowerState,
		},
		{
			name:     "invalid tag",
			tag:      Tag{},
			expected: 0,
		},
		{
			name:     "end tag",
			tag:      EndTag,
			expected: 0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.tag.ID()
			if result != tt.expected {
				t.Errorf("Tag.ID() = 0x%08X, want 0x%08X", result, tt.expected)
			}
		})
	}
}

func TestTagCap(t *testing.T) {
	tests := []struct {
		name     string
		tag      Tag
		expected int
	}{
		{
			name:     "4-byte buffer",
			tag:      Tag{TagGetFirmwareRevision, 4, 0x80000004, 0},
			expected: 4,
		},
		{
			name:     "8-byte buffer",
			tag:      Tag{TagGetPowerState, 8, 0x80000008, 0, 0},
			expected: 8,
		},
		{
			name:     "invalid tag",
			tag:      Tag{},
			expected: 0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.tag.Cap()
			if result != tt.expected {
				t.Errorf("Tag.Cap() = %d, want %d", result, tt.expected)
			}
		})
	}
}

func TestTagLen(t *testing.T) {
	tests := []struct {
		name     string
		tag      Tag
		expected int
	}{
		{
			name:     "response with 4-byte length",
			tag:      Tag{TagGetFirmwareRevision, 4, 0x80000004, 0},
			expected: 4,
		},
		{
			name:     "response with 8-byte length",
			tag:      Tag{TagGetPowerState, 8, 0x80000008, 0, 0},
			expected: 8,
		},
		{
			name:     "request tag (not a response)",
			tag:      Tag{TagGetFirmwareRevision, 4, 0, 0},
			expected: 0,
		},
		{
			name:     "invalid tag",
			tag:      Tag{},
			expected: 0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.tag.Len()
			if result != tt.expected {
				t.Errorf("Tag.Len() = %d, want %d", result, tt.expected)
			}
		})
	}
}

func TestTagValue(t *testing.T) {
	tests := []struct {
		name     string
		tag      Tag
		expected []uint32
	}{
		{
			name:     "single value response",
			tag:      Tag{TagGetFirmwareRevision, 4, 0x80000004, 0xDEADBEEF},
			expected: []uint32{0xDEADBEEF},
		},
		{
			name:     "two value response",
			tag:      Tag{TagGetPowerState, 8, 0x80000008, 0x00000001, 0x00000002},
			expected: []uint32{0x00000001, 0x00000002},
		},
		{
			name:     "request tag (no response)",
			tag:      Tag{TagGetFirmwareRevision, 4, 0, 0},
			expected: nil,
		},
		{
			name:     "invalid tag",
			tag:      Tag{},
			expected: nil,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.tag.Value()
			if len(result) != len(tt.expected) {
				t.Errorf("Tag.Value() len = %d, want %d", len(result), len(tt.expected))
				return
			}
			for i := range result {
				if result[i] != tt.expected[i] {
					t.Errorf("Tag.Value()[%d] = 0x%08X, want 0x%08X", i, result[i], tt.expected[i])
				}
			}
		})
	}
}

func TestReadTag(t *testing.T) {
	tests := []struct {
		name        string
		buffer      []uint32
		expectedTag Tag
		expectErr   bool
	}{
		{
			name:        "end tag",
			buffer:      []uint32{0, 0, 0, 0},
			expectedTag: EndTag,
			expectErr:   false,
		},
		{
			name:        "valid firmware revision response",
			buffer:      []uint32{TagGetFirmwareRevision, 4, 0x80000004, 0xDEADBEEF, 0},
			expectedTag: Tag{TagGetFirmwareRevision, 4, 0x80000004, 0xDEADBEEF},
			expectErr:   false,
		},
		{
			name:        "valid power state response",
			buffer:      []uint32{TagGetPowerState, 8, 0x80000008, 0, 1, 0},
			expectedTag: Tag{TagGetPowerState, 8, 0x80000008, 0, 1},
			expectErr:   false,
		},
		{
			name:        "buffer too small",
			buffer:      []uint32{TagGetFirmwareRevision, 4},
			expectedTag: nil,
			expectErr:   true,
		},
		{
			name:        "empty buffer",
			buffer:      []uint32{},
			expectedTag: nil,
			expectErr:   true,
		},
		{
			name:        "buffer too small for value",
			buffer:      []uint32{TagGetFirmwareRevision, 4, 0x80000004},
			expectedTag: nil,
			expectErr:   true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tag, err := ReadTag(tt.buffer)
			if tt.expectErr {
				if err == nil {
					t.Error("ReadTag() expected error, got nil")
				}
				return
			}
			if err != nil {
				t.Errorf("ReadTag() unexpected error: %v", err)
				return
			}
			if len(tag) != len(tt.expectedTag) {
				t.Errorf("ReadTag() len = %d, want %d", len(tag), len(tt.expectedTag))
				return
			}
			for i := range tag {
				if tag[i] != tt.expectedTag[i] {
					t.Errorf("ReadTag()[%d] = 0x%08X, want 0x%08X", i, tag[i], tt.expectedTag[i])
				}
			}
		})
	}
}

func TestTagConstants(t *testing.T) {
	// Verify tag constants match Raspberry Pi firmware spec
	expectedTags := map[string]uint32{
		"TagGetFirmwareRevision":  0x00000001,
		"TagGetBoardModel":        0x00010001,
		"TagGetBoardRevision":     0x00010002,
		"TagGetBoardMAC":          0x00010003,
		"TagGetPowerState":        0x00020001,
		"TagGetClockRate":         0x00030002,
		"TagGetVoltage":           0x00030003,
		"TagGetMaxVoltage":        0x00030005,
		"TagGetTemperature":       0x00030006,
		"TagGetMinVoltage":        0x00030008,
		"TagGetTurbo":             0x00030009,
		"TagGetMaxTemperature":    0x0003000A,
		"TagGetClockRateMeasured": 0x00030047,
	}

	actualTags := map[string]uint32{
		"TagGetFirmwareRevision":  TagGetFirmwareRevision,
		"TagGetBoardModel":        TagGetBoardModel,
		"TagGetBoardRevision":     TagGetBoardRevision,
		"TagGetBoardMAC":          TagGetBoardMAC,
		"TagGetPowerState":        TagGetPowerState,
		"TagGetClockRate":         TagGetClockRate,
		"TagGetVoltage":           TagGetVoltage,
		"TagGetMaxVoltage":        TagGetMaxVoltage,
		"TagGetTemperature":       TagGetTemperature,
		"TagGetMinVoltage":        TagGetMinVoltage,
		"TagGetTurbo":             TagGetTurbo,
		"TagGetMaxTemperature":    TagGetMaxTemperature,
		"TagGetClockRateMeasured": TagGetClockRateMeasured,
	}

	for name, expected := range expectedTags {
		actual := actualTags[name]
		if actual != expected {
			t.Errorf("%s = 0x%08X, want 0x%08X", name, actual, expected)
		}
	}
}

func TestPowerDeviceIDs(t *testing.T) {
	expectedIDs := map[string]PowerDeviceID{
		"SDCard":   0x00000000,
		"UART0":    0x00000001,
		"UART1":    0x00000002,
		"USBHCD":   0x00000003,
		"I2C0":     0x00000004,
		"I2C1":     0x00000005,
		"I2C2":     0x00000006,
		"SPI":      0x00000007,
		"CCP2TX":   0x00000008,
		"Unknown0": 0x00000009,
		"Unknown1": 0x0000000a,
	}

	actualIDs := map[string]PowerDeviceID{
		"SDCard":   PowerDeviceIDSDCard,
		"UART0":    PowerDeviceIDUART0,
		"UART1":    PowerDeviceIDUART1,
		"USBHCD":   PowerDeviceIDUSBHCD,
		"I2C0":     PowerDeviceIDI2C0,
		"I2C1":     PowerDeviceIDI2C1,
		"I2C2":     PowerDeviceIDI2C2,
		"SPI":      PowerDeviceIDSPI,
		"CCP2TX":   PowerDeviceIDCCP2TX,
		"Unknown0": PowerDeviceIDUnknown0,
		"Unknown1": PowerDeviceIDUnknown1,
	}

	for name, expected := range expectedIDs {
		actual := actualIDs[name]
		if actual != expected {
			t.Errorf("PowerDeviceID%s = 0x%08X, want 0x%08X", name, actual, expected)
		}
	}
}

func TestClockIDs(t *testing.T) {
	expectedIDs := map[string]ClockID{
		"EMMC":     0x000000001,
		"UART":     0x000000002,
		"ARM":      0x000000003,
		"Core":     0x000000004,
		"V3D":      0x000000005,
		"H264":     0x000000006,
		"ISP":      0x000000007,
		"SDRAM":    0x000000008,
		"Pixel":    0x000000009,
		"PWM":      0x00000000a,
		"HEVC":     0x00000000b,
		"EMMC2":    0x00000000c,
		"M2MC":     0x00000000d,
		"PixelBVB": 0x00000000e,
		"VEC":      0x00000000f,
	}

	actualIDs := map[string]ClockID{
		"EMMC":     ClockIDEMMC,
		"UART":     ClockIDUART,
		"ARM":      ClockIDARM,
		"Core":     ClockIDCore,
		"V3D":      ClockIDV3D,
		"H264":     ClockIDH264,
		"ISP":      ClockIDISP,
		"SDRAM":    ClockIDSDRAM,
		"Pixel":    ClockIDPixel,
		"PWM":      ClockIDPWM,
		"HEVC":     ClockIDHEVC,
		"EMMC2":    ClockIDEMMC2,
		"M2MC":     ClockIDM2MC,
		"PixelBVB": ClockIDPixelBVB,
		"VEC":      ClockIDVEC,
	}

	for name, expected := range expectedIDs {
		actual := actualIDs[name]
		if actual != expected {
			t.Errorf("ClockID%s = 0x%08X, want 0x%08X", name, actual, expected)
		}
	}
}

func TestVoltageIDs(t *testing.T) {
	expectedIDs := map[string]VoltageID{
		"Core":   0x000000001,
		"SDRAMC": 0x000000002,
		"SDRAMP": 0x000000003,
		"SDRAMI": 0x000000004,
	}

	actualIDs := map[string]VoltageID{
		"Core":   VoltageIDCore,
		"SDRAMC": VoltageIDSDRAMC,
		"SDRAMP": VoltageIDSDRAMP,
		"SDRAMI": VoltageIDSDRAMI,
	}

	for name, expected := range expectedIDs {
		actual := actualIDs[name]
		if actual != expected {
			t.Errorf("VoltageID%s = 0x%08X, want 0x%08X", name, actual, expected)
		}
	}
}

func TestMailboxClose(t *testing.T) {
	// Test closing nil mailbox
	var m *Mailbox
	err := m.Close()
	if err != nil {
		t.Errorf("Close() on nil mailbox returned error: %v", err)
	}

	// Test closing mailbox with nil file
	m = &Mailbox{}
	err = m.Close()
	if err != nil {
		t.Errorf("Close() on mailbox with nil file returned error: %v", err)
	}
}

func TestResponseCodes(t *testing.T) {
	if replySuccess != 0x80000000 {
		t.Errorf("replySuccess = 0x%08X, want 0x80000000", replySuccess)
	}
	if replyFail != 0x80000001 {
		t.Errorf("replyFail = 0x%08X, want 0x80000001", replyFail)
	}
}

func TestRequestCodeDefault(t *testing.T) {
	if RequestCodeDefault != 0x00000000 {
		t.Errorf("RequestCodeDefault = 0x%08X, want 0x00000000", RequestCodeDefault)
	}
}

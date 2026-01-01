package ioctl

import (
	"testing"
)

func TestIO(t *testing.T) {
	tests := []struct {
		name     string
		typ      uint
		nr       uint
		expected uint
	}{
		{
			name:     "zero values",
			typ:      0,
			nr:       0,
			expected: 0,
		},
		{
			name:     "type only",
			typ:      'd',
			nr:       0,
			expected: uint('d') << iocTypeshift,
		},
		{
			name:     "nr only",
			typ:      0,
			nr:       1,
			expected: 1,
		},
		{
			name:     "both type and nr",
			typ:      'd',
			nr:       5,
			expected: (uint('d') << iocTypeshift) | 5,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := IO(tt.typ, tt.nr)
			if result != tt.expected {
				t.Errorf("IO(%d, %d) = 0x%08X, want 0x%08X", tt.typ, tt.nr, result, tt.expected)
			}
		})
	}
}

func TestIOR(t *testing.T) {
	tests := []struct {
		name     string
		typ      uint
		nr       uint
		size     uint
		expected uint
	}{
		{
			name:     "zero values",
			typ:      0,
			nr:       0,
			size:     0,
			expected: iocRead << iocDirshift,
		},
		{
			name:     "with size",
			typ:      'd',
			nr:       0,
			size:     4,
			expected: (iocRead << iocDirshift) | (uint('d') << iocTypeshift) | (4 << iocSizeshift),
		},
		{
			name:     "all parameters",
			typ:      'v',
			nr:       1,
			size:     8,
			expected: (iocRead << iocDirshift) | (uint('v') << iocTypeshift) | 1 | (8 << iocSizeshift),
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := IOR(tt.typ, tt.nr, tt.size)
			if result != tt.expected {
				t.Errorf("IOR(%d, %d, %d) = 0x%08X, want 0x%08X", tt.typ, tt.nr, tt.size, result, tt.expected)
			}
		})
	}
}

func TestIOW(t *testing.T) {
	tests := []struct {
		name     string
		typ      uint
		nr       uint
		size     uint
		expected uint
	}{
		{
			name:     "zero values",
			typ:      0,
			nr:       0,
			size:     0,
			expected: iocWrite << iocDirshift,
		},
		{
			name:     "with size",
			typ:      'd',
			nr:       0,
			size:     4,
			expected: (iocWrite << iocDirshift) | (uint('d') << iocTypeshift) | (4 << iocSizeshift),
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := IOW(tt.typ, tt.nr, tt.size)
			if result != tt.expected {
				t.Errorf("IOW(%d, %d, %d) = 0x%08X, want 0x%08X", tt.typ, tt.nr, tt.size, result, tt.expected)
			}
		})
	}
}

func TestIOWR(t *testing.T) {
	tests := []struct {
		name     string
		typ      uint
		nr       uint
		size     uint
		expected uint
	}{
		{
			name:     "zero values",
			typ:      0,
			nr:       0,
			size:     0,
			expected: (iocRead | iocWrite) << iocDirshift,
		},
		{
			name:     "mailbox ioctl pattern",
			typ:      'd',
			nr:       0,
			size:     8,
			expected: ((iocRead | iocWrite) << iocDirshift) | (uint('d') << iocTypeshift) | (8 << iocSizeshift),
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := IOWR(tt.typ, tt.nr, tt.size)
			if result != tt.expected {
				t.Errorf("IOWR(%d, %d, %d) = 0x%08X, want 0x%08X", tt.typ, tt.nr, tt.size, result, tt.expected)
			}
		})
	}
}

func TestIocDirectionBits(t *testing.T) {
	// Verify direction bits are correctly positioned
	ioResult := IO('x', 1)
	iorResult := IOR('x', 1, 4)
	iowResult := IOW('x', 1, 4)
	iowrResult := IOWR('x', 1, 4)

	// Extract direction bits
	ioDir := (ioResult >> iocDirshift) & ((1 << iocDirbits) - 1)
	iorDir := (iorResult >> iocDirshift) & ((1 << iocDirbits) - 1)
	iowDir := (iowResult >> iocDirshift) & ((1 << iocDirbits) - 1)
	iowrDir := (iowrResult >> iocDirshift) & ((1 << iocDirbits) - 1)

	if ioDir != iocNone {
		t.Errorf("IO direction = %d, want %d (iocNone)", ioDir, iocNone)
	}
	if iorDir != iocRead {
		t.Errorf("IOR direction = %d, want %d (iocRead)", iorDir, iocRead)
	}
	if iowDir != iocWrite {
		t.Errorf("IOW direction = %d, want %d (iocWrite)", iowDir, iocWrite)
	}
	if iowrDir != (iocRead | iocWrite) {
		t.Errorf("IOWR direction = %d, want %d (iocRead|iocWrite)", iowrDir, iocRead|iocWrite)
	}
}

func TestIocConstants(t *testing.T) {
	// Verify constants match Linux ioctl.h conventions
	if iocNrbits != 8 {
		t.Errorf("iocNrbits = %d, want 8", iocNrbits)
	}
	if iocTypebits != 8 {
		t.Errorf("iocTypebits = %d, want 8", iocTypebits)
	}
	if iocSizebits != 14 {
		t.Errorf("iocSizebits = %d, want 14", iocSizebits)
	}
	if iocDirbits != 2 {
		t.Errorf("iocDirbits = %d, want 2", iocDirbits)
	}

	// Verify shifts
	if iocNrshift != 0 {
		t.Errorf("iocNrshift = %d, want 0", iocNrshift)
	}
	if iocTypeshift != 8 {
		t.Errorf("iocTypeshift = %d, want 8", iocTypeshift)
	}
	if iocSizeshift != 16 {
		t.Errorf("iocSizeshift = %d, want 16", iocSizeshift)
	}
	if iocDirshift != 30 {
		t.Errorf("iocDirshift = %d, want 30", iocDirshift)
	}
}

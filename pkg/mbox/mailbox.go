/*
Package mbox implements the Mailbox protocol used to communicate between the VideoCore GPU and
ARM processor on a Raspberry Pi.

Supports Raspberry Pi 1 through Raspberry Pi 5.

https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
*/
package mbox

import (
	"errors"
	"fmt"
	"io/fs"
	"os"
	"sync"
	"unsafe"

	"github.com/CHA0S-CORP/rpi_exporter/pkg/ioctl"
)

const (
	RequestCodeDefault uint32 = 0x00000000
)

const (
	TagGetFirmwareRevision  uint32 = 0x00000001
	TagGetBoardModel        uint32 = 0x00010001
	TagGetBoardRevision     uint32 = 0x00010002
	TagGetBoardMAC          uint32 = 0x00010003
	TagGetPowerState        uint32 = 0x00020001
	TagGetClockRate         uint32 = 0x00030002
	TagGetVoltage           uint32 = 0x00030003
	TagGetMaxVoltage        uint32 = 0x00030005
	TagGetTemperature       uint32 = 0x00030006
	TagGetMinVoltage        uint32 = 0x00030008
	TagGetTurbo             uint32 = 0x00030009
	TagGetMaxTemperature    uint32 = 0x0003000A
	TagGetClockRateMeasured uint32 = 0x00030047
)

const (
	replySuccess uint32 = 0x80000000
	replyFail    uint32 = 0x80000001
)

var Debug = false

var (
	ErrNotImplemented = errors.New("vcio: not implemented")
	ErrRequestBuffer  = errors.New("vcio: error parsing request buffer")
)

var mbIoctl = ioctl.IOWR('d', 0, uint(unsafe.Sizeof(new(byte))))

type Tag []uint32

var EndTag = Tag{0}

func (t Tag) ID() uint32 {
	if !t.IsValid() {
		return 0
	}
	return t[0]
}

// Cap returns the length of the value buffer in bytes.
func (t Tag) Cap() int {
	if !t.IsValid() {
		return 0
	}
	return int(t[1])
}

// Len is the length of a response value in bytes. The actual bytes written by a response will be
// the lesser of Len and Cap. If Len is greater than Cap, retry the request with a bigger value
// buffer.
//
// To get the length of the entire tag in uint32s, including headers, use len(Tag).
func (t Tag) Len() int {
	if !t.IsValid() || !t.IsResponse() {
		return 0
	}
	return int(t[2] & 0x7FFFFFFF)
}

func (t Tag) IsResponse() bool {
	if !t.IsValid() {
		return false
	}
	return t[2]&0x80000000 == 0x80000000
}

// Value returns the value buffer as 32-bit words.
func (t Tag) Value() []uint32 {
	if !t.IsValid() {
		return nil
	}
	// Round up to include partial words: (len + 3) / 4
	words := (t.Len() + 3) / 4
	if words == 0 {
		return nil
	}
	return t[3 : 3+words]
}

func (t Tag) IsEnd() bool { return len(t) == 1 && t[0] == 0 }

func (t Tag) IsValid() bool {
	if len(t) == 0 {
		return false // Nil or empty
	}
	if len(t) == 1 {
		return t.IsEnd() // End tag
	}
	if len(t) < 3 {
		return false // Too short
	}
	if len(t) != int(3+t[1]/4) {
		// Incorrect size with value buffer
		return false
	}
	return true
}

func ReadTag(b []uint32) (Tag, error) {
	if len(b) > 0 && b[0] == 0 {
		return EndTag, nil
	}
	if len(b) < 3 {
		return nil, fmt.Errorf("vcio: tag buffer is too small")
	}
	sz := 3 + int(b[1]/4)
	if len(b) < sz {
		return nil, fmt.Errorf("vcio: tag buffer is too small")
	}
	return Tag(b[:sz]), nil
}

// Mailbox implements the Mailbox protocol used by the VideoCore and ARM on a Raspberry Pi.
type Mailbox struct {
	mu           sync.Mutex
	f            *os.File
	bufUnaligned [48]uint32
	buf          []uint32
}

func Open() (f *Mailbox, err error) {
	var ff *os.File
	ff, err = os.OpenFile("/dev/vcio", os.O_RDONLY, os.ModePerm)
	if errors.Is(err, fs.ErrNotExist) {
		return nil, ErrNotImplemented
	}
	if err != nil {
		return nil, err
	}
	return &Mailbox{f: ff}, nil
}

func (c *Mailbox) Close() (err error) {
	if c == nil || c.f == nil {
		return nil
	}
	err = c.f.Close()
	c.f = nil
	return
}

// Do sends a single command tag and returns all response tags. Returned memory is only usable until
// the next request is made.
func (m *Mailbox) Do(tagID uint32, bufferBytes int, args ...uint32) ([]Tag, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	// Align buffer to 16-byte boundary
	if m.buf == nil {
		baseAddr := uintptr(unsafe.Pointer(&m.bufUnaligned[0]))
		// Calculate how many bytes we need to skip to reach 16-byte alignment
		// Each uint32 is 4 bytes, so we divide by 4 to get the offset in uint32 units
		offsetBytes := (16 - (baseAddr & 15)) & 15
		offsetWords := int(offsetBytes / 4)
		m.buf = m.bufUnaligned[offsetWords : len(m.bufUnaligned)-offsetWords]
	}

	// Compute value buffer length (ensure 32-bit aligned)
	if bufferBytes%4 != 0 {
		bufferBytes = (bufferBytes + 3) &^ 3
	}
	if bufferBytes < len(args)*4 {
		bufferBytes = len(args) * 4
	}

	// Calculate message size: header (2) + tag header (3) + value buffer + end tag (1)
	valueWords := bufferBytes / 4
	msgWords := 2 + 3 + valueWords + 1
	msgBytes := msgWords * 4

	// Zero the buffer before use
	for i := range m.buf {
		m.buf[i] = 0
	}

	// Write request header
	m.buf[0] = uint32(msgBytes)   // actual message size in bytes
	m.buf[1] = RequestCodeDefault // request code

	// Write request tag
	m.buf[2] = tagID
	m.buf[3] = uint32(bufferBytes) // Value buffer size in bytes
	m.buf[4] = 0                   // This is a request
	copy(m.buf[5:], args)          // Write value buffer
	// End tag at m.buf[5+valueWords] is already zero from buffer zeroing

	debugf("TX (msgBytes=%d, valueWords=%d):\n", msgBytes, valueWords)
	for i, v := range m.buf[:msgWords] {
		debugf("  %02d: 0x%08X\n", i, v)
	}

	// Send message via ioctl
	debugf("ioctl: fd=%d, op=0x%08X, buf=%p\n", m.f.Fd(), mbIoctl, &m.buf[0])
	err := ioctl.Ioctl(m.f.Fd(), uintptr(mbIoctl), uintptr(unsafe.Pointer(&m.buf[0])))
	if err != nil {
		debugf("ioctl error: %v\n", err)
	}
	if err != nil {
		return nil, err
	}

	debugf("RX:\n")
	for i, v := range m.buf[:16] {
		debugf("  %02d: 0x%08X\n", i, v)
	}

	// Check response header
	if m.buf[1] == replyFail {
		return nil, ErrRequestBuffer
	}
	if m.buf[1]&replySuccess != replySuccess {
		return nil, fmt.Errorf("vcio: unexpected response code: 0x%08x", m.buf[1])
	}

	b := m.buf[2:]
	var tags []Tag
	for {
		t, err := ReadTag(b)
		if err != nil {
			return nil, err
		}
		if t.IsEnd() {
			break
		}
		if tags == nil {
			tags = make([]Tag, 0, 1)
		}
		tags = append(tags, t)
		b = b[len(t):]
	}

	return tags, nil
}

func (m *Mailbox) getUint32(tagID uint32) (uint32, error) {
	tags, err := m.Do(tagID, 4)
	if err != nil {
		return 0, err
	}
	if len(tags) == 0 {
		return 0, fmt.Errorf("vcio: no tags returned for tag ID 0x%08x", tagID)
	}
	val := tags[0].Value()
	if len(val) == 0 {
		return 0, fmt.Errorf("vcio: empty value buffer for tag ID 0x%08x", tagID)
	}
	return val[0], nil
}

func (m *Mailbox) getUint32ByID(tagID, id uint32) (uint32, error) {
	tags, err := m.Do(tagID, 8, id)
	if err != nil {
		return 0, err
	}
	if len(tags) == 0 {
		return 0, fmt.Errorf("vcio: no tags returned for tag ID 0x%08x", tagID)
	}
	val := tags[0].Value()
	if len(val) < 2 {
		return 0, fmt.Errorf("vcio: value buffer too small for tag ID 0x%08x (got %d, need 2)", tagID, len(val))
	}
	if val[0] != id {
		return 0, fmt.Errorf("vcio: response ID mismatch for tag ID 0x%08x (got 0x%08x, want 0x%08x)", tagID, val[0], id)
	}
	return val[1], nil
}

func debugf(format string, a ...interface{}) {
	if !Debug {
		return
	}
	fmt.Fprintf(os.Stderr, format, a...)
}

// GetFirmwareRevision returns the firmware revision of the VideoCore component.
func (m *Mailbox) GetFirmwareRevision() (uint32, error) {
	return m.getUint32(TagGetFirmwareRevision)
}

// GetBoardModel returns the model number of the system board.
func (m *Mailbox) GetBoardModel() (uint32, error) {
	return m.getUint32(TagGetBoardModel)
}

// GetBoardRevision returns the revision number of the system board.
func (m *Mailbox) GetBoardRevision() (uint32, error) {
	return m.getUint32(TagGetBoardRevision)
}

type PowerDeviceID uint32

const (
	PowerDeviceIDSDCard PowerDeviceID = 0x00000000
	PowerDeviceIDUART0  PowerDeviceID = 0x00000001
	PowerDeviceIDUART1  PowerDeviceID = 0x00000002
	PowerDeviceIDUSBHCD PowerDeviceID = 0x00000003
	PowerDeviceIDI2C0   PowerDeviceID = 0x00000004
	PowerDeviceIDI2C1   PowerDeviceID = 0x00000005
	PowerDeviceIDI2C2   PowerDeviceID = 0x00000006
	PowerDeviceIDSPI    PowerDeviceID = 0x00000007
	PowerDeviceIDCCP2TX PowerDeviceID = 0x00000008
	// RPi 4/5 additional power devices
	PowerDeviceIDUnknown0 PowerDeviceID = 0x00000009
	PowerDeviceIDUnknown1 PowerDeviceID = 0x0000000a
)

type PowerState uint32

const (
	PowerStateOff     uint32 = 0x00000000
	PowerStateOn      uint32 = 0x00000001
	PowerStateMissing uint32 = 0x00000002
)

func (m *Mailbox) GetPowerState(id PowerDeviceID) (PowerState, error) {
	tags, err := m.Do(TagGetPowerState, 8, uint32(id))
	if err != nil {
		return 0, err
	}
	if len(tags) == 0 {
		return 0, fmt.Errorf("vcio: no tags returned for GetPowerState")
	}
	val := tags[0].Value()
	if len(val) < 2 {
		return 0, fmt.Errorf("vcio: value buffer too small for GetPowerState (got %d, need 2)", len(val))
	}
	return PowerState(val[1] & 0x03), nil
}

type ClockID uint32

const (
	ClockIDEMMC     ClockID = 0x000000001
	ClockIDUART     ClockID = 0x000000002
	ClockIDARM      ClockID = 0x000000003
	ClockIDCore     ClockID = 0x000000004
	ClockIDV3D      ClockID = 0x000000005
	ClockIDH264     ClockID = 0x000000006
	ClockIDISP      ClockID = 0x000000007
	ClockIDSDRAM    ClockID = 0x000000008
	ClockIDPixel    ClockID = 0x000000009
	ClockIDPWM      ClockID = 0x00000000a
	ClockIDHEVC     ClockID = 0x00000000b
	ClockIDEMMC2    ClockID = 0x00000000c
	ClockIDM2MC     ClockID = 0x00000000d
	ClockIDPixelBVB ClockID = 0x00000000e
	// RPi 5 RP1 southbridge clocks
	ClockIDVEC ClockID = 0x00000000f
)

func (m *Mailbox) GetClockRate(id ClockID) (hz int, err error) {
	v, err := m.getUint32ByID(TagGetClockRate, uint32(id))
	if err != nil {
		return 0, err
	}
	return int(v), nil
}

func (m *Mailbox) GetClockRateMeasured(id ClockID) (hz int, err error) {
	v, err := m.getUint32ByID(TagGetClockRateMeasured, uint32(id))
	if err != nil {
		return 0, err
	}
	return int(v), nil
}

func (m *Mailbox) getTemperature(tag uint32) (float32, error) {
	v, err := m.getUint32ByID(tag, 0)
	if err != nil {
		return 0, err
	}
	return float32(v) / 1000, nil
}

// GetTemperature returns the temperature of the SoC in degrees celcius.
func (m *Mailbox) GetTemperature() (float32, error) {
	return m.getTemperature(TagGetTemperature)
}

// GetMaxTemperature returns the maximum safe temperature of the SoC in degrees celcius.
//
// Overclock may be disabled above this temperature.
func (m *Mailbox) GetMaxTemperature() (float32, error) {
	return m.getTemperature(TagGetMaxTemperature)
}

type VoltageID uint32

const (
	VoltageIDCore   VoltageID = 0x000000001
	VoltageIDSDRAMC VoltageID = 0x000000002
	VoltageIDSDRAMP VoltageID = 0x000000003
	VoltageIDSDRAMI VoltageID = 0x000000004
)

func (m *Mailbox) getVoltage(tag uint32, id VoltageID) (float32, error) {
	v, err := m.getUint32ByID(tag, uint32(id))
	if err != nil {
		return 0, err
	}
	return float32(v) / 1000000, nil
}

// GetVoltage returns the voltage of the given component.
func (m *Mailbox) GetVoltage(id VoltageID) (float32, error) {
	return m.getVoltage(TagGetVoltage, id)
}

// GetMaxVoltage returns the minimum supported voltage of the given component.
func (m *Mailbox) GetMinVoltage(id VoltageID) (float32, error) {
	return m.getVoltage(TagGetMinVoltage, id)
}

// GetMaxVoltage returns the maximum supported voltage of the given component.
func (m *Mailbox) GetMaxVoltage(id VoltageID) (float32, error) {
	return m.getVoltage(TagGetMaxVoltage, id)
}

func (m *Mailbox) GetTurbo() (bool, error) {
	tags, err := m.Do(TagGetTurbo, 8, 0)
	if err != nil {
		return false, err
	}
	if len(tags) == 0 {
		return false, fmt.Errorf("vcio: no tags returned for GetTurbo")
	}
	val := tags[0].Value()
	if len(val) < 2 {
		return false, fmt.Errorf("vcio: value buffer too small for GetTurbo (got %d, need 2)", len(val))
	}
	return val[1] == 1, nil
}
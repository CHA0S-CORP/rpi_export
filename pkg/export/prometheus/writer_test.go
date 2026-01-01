package prometheus

import (
	"bytes"
	"strings"
	"testing"

	"github.com/CHA0S-CORP/rpi_exporter/pkg/mbox"
)

func TestFormatTemp(t *testing.T) {
	tests := []struct {
		input    float32
		expected string
	}{
		{0, "0.000"},
		{25.5, "25.500"},
		{100.123, "100.123"},
		{-10.5, "-10.500"},
		{45.6789, "45.679"},
	}

	for _, tt := range tests {
		result := formatTemp(tt.input)
		if result != tt.expected {
			t.Errorf("formatTemp(%v) = %q, want %q", tt.input, result, tt.expected)
		}
	}
}

func TestFormatVolts(t *testing.T) {
	tests := []struct {
		input    float32
		expected string
	}{
		{0, "0.000000"},
		{1.2, "1.200000"},
		{0.000001, "0.000001"},
		{1.234567, "1.234567"},
	}

	for _, tt := range tests {
		result := formatVolts(tt.input)
		if result != tt.expected {
			t.Errorf("formatVolts(%v) = %q, want %q", tt.input, result, tt.expected)
		}
	}
}

func TestFormatBool(t *testing.T) {
	if formatBool(true) != "1" {
		t.Errorf("formatBool(true) = %q, want %q", formatBool(true), "1")
	}
	if formatBool(false) != "0" {
		t.Errorf("formatBool(false) = %q, want %q", formatBool(false), "0")
	}
}

func TestCelsiusToFahrenheit(t *testing.T) {
	tests := []struct {
		celsius    float64
		fahrenheit float64
	}{
		{0, 32},
		{100, 212},
		{-40, -40},
		{25, 77},
		{37, 98.6},
	}

	for _, tt := range tests {
		result := celsiusToFahrenheit(tt.celsius)
		if result != tt.fahrenheit {
			t.Errorf("celsiusToFahrenheit(%v) = %v, want %v", tt.celsius, result, tt.fahrenheit)
		}
	}
}

func TestWriteHeader(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf}

	w.writeHeader("test_metric", "A test metric description", metricTypeGauge)

	output := buf.String()
	if !strings.Contains(output, "# HELP test_metric A test metric description") {
		t.Errorf("writeHeader missing HELP line, got: %s", output)
	}
	if !strings.Contains(output, "# TYPE test_metric gauge") {
		t.Errorf("writeHeader missing TYPE line, got: %s", output)
	}
	if w.name != "test_metric" {
		t.Errorf("writeHeader name = %q, want %q", w.name, "test_metric")
	}
}

func TestWriteHeaderWithLabels(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf}

	w.writeHeader("labeled_metric", "A labeled metric", metricTypeCounter, "label1", "label2")

	if len(w.labels) != 2 {
		t.Errorf("writeHeader labels len = %d, want 2", len(w.labels))
	}
	if w.labels[0] != "label1" || w.labels[1] != "label2" {
		t.Errorf("writeHeader labels = %v, want [label1, label2]", w.labels)
	}
}

func TestWriteSampleNoLabels(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf, name: "simple_metric", labels: nil}

	w.writeSample(42)

	output := buf.String()
	expected := "simple_metric 42\n"
	if output != expected {
		t.Errorf("writeSample() = %q, want %q", output, expected)
	}
}

func TestWriteSampleWithLabels(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf, name: "labeled_metric", labels: []string{"id", "type"}}

	w.writeSample(123, "device0", "sensor")

	output := buf.String()
	expected := `labeled_metric{id="device0",type="sensor"} 123` + "\n"
	if output != expected {
		t.Errorf("writeSample() = %q, want %q", output, expected)
	}
}

func TestWriteSampleWrongLabelCount(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf, name: "labeled_metric", labels: []string{"id", "type"}}

	w.writeSample(123, "only_one")

	if w.err == nil {
		t.Error("writeSample() with wrong label count should set error")
	}
}

func TestWriteSampleWithError(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf, name: "metric", labels: nil, err: bytes.ErrTooLarge}

	initialLen := buf.Len()
	w.writeSample(42)

	if buf.Len() != initialLen {
		t.Error("writeSample() should not write when error is set")
	}
}

func TestVoltageLabelsByID(t *testing.T) {
	expectedLabels := map[mbox.VoltageID]string{
		mbox.VoltageIDCore:   "core",
		mbox.VoltageIDSDRAMC: "sdram_c",
		mbox.VoltageIDSDRAMI: "sdram_i",
		mbox.VoltageIDSDRAMP: "sdram_p",
	}

	for id, expected := range expectedLabels {
		actual := voltageLabelsByID[id]
		if actual != expected {
			t.Errorf("voltageLabelsByID[%d] = %q, want %q", id, actual, expected)
		}
	}
}

func TestPowerLabelsByID(t *testing.T) {
	expectedLabels := map[mbox.PowerDeviceID]string{
		mbox.PowerDeviceIDSDCard: "sd_card",
		mbox.PowerDeviceIDUART0:  "uart0",
		mbox.PowerDeviceIDUART1:  "uart1",
		mbox.PowerDeviceIDUSBHCD: "usb_hcd",
		mbox.PowerDeviceIDI2C0:   "i2c0",
		mbox.PowerDeviceIDI2C1:   "i2c1",
		mbox.PowerDeviceIDI2C2:   "i2c2",
		mbox.PowerDeviceIDSPI:    "spi",
		mbox.PowerDeviceIDCCP2TX: "ccp2tx",
	}

	for id, expected := range expectedLabels {
		actual := powerLabelsByID[id]
		if actual != expected {
			t.Errorf("powerLabelsByID[%d] = %q, want %q", id, actual, expected)
		}
	}
}

func TestClockLabelsByID(t *testing.T) {
	expectedLabels := map[mbox.ClockID]string{
		mbox.ClockIDEMMC:     "emmc",
		mbox.ClockIDUART:     "uart",
		mbox.ClockIDARM:      "arm",
		mbox.ClockIDCore:     "core",
		mbox.ClockIDV3D:      "v3d",
		mbox.ClockIDH264:     "h264",
		mbox.ClockIDISP:      "isp",
		mbox.ClockIDSDRAM:    "sdram",
		mbox.ClockIDPixel:    "pixel",
		mbox.ClockIDPWM:      "pwm",
		mbox.ClockIDHEVC:     "hevc",
		mbox.ClockIDEMMC2:    "emmc2",
		mbox.ClockIDM2MC:     "m2mc",
		mbox.ClockIDPixelBVB: "pixel_bvb",
		mbox.ClockIDVEC:      "vec",
	}

	for id, expected := range expectedLabels {
		actual := clockLabelsByID[id]
		if actual != expected {
			t.Errorf("clockLabelsByID[%d] = %q, want %q", id, actual, expected)
		}
	}
}

func TestMetricTypeConstants(t *testing.T) {
	if metricTypeGauge != "gauge" {
		t.Errorf("metricTypeGauge = %q, want %q", metricTypeGauge, "gauge")
	}
	if metricTypeCounter != "counter" {
		t.Errorf("metricTypeCounter = %q, want %q", metricTypeCounter, "counter")
	}
}

func TestWriteExporterMetrics(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf}

	w.writeExporterMetrics(0.123456)

	output := buf.String()

	if !strings.Contains(output, "rpi_exporter_scrape_duration_seconds") {
		t.Error("missing rpi_exporter_scrape_duration_seconds metric")
	}
	if !strings.Contains(output, "0.123456") {
		t.Error("missing scrape duration value")
	}
	if !strings.Contains(output, "rpi_exporter_scrapes_total") {
		t.Error("missing rpi_exporter_scrapes_total metric")
	}
	if !strings.Contains(output, "rpi_exporter_scrape_errors_total") {
		t.Error("missing rpi_exporter_scrape_errors_total metric")
	}
}

func TestConfigStruct(t *testing.T) {
	cfg := &Config{}
	if cfg.SenseHat != nil {
		t.Error("empty Config should have nil SenseHat")
	}
}

func TestMultipleSamples(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf}

	w.writeHeader("multi_sample_metric", "A metric with multiple samples", metricTypeGauge, "id")
	w.writeSample(1, "first")
	w.writeSample(2, "second")
	w.writeSample(3, "third")

	output := buf.String()

	if !strings.Contains(output, `id="first"`) {
		t.Error("missing first sample")
	}
	if !strings.Contains(output, `id="second"`) {
		t.Error("missing second sample")
	}
	if !strings.Contains(output, `id="third"`) {
		t.Error("missing third sample")
	}
}

func TestPrometheusFormatCompliance(t *testing.T) {
	var buf bytes.Buffer
	w := &expWriter{w: &buf}

	w.writeHeader("test_metric", "Test help text", metricTypeGauge, "label")
	w.writeSample(42.5, "value")

	output := buf.String()
	lines := strings.Split(strings.TrimSpace(output), "\n")

	if !strings.HasPrefix(lines[0], "# HELP ") {
		t.Errorf("first line should be HELP comment, got: %s", lines[0])
	}
	if !strings.HasPrefix(lines[1], "# TYPE ") {
		t.Errorf("second line should be TYPE comment, got: %s", lines[1])
	}
	if !strings.Contains(lines[2], `label="value"`) {
		t.Errorf("sample missing proper label format, got: %s", lines[2])
	}
}

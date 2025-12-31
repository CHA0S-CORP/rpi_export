/*
Package prometheus provides utilities for authoring a Prometheus metric exporter
for Raspberry Pi hardware metrics including VideoCore and optional Sense HAT sensors.
*/
package prometheus

import (
	"fmt"
	"io"
	"sync/atomic"
	"time"

	"github.com/CHA0S-CORP/rpi_exporter/pkg/mbox"
	"github.com/CHA0S-CORP/rpi_exporter/pkg/sensehat"
)

// Exporter metrics
var (
	scrapeCount        uint64
	scrapeErrorCount   uint64
	lastScrapeDuration float64
)

const (
	metricTypeGauge   = "gauge"
	metricTypeCounter = "counter"
)

var voltageLabelsByID = map[mbox.VoltageID]string{
	mbox.VoltageIDCore:   "core",
	mbox.VoltageIDSDRAMC: "sdram_c",
	mbox.VoltageIDSDRAMI: "sdram_i",
	mbox.VoltageIDSDRAMP: "sdram_p",
}

var powerLabelsByID = map[mbox.PowerDeviceID]string{
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

var clockLabelsByID = map[mbox.ClockID]string{
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

func formatTemp(t float32) string  { return fmt.Sprintf("%.03f", t) }
func formatVolts(v float32) string { return fmt.Sprintf("%.06f", v) }

func formatBool(b bool) string {
	if b {
		return "1"
	}
	return "0"
}

func celsiusToFahrenheit(c float64) float64 {
	return (c * 9 / 5) + 32
}

type expWriter struct {
	w      io.Writer
	name   string
	labels []string
	err    error
}

// Config holds configuration for the Prometheus writer.
type Config struct {
	SenseHat *sensehat.SenseHat // nil to disable Sense HAT metrics
}

// Write all metrics in Prometheus text-based exposition format.
func Write(w io.Writer, cfg *Config) error {
	start := time.Now()
	atomic.AddUint64(&scrapeCount, 1)

	ew := &expWriter{w: w}
	err := ew.write(cfg)

	duration := time.Since(start).Seconds()
	lastScrapeDuration = duration

	if err != nil {
		atomic.AddUint64(&scrapeErrorCount, 1)
		return err
	}

	// Write exporter metrics
	ew.writeExporterMetrics(duration)

	return ew.err
}

func (w *expWriter) writeHeader(name, help, metricType string, labels ...string) {
	w.name = name
	w.labels = labels
	fmt.Fprintf(w.w, "# HELP %s %s\n", name, help)
	fmt.Fprintf(w.w, "# TYPE %s %v\n", name, metricType)
}

func (w *expWriter) writeSample(val interface{}, labels ...string) {
	if w.err != nil {
		return
	}
	if len(labels) != len(w.labels) {
		w.err = fmt.Errorf("incorrect metrics label count: got %d, want %d", len(labels), len(w.labels))
		return
	}
	fmt.Fprintf(w.w, w.name)
	if len(w.labels) > 0 {
		fmt.Fprintf(w.w, "{")
		for i, key := range w.labels {
			if i > 0 {
				fmt.Fprintf(w.w, ",")
			}
			fmt.Fprintf(w.w, "%s=\"%s\"", key, labels[i])
		}
		fmt.Fprintf(w.w, "}")
	}
	fmt.Fprintf(w.w, " %v\n", val)
}

func (w *expWriter) write(cfg *Config) error {
	// Write VideoCore metrics
	if err := w.writeVideoCoreMetrics(); err != nil {
		return err
	}

	// Write Sense HAT metrics if enabled
	if cfg != nil && cfg.SenseHat != nil {
		if err := w.writeSenseHatMetrics(cfg.SenseHat); err != nil {
			return err
		}
	}

	return w.err
}

func (w *expWriter) writeVideoCoreMetrics() error {
	m, err := mbox.Open()
	if err != nil {
		return err
	}
	defer m.Close()

	/*
	 * Hardware.
	 */
	w.writeHeader("rpi_vc_revision", "Firmware revision of the VideoCore device.", metricTypeGauge)
	rev, err := m.GetFirmwareRevision()
	if err != nil {
		return err
	}
	w.writeSample(rev)

	w.writeHeader("rpi_board_model", "Board model.", metricTypeGauge)
	model, err := m.GetBoardModel()
	if err != nil {
		return err
	}
	w.writeSample(model)

	w.writeHeader("rpi_board_revision", "Board revision.", metricTypeGauge)
	rev, err = m.GetBoardRevision()
	if err != nil {
		return err
	}
	w.writeSample(rev)

	/*
	 * Power.
	 */
	w.writeHeader(
		"rpi_power_state",
		"Component power state (0: off, 1: on, 2: missing).",
		metricTypeGauge,
		"id",
	)
	for id, label := range powerLabelsByID {
		powerState, err := m.GetPowerState(id)
		if err != nil {
			return err
		}
		w.writeSample(powerState, label)
	}

	/*
	 * Clocks.
	 */
	w.writeHeader("rpi_clock_rate_hz", "Clock rate in Hertz.", metricTypeGauge, "id")
	for id, label := range clockLabelsByID {
		clockRate, err := m.GetClockRate(id)
		if err != nil {
			return err
		}
		w.writeSample(clockRate, label)
	}

	w.writeHeader("rpi_clock_rate_measured_hz", "Measured clock rate in Hertz.", metricTypeGauge, "id")
	for id, label := range clockLabelsByID {
		clockRate, err := m.GetClockRateMeasured(id)
		if err != nil {
			return err
		}
		w.writeSample(clockRate, label)
	}

	w.writeHeader("rpi_turbo", "Turbo state.", metricTypeGauge)
	turbo, err := m.GetTurbo()
	if err != nil {
		return err
	}
	w.writeSample(formatBool(turbo))

	/*
	 * Temperature sensors.
	 */
	w.writeHeader(
		"rpi_temperature_c",
		"Temperature of the SoC in degrees celsius.",
		metricTypeGauge,
		"id",
	)
	temp, err := m.GetTemperature()
	if err != nil {
		return err
	}
	w.writeSample(formatTemp(temp), "soc")

	w.writeHeader(
		"rpi_temperature_f",
		"Temperature of the SoC in degrees fahrenheit.",
		metricTypeGauge,
		"id",
	)
	w.writeSample(formatTemp(temp*9/5+32), "soc")

	w.writeHeader(
		"rpi_max_temperature_c",
		"Maximum temperature of the SoC in degrees celsius.",
		metricTypeGauge,
		"id",
	)
	maxTemp, err := m.GetMaxTemperature()
	if err != nil {
		return err
	}
	w.writeSample(formatTemp(maxTemp), "soc")

	w.writeHeader(
		"rpi_max_temperature_f",
		"Maximum temperature of the SoC in degrees fahrenheit.",
		metricTypeGauge,
		"id",
	)
	w.writeSample(formatTemp(maxTemp*9/5+32), "soc")

	/*
	 * Voltages
	 */
	w.writeHeader("rpi_voltage", "Current component voltage.", metricTypeGauge, "id")
	for id, label := range voltageLabelsByID {
		volts, err := m.GetVoltage(id)
		if err != nil {
			return err
		}
		w.writeSample(formatVolts(volts), label)
	}

	w.writeHeader("rpi_voltage_min", "Minimum supported component voltage.", metricTypeGauge, "id")
	for id, label := range voltageLabelsByID {
		volts, err := m.GetMinVoltage(id)
		if err != nil {
			return err
		}
		w.writeSample(formatVolts(volts), label)
	}

	w.writeHeader("rpi_voltage_max", "Maximum supported component voltage.", metricTypeGauge, "id")
	for id, label := range voltageLabelsByID {
		volts, err := m.GetMaxVoltage(id)
		if err != nil {
			return err
		}
		w.writeSample(formatVolts(volts), label)
	}

	return nil
}

func (w *expWriter) writeSenseHatMetrics(hat *sensehat.SenseHat) error {
	/*
	 * Temperature metrics (Fahrenheit)
	 */
	w.writeHeader(
		"sensehat_temperature_humidity_fahrenheit",
		"Temperature from humidity sensor in Fahrenheit.",
		metricTypeGauge,
	)
	tempH, err := hat.GetTemperatureFromHumidity()
	if err != nil {
		return err
	}
	w.writeSample(fmt.Sprintf("%.03f", celsiusToFahrenheit(tempH)))

	w.writeHeader(
		"sensehat_temperature_pressure_fahrenheit",
		"Temperature from pressure sensor in Fahrenheit.",
		metricTypeGauge,
	)
	tempP, err := hat.GetTemperatureFromPressure()
	if err != nil {
		return err
	}
	w.writeSample(fmt.Sprintf("%.03f", celsiusToFahrenheit(tempP)))

	// Average temperature
	avgTempC := (tempH + tempP) / 2
	w.writeHeader(
		"sensehat_temperature_average_fahrenheit",
		"Average of humidity and pressure sensor temps in Fahrenheit.",
		metricTypeGauge,
	)
	w.writeSample(fmt.Sprintf("%.03f", celsiusToFahrenheit(avgTempC)))

	// CPU-adjusted temperature
	cpuTempC, err := sensehat.GetCPUTemperature()
	if err == nil {
		w.writeHeader(
			"sensehat_cpu_temperature_fahrenheit",
			"Raspberry Pi CPU temperature in Fahrenheit.",
			metricTypeGauge,
		)
		w.writeSample(fmt.Sprintf("%.03f", celsiusToFahrenheit(cpuTempC)))

		// Sense HAT sits close to CPU and reads high - apply correction
		adjustedC := avgTempC - ((cpuTempC - avgTempC) / 1.5)
		w.writeHeader(
			"sensehat_temperature_adjusted_fahrenheit",
			"CPU-adjusted temperature estimate in Fahrenheit.",
			metricTypeGauge,
		)
		w.writeSample(fmt.Sprintf("%.03f", celsiusToFahrenheit(adjustedC)))
	}

	/*
	 * Environmental metrics
	 */
	w.writeHeader(
		"sensehat_humidity_percent",
		"Relative humidity percentage.",
		metricTypeGauge,
	)
	humidity, err := hat.GetHumidity()
	if err != nil {
		return err
	}
	w.writeSample(fmt.Sprintf("%.03f", humidity))

	w.writeHeader(
		"sensehat_pressure_millibars",
		"Atmospheric pressure in millibars.",
		metricTypeGauge,
	)
	pressure, err := hat.GetPressure()
	if err != nil {
		return err
	}
	w.writeSample(fmt.Sprintf("%.03f", pressure))

	/*
	 * Orientation metrics
	 */
	pitch, roll, yaw, err := hat.GetOrientation()
	if err != nil {
		return err
	}

	w.writeHeader("sensehat_orientation_pitch_degrees", "Pitch orientation in degrees.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.03f", pitch))

	w.writeHeader("sensehat_orientation_roll_degrees", "Roll orientation in degrees.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.03f", roll))

	w.writeHeader("sensehat_orientation_yaw_degrees", "Yaw orientation in degrees.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.03f", yaw))

	/*
	 * Accelerometer metrics
	 */
	ax, ay, az, err := hat.GetAccelerometer()
	if err != nil {
		return err
	}

	w.writeHeader("sensehat_accelerometer_x_g", "Accelerometer X-axis in Gs.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", ax))

	w.writeHeader("sensehat_accelerometer_y_g", "Accelerometer Y-axis in Gs.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", ay))

	w.writeHeader("sensehat_accelerometer_z_g", "Accelerometer Z-axis in Gs.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", az))

	/*
	 * Gyroscope metrics
	 */
	gx, gy, gz, err := hat.GetGyroscope()
	if err != nil {
		return err
	}

	w.writeHeader("sensehat_gyroscope_x_dps", "Gyroscope X-axis in degrees/second.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", gx))

	w.writeHeader("sensehat_gyroscope_y_dps", "Gyroscope Y-axis in degrees/second.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", gy))

	w.writeHeader("sensehat_gyroscope_z_dps", "Gyroscope Z-axis in degrees/second.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", gz))

	/*
	 * Magnetometer metrics
	 */
	mx, my, mz, err := hat.GetMagnetometer()
	if err != nil {
		return err
	}

	w.writeHeader("sensehat_magnetometer_x_microtesla", "Magnetometer X-axis in microteslas.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", mx))

	w.writeHeader("sensehat_magnetometer_y_microtesla", "Magnetometer Y-axis in microteslas.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", my))

	w.writeHeader("sensehat_magnetometer_z_microtesla", "Magnetometer Z-axis in microteslas.", metricTypeGauge)
	w.writeSample(fmt.Sprintf("%.06f", mz))

	/*
	 * Compass
	 */
	w.writeHeader("sensehat_compass_north_degrees", "Direction of North in degrees.", metricTypeGauge)
	compass, err := hat.GetCompass()
	if err != nil {
		return err
	}
	w.writeSample(fmt.Sprintf("%.03f", compass))

	/*
	 * IMU calibration status
	 */
	w.writeHeader("sensehat_imu_calibrated", "IMU calibration status (1=calibrated).", metricTypeGauge)
	if hat.IsIMUCalibrated() {
		w.writeSample(1)
	} else {
		w.writeSample(0)
	}

	/*
	 * Color sensor (Sense HAT v2 only)
	 */
	w.writeHeader("sensehat_color_sensor_available", "Color sensor available (1=yes, 0=no).", metricTypeGauge)
	if hat.HasColorSensor() {
		w.writeSample(1)

		r, g, b, c, err := hat.GetColor()
		if err == nil {
			w.writeHeader("sensehat_color_red", "Color sensor red channel (0-255).", metricTypeGauge)
			w.writeSample(r)

			w.writeHeader("sensehat_color_green", "Color sensor green channel (0-255).", metricTypeGauge)
			w.writeSample(g)

			w.writeHeader("sensehat_color_blue", "Color sensor blue channel (0-255).", metricTypeGauge)
			w.writeSample(b)

			w.writeHeader("sensehat_color_clear", "Color sensor clear/brightness channel (0-255).", metricTypeGauge)
			w.writeSample(c)
		}
	} else {
		w.writeSample(0)
	}

	return nil
}

func (w *expWriter) writeExporterMetrics(scrapeDuration float64) {
	/*
	 * Exporter metrics
	 */
	w.writeHeader(
		"rpi_exporter_scrape_duration_seconds",
		"Duration of the last scrape in seconds.",
		metricTypeGauge,
	)
	w.writeSample(fmt.Sprintf("%.06f", scrapeDuration))

	w.writeHeader(
		"rpi_exporter_scrapes_total",
		"Total number of scrapes.",
		metricTypeCounter,
	)
	w.writeSample(atomic.LoadUint64(&scrapeCount))

	w.writeHeader(
		"rpi_exporter_scrape_errors_total",
		"Total number of scrape errors.",
		metricTypeCounter,
	)
	w.writeSample(atomic.LoadUint64(&scrapeErrorCount))
}
# rpi_exporter

A lightweight Prometheus exporter for Raspberry Pi hardware metrics with optional Sense HAT support.

## Features

### VideoCore Metrics (always enabled)
- Component power states
- Clock rates (configured and measured)
- SoC temperatures (current and max)
- Component voltages (current, min, max)
- Turbo mode status
- Board model and revision
- Firmware revision

### Sense HAT Metrics (optional, `-sensehat` flag)
- Temperature (humidity sensor, pressure sensor, CPU-adjusted) in Fahrenheit
- Humidity percentage
- Atmospheric pressure (millibars)
- Orientation (pitch, roll, yaw)
- Accelerometer (x, y, z in Gs)
- Gyroscope (x, y, z in degrees/second)
- Magnetometer (x, y, z in microteslas)
- Compass heading
- Color sensor RGB + clear (Sense HAT v2 only)
- IMU calibration status

`rpi_exporter` is written in Go with zero external dependencies. It interfaces directly with the VideoCore mailbox and I2C devices for lightweight, fast metric collection.

## Installation

### Build from source

```shell
# 32-bit ARM (Raspberry Pi OS default)
make rpi_exporter

# 64-bit ARM (Pi 4/5 with 64-bit OS)
make rpi_exporter-arm64

# Install to /opt/node_exporter
make install
```

### Docker

```shell
# Build
make docker-build

# Run (VideoCore metrics only)
docker run -d \
  --name rpi-exporter \
  --device /dev/vcio \
  -p 9110:9110 \
  rpi_exporter:latest

# Run with Sense HAT enabled
docker run -d \
  --name rpi-exporter \
  --device /dev/vcio \
  --device /dev/i2c-1 \
  -p 9110:9110 \
  rpi_exporter:latest -addr=:9110 -sensehat
```

Or use docker-compose:

```shell
# VideoCore only
docker-compose up -d rpi-exporter

# With Sense HAT
docker-compose --profile sensehat up -d rpi-exporter-sensehat
```

## Usage

```
Usage of rpi_exporter:
  -addr string
        Listen on address (e.g., :9110)
  -debug
        Print debug messages
  -sensehat
        Enable Sense HAT metrics
```

### Examples

```shell
# Print metrics to stdout
./rpi_exporter

# Start HTTP server on port 9110
./rpi_exporter -addr=:9110

# With Sense HAT metrics
./rpi_exporter -addr=:9110 -sensehat

# Debug mode
./rpi_exporter -addr=:9110 -sensehat -debug
```

## Configure systemd

```ini
# /etc/systemd/system/rpi_exporter.service
[Unit]
Description=Raspberry Pi Exporter
After=network.target

[Service]
Type=simple
ExecStart=/opt/node_exporter/rpi_exporter -addr=:9110 -sensehat
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```shell
sudo systemctl daemon-reload
sudo systemctl enable rpi_exporter.service
sudo systemctl start rpi_exporter.service
```

## Configure Prometheus

```yaml
# /etc/prometheus/prometheus.yml
scrape_configs:
  - job_name: "rpi_exporter"
    scrape_interval: 5s
    static_configs:
      - targets: ["localhost:9110"]
```

```shell
sudo systemctl restart prometheus.service
```

## Metrics Reference

### VideoCore Metrics

| Metric | Labels | Description |
|--------|--------|-------------|
| `rpi_vc_revision` | | Firmware revision |
| `rpi_board_model` | | Board model number |
| `rpi_board_revision` | | Board revision number |
| `rpi_power_state` | `id` | Power state (0=off, 1=on, 2=missing) |
| `rpi_clock_rate_hz` | `id` | Configured clock rate |
| `rpi_clock_rate_measured_hz` | `id` | Measured clock rate |
| `rpi_turbo` | | Turbo mode (0/1) |
| `rpi_temperature_c` | `id` | SoC temperature (Celsius) |
| `rpi_temperature_f` | `id` | SoC temperature (Fahrenheit) |
| `rpi_max_temperature_c` | `id` | Max safe temperature (Celsius) |
| `rpi_max_temperature_f` | `id` | Max safe temperature (Fahrenheit) |
| `rpi_voltage` | `id` | Current voltage |
| `rpi_voltage_min` | `id` | Minimum supported voltage |
| `rpi_voltage_max` | `id` | Maximum supported voltage |

### Sense HAT Metrics

| Metric | Description |
|--------|-------------|
| `sensehat_temperature_humidity_fahrenheit` | Temperature from humidity sensor |
| `sensehat_temperature_pressure_fahrenheit` | Temperature from pressure sensor |
| `sensehat_temperature_average_fahrenheit` | Average of both sensors |
| `sensehat_temperature_adjusted_fahrenheit` | CPU heat-adjusted estimate |
| `sensehat_cpu_temperature_fahrenheit` | CPU temperature |
| `sensehat_humidity_percent` | Relative humidity |
| `sensehat_pressure_millibars` | Atmospheric pressure |
| `sensehat_orientation_pitch_degrees` | Pitch angle |
| `sensehat_orientation_roll_degrees` | Roll angle |
| `sensehat_orientation_yaw_degrees` | Yaw angle |
| `sensehat_accelerometer_{x,y,z}_g` | Acceleration in Gs |
| `sensehat_gyroscope_{x,y,z}_dps` | Angular velocity |
| `sensehat_magnetometer_{x,y,z}_microtesla` | Magnetic field |
| `sensehat_compass_north_degrees` | Compass heading |
| `sensehat_imu_calibrated` | IMU status (0/1) |
| `sensehat_color_sensor_available` | Color sensor present (0/1) |
| `sensehat_color_{red,green,blue,clear}` | Color channels (0-255) |

## Hardware Requirements

### VideoCore (always required)
- Raspberry Pi 1-5
- `/dev/vcio` device access

### Sense HAT (optional)
- Sense HAT v1 or v2
- I2C enabled (`sudo raspi-config nonint do_i2c 0`)
- `/dev/i2c-1` device access

I2C addresses used:
- `0x5C` - LPS25H (pressure/temperature)
- `0x5F` - HTS221 (humidity/temperature)  
- `0x6A` - LSM9DS1 accelerometer/gyroscope
- `0x1C` - LSM9DS1 magnetometer
- `0x29` - TCS34725 color sensor (v2 only)

## License

MIT License - see [LICENSE](LICENSE)

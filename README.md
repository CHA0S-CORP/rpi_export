# rpi_exporter

A lightweight Prometheus exporter for Raspberry Pi hardware metrics with optional Sense HAT support.

Written in pure Go with zero external dependencies. Interfaces directly with the VideoCore mailbox and I2C devices for fast, efficient metric collection.

## Quick Start

```shell
# Build for your Pi
make rpi_exporter-arm64  # 64-bit OS (Pi 4/5)
make rpi_exporter        # 32-bit OS

# Run as HTTP server
./rpi_exporter -addr=:9110

# Test it
curl http://localhost:9110/metrics
```

## Features

### VideoCore Metrics (always enabled)

Collected via the VideoCore mailbox interface (`/dev/vcio`):

- Component power states
- Clock rates (configured and measured)
- SoC temperatures (current and max)
- Component voltages (current, min, max)
- Turbo mode status
- Board model and revision
- Firmware revision

### Sense HAT Metrics (optional)

Enabled with `-sensehat` flag, requires I2C:

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

## Compatibility

| Raspberry Pi | VideoCore | Sense HAT |
|--------------|-----------|-----------|
| Pi 1         | Yes       | Yes       |
| Pi 2         | Yes       | Yes       |
| Pi 3         | Yes       | Yes       |
| Pi 4         | Yes       | Yes       |
| Pi 5         | Yes       | Yes       |
| Pi Zero      | Yes       | Yes       |
| Pi Zero 2    | Yes       | Yes       |

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

### Kubernetes

A complete DaemonSet configuration is available in `k8s-daemonset.yaml`, including:

- DaemonSet with node selector for arm64
- ServiceMonitor for Prometheus Operator
- Health and readiness probes
- Resource limits (64Mi memory, 100m CPU)

```shell
kubectl apply -f k8s-daemonset.yaml
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
# Print metrics to stdout (one-shot mode)
./rpi_exporter

# Start HTTP server on port 9110
./rpi_exporter -addr=:9110

# With Sense HAT metrics
./rpi_exporter -addr=:9110 -sensehat

# Debug mode
./rpi_exporter -addr=:9110 -sensehat -debug
```

## HTTP Endpoints

When running as a server (`-addr` flag):

| Endpoint   | Description                                      |
|------------|--------------------------------------------------|
| `/metrics` | Prometheus metrics                               |
| `/health`  | Health check (always returns 200)                |
| `/ready`   | Readiness probe (checks VideoCore accessibility) |

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

**Power state IDs:** `sd_card`, `uart0`, `uart1`, `usb_hcd`, `i2c0`, `i2c1`, `i2c2`, `spi`, `ccp2tx`

**Clock IDs:** `emmc`, `uart`, `arm`, `core`, `v3d`, `h264`, `isp`, `sdram`, `pixel`, `pwm`, `hevc`, `emmc2`, `m2mc`, `pixel_bvs`

**Voltage IDs:** `core`, `sdram_c`, `sdram_p`, `sdram_i`

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

The VideoCore mailbox is the communication interface between the ARM CPU and the VideoCore GPU on Raspberry Pi. It provides access to hardware information and configuration.

### Sense HAT (optional)

- Sense HAT v1 or v2
- I2C enabled (`sudo raspi-config nonint do_i2c 0`)
- `/dev/i2c-1` device access

I2C addresses used:

| Address | Chip      | Function                    |
|---------|-----------|-----------------------------|
| `0x5C`  | LPS25H    | Pressure/temperature        |
| `0x5F`  | HTS221    | Humidity/temperature        |
| `0x6A`  | LSM9DS1   | Accelerometer/gyroscope     |
| `0x1C`  | LSM9DS1   | Magnetometer                |
| `0x29`  | TCS34725  | Color sensor (v2 only)      |

## Troubleshooting

### "permission denied" accessing /dev/vcio

Run as root or add your user to the `video` group:

```shell
sudo usermod -aG video $USER
# Log out and back in
```

### "permission denied" accessing /dev/i2c-1

Add your user to the `i2c` group:

```shell
sudo usermod -aG i2c $USER
# Log out and back in
```

### I2C device not found

Enable I2C on your Pi:

```shell
sudo raspi-config nonint do_i2c 0
sudo reboot
```

Verify I2C is enabled:

```shell
ls -la /dev/i2c*
```

### Sense HAT not detected

Check I2C devices are visible:

```shell
sudo apt install i2c-tools
i2cdetect -y 1
```

You should see devices at addresses 0x1c, 0x29 (v2 only), 0x5c, 0x5f, 0x6a.

### Docker container can't access devices

Ensure you're passing the required devices:

```shell
docker run --device /dev/vcio --device /dev/i2c-1 ...
```

For Kubernetes, the container needs to run privileged to access hardware devices.

### High CPU temperature readings from Sense HAT

The Sense HAT sits directly on top of the Pi, so temperature readings are affected by CPU heat. Use `sensehat_temperature_adjusted_fahrenheit` for a compensated estimate, or mount the Sense HAT away from the Pi using a ribbon cable.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     rpi_exporter                        │
├─────────────────────────────────────────────────────────┤
│  cmd/rpi_exporter/main.go                               │
│  - CLI parsing                                          │
│  - HTTP server (optional)                               │
│  - Metric collection orchestration                      │
├─────────────────────────────────────────────────────────┤
│  pkg/export/prometheus/writer.go                        │
│  - Prometheus text format serialization                 │
├──────────────────────┬──────────────────────────────────┤
│  pkg/mbox/           │  pkg/sensehat/                   │
│  - VideoCore mailbox │  - I2C sensor interface          │
│  - /dev/vcio access  │  - /dev/i2c-1 access             │
│  - Hardware info     │  - HTS221, LPS25H, LSM9DS1       │
│  - Power/clocks      │  - TCS34725 (v2)                 │
│  - Temperature       │  - IMU fusion                    │
└──────────────────────┴──────────────────────────────────┘
         │                        │
         ▼                        ▼
    /dev/vcio               /dev/i2c-1
    (VideoCore)             (I2C bus)
```

## License

MIT License - see [LICENSE](LICENSE)

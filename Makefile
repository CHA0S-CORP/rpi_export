all: rpi_exporter

# Default: Linux on Raspberry Pi OS (32-bit ARMv7)
rpi_exporter:
	GOOS=linux \
	GOARCH=arm \
	GOARM=7 \
	go build -o rpi_exporter ./cmd/rpi_exporter

# 64-bit ARM (Raspberry Pi 4/5 with 64-bit OS)
rpi_exporter-arm64:
	GOOS=linux \
	GOARCH=arm64 \
	go build -o rpi_exporter ./cmd/rpi_exporter

# Native build (for testing on x86)
rpi_exporter-native:
	go build -o rpi_exporter ./cmd/rpi_exporter

install: rpi_exporter
	install \
		-m 755 \
		-o node_exporter \
		-g node_exporter \
		rpi_exporter \
		/opt/node_exporter/rpi_exporter

docker-build:
	GOOS=linux GOARCH=arm64 go build -o rpi_exporter ./cmd/rpi_exporter
	docker buildx build --platform linux/arm64 -t rpi_exporter:latest .

clean:
	rm -f rpi_exporter

.PHONY: all clean install docker-build rpi_exporter-arm64 rpi_exporter-native
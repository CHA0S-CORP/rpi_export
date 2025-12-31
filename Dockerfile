FROM arm64v8/debian:bookworm-slim

ARG OS=linux
ARG ARCH=arm64

EXPOSE 9110

WORKDIR /opt

ADD rpi_exporter /opt/rpi_exporter
RUN chmod +x /opt/rpi_exporter

# Default: VideoCore metrics only
# Add -sensehat flag to enable Sense HAT metrics
# Container needs device access: --device /dev/vcio --device /dev/i2c-1
ENTRYPOINT ["/opt/rpi_exporter"]
CMD ["-addr=:9110"]

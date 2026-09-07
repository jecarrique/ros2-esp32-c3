# Dockerfile for ESP32-C3 + micro-ROS development environment
# Based on the official Espressif ESP-IDF v6.0 image
FROM espressif/idf:v6.0

ENV DEBIAN_FRONTEND=noninteractive

# Install extra tooling used by the DevContainer / Codespaces workflow
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    udev \
    usbutils \
    python3-pip \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Eliminar compiladores de Xtensa no utilizados para liberar varios gigabytes
RUN rm -rf /opt/esp/tools/xtensa-esp-elf*

# Make sure the ESP-IDF environment is sourced automatically in every shell
RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> /etc/bash.bashrc

WORKDIR /workspaces/ros2-esp32-c3

FROM espressif/idf:v6.0

# Define la arquitectura que vas a utilizar
ENV IDF_TARGET=esp32c3

# Si ejecutas el instalador manualmente, especifica solo el target y limpia paquetes descargados
RUN $IDF_PATH/install.sh esp32c3 \
    && rm -rf $IDF_TOOLS_PATH/dist

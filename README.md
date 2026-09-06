# ros2-esp32-c3

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/TU_USUARIO/TU_REPO)

Proyecto de **ESP-IDF v6.0** para **ESP32-C3 (ESP32-C3-DevKitC-02)** que integra **micro-ROS**
con **ROS 2 Jazzy** a través de un transporte **UDP sobre Wi-Fi**. El firmware publica:

- `/imu/data` (`sensor_msgs/msg/Imu`): aceleración y velocidad angular leídas de una IMU
  Adafruit 3463 (FXOS8700 + FXAS21002) conectada por I2C.
- `/button/state` (`std_msgs/msg/Bool`): estado de un pulsador conectado a GND (`true` =
  presionado, `false` = en reposo/VCC).

El repositorio incluye una configuración de **DevContainer** lista para **GitHub Codespaces**,
con la imagen oficial de ESP-IDF v6.0, extensiones de VS Code y tareas (`build`, `flash`,
`monitor`, agente micro-ROS) preconfiguradas.

## Tabla de contenidos

- [Hardware](#hardware)
- [Estructura del proyecto](#estructura-del-proyecto)
- [Tutorial de uso](#tutorial-de-uso)
- [Tutorial de modificación](#tutorial-de-modificación)

## Hardware

| Periférico                          | Interfaz | Pines / Notas                                   |
| ------------------------------------ | -------- | ------------------------------------------------ |
| IMU Adafruit 3463 (FXOS8700+FXAS21002)| I2C (`I2C_NUM_0`) | SCL = GPIO6, SDA = GPIO7 |
| Botón / Switch                       | GPIO digital | GPIO5, pull-up interno. Reposo = HIGH (VCC), presionado = LOW (GND) |

## Estructura del proyecto

```
.
├── .devcontainer/         # DevContainer para VS Code / GitHub Codespaces
│   ├── Dockerfile
│   └── devcontainer.json
├── .vscode/
│   └── tasks.json         # Build / Flash / Monitor / Agente micro-ROS
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml  # Dependencia: micro_ros_espidf_component
│   ├── Kconfig.projbuild  # SSID, password, IP y puerto del agente
│   ├── fxos8700_fxas21002.h/.c  # Driver I2C de la IMU
│   └── main.c             # NVS, Wi-Fi, transporte micro-ROS, publicadores
├── CMakeLists.txt
├── partitions.csv
├── sdkconfig.defaults
└── README.md
```

## Tutorial de uso

### 1. Abrir el proyecto en VS Code DevContainer o GitHub Codespaces

**Opción A: GitHub Codespaces**

1. Haz clic en el botón *"Open in GitHub Codespaces"* al inicio de este README.
2. Espera a que Codespaces construya el contenedor (usa la imagen oficial `espressif/idf:v6.0`).
3. Una vez listo, tendrás una terminal con el entorno de ESP-IDF ya activado.

**Opción B: VS Code DevContainer local**

1. Instala la extensión [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) en VS Code.
2. Clona este repositorio y ábrelo en VS Code.
3. Ejecuta `Dev Containers: Reopen in Container` desde la paleta de comandos (`Ctrl+Shift+P`).

### 2. Configurar Wi-Fi y el agente micro-ROS

Puedes usar `idf.py menuconfig` → **"ROS 2 / micro-ROS Configuration"**, o editar directamente
`sdkconfig` (generado a partir de `sdkconfig.defaults`) o `sdkconfig.defaults`:

```bash
idf.py menuconfig
```

Opciones disponibles:

- `WIFI_SSID`: nombre de tu red Wi-Fi.
- `WIFI_PASSWORD`: contraseña de tu red Wi-Fi.
- `MICRO_ROS_AGENT_IP`: dirección IP del equipo donde corre el agente micro-ROS (por ejemplo, la
  IP de tu PC con ROS 2 Jazzy).
- `MICRO_ROS_AGENT_PORT`: puerto UDP en el que escucha el agente (por defecto `8888`).

### 3. Compilar y flashear

Desde la terminal (o usando las tareas de VS Code, ver más abajo):

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

Desde VS Code, abre la paleta de comandos → `Tasks: Run Task` y selecciona:

- **Build Project**: ejecuta `idf.py build`.
- **Flash ESP32-C3**: ejecuta `idf.py flash` (requiere configurar `idf.port` en la configuración
  de VS Code, o modificar la tarea con tu puerto serie, p. ej. `/dev/ttyUSB0` o `COM3`).
- **Monitor Serial**: ejecuta `idf.py monitor` para ver los logs por consola serie.

### 4. Iniciar el Agente micro-ROS para ROS 2 Jazzy (Docker)

En tu PC con ROS 2 Jazzy y Docker instalado:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
```

O bien usa la tarea de VS Code **"Start Micro-ROS Jazzy Agent (Docker)"**.

> Asegúrate de que el puerto coincida con `MICRO_ROS_AGENT_PORT` configurado en el firmware, y
> que la IP configurada en `MICRO_ROS_AGENT_IP` corresponda a la máquina donde corre este
> contenedor.

### 5. Verificar los tópicos publicados

Con el agente corriendo y la placa conectada a la misma red Wi-Fi:

```bash
ros2 topic list
ros2 topic echo /imu/data
ros2 topic echo /button/state
```

## Tutorial de modificación

### Cambiar los pines I2C / GPIO

- Los pines de la IMU (`SCL`, `SDA`) y del botón se definen como macros al inicio de
  `main/main.c`:

  ```c
  #define I2C_SDA_GPIO         7
  #define I2C_SCL_GPIO         6
  #define BUTTON_GPIO          GPIO_NUM_5
  ```

  Modifica estos valores según tu cableado y vuelve a compilar (`idf.py build`).

### Cambiar la frecuencia de publicación

- La frecuencia de publicación de `/imu/data` y `/button/state` se controla con la macro
  `PUBLISH_PERIOD_MS` en `main/main.c` (por defecto `100` ms = 10 Hz):

  ```c
  #define PUBLISH_PERIOD_MS    100  /* 10 Hz */
  ```

  Cambia este valor (en milisegundos) y recompila para ajustar la tasa de publicación.

### Agregar o cambiar un tipo de mensaje de ROS 2

1. Incluye el header del nuevo tipo de mensaje, por ejemplo:

   ```c
   #include <std_msgs/msg/int32.h>
   ```

2. Declara el publicador y el mensaje en `micro_ros_task()`:

   ```c
   rcl_publisher_t my_publisher;
   RCCHECK(rclc_publisher_init_default(
       &my_publisher, &node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
       "my_topic"));

   std_msgs__msg__Int32 my_msg;
   ```

3. Publica el mensaje dentro del bucle principal con `rcl_publish(&my_publisher, &my_msg, NULL);`.
4. Si el tipo de mensaje no está incluido por defecto, agrega el paquete ROS 2 correspondiente en
   `main/idf_component.yml` (bajo `dependencies`) o mediante `extra_packages`/`app-colcon.meta`
   según la documentación de
   [`micro_ros_espidf_component`](https://github.com/micro-ROS/micro_ros_espidf_component).

### Description

- This code is being developed in ROS2 Humble
- It is part of the projects developed by the research group Optimization and Control of Distributed Systems
- Working with Navio2, Raspberry pi 4 and Ardupilot

# ASV_Loyola_US_low_level

![](https://www.uloyola.es/templates/v6/images/isologo_loyola_principal.svg)

## Tabla de Contenidos

1. [Requisitos](#requisitos)
2. [Instalación de dependecias](#instalación-de-dependecias)
3. [Clona el proyecto](#clona-el-repositorio)
4. [Compilacion](#compilación)

## Requisitos

Antes de comenzar, asegúrate de tener los siguientes requisitos instalados en tu máquina:

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Python 3](https://www.python.org/downloads/)
- [Guribi Academic](https://support.gurobi.com/hc/en-us/articles/4534161999889-How-do-I-install-Gurobi-Optimizer/)

## Instalación de dependecias

A continuación, se describen algunos de los pasos para la instalación de las principales dependencias del este proyecto, y la configuración de alguno periféricos como los son la (IMU) y (modulo Xbee).

### Configuración de periféricos

Con estos comandos se definirá una regla para identificar los módulos XBee e IMU al conectarlos, lo cual facilitará su conexión con su equipo. En caso de que no piense utilizarlos (desarrollo exclusivo en ordenador o solo lectura de rosbags), podrá omitir este paso. Primero descargue los siguientes archivos `bind_device.sh`, `imu_usb.rules` y `xbee_usb.rules` de este [repositorio](https://github.com/manuelgantiva/asv_UL_Docker/tree/main/docker) y almacenelos en una carpeta llamada `rules`:

```bash
cd /rules
sudo chmod 777 bind_device.sh
sudo sh bind_device.sh
cd ..
```

### Mavros package

Dado que la lectura de los sensores y escritura de actuadores se realiza por medio de software [Ardupilot](https://ardupilot.org/rover/docs/boat-configuration.html), este proyecto requiere de la instalación del paquete `Mavros`, para su correcta compilación y funcionamiento, a continuación, se describen los pasos para su instalación:

```bash
sudo apt install ros-humble-mavros
```
Luego instale los conjuntos de datos de GeographicLib ejecutando el script `install_geographiclib_datasets.sh`:

```bash
ros2 run mavros install_geographiclib_datasets.sh

# Alternative:
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

### XBee Python library

Esta libreria se utiliza para facilitar la conexión con los módulos Xbee, aunque estos no se utilizen, se recomienda su instalacion para no afectar las dependencias:

```bash
pip install digi-xbee
```


## Clona el repositorio

Clona este repositorio en tu espacio de trabajo de ROS 2

```bash
mkdir ASV/src
cd ASV/src
git clone -b hito2 https://github.com/manuelgantiva/ASV_Loyola_US_low_level.git .
```

### Compilación

Una vez clonado, navega al espacio de trabajo y compila con `colcon`


```bash
cd ..
colcon build ----executor sequential
```
En caso que quieras compilar un solo paquete podras utilizar el comando

```bash
cd ..
colcon build --packages-select [nombre_paquete]
```


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
    
   [WLS License]: <https://www.gurobi.com/features/academic-wls-license/>
   

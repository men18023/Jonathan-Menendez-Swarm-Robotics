# Algoritmos de Robótica de Enjambre

Este repositorio contiene documentación relacionada con el proyecto de graduación:

**Validación de los algoritmos de robótica de enjambre Particle Swarm Optimization y Ant Colony Optimization con sistemas robóticos físicos en el ecosistema Robotat**

## Descripción

Este proyecto se centra en la validación de algoritmos de robótica de enjambre, específicamente Particle Swarm Optimization y Ant Colony Optimization, utilizando el sistema de captura de movimiento OptiTrack disponible para la mesa de pruebas del Robotat en la Universidad del Valle de Guatemala

<div align="center">
<img src="https://github.com/men18023/Jonathan-Menendez-Swarm-Robotics/assets/68084833/be87a6c0-01f1-47dc-935d-0feb88acacbd" alt="Mesa de pruebas Robotat" width="500">
</div>
  
## Índice

- [Introducción](#introducción)
- [Contenido](#contenido)
- [Instalación](#instalación)
- [Uso](#uso)
- [Resultados](#resultados)


## Introducción

La Universidad del Valle de Guatemala ha realizado avances en la robótica de enjambre, donde se ha dado un enfoque a los algoritmos como lo son Particle Swarm Optimization y Ant Colony Optimization para la generación de trayectorias en plataformas móviles. En un nuevo estudio, se realizaron pruebas físicas de estos algoritmos con robots [Pololu 3pi+](https://www.pololu.com/product/4975). Se realizó la optimización de parámetros en los algoritmos y los controladores para mejorar el rendimiento en físico. También se realizaron pruebas para evaluar la robustez y adaptabilidad de cada algoritmo.

## Contenido

- `ACO_3pi` es la carpeta principal y se divide en:
  - `ACO` es una subcarpeta que contiene los archivos para generación de una trayectoria desde un punto inicial a un punto final por medio del alogoritmo ACO y un archivo `README.md`.
  - `imágenes` es otra subcarpeta que contiene imágenes de resultados y pruebas realizadas con el algoritmo ACO.
  - `workspaces` es una subcarpeta que contiene los "workspace" con las variables utilizadas para cada prueba realizada, cada una relacionada con imágenes de la subcarpeta con el mismo nombre.
  - `Anexos` contiene archivos antiguos y de prueba utilizados durante el desarrollo de los archivos finales.
  - Archivos para la ejecución de trayectoria con los robots físicos.
  - El archivos `README.md` que explica el funcionamiento de los archivos para ejecución de las pruebas.
  
- `MPSO_3pi` es la carpeta principal y se divide en:
  - `Anexos` contiene archivos antiguos y de prueba utilizados durante el desarrollo de los archivos finales.
  - Archivos para la ejecución de trayectorias con los robots físicos utilizando el algoritmo de MPSO.
  - El archivos `README.md` que explica el funcionamiento de los archivos para ejecución de las pruebas.


- `Documentación` es la carpeta para almacenar documentación relacionada a este trabajo de graduación:
  - `Documentación Antecedentes` contiene todos los doumentos utilizados para la sección de Antecedentes para el documento final de tesis.
  - `Documentación Marco Teórico` contiene todos los doumentos utilizados para la sección de Marco Teórico para el documento final de tesis.
  
## Instalación
El desarrollo e implementación de los algoritmos a nivel físivo se trabajo utilizando el software de MATLAB, en su versión R2021a. 
ACO
- Descargar la carpeta *ACO_3pi* que contiene los archivos *ACO_3pi.m* y las funciones de controladores para los robots y comandos para comunicación con el sistema
OptiTrack
- Asegurarse que el servidor del Robotat y el sistema de OptiTrack estén en funcionamiento
- Elegir el robot Pololu 3pi+ que se desee utilizar.
- Confirmar que el microcontrolador ESP32 cuente con la configuración para comunicarse con el servidor.
- Colocar el marcador del sistema OptiTrack en el robot. Verificar que el número entre el robot, ESP32 y marcador coincidan.
- Abrir en MATLAB la carpeta descargada en el primer paso.
- Correr el archivo ACO.m, donde se debe elegir los nodos de inicio y final para encontrar la trayectoria por medio del algoritmo.
- Iniciar el agente en una posición cercana al nodo inicial seleccionado en el paso previo, idealmente.
- Correr el código del archivo ACO_pololu, donde debe configurar los parámetros que se deseen probar.

PSO
- Descargar la carpeta *MPSO_3pi* que contiene los archivos *MPSO_3pi.m* y las funciones de controladores para los robots y comandos para comunicación con el sistema
OptiTrack
- Asegurarse que el servidor del Robotat y el sistema de OptiTrack estén en funciona-
miento
- Elegir los robots Pololu 3pi+ que se deseen utilizar. Se debe elegir una secuencia continua entre los robots disponibles del 1 al 10
- Confirmar que los microcontroladores ESP32 cuente con la configuración para comunicarse con el servidor.
- Colocar los marcadores del sistema OptiTrack en cada uno de los robots. Verificar que el número entre el robot, ESP32 y marcador coincidan
- Encender y colocar los agentes en las posiciones iniciales deseadas
- Abrir en MATLAB la carpeta descargada en el primer paso
- Correr el código del archivo MPSO_pololu, donde debe ajustar algunas variables para indicar que robots fueron seleccionados y los parámetros del algoritmo que se deseen
probar


## Uso
El desarrollo e implementación de los algoritmos a nivel físivo se trabajo utilizando el software de MATLAB, en su versión R2021a. 





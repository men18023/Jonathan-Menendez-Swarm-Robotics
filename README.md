# Algoritmos de Robótica de Enjambre

Este repositorio contiene documentación relacionada con el proyecto de graduación:

**Validación de los algoritmos de robótica de enjambre Particle Swarm Optimization y Ant Colony Optimization con sistemas robóticos físicos en el ecosistema Robotat**

## Descripción

Este proyecto se centra en la validación de algoritmos de robótica de enjambre, específicamente Particle Swarm Optimization y Ant Colony Optimization, utilizando el sistema de captura de movimiento OptiTrack disponible para la mesa de pruebas del Robotat en la Universidad del Valle de Guatemala

<div align="center">
<img src="https://github.com/men18023/Jonathan-Menendez-Swarm-Robotics/assets/68084833/be87a6c0-01f1-47dc-935d-0feb88acacbd" alt="Mesa de pruebas Robotat" width="500">
</div>

<div align="center">
<img src="ACO_3pi/Imagenes/ACO.gif" alt="Funcionamiento final ACO" width="500">
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

ACO
El algoritmo de Ant Colony Optimization se divide en dos partes:
- Generación de trayectoria con parámetros de ACO. 
- Ejecución de trayectoria con controlador para robot Pololu 3pi+.

PSO


## Uso

Ambos algoritmos usan una serie de funciones de MATLAB que permiten la comunicación con el sistema de 

## Resultados





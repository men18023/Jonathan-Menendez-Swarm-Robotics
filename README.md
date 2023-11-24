# Algoritmos de Robótica de Enjambre

Este repositorio contiene documentación relacionada con el proyecto de graduación:

**Validación de los algoritmos de robótica de enjambre Particle Swarm Optimization y Ant Colony Optimization con sistemas robóticos físicos en el ecosistema Robotat**

## Descripción

Este proyecto se centra en la validación de algoritmos de robótica de enjambre, específicamente Particle Swarm Optimization y Ant Colony Optimization, utilizando el sistema de captura de movimiento OptiTrack disponible para la mesa de pruebas del Robotat en la Universidad del Valle de Guatemala

<div align="center">
<img src="https://github.com/men18023/Jonathan-Menendez-Swarm-Robotics/assets/68084833/be87a6c0-01f1-47dc-935d-0feb88acacbd" alt="Mesa de pruebas Robotat" width="500">
</div>

#Particle Swarm Optimization

El algoritmo original fue introducido en 1995 por James Kennedy y Russell Eberhart. Esta técnica utiliza un mecanismo simple que imita el comportamiento de enjambre de algunos grupos de animales, como lo son las parvadas y los cardúmenes, para guiar a las partículas a buscar soluciones óptimas globales.

En PSO, las partículas se colocan en un espacio de búsqueda de un problema o función, y cada entidad evalúa una función objetivo en su ubicación actual. Luego, cada partícula determina su posición en el espacio de búsqueda combinando algún aspecto de su historial de ubicaciones actuales y las mejores ubicaciones con las de uno o más miembros del enjambre, incluyendo algunas perturbaciones aleatorias. La siguiente iteración se produce después de que se hayan movido todas las partículas. En su última instancia, es probable que el enjambre se acerque al valor óptimo de la función ajustada.

#Ant Colony Optimization
Este algoritmo está basado en el comportamiento natural de una colonia de hormigas, donde se observa como las hormigas tienen la capacidad de encontrar un camino óptimo entre el hormiguero y una fuente de alimento. A partir de este comportamiento, Marco Dorigo desarrolló un algoritmo denominadoAnt System(SA), que luego de basarse en un
método metaheurístico, en donde se ven a las hormigas como base del algoritmo ACO, que permite la solución de problemas discretos de optimización.

<div align="center">
<img src="ACO_3pi/Imagenes/ACO.gif" alt="Mesa de pruebas Robotat" width="500">
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





**Particle Swarm Optimization**

El algoritmo original fue introducido en 1995 por James Kennedy y Russell Eberhart. Esta técnica utiliza un mecanismo simple que imita el comportamiento de enjambre de algunos grupos de animales, como lo son las parvadas y los cardúmenes, para guiar a las partículas a buscar soluciones óptimas globales.

En PSO, las partículas se colocan en un espacio de búsqueda de un problema o función, y cada entidad evalúa una función objetivo en su ubicación actual. Luego, cada partícula determina su posición en el espacio de búsqueda combinando algún aspecto de su historial de ubicaciones actuales y las mejores ubicaciones con las de uno o más miembros del enjambre, incluyendo algunas perturbaciones aleatorias. La siguiente iteración se produce después de que se hayan movido todas las partículas. En su última instancia, es probable que el enjambre se acerque al valor óptimo de la función ajustada.

<div align="center">
<img src="Imagenes/PSO.png" alt="Trayectoria Generada por ACO.m" width="500">
</div>

**Descripción de Funciones**

* *robotat_connect.m*

Esta función en MATLAB establece una conexión con un robot 3Pi, identificado por un número de agente único. Verifica la validez del ID del agente, determina la dirección IP en función del rango del ID y crea un cliente TCP para establecer la conexión.

* *robotat_get_pose.m*

Esta función en MATLAB obtiene datos de posición (pose) de robots 3Pi a través de una conexión TCP. Envía una solicitud al servidor con los IDs de los agentes, espera la respuesta y decodifica los datos JSON recibidos. Puede manejar representaciones de rotación en cuaterniones o ángulos de Euler, convirtiendo automáticamente si es necesario.

* *robotat_disconnect.m*


Esta función en MATLAB realiza la desconexión del cliente TCP con el servidor Robotat. Envía la señal 'EXIT' al servidor, imprime un mensaje de desconexión y limpia la variable de entrada en el espacio de trabajo de MATLAB.

* *robotat_3pi_connect.m*

Esta función en MATLAB establece una conexión con un robot 3Pi, identificado por un número de agente único. Verifica la validez del ID del agente, determina la dirección IP en función del rango del ID y crea un cliente TCP para establecer la conexión. Además, ahora incluye una verificación adicional para asegurar la validez y apertura de la conexión TCP.

* *robotat_3pi_set_wheels_velocities.m*

Esta función en MATLAB establece las velocidades de las ruedas izquierda y derecha de un robot 3Pi, asegurándose de que estén dentro de los límites permitidos. Utiliza un formato de codificación CBOR para enviar las velocidades al robot a través de la conexión TCP establecida. Adicionalmente, emite advertencias si las velocidades exceden los límites especificados.

* *robotat_3pi_force_stop.m*

Esta función en MATLAB detiene forzosamente las ruedas izquierda y derecha de un robot 3Pi, estableciendo las velocidades a cero. Utiliza el formato de codificación CBOR para enviar las velocidades al robot a través de la conexión TCP establecida.

* *robotat_3pi_disconnect.m*

Esta función en MATLAB realiza la desconexión del robot 3Pi, eliminando la variable asociada en el espacio de trabajo de MATLAB y mostrando un mensaje de desconexión.

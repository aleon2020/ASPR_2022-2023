# ASR-1-SeekAndCapture
![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/License-Apache%202.0-blue)
![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)
![Build status](https://github.com/Docencia-fmrico/seekandcapture-forocoches/actions/workflows/lint.yml/badge.svg?branch=main)

Práctica 1 – Arquitectura Software para Robots

En esta práctica el robot debe:

1. Buscar a las personas a su alrededor.
2. Cuando detecta nua, debe ir hacia ella, incluso si ésta se mueve.
3. Cuando esté cerca, debe hacer alguna señal (sonido, mensaje,..)
4. El robot busca a otra persona diferente

* El software debe tener un subsistema de percepción que detecte a la persona en la imagen, transforme esa detección a una detección 3D y genere un frame desde "odom" para la detección. Usa los nodos del paquete perception_asr para esto.
* Debe haber un subsistema de actuación que, a partir de los frames de las percepciones de personas, elija una y genere los comandos para seguirla. Debe pararse a ~1 metro y hacer la señal.

Puntuación:

* Sistema de percepción funcionando correctamente [+1]
* El robot se aproxima a una persona [+2]
* El robot usa PIDs para seguir a la persona [+1]
* El robot se para y avisa cuando ha "cogido" a una persona [+1]
* El robot usa Behavior Trees para organizar su comportamiento [+2]
* El robot busca y sigue a otra persona cuando ha alcanzado a una [+2]
* Todo se lanza con un solo launcher. Tiene un sistema de depuración efectivo [+1]

***

# SeekAndCapture-Forocoches V.1.0.0
<p align="center">
  <img src="https://user-images.githubusercontent.com/92941012/227970105-c6a33f31-2ce9-412d-895c-99a95766586b.png" width="600"/>
</p>

¡Bienvenidos al repositorio de SeekAndCapture-Forocoches! Somos un equipo altamente cualificado y dedicado que ha trabajado arduamente en este proyecto. Este es un modelo de sistema desarrollado en ROS-2 que permite a un robot seguir a varias personas utilizando la cámara Astra del Kobuki y algoritmos basados en transformaciones mediante la librería darknet-ros.

Este proyecto es el resultado de la colaboración entre dos equipos que enfocados en la parte perceptiva y dinámica de los comportamientos del robot, respectivamente. Hemos mejorado y perfeccionado ambas partes para que el robot pueda seguir a varias personas y satisfacer los requisitos de esta práctica.

<div align="center">
  <video src="https://user-images.githubusercontent.com/92941012/227981005-558a5dc9-346e-4f47-b4e3-389057dcda6b.mp4" width="200" poster="https://user-images.githubusercontent.com/92941012/227982978-544fed1a-1262-4535-9262-3fefc7ea7b3d.png" controls></video>
</div>

## Instalación
Este repositorio contiene todos los componentes necesarios para utilizar este modelo de sistema en tu propio robot. Hemos hecho todo lo posible para facilitar la instalación y uso del paquete. Sigue estos pasos para instalarlo:

1. Clona el repositorio principal:
```sh
git clone https://github.com/tu_usuario/tu_repositorio.git
cd seekandcapture-forocoches
```
2. Ejecuta el instalador automático
```sh
./setup.sh
```

Opcionalmente tambien puedes instalar manualmente el repositorio si ya posees una o varias dependencias dentro de tu equipo. Para ello puedes utilizar **vcs import** con el fichero __third_parties.repos__ y seleccionar manualmente las instalaciones necesarias. Recuerda, es importante descargar de forma recursiva la dependencia de darknet, y una vez descargados, deberas instalar dichos paquetes en la terminal que estes utilizando. Puedes encontrar mas información en:
https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

Una vez instalado el repositorio, podrás poner en marcha tu robot y permitir que este siga a todas las personas que detecte. Asegúrate de que el robot tenga la cámara Astra y que esté configurado correctamente para usarla.

## Modo de uso
Para lanzar el paquete SeekAndCapture-Forocoches en ROS2, es necesario utilizar el launcher seek_and_capture_full.launch.py. Este launcher se encargará de lanzar todos los paquetes necesarios para el funcionamiento del sistema, incluyendo los paquetes de percepción y comportamiento.

Además, es importante asegurarse de conectar los correspondientes paquetes de sincronización con el robot Kobuki para que el sistema pueda recibir y enviar datos correctamente.

```sh
ros2 launch seekandcapture-forocoches seek_and_capture_full.launch.py
```

## Comportamiento (Perception)
El nodo IsPerson es el nodo que se encarga de detectar personas en un entorno a través de una cámara y publicar sus posiciones en el marco de referencia del robot. Este nodo recibe mensajes de detección de personas en 3D de la camara preprocesado por el paquete perception_asr y utiliza la información de la transformación de la cámara con respecto al robot para calcular la posición de la persona en el marco de referencia del robot.

El nodo IsPerson hereda de la clase BT::ActionNodeBase y contiene una serie de métodos para el procesamiento de datos. El método de tick() es el que se ejecuta cuando el nodo es activado y su función es determinar si hay personas detectadas en la última detección recibida y publicar la posición de la persona detectada en el marco de referencia del robot. Si no se ha recibido una detección de persona reciente, devuelve un estado de falla.

El método de tf_already_exists_in_position() se encarga de comprobar si una persona detectada en una posición determinada ya ha sido publicada anteriormente. Esto se hace verificando si la distancia entre la nueva posición de la persona y la posición de todas las personas ya detectadas es inferior a un cierto umbral. Si la distancia es menor que el umbral, se considera que la persona ya ha sido publicada y la detección actual se omite. Este método se utiliza para evitar la publicación de la misma persona varias veces.

Para sobrellevar el problema de multiples objetivos en nuestro mapa, nos hemos aprovechado de una generación en tiempo de ejecución de nombres de frames, que se iran generando a medida que el robot vaya detectando nuevos destinos. Estos frames tendran una determinada sensibilidad de generación es decir, deberan tener una minima distancia para poder detectarse como distintos frames, lo que omite detecciones erroneas por parte de darknet
```c++
  // Publication of detected target
  
  // --- Calculation ---
  tf2::Transform odom2person = odom2robot * robot2person;
  geometry_msgs::msg::TransformStamped odom2person_msg;
  odom2person_msg.transform = tf2::toMsg(odom2person);

  // --- Generating new frame (based in number of detections) ---
  odom2person_msg.header = detection.header;
  odom2person_msg.header.frame_id = "odom";
  odom2person_msg.child_frame_id = "person_detected" + std::to_string(person_detected);

  // --- Publishing frame ---
  config().blackboard->set(
    "person_frame",
    "person_detected" + std::to_string(person_detected));
  person_detected++;
  tf_broadcaster_->sendTransform(odom2person_msg);
  debug_msg_.data = DebugNode::PERSON_DETECTED;
  debug_pub_->publish(debug_msg_);
  return BT::NodeStatus::SUCCESS;
```

## Comportamiento (Behavior Tree)
El Behavior Tree implementado para dirigir el comportamiento del robot consta de cuatro nodos principales: IsPerson, FindPerson, Navigation y Turn.

El raíz del Behavior Tree da al nodo de control del tipo KeepRunningUntilFailure, que asegura que el árbol siga ejecutándose mientras no haya un fallo. Dentro de este tenemos un nodo de control Sequence, que ejecuta las tareas en secuencia, y un nodo de control ReactiveFallback, que se ejecuta hasta que se detecta a una persona.

Si no hay ninguna persona detectada, el nodo FindPerson se ejecuta para rotar el robot y así encontrar a alguna persona. Una vez que se detecta a una persona, el nodo IsPerson guarda su frame para que este sea usado más adelante.

Fuera ya del ReactiveFallback, el nodo Navigate se ejecuta para guiar al robot hacia la persona utilizando el frame de la persona detectada guardada anteriormente. Cuando se completa el movimento, se da por encontrada a la persona y se procede a girar para así prepararse para el nuevo ciclo. Esta última parte es necesitada por un bug en la detección de Darknet.

En resumen, este Behavior Tree implementado para el robot cumple con el objetivo de encontrar a una persona, acercarse a ella y hacer una señal, y luego buscar a otra persona para repetir el proceso.

<p align="center">
  <img src="https://user-images.githubusercontent.com/92941012/228001946-e27ad7a8-4deb-4300-b8d7-8f8afb30368e.gif" width="600"/>
</p>

## Autores
* Javier Izquierdo
* Alberto León
* Luis Moreno
* Sebastian Mayorquín

## Contribuciones
Las contribuciones son bienvenidas. Si deseas contribuir a este proyecto, por favor, crea un pull request. Asegúrate de seguir las directrices de contribución antes de hacerlo.
__By Forocoches__

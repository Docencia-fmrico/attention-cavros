[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-c66648af7eb3fe8bc4f294546bfd86ef473780cde1dea487d3c4ff354943c9ae.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7776059&assignment_repo_type=AssignmentRepo)
# Atención
## Introducción
En esta práctica trataremos de dotar a nuestro robot de capacidad de atención. El objetivo será que el robot sea capaz de detectar los objetos que tenga cercanos y mire a los objetos durante unos segundos. El robot podrá estar en movimiento mientras hace uso de dicha atención.

Para ello, hemos decidido diseñar un sistema con 3 nodos interconectados mediante knowledge graphs:
- PoseInMap, para la posición del robot con respecto al mapa.
- Detector, encargado de la obtención de los objetos de interés.
- HeadController, que gestiona el movimiento del cuello del robot.

## PoseInMap
(Rellenar ...)

## Detector
Este proceso se encarga de resolver el problema de detección de objetos, seleccionando solo aquellos buscados. Para ello, se emplean lifecycle nodes, con el fin de poder controlar mejor el sistema.

De este modo:
- Configuramos el nodo leyendo dos parámetros de un fichero de configuración .yaml (objetos de interés y radio de detección).

```
detector_node:
  ros__parameters:
    target_objects: ["PotatoChipChair", "Chair", "AdjTable"]
    detection_distance: 5.0
```

- Luego seleccionamos la información mediante el uso de 2 filtros, el primero para comparar solo los objetos buscados, y el segundo para obtener solo aquellos que estén en el círculo de detección.
- Por último, se almacena la información en 2 vectores, uno de ID's y otro con los datos espaciales relevantes.

A continuación, se muestra una representación del funcionamiento del proceso:

<p align="center">
<img src="https://github.com/Docencia-fmrico/attention-cavros/blob/readme/media/detection.png?raw=true" width=50% height=50% />
</p>

## HeadController
(Rellenar ...)

## Knowledge Graphs
(Rellenar ...)

## Contribuidores
* Cristian Sánchez Rodríguez
* Blanca Soria Rubio
* Victor de la Torre Rosa
* Rubén Montilla Fernández

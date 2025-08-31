# Programación Lineal Entera (PLE)

## Problema 1:

Una empresa de logística opera con un Service Center (SVC) principal y una red de $n$ nodos de entrega (o delivery cells) para gestionar la última milla.

Hoy se debe planificar la entrega de $M$ paquetes.
Para cada paquete, la empresa debe tomar una decisión estratégica:
¿se asigna a la ruta de un vehículo que sale directamente del SC, o se envía primero a un nodo para que este gestione la entrega final?

El objetivo es minimizar el costo total de la entrega de todos los paquetes, respetando las capacidades operativas de cada nodo.

* Conjuntos:

    * $I:$ Conjunto de todos los nodos de entrega disponibles.
    * $J:$ Conjunto de todos los paquetes que deben ser entregados.

* Parámetros (Datos de entrada):

    * $c_i:$ Costo fijo por entregar un paquete desde el nodo $i$.
    * $S:$ Costo promedio por entregar un paquere desde el Service Center.
    * $k_i:$ Capacidad máxima (en número de paquetes) del nodo $i$.
    * $a_{ij}:$ Parámetro de cobertura. Es $1$ si el nodo $i$ puede entregar el paquete $j$ (su destino está en su zona), y $0$ en caso contrario.

## Entrada:

El programa deberá leer un archivo de texto simple (ej. problema.in) que puede contener una o más instancias de prueba.

* El archivo contendrá una secuencia de bloques, donde cada bloque define una instancia.
* El final de todas las instancias se indicará con una línea que contiene 0 0.

### Estructura de un Bloque de Instancia:

1. Línea 1: Dos enteros N y M, separados por un espacio, donde N es el número de nodos y M es el número de paquetes.
2. Línea 2: Un número decimal S, el costo de entrega por paquete desde el Service Center.
3. Siguientes N líneas: Describen los nodos (del 0 al N-1). Cada línea contiene la capacidad (entero) y el costo (decimal) del nodo, separados por un espacio.
4. Siguientes N líneas: Describen la cobertura de cada nodo. Cada línea comienza con un entero k (la cantidad de paquetes que el nodo puede cubrir), seguido de k enteros que son los IDs de dichos paquetes (del 0 al M-1).


### Notas Importantes:

* Se utiliza indexación basada en cero. El primer nodo es el Nodo 0, el primer paquete es el Paquete 0.
* Un paquete solo puede ser asignado a un nodo si dicho nodo lo tiene en su lista de cobertura.

## Salida:

Tu programa debe generar un archivo de texto (ej. problema.out) con los resultados de cada instancia, en el mismo orden en que fueron leídas.

### Estructura del Archivo de Salida:

* Para cada instancia del archivo de entrada, deberás imprimir un bloque de salida.
* Cada bloque comenzará con el encabezado Caso X, donde X es el número de la instancia (comenzando en 1).
* Deja una línea en blanco entre la salida de cada caso.

### Estructura de un Bloque de Salida:

1. Línea 1: El costo total mínimo de la asignación (un número decimal).
2. Siguientes M líneas: La asignación para cada paquete (del 0 al M-1). Cada línea contendrá el id_paquete y el id_ubicacion asignada.
    * El id_ubicacion será el índice del nodo (de 0 a N-1).
    * El id_ubicacion será -1 para indicar que fue asignado al Service Center.

## Solución:

Cada paquete $j \in J$ puede ser entregado por:
* El SC a un costo promedio $S$
* El nodo $i$ a un costo fijo $C_i$

Pero podemos interpretar al SC como un nodo con capacidad infinita (basta con capacidad $M$) y costo $S$ para facilitar algunas restricciones.

Notamos $I^+$ al conjunto de nodos extendido con el SC representado como nodo:

$c_n = S  \\
k_n = M \\
a_{nj} = 1 \forall j \in J
$


Notamos $b_{ij} = 1$ si el nodo $i$ entrega el paquete $j$ y, $b_{ij} = 0$ en otro caso.

### Minimizamos:

$Z= \sum_{i \in I^+}{
    \sum_{j \in J}{
        b_{ij}*c_i
    }
}$

### Restricciones:

* Cada paquete se entrega por exactamente 1 nodo:

    $1 = \sum_{i \in I^+}(b_{ij}) \\ \forall j \in J$

* Cada nodo no entrega más paquetes que su capacidad lo permite:

    $k_i \geq \sum_{j \in J}(b_{ij}) \\ \forall i \in I^+$
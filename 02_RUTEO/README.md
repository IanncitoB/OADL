# Asignación de paquetes y ruteo eficiente

## Problema 2

La empresa "LogiSpeed" busca una solución global e integrada para su operación de última milla.

Optimizar la asignación (Problema 1) y el ruteo de forma separada no es suficiente.

Una decisión que parece óptima para la asignación podría generar rutas muy ineficientes, y viceversa.

El nuevo desafío es utilizar toda la información disponible para encontrar simultáneamente la **mejor asignación** y las **rutas más eficientes**, minimizando el costo total.

El objetivo es desarrollar un programa que resuelva este problema integrado, el cual consta de dos partes interconectadas:

### PARTE A - Ruteo de una Asignación Base

Generar un conjunto de rutas de entrega óptimas para una asignación de paquetes dada, minimizando los costos de la flota y respetando las restricciones.

### PARTE B - Optimización Global (Challenge)

Implementar una estrategia para modificar la asignación inicial si al hacerlo se reduce el costo global total (Costo de Asignación Final + Costo de Ruteo Final).

## Entrada:

### Asignación (P1):
- Costos de asignación por paquete ($S$, $c_i$).
- Capacidad de los nodos ($k_i$).
- Matriz de cobertura ($a_{ij}$).

### Parámetros de Ruteo (P2):
- Costo fijo por vehículo.
- Costo variable por paquete entregado.
- Capacidad de vehículos (en paquetes).

El archivo problema2.in permite describir una lista de instancias, donde cada una tendrá la información de Bloque 1 + Bloque 2. El final del input se marca con una línea 0 0.

### Bloque 1: Datos de Asignación (P1):
- Línea 1: N M (Nodos, Paquetes)
- Línea 2: S (Costo Asignación SC)
- N líneas sig.: capacidad nodo costo nodo
- N líneas sig.: k id_paq1 id_paq2 ... (Cobertura)

### Bloque 2: Datos de Ruteo (P2):
- Línea sig.: capacidad vehículo
- Línea sig.: costo fijo vehículo costo variable paquete
- Línea sig.: -1 x y (Coords. SC)
- N líneas sig.: id_nodo x y
- M líneas sig.: id_paquete x y

### Fin del archivo:
- Última Línea: 0 0

## Salida:

El archivo problema2.out debe contener un bloque de salida para cada instancia, comenzando con Caso X.

### Estructura de un Bloque de Salida
- Línea 1: Costo global total mínimo.
- Línea 2: K (Número de orígenes con rutas).
- K bloques sig.: Describen las rutas por origen.

### Formato de Rutas por Origen
- Línea 1: id origen num vehículos
- Líneas sig.: Una línea por vehículo con los id paquete de su ruta.

## Solución:

La solución se basa fuertemente en el ejercicio anterior ```01_PLE```.

### Asignación:

Para asignar los paquetes a cada nodo proveedor utilizamos la librería ```PuLP``` teniendo en cuenta:

- ```Nuevo``` Hiperparámetro:
    - $\alpha$: Busca penalizar los envíos asignados que estén lejos del nodo. 
        - Cuando $\alpha = 0$ la minimización solamente tiene en cuenta los costos de asignación y la cantidad de vehículos utilizados
        - Entre más grande sea $\alpha$, se le da mayor importancia al ruteo.  
- Variables:
    - $b_{ij} = 1$ sii el nodo i entrega el paquete j. Binaria
    - ```Nueva``` $v_i$ cantidad de vehículos usados por nodo i. Entera

- Costos:
    - Asignación: 

        $\sum_{i \in I^+}{
            \sum_{j \in J}{
                b_{ij}*c_i
            }
        }$
    - ```Nuevo``` Costo fijo vehículos:

        $\sum_{i \in I^+}{
            cvf * v_i
        }$
    - ```Nuevo``` Distancias (ruteo):

        $\sum_{i \in I^+}{
            \sum_{j \in J}{
                cvp * dist(i,j) * b_{ij} * 2 * \alpha
            }
        }$

        Este costo está asociado al peor caso: cada nodo realiza ida y vuelta a cada paquete asignado, porque sino habría que calcular la mejor ruta para cada exploración de asignaciones (computacionalmente irrealizable)

- Restricciones:
    - Cada paquete se entrega por exactamente 1 nodo:

        $1 = \sum_{i \in I^+}(b_{ij}) \\ \forall j \in J$

    - Cada nodo no entrega más paquetes que su capacidad lo permite:

        $k_i \geq \sum_{j \in J}(b_{ij}) \\ \forall i \in I^+$

    - Solo se entregan paquetes dentro del área cubierta por el nodo:

        $b_{ij} \leq a_{ij} \\ \forall i \in I^+ \\ \forall j \in J$
    
    - ```Nueva``` cada nodo no entrega más paquetes que los que quepan en sus vehículos contratados:

        $\sum_{j \in J}(b_{ij}) \leq kv * v_i \\ \forall i \in I^+$


### Ruteo:

Para una asignación dada (nodo proveedor -> cliente/paquete), debemos obtener las rutas que realizará cada vehículo.

Utilizando la librería ```pyvrp```, resolvemos el ruteo de cada nodo agregando:

- Vehículos:
    - Disponibles: infinitos (```len(paquetes) a entregar```)
    - Capacidad: ```kv``` dada en input
    - Costo Fijo: ```cfv``` dado en input
- Depósitos:
    - Único depósito (el nodo) con sus coordenadas
- Clientes:
    - Coordenadas X, Y de cada paquete a entregar por ese nodo
    - Delivery: ```1```. Cada cliente espera exactamente 1 paquete.
- Aristas:
    - Grafo completo de todas las ```locations``` (depósitos + clientes)
    - Cada arista tiene peso ```distancia * cvp```

Opcionalmente se pueden graficar las rutas de cada nodo y de todos los nodos en simultáneo.

### Juntando:

1) Exploramos diferentes valores de $\alpha$ para obtener varias asignaciones candidatas. Notar que valores $\alpha$ cercanos entre sí, suelen repetir asignaciones. 

2) Para cada asignación candidata, se calcula el ruteo óptimo.

3) Nos quedamos con aquella asignación + ruteo que sea más conveniente

### Gráficos e información:

En ```solucion.py``` se encuentran hiperparámetros configurables que ayudan la visualización del problema.

**Altamente** recomendable utilizar:
     
```PLOTTING_NODE_ROUTES = True```

```PLOTTING_NODE_ROUTES = True```

```ALPHA_EVALUATION_VERBOSE = True```
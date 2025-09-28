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


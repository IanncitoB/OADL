from pulp import LpProblem, LpVariable, LpMinimize, lpSum, LpStatus, value, PULP_CBC_CMD
import sys
import numpy as np

def solve(n, m, nodos, cobertura):
    # Creación del modelo de optimización
    modelo = LpProblem("Entrega_De_Paquetes", LpMinimize)

    # Variables de decisión: b_ij = 1 si el nodo i entrega el paquete j
    b = LpVariable.dicts("b", ((i, j) for i in range(n+1) for j in range(m)), cat='Binary')

    # Definición de la función objetivo: minimizar el costo total
    modelo += lpSum(nodos[i]['costo'] * lpSum(b[i, j] for j in range(m)) for i in range(n+1)), "Costo_Total"

    # Restricciones:
    # 1) Cada paquete se entrega por exactamente 1 nodo:
    for j in range(m):
        modelo += lpSum(b[i, j] for i in range(n+1)) == 1, f"Entrega_Unica_Paquete_{j}"

    # 2) Cada nodo no entrega más paquetes que su capacidad lo permite:
    for i in range(n+1): # Podemos omitir el nodo n (Service Center) ya que tiene capacidad suficiente
        modelo += lpSum(b[i, j] for j in range(m)) <= nodos[i]['capacidad'], f"Capacidad_Nodo_{i}"

    # 3) Solo se entregan paquetes dentro del área cubierta por el nodo:
    for i in range(n+1): # Podemos omitir el nodo n (Service Center) ya que cubre todos los paquetes
        for j in range(m):
            modelo += b[i, j] <= cobertura[i, j], f"Cobertura_Nodo_{i}_Paquete_{j}"

    # Resolución del modelo
    modelo.solve(PULP_CBC_CMD(msg=0))

    # El valor del objetivo es el costo mínimo
    costo_minimo = value(modelo.objective)
    print(f"{costo_minimo:.2f}") # Costo mínimo con 2 decimales
    # Asignación de paquetes a nodos
    for j in range(m):
        for i in range(n+1):
            if value(b[i, j]) == 1:
                print(f"{j} {i}")
                break


if __name__ == '__main__':
    input_file = './01_PLE/ejemplo_problema.in'
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    output_file = input_file.replace('.in', '.out')
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    sys.stdout = open(output_file, 'w')
    caso = 1
    with open(input_file, 'r') as f:
        n, m = map(int, f.readline().strip().split()) # número de nodos, número de paquetes
        while (n != 0):
            # Capacidades y costos de los nodos
            S = float(f.readline().strip().split()[0]) # costo de entrega Service Center
            nodos = []
            for i in range(n):
                k, c = f.readline().strip().split() # capacidad y costo de cada nodo
                k = int(k)
                c = float(c)
                nodos.append({'capacidad': k, 'costo': c})
            # Agregamos nodo que actúa como Service Center
            nodos.append({'capacidad': m, 'costo': S})

            # Cobertura de cada nodo
            cobertura = np.zeros((n+1, m), dtype=bool)
            for i in range(n):
                datos = list(map(int, f.readline().strip().split()))
                t = datos[0]
                for j in range(1,t+1):
                    cobertura[i, datos[j]] = True
            # El Service Center cubre todos los paquetes
            cobertura[n, :] = True

            print(f'Caso {caso}')
            solve(n, m, nodos, cobertura)

            caso += 1
            n, m = map(int, f.readline().strip().split()) # número de nodos, número de paquetes


from pulp import LpProblem, LpVariable, LpMinimize, lpSum, LpStatus, value, PULP_CBC_CMD
import sys
import numpy as np

if __name__ == '__main__':
    input_file = './01_PLE/ejemplo_problema.in'
    # if len(sys.argv) > 1:
    #     input_file = sys.argv[1]
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
                for j in range(t):
                    cobertura[i, datos[j]] = True
            # El Service Center cubre todos los paquetes
            cobertura[n, :] = True

        
        print(f'NODOS:\n{nodos}')


# 02_RUTEO/solucion.py
import sys
import numpy as np
from package_assignment import PackageAssignmentSolver as PASolver
from pyvrp import Model

import matplotlib.pyplot as plt
from pyvrp.stop import MaxRuntime
from pyvrp.plotting import plot_coordinates, plot_solution
import math

def solve_routing(n, m, nodos_coordenadas, paquetes_coordenadas, assignments, kv, cfv, cvp):
    # Agrupamos los paquetes por nodo asignado
    paquetes_por_nodo = {}
    for j, i in enumerate(assignments):
        if i not in paquetes_por_nodo:
            paquetes_por_nodo[i] = []
        paquetes_por_nodo[i].append(j)

    models = []
    for nodo, paquetes in paquetes_por_nodo.items():
        model = Model()
        model.add_vehicle_type(num_available=len(paquetes_por_nodo[nodo]), capacity=kv, fixed_cost=cfv)
        depot = model.add_depot(x=nodos_coordenadas[nodo][0], y=nodos_coordenadas[nodo][1])
        clients = [model.add_client(x=paquetes_coordenadas[j][0], y=paquetes_coordenadas[j][1], delivery=1) for j in paquetes]
        for frm in model.locations:
            for to in model.locations:
                distance = math.hypot(frm.x - to.x, frm.y - to.y)  # Distancia Euclidiana
                model.add_edge(frm, to, distance=distance * cvp)
                
        res = model.solve(stop=MaxRuntime(10), display=True)  # Limitar a 10 segundos
        if res is None or res.best is None:
            print("[DBG]\tNo se encontró solución de ruteo para el nodo {nodo}.")
            continue
        
        models.append((model, res))
        _, axs = plt.subplots(1, 2, figsize=(16, 8))
        plot_coordinates(model.data(), ax=axs[0])
        plot_solution(res.best, model.data(), ax=axs[1])
        plt.show()
    
    # plot all routes together
    if len(models) > 0:
        _, ax = plt.subplots(1, 1, figsize=(8, 8))
        for model, res in models:
            plot_solution(res.best, model.data(), ax=ax)
        plt.show()
    else:
        print("[DBG]\tNo se encontró solución de ruteo para ningún nodo.")
    
    # Calculamos costo total:


if __name__ == '__main__':
    input_file = './02_RUTEO/ejemplo_problema.in'
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    output_file = input_file.replace('.in', '.out')
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    # sys.stdout = open(output_file, 'w')
    caso = 1
    with open(input_file, 'r') as f:
        n, m = map(int, f.readline().strip().split()) # número de nodos, número de paquetes
        while (n != 0):
            # BLOQUE 1: datos de Asignación:

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

            # BLOQUE 2: datos de ruteo

            kv = int(f.readline().strip()) # capacidad vehículos en paquetes
            # Costo fijo vehículo y costo variable paquete
            cfv, cvp = map(float, f.readline().strip().split())

            # Coordenadas
            nodos_coordenadas = [(0, 0)] * (n + 1) 

            for _ in range(n + 1):
                i, x, y = f.readline().strip().split()
                i = int(i)
                x = float(x)
                y = float(y)
                nodos_coordenadas[i] = (x, y)
                # Notar que el nodo correspondiente al SC está en la posición [-1], es decir, la última posición
            
            paquetes_coordenadas = [(0, 0)] * m
            for _ in range(m):
                j, x, y = f.readline().strip().split()
                j = int(j)
                x = float(x)
                y = float(y)
                paquetes_coordenadas[j] = (x, y)
            
            print(f'Caso {caso}')

            solver = PASolver(
                n, m, nodos, cobertura, solver_msg=False
            )
            result = solver.solve()
            if result['status'] != 'Optimal':
                print("[DBG]\tNo se encontró solución óptima.")
            else:
                print(f"Costo asignación: {result['costo_minimo']:.2f}") # Costo de asignación
                assignments = result['assignments']
                print(f'Assignments: {assignments}')

            # Con la asignación, resolver el problema de ruteo

            solve_routing(
                n, m, nodos_coordenadas, paquetes_coordenadas, assignments,
                kv, cfv, cvp
            )


            caso += 1
            n, m = map(int, f.readline().strip().split()) # número de nodos, número de paquetes
# 02_RUTEO/solucion.py
import sys
import math
import numpy as np
from assignments_vrp import PackageSolver
from pyvrp import Model
from pyvrp.stop import MaxRuntime
from pyvrp.plotting import plot_coordinates, plot_solution
import matplotlib.pyplot as plt

def solve_routing(n, m, nodos_coordenadas, paquetes_coordenadas, assignments, kv, cfv, cvp):
    # Agrupamos los paquetes por nodo asignado
    paquetes_por_nodo = {}
    for j, i in enumerate(assignments):
        if i is None:
            continue
        # i == -1 => Service Center (último nodo)
        node_idx = i if i >= 0 else n
        paquetes_por_nodo.setdefault(node_idx, []).append(j)

    models = []
    total_routing_cost = 0.0
    for nodo_idx, paquetes in paquetes_por_nodo.items():
        # Si no hay paquetes, saltar
        if len(paquetes) == 0:
            continue

        # número mínimo de vehículos a considerar
        num_veh = max(1, (len(paquetes) + kv - 1) // kv)

        model = Model()
        model.add_vehicle_type(num_available=num_veh, capacity=kv, fixed_cost=cfv)
        model.add_depot(x=nodos_coordenadas[nodo_idx][0], y=nodos_coordenadas[nodo_idx][1])

        # crear clientes (delivery=1)
        for j in paquetes:
            model.add_client(x=paquetes_coordenadas[j][0], y=paquetes_coordenadas[j][1], delivery=1)

        # agregar aristas (distancias)
        for frm in model.locations:
            for to in model.locations:
                d = math.hypot(frm.x - to.x, frm.y - to.y)
                model.add_edge(frm, to, distance=d * cvp)

        res = model.solve(stop=MaxRuntime(10), display=False)  # 10 segundos por nodo
        if res is None or getattr(res, "best", None) is None:
            print(f"[DBG]\tNo se encontró solución de ruteo para el nodo {nodo_idx}.")
            continue

        models.append((model, res))

        best = res.best

        # --- Extracción robusta de rutas (para distintas versiones de pyvrp) ---
        routes = None
        try:
            if hasattr(best, "routes"):
                attr = best.routes
                routes = attr() if callable(attr) else attr
            elif hasattr(best, "solution"):
                attr = best.solution
                routes = attr() if callable(attr) else attr
            else:
                # buscar en los names del objeto algo parecido a 'route' o 'routes'
                candidate = None
                for name in dir(best):
                    if "route" in name.lower():
                        candidate = getattr(best, name)
                        break
                if candidate is not None:
                    routes = candidate() if callable(candidate) else candidate
        except Exception:
            routes = None

        # --- contar vehículos usados y tratar rutas ---
        vehicles_used = num_veh  # fallback
        route_cost = None
        try:
            # intentar obtener costo directamente (si existe)
            route_cost = getattr(best, "cost", None)
            if route_cost is None:
                route_cost = getattr(best, "objective", None)
            if route_cost is None:
                route_cost = getattr(best, "value", None)
            # si encontramos rutas iterable, intentar contar vehículos usados
            if routes is not None:
                count = 0
                for r in routes:
                    # r puede ser objeto con .nodes, .visits, o ser lista
                    nodes = None
                    if hasattr(r, "nodes"):
                        nodes = getattr(r, "nodes")
                    elif hasattr(r, "visits"):
                        nodes = getattr(r, "visits")
                    elif isinstance(r, (list, tuple)):
                        nodes = r
                    # determinar longitud
                    l = len(nodes) if nodes is not None else None
                    # considerar ruta "usada" si tiene más de 2 nodos (depósito + al menos 1 cliente)
                    if l is not None and l > 2:
                        count += 1
                    else:
                        # si no pudimos evaluar nodes, intentar con len(r) como fallback
                        try:
                            if len(r) > 2:
                                count += 1
                        except Exception:
                            pass
                vehicles_used = max(1, count)
        except Exception:
            vehicles_used = num_veh

        if route_cost is not None:
            try:
                total_routing_cost += float(route_cost)
            except Exception:
                # si no pudimos convertir, fallback a aproximación
                route_cost = None

        if route_cost is None:
            # aproximación: costo fijo por vehículos usados + cvp * 2*sum distancia depot->client
            total_routing_cost += vehicles_used * cfv
            depot_x, depot_y = nodos_coordenadas[nodo_idx]
            approx_dist = sum(math.hypot(paquetes_coordenadas[j][0]-depot_x, paquetes_coordenadas[j][1]-depot_y) for j in paquetes)
            total_routing_cost += cvp * 2 * approx_dist

        # try:
        #     _, axs = plt.subplots(1, 2, figsize=(16, 8))
        #     plot_coordinates(model.data(), ax=axs[0])
        #     plot_solution(res.best, model.data(), ax=axs[1])
        #     plt.show()
        # except Exception as e:
        #     print(f"[DBG]\tError al plotear nodo {nodo_idx}: {e}")

    # plot all routes together
    if len(models) > 0:
        try:
            _, ax = plt.subplots(1, 1, figsize=(8, 8))
            for model, res in models:
                try:
                    plot_solution(res.best, model.data(), ax=ax)
                except Exception:
                    pass
            plt.show()
        except Exception as e:
            print(f"[DBG]\tError al plotear todas las rutas: {e}")
    else:
        print("[DBG]\tNo se encontró solución de ruteo para ningún nodo.")

    print(f"[DBG] Costo de ruteo (aprox): {total_routing_cost:.2f}")
    return total_routing_cost


if __name__ == '__main__':
    input_file = './02_RUTEO/ejemplo_problema copy.in'
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    output_file = input_file.replace('.in', '.out')
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    # sys.stdout = open(output_file, 'w')
    caso = 1
    with open(input_file, 'r') as f:
        n, m = map(int, f.readline().strip().split()) # número de nodos, número de paquetes
        while (n != 0 or m != 0):
            # BLOQUE 1: Asignación
            # Capacidades y costos de los nodos
            S = float(f.readline().strip().split()[0]) # costo de entrega Service Center
            nodos = []
            for i in range(n):
                k, c = f.readline().strip().split() # capacidad y costo de cada nodo
                k = int(k)
                c = float(c)
                nodos.append({'capacidad': k, 'costo': c, 'posicion': (0.0, 0.0)})
            # Agregamos nodo que actúa como Service Center
            nodos.append({'capacidad': m, 'costo': S, 'posicion': (0.0, 0.0)})

            # Cobertura de cada nodo
            cobertura = np.zeros((n+1, m), dtype=bool)
            for i in range(n):
                datos = list(map(int, f.readline().strip().split()))
                t = datos[0]
                for j in range(1,t+1):
                    cobertura[i, datos[j]] = True
            # El Service Center cubre todos los paquetes
            cobertura[n, :] = True

            # BLOQUE 2: Ruteo
            kv = int(f.readline().strip())
            cfv, cvp = map(float, f.readline().strip().split())

            nodos_coordenadas = [(0,0)] * (n + 1)
            for _ in range(n + 1):
                i, x, y = f.readline().strip().split()
                idx = int(i)
                x, y = float(x), float(y)
                nodos_coordenadas[idx] = (x, y)
                # actualizar posición en nodos
                nodos[idx]['posicion'] = (x, y)

            paquetes_coordenadas = [(0,0)] * m
            for _ in range(m):
                j, x, y = f.readline().strip().split()
                idx = int(j)
                paquetes_coordenadas[idx] = (float(x), float(y))

            print(f'Caso {caso}')

            solver = PackageSolver(
                nodos=nodos,
                paquetes_coordenadas=paquetes_coordenadas,
                cobertura=cobertura,
                kv=kv, cfv=cfv, cvp=cvp,
                solver_msg=True,
                alpha=3
            )
            result = solver.solve()
            if result['status'] not in ('Optimal', 'Feasible'):
                print("[DBG]\tNo se encontró solución óptima.")
            else:
                print(f"Costo (asignación + aproximación VRP): {result['costo_minimo']:.2f}")
                assignments = result['assignments']
                print(f'Assignments: {assignments}')
                print(f'Vehículos estimados por nodo: {result["vehicle_count"]}')

                # Con la asignación, resolver el problema de ruteo real por nodo
                routing_cost = solve_routing(n, m, nodos_coordenadas, paquetes_coordenadas, assignments, kv, cfv, cvp)
                print(f"Costo total aproximado (asignación + ruteo real): {result['costo_minimo'] + routing_cost:.2f}")

            caso += 1
            n, m = map(int, f.readline().strip().split())

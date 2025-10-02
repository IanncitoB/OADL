# 02_RUTEO/solucion.py
import sys
import math
import numpy as np
from assignments_vrp import PackageSolver
from pyvrp import Model
from pyvrp.stop import MaxRuntime
from pyvrp.plotting import plot_coordinates, plot_solution
import matplotlib.pyplot as plt

# Hiperparámetros
PACKAGE_SOLVER_VERBOSE      = False     # Mensajes de librería PuLP
ROUTING_SOLVER_VERBOSE      = False     # Mensajes de librería pyVRP
ROUTING_SOLVER_MAX_RUNTIME  = 1         # 1 segundo por nodo
PLOTTING_NODE_ROUTES        = False     # Rutas de cada nodo
PLOTTING_ALL_ROUTES         = False     # Rutas de todos los nodos
ALPHA_EVALUATION_VERBOSE    = False     # Mensajes búsqueda del alpha óptimo
alpha_values = np.linspace(start=0, stop=2, num=20) # 20 valores en [0, 2] uniformemente espaciados

# PLOTTING
def annotate_locations(ax, locations, fontsize=8, zorder=10):
    """
    Añade el nombre de cada location en su posición al eje `ax`.
    `locations` puede ser una lista de objetos con atributos .x, .y, .name
    o una lista de tuplas (x, y) / (x, y, name).
    Evita dibujar la misma etiqueta más de una vez.
    """
    seen = set()
    for loc in locations:
        # obtener x, y, name de forma flexible
        x = getattr(loc, "x", None)
        y = getattr(loc, "y", None)
        name = getattr(loc, "name", None)

        if (x, y) in seen or x is None or y is None:
            continue
        seen.add((x, y))

        # dibujar el texto; lo colocamos ligeramente sobre el punto
        ax.text(x, y, name, fontsize=fontsize, ha='center', va='bottom', zorder=zorder,
                bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.6))


def solve_routing(n, m, nodos_coordenadas, paquetes_coordenadas, assignments, kv, cfv, cvp):
    assert n + 1 == len(nodos_coordenadas)
    assert m == len(paquetes_coordenadas)
    # Agrupamos los paquetes por nodo asignado
    paquetes_por_nodo = {}
    for j, i in enumerate(assignments):
        # i == -1 => Service Center (último nodo)
        nodo_idx = i if i >= 0 else n
        paquetes_por_nodo.setdefault(nodo_idx, []).append(j)

    models = []
    total_routing_cost = 0.0
    vehicles_used = 0
    all_routes = {}
    all_locations = {}
    for nodo_idx, paquetes in paquetes_por_nodo.items():
        # Si no hay paquetes, saltar
        if len(paquetes) == 0:
            continue

        # número máximo de vehículos a considerar
        num_veh = len(paquetes)

        model = Model()
        model.add_vehicle_type(num_available=num_veh, capacity=kv, fixed_cost=cfv)
        model.add_depot(x=nodos_coordenadas[nodo_idx][0], y=nodos_coordenadas[nodo_idx][1], name=f'{nodo_idx if nodo_idx < n else "-1"}')

        # crear clientes (delivery=1)
        for j in paquetes:
            client = model.add_client(x=paquetes_coordenadas[j][0], y=paquetes_coordenadas[j][1], delivery=1, name=f'{j}')

        # agregar aristas (distancias)
        for frm in model.locations:
            for to in model.locations:
                d = math.hypot(frm.x - to.x, frm.y - to.y)
                model.add_edge(frm, to, distance=d * cvp)

        res = model.solve(stop=MaxRuntime(ROUTING_SOLVER_MAX_RUNTIME), display=ROUTING_SOLVER_VERBOSE)
        if res is None or getattr(res, "best", None) is None:
            print(f"[DBG]\tNo se encontró solución de ruteo para el nodo {nodo_idx}.")
            continue

        models.append((model, res))

        best = res.best    
        routes = best.routes()
        
        route_cost = getattr(best, "distance_cost", None)
        if route_cost is None:
            route_cost = getattr(best, "distance", None)
        
        route_cost = route_cost() if callable(route_cost) else route_cost
        
        vehicles_used += sum([route.num_trips() for route in routes])
        if not nodo_idx in all_routes.keys():
            all_routes[nodo_idx] = []
        all_routes[nodo_idx] += routes

        if not nodo_idx in all_locations.keys():
            all_locations[nodo_idx] = []
        all_locations[nodo_idx] += model.locations

        total_routing_cost += float(route_cost)
        # PLOT rutas por nodo
        if PLOTTING_NODE_ROUTES:
            try:
                _, axs = plt.subplots(1, 2, figsize=(16, 8))
                plot_coordinates(model.data(), ax=axs[0])
                plot_solution(res.best, model.data(), ax=axs[1])
                try:
                    annotate_locations(axs[0], model.locations)
                    annotate_locations(axs[1], model.locations)
                except Exception as e:
                    print(f"[DBG]\tWarning anotando ubicaciones nodo {nodo_idx}: {e}")
                plt.suptitle(f'NODO: {nodo_idx}')
                plt.show()
            except Exception as e:
                print(f"[DBG]\tError al plotear nodo {nodo_idx}: {e}")

    # plot all routes together
    if PLOTTING_ALL_ROUTES:
        if len(models) > 0:
            try:
                _, ax = plt.subplots(1, 1, figsize=(8, 8))
                for model, res in models:
                    try:
                        plot_solution(res.best, model.data(), ax=ax)
                    except Exception:
                        pass
                try:
                    # concatenar todas las locations y evitar duplicados
                    combined = []
                    seen_xy = set()
                    for model, _ in models:
                        for loc in getattr(model, "locations", []):
                            x = getattr(loc, "x", None)
                            y = getattr(loc, "y", None)
                            if (x, y) in seen_xy or x is None or y is None:
                                continue
                            seen_xy.add((x, y))
                            combined.append(loc)
                    annotate_locations(ax, combined)
                except Exception as e:
                    print(f"[DBG]\tWarning anotando ubicaciones global: {e}")
                plt.title('Todas las rutas')
                plt.show()
            except Exception as e:
                print(f"[DBG]\tError al plotear todas las rutas: {e}")
        else:
            print("[DBG]\tNo se encontró solución de ruteo para ningún nodo.")

    total_routing_cost += vehicles_used * cvp

    return total_routing_cost, all_routes, all_locations


if __name__ == '__main__':
    input_file = './02_RUTEO/ejemplo_problema.in'
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    output_file = input_file.replace('.in', '.out')
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    sys.stdout = open(output_file, 'w')
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
            kv = int(f.readline().strip()) # capacidad del vehículo
            cfv, cvp = map(float, f.readline().strip().split()) # costo fijo vehículo, costo variable paquete

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

            unique_assignments = {}
            best = None
            for alpha in alpha_values:
                alpha = float(alpha)
                solver = PackageSolver(
                    nodos=nodos,
                    paquetes_coordenadas=paquetes_coordenadas,
                    cobertura=cobertura,
                    kv=kv, cfv=cfv, cvp=cvp,
                    solver_msg=PACKAGE_SOLVER_VERBOSE,
                    alpha=alpha
                )

                result = solver.solve()
                status = result['status']
                if result['status'] not in ('Optimal', 'Feasible'):
                    print(f"[DBG]\talpha={alpha:.6g}: no se obtuvo solución ({status}).")
                    continue
            
                assignments = result['assignments']
                key = tuple(assignments)
                if key not in unique_assignments:
                    costo_asignacion = result['costo_asignacion']
                    costo_ruteo, all_routes, locations = solve_routing(n, m, nodos_coordenadas, paquetes_coordenadas, assignments, kv, cfv, cvp)
                    costo_total = costo_asignacion + costo_ruteo
                    unique_assignments[key] = {
                        'alphas': [alpha],
                        'costo_asignacion': costo_asignacion,
                        'costo_ruteo': costo_ruteo,
                        'costo_total': costo_total,
                        'routes': all_routes,
                        'locations': locations
                    }
                    if ALPHA_EVALUATION_VERBOSE:
                        print(f"[NEW] alpha={alpha:.6g} -> (costo_asign={costo_asignacion:.2f}, costo_ruteo={costo_ruteo:.2f})\t\ttotal: {costo_total}")
                    if best is None or unique_assignments[best]['costo_total'] > costo_total:
                        best = key
                else:
                    unique_assignments[key]['alphas'].append(alpha)
                    if ALPHA_EVALUATION_VERBOSE:
                        print(f"[HIT] alpha={alpha:.6g} -> ({len(unique_assignments[key]['alphas'])} alphas)\t\t\t\t\ttotal: {unique_assignments[key]['costo_total']}")

            print(unique_assignments[best]['costo_total'])
            best_all_routes = unique_assignments[best]['routes']
            len(best_all_routes)

            for node, routes in best_all_routes.items():
                locations = unique_assignments[best]['locations']
                node_name = locations[node][0].name
                routes_named = [[locations[node][visit].name for visit in route.visits()] for route in routes]
                print(node_name, len(routes))
                for route in routes_named:
                    print(*route)
            caso += 1
            n, m = map(int, f.readline().strip().split())

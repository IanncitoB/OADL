# 02_RUTEO/assignments+vrp.py
from typing import List, NamedTuple, Tuple, Dict, Optional
import numpy as np
from pulp import (
    LpProblem, LpVariable, LpMinimize, lpSum, value, PULP_CBC_CMD, LpStatus
)


class Nodo(NamedTuple):
    capacidad: int
    costo: float
    posicion: Tuple[int,int]

class PackageSolver:
    """
    Clase para resolver el problema de asignación de paquetes a clientes, considerando sus rutas.

    Constructor arguments:
        nodos: lista de dicts con keys 'capacidad' (int), 'costo' (float) y 'posicion' (int,int).
                Debe incluir el Service Center como último elemento (índice n).
                Longitud esperada: n+1 (último es Service Center).
        paquetes_coordenadas: lista de posiciones (int,int) del destino de los paquetes.
                Longitud esperada: m
        cobertura: matriz booleana (n+1, m) indicando si nodo i cubre paquete j.
                   Acepta numpy array o lista de listas; se convertirá a np.bool_.
    Solver options:
        solver_msg: bool, si True muestra mensajes del solver CBC.
    """

    def __init__(
        self,
        nodos: List[Nodo],
        paquetes_coordenadas: List[Tuple[int,int]],
        cobertura,
        solver_msg: bool = False
    ):
        self.n = n = len(nodos) - 1
        self.m = m = len(paquetes_coordenadas)
        self.nodos = nodos

        # cobertura -> numpy boolean array shape (n+1, m)
        cov = np.array(cobertura, dtype=bool)
        if cov.shape != (n + 1, m):
            raise ValueError(f"La cobertura debe tener forma ({n+1}, {m}), pero tiene {cov.shape}.")
        self.cobertura = cov
        self.solver_msg = solver_msg

        # Variables internas llenadas en solve()
        self._model = None
        self._b_vars = None
        self.status = None
    
    def __modelo_init(self):
        """
        Inicializa el modelo de asignación de paquetes.
        Considera:
            - Restricciones de capacidad
            - Restricciones de cobertura
        NO considera:
            - Distancias entre nodos
            - Costos de vehículos
            - Ruteo
        """
        modelo = LpProblem("Entrega_De_Paquetes", LpMinimize)

        # Variables binarias b[i,j]
        b = LpVariable.dicts("b", ((i, j) for i in range(self.n + 1) for j in range(self.m)),
                             cat='Binary')

        # Función objetivo: minimizar costo total
        modelo += lpSum(self.nodos[i]['costo'] * lpSum(b[i, j] for j in range(self.m))
                        for i in range(self.n + 1)), "Costo_Total"

        # Restricción 1: cada paquete entregado exactamente 1 vez
        for j in range(self.m):
            modelo += lpSum(b[i, j] for i in range(self.n + 1)) == 1, f"Entrega_Unica_Paquete_{j}"

        # Restricción 2: capacidades de los nodos
        for i in range(self.n + 1):
            modelo += lpSum(b[i, j] for j in range(self.m)) <= self.nodos[i]['capacidad'], f"Capacidad_Nodo_{i}"

        # Restricción 3: cobertura
        for i in range(self.n + 1):
            for j in range(self.m):
                if not self.cobertura[i, j]:
                    # si no cubre, entonces b[i,j] <= 0  => b[i,j] == 0 (por binaria)
                    modelo += b[i, j] <= 0, f"Cobertura_Nodo_{i}_Paquete_{j}"
                # si cubre, no hay restricción adicional
        return modelo, b

    def solve(self) -> Dict:
        """
        Resuelve el modelo y devuelve un diccionario con:
            - 'status': estado del solver (string)
            - 'costo_minimo': float (o None si no tiene solución)
            - 'assignments': lista de longitud n+1 que contiene listas de id_paquetes a ser entregados por cada nodo
            - 'routes': lista de lista de todas las rutas que hace cada nodo:
                        ruta = List[Tuple[int,int]]
        Lanza excepción si el modelo es infeasible o no optimal.
        """
        modelo, b = self.__modelo_init()

        # Resolver primero minimizando el costo de asignación (sin importar ruteo)
        solver = PULP_CBC_CMD(msg=self.solver_msg)
        modelo.solve(solver)

        self._model = modelo
        self._b_vars = b
        self.status = LpStatus[modelo.status]

        # Si no es optimal/feasible, no resolvemos ruteo
        if self.status not in ("Optimal", "Feasible"):
            return {"status": self.status, "costo_minimo": None, "assignments": None, "routes": None}

        costo_asignacion = float(value(modelo.objective))
        
        # Para cada nodo k, devuelve la lista de paquetes que tiene asignado
        assignments = [
                        [paq for (paq,value) in enumerate(b[k]) if value] 
                    for k in range(len(b))
                    ]

        costo_total = costo_asignacion + 0 # Completar con costo de VRP
        routes = None 
        return {"status": self.status, "costo_minimo": costo_total, "assignments": assignments, 'routes': routes}


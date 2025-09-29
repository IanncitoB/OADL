# 02_RUTEO/assignments_vrp.py
from typing import List, Tuple, Dict, Optional
import math
import numpy as np
from pulp import (
    LpProblem, LpVariable, LpMinimize, lpSum, value, PULP_CBC_CMD, LpStatus, LpInteger
)

class PackageSolver:
    """
    Solver de asignación con estimación de costos de ruteo.

    nodos: lista de dicts {'capacidad': int, 'costo': float, 'posicion': (x,y)}
           Debe incluir el Service Center como último elemento (índice n).
    paquetes_coordenadas: lista de tuplas (x,y) de longitud m
    cobertura: array booleano shape (n+1, m)
    kv: capacidad de cada vehículo (paquetes por vehículo)
    cfv: costo fijo por vehículo
    cvp: costo variable por distancia por paquete
    alpha: hiperparámetro en (0, 1]. Penalización por elección de nodos lejanos al cliente
    """
    def __init__(
        self,
        nodos: List[Dict],
        paquetes_coordenadas: List[Tuple[float,float]],
        cobertura,
        kv: int = 1,
        cfv: float = 0.0,
        cvp: float = 0.0,
        solver_msg: bool = False,
        alpha: float = 1.0
    ):
        self.n = len(nodos) - 1  # número de nodos locales (service center = índice n)
        self.m = len(paquetes_coordenadas)
        self.nodos = nodos
        self.kv = kv
        assert kv > 0
        self.alpha = alpha
        self.cfv = cfv
        self.cvp = cvp

        cov = np.array(cobertura, dtype=bool)
        if cov.shape != (self.n + 1, self.m):
            raise ValueError(f"La cobertura debe tener forma ({self.n+1}, {self.m}), pero tiene {cov.shape}.")
        self.cobertura = cov
        self.paquetes = paquetes_coordenadas
        self.solver_msg = solver_msg

        self._model = None
        self._b_vars = None
        self._v_vars = None
        self.status = None

    def _distance(self, nodo_idx: int, paquete_idx: int) -> float:
        nx, ny = self.nodos[nodo_idx]['posicion']
        px, py = self.paquetes[paquete_idx]
        return math.hypot(nx - px, ny - py)

    def solve(self) -> Dict:
        modelo = LpProblem("Asignacion_con_VRP_approx", LpMinimize)

        # Variables de decisión: b_ij = 1 si el nodo i entrega el paquete j
        b = LpVariable.dicts("b", ((i, j) for i in range(self.n + 1) for j in range(self.m)), cat='Binary')

        # v[i] entero >=0: cantidad de vehículos usados por nodo i
        v = LpVariable.dicts("v", (i for i in range(self.n + 1)), lowBound=0, cat=LpInteger)

        # Objetivo:
        #   costo asignacion nodo (por paquete) + costo fijo por vehiculo + costo variable por distancia
        costo_asignacion = lpSum(self.nodos[i]['costo'] * lpSum(b[i, j] for j in range(self.m))
                                 for i in range(self.n + 1))
        costo_fijo_vehiculos = lpSum(self.cfv * v[i] for i in range(self.n + 1))
        # En el peor caso, el ruteo resulta de un viaje por paquete (ida y vuelta)
        # Esta desición entiendo que penaliza enviar paquetes demasiado lejos, salvo que el costo de asignación sea muy bajo y compense
        # La elección del alpha es crucial para que relaje esta restricción y pueda explorar asignaciones lejanas
        costo_distancias = lpSum(self.cvp * self._distance(i, j) * b[i, j] * 2 * self.alpha
                                 for i in range(self.n + 1) for j in range(self.m))

        # Definición de la función objetivo: minimizar el costo total
        modelo += (costo_asignacion + costo_fijo_vehiculos + costo_distancias), "Costo_Total_AproxVRP"

        # Restricción: cada paquete exactamente una vez
        for j in range(self.m):
            modelo += lpSum(b[i, j] for i in range(self.n + 1)) == 1, f"EntregaUnica_{j}"

        # Restricción: capacidades físicas de nodos
        for i in range(self.n + 1):
            modelo += lpSum(b[i, j] for j in range(self.m)) <= self.nodos[i]['capacidad'], f"CapacidadNodo_{i}"

        # Relacion v[i] con paquetes: sum_j b[i,j] <= kv * v[i]
        for i in range(self.n + 1):
            modelo += lpSum(b[i, j] for j in range(self.m)) <= self.kv * v[i], f"VehiculosSuf_{i}"

        # Cobertura
        for i in range(self.n + 1):
            for j in range(self.m):
                if not self.cobertura[i, j]:
                    modelo += b[i, j] <= 0, f"Cobertura_{i}_{j}"

        # Resolver
        solver = PULP_CBC_CMD(msg=self.solver_msg)
        modelo.solve(solver)

        self._model = modelo
        self._b_vars = b
        self._v_vars = v
        self.status = LpStatus[modelo.status]

        if self.status not in ("Optimal", "Feasible"):
            return {"status": self.status, "costo_minimo": None, "assignments": None, "vehicle_count": None}

        costo_val = float(value(modelo.objective))

        assignments: List[int] = [-2] * self.m
        for j in range(self.m):
            assigned = None
            for i in range(self.n + 1):
                val = value(b[(i, j)])
                if val is not None and round(val) == 1:
                    assigned = i
                    break
            if assigned is None:
                assignments[j] = None
            else:
                assignments[j] = assigned if assigned < self.n else -1  # -1 para Service Center

        vehicle_count = [int(round(value(v[i]))) if value(v[i]) is not None else 0 for i in range(self.n + 1)]

        return {"status": self.status, "costo_minimo": costo_val, "assignments": assignments, "vehicle_count": vehicle_count}
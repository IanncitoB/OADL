# 02_RUTEO/package_assignment.py
from typing import List, Tuple, Dict, Optional
import numpy as np
from pulp import (
    LpProblem, LpVariable, LpMinimize, lpSum, value, PULP_CBC_CMD, LpStatus
)


class PackageAssignmentSolver:
    """
    Clase para resolver el problema de asignación de paquetes a nodos proveedores.

    Constructor arguments:
        n: número de nodos "locales" (excluye el Service Center).
        m: número de paquetes.
        nodos: lista de dicts con keys 'capacidad' (int) y 'costo' (float).
               Debe incluir el Service Center como último elemento (índice n)
               si lo provees; si no está, la clase asumirá que lo agregas manualmente.
               Longitud esperada: n+1 (último es Service Center).
        cobertura: matriz booleana (n+1, m) indicando si nodo i cubre paquete j.
                   Acepta numpy array o lista de listas; se convertirá a np.bool_.
    Solver options:
        solver_msg: bool, si True muestra mensajes del solver CBC.
    """

    def __init__(
        self,
        n: int,
        m: int,
        nodos: List[Dict[str, float]],
        cobertura,
        solver_msg: bool = False
    ):
        self.n = n
        self.m = m
        # normalizar nodos: debe tener n+1 elementos (incluye service center)
        if len(nodos) != n + 1:
            raise ValueError(f"Se esperaban {n+1} nodos (incluyendo Service Center), "
                             f"pero se recibieron {len(nodos)}.")
        self.nodos = nodos

        # cobertura -> numpy boolean array shape (n+1, m)
        cov = np.array(cobertura, dtype=bool)
        if cov.shape != (n + 1, m):
            raise ValueError(f"La cobertura debe tener forma ({n+1}, {m}), "
                             f"pero tiene {cov.shape}.")
        self.cobertura = cov
        self.solver_msg = solver_msg

        # Variables internas llenadas en solve()
        self._model = None
        self._b_vars = None
        self.status = None

    def solve(self) -> Dict:
        """
        Resuelve el modelo y devuelve un diccionario con:
            - 'status': estado del solver (string)
            - 'costo_minimo': float (o None si no tiene solución)
            - 'assignments': lista de longitud m con el índice del nodo asignado,
                             o -1 si asignado al Service Center.
        Lanza excepción si el modelo es infeasible o no optimal.
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

        # Resolver
        solver = PULP_CBC_CMD(msg=1 if self.solver_msg else 0)
        modelo.solve(solver)

        self._model = modelo
        self._b_vars = b
        self.status = LpStatus[modelo.status]

        # Si no es optimal/feasible, devolver status y None
        if self.status not in ("Optimal", "Feasible"):
            return {"status": self.status, "costo_minimo": None, "assignments": None}

        costo_minimo = float(value(modelo.objective))

        # Recuperar asignaciones: para cada paquete j el i tal que b[i,j] == 1
        assignments: List[int] = [-2] * self.m  # marcador temporal
        for j in range(self.m):
            assigned = None
            for i in range(self.n + 1):
                val = value(b[i, j])
                # value puede retornar 0.0/1.0 o None; comparar con 1 con tolerancia
                if val is not None and round(val) == 1:
                    assigned = i
                    break
            if assigned is None:
                # Debería no pasar si modelo es factible
                assignments[j] = None
            else:
                # si assigned == self.n (Service Center) devolvemos -1 para mantener compatibilidad
                assignments[j] = assigned if assigned < self.n else -1

        return {"status": self.status, "costo_minimo": costo_minimo, "assignments": assignments}

    def solve_and_print(self, case_name: Optional[str] = None) -> None:
        """
        Ejecuta solve() e imprime el resultado en el formato similar al script original.
        """
        res = self.solve()
        if case_name:
            print(case_name)
        if res["costo_minimo"] is None:
            print("No se encontró solución óptima (status: {})".format(res["status"]))
            return
        print(f"{res['costo_minimo']:.2f}")
        for j, i in enumerate(res["assignments"]):
            # si i == -1 => Service Center
            print(f"{j} {i}")


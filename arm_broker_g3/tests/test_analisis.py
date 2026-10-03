""" Pruebas de las herramientas de análisis del ítem 3 (metricas.py y simular_politicas.py) """

"""No necesitan ROS 2: se generan CSV sintéticos con el mismo formato que exportar_csv.py

    cd src/arm_broker_g3 && python3 -m unittest discover -s test -v
"""
import csv
import os
import sys
import tempfile
import unittest

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
sys.path.insert(0, os.path.join(RAIZ, 'src', 'arm_broker_g3'))
sys.path.insert(0, os.path.join(RAIZ, 'analisis'))
sys.path.insert(0, os.path.join(RAIZ, 'herramientas'))

import metricas as M                                  # noqa: E402
import simular_politicas as S                         # noqa: E402

CABECERA = ['t_ns', 'executing_client', 'executing_goal_id', 'executing_elapsed_s',
            'queue_length', 'queued_goal_ids', 'queued_clients', 'queued_priorities',
            'queued_wait_s', 'total_accepted', 'total_rejected', 'total_completed']


def fila_cola(t, ejecutando=('', ''), cola=(), aceptados=0, rechazados=0, completados=0):
    """cola = [(goal_id, cliente, prioridad, espera_s)]"""
    return [int(t * 1e9), ejecutando[0], ejecutando[1], '0.0', len(cola),
            '|'.join(c[0] for c in cola), '|'.join(c[1] for c in cola),
            '|'.join(str(c[2]) for c in cola), '|'.join(f'{c[3]:.3f}' for c in cola),
            aceptados, rechazados, completados]


def escribir(ruta, cabecera, filas):
    with open(ruta, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cabecera)
        w.writerows(filas)


class TestMetricasSobreCSV(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.q = os.path.join(self.dir.name, 'queue_state.csv')
        self.j = os.path.join(self.dir.name, 'joint_states.csv')

    def tearDown(self):
        self.dir.cleanup()

    def cola_limpia(self):
        """Dos goals ejecutados de a uno: g1 (A) de 1 a 2 s y g2 (B) de 2 a 3 s, con la cola
        muestreada a 5 Hz como la publica el broker"""
        filas = [fila_cola(0.0, cola=[('g1', 'A', 1, 0.0), ('g2', 'B', 2, 0.0)], aceptados=2)]
        for k in range(5, 10):
            filas.append(fila_cola(k / 5, ('A', 'g1'), [('g2', 'B', 2, k / 5)], aceptados=2))
        for k in range(10, 15):
            filas.append(fila_cola(k / 5, ('B', 'g2'), aceptados=2, completados=1))
        filas.append(fila_cola(3.0, aceptados=2, completados=2))
        escribir(self.q, CABECERA, filas)
        return M.filas_cola(self.q)


    def test_sin_violaciones_en_una_corrida_limpia(self):
        filas = self.cola_limpia()
        escribir(self.j, ['t_ns', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
                 [[int(t * 1e9)] + [0] * 6 for t in (1.1, 1.5, 1.9, 2.1, 2.5, 2.9)])
        v = M.violaciones_exclusion(filas, self.j)
        self.assertEqual((v['intercaladas'], v['sin_ejecucion']), (0, 0))
        self.assertEqual(v['mensajes_joint'], 6)

    def test_detecta_goals_intercalados(self):
        escribir(self.q, CABECERA, [
            fila_cola(1.0, ('A', 'g1')), fila_cola(1.2, ('B', 'g2')),
            fila_cola(1.4, ('A', 'g1'))])          # g1 vuelve a ejecutarse tras g2
        v = M.violaciones_exclusion(M.filas_cola(self.q), None)
        self.assertEqual(v['intercaladas'], 1)
        self.assertIsNone(v['sin_ejecucion'])


    def test_resumen_completo_y_figura(self):
        self.cola_limpia()
        d1 = M.resumen('fifo', self.q)
        self.assertEqual(d1['violaciones']['intercaladas'], 0)
        self.assertEqual(d1['rechazados'], 0)
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            self.skipTest('sin matplotlib no se prueba la figura')
        d2 = dict(d1, nombre='round_robin')
        salida = os.path.join(self.dir.name, 'fig.png')
        M.figura([d1, d2], salida)
        self.assertTrue(os.path.getsize(salida) > 10000)


class TestSimulador(unittest.TestCase):
    def orden(self, politica):
        entradas = S.llegadas([('A', 1), ('B', 2), ('C', 3)], 3, 'escalonado', 0.5)
        return S.metricas(S.simular(politica, entradas, 1.0))

    def test_ordenes_de_las_dos_politicas(self):
        self.assertEqual(self.orden('fifo')['orden'],
                         ['A1', 'A2', 'A3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3'])
        self.assertEqual(self.orden('round_robin')['orden'],
                         ['A1', 'B1', 'C1', 'A2', 'B2', 'C2', 'A3', 'B3', 'C3'])


if __name__ == '__main__':
    unittest.main()

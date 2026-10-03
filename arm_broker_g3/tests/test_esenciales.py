""" Pruebas esenciales del Reto 2 — una por fila de la tabla de pruebas esenciales """

"""Ítem 1 (FK):      el cálculo de la FK y del error, y las predicciones de las 3 poses del diseño previo
Ítem 2 (Broker):   goal válido, 3 rechazos con motivo, encolar ≠ ejecutar, exclusión mutua,
                   publicador único y orden FIFO / Round Robin
Los ítems 3 y 4 tienen su parte sin robot en test_analisis.py y test_auditar_ik.py; las 3 poses FÍSICAS del ítem 1,
las 2 corridas del ítem 3 y la prueba física del ítem 4 necesitan el robot y no son pruebas unitarias

Solo las esenciales:
    cd src/arm_broker_g3 && python3 -m unittest discover -s test -p "test_esenciales.py" -v
"""
import os
import random
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simulacion_ros as tb                                       # noqa: E402
from simulacion_ros import (HAY_ROS, OK, Base, GoalResponse,     # noqa: E402
                                  arrancar_worker, enviar, pose)

from arm_broker_g3 import fk                                               # noqa: E402

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
sys.path.insert(0, os.path.join(RAIZ, 'herramientas'))
from auditar_ik import error_cartesiano                                 # noqa: E402


# Ítem 1 — FK
class TestItem1FK(unittest.TestCase):
    def test_fk_pose_cero(self):
        """Con q = 0 el brazo apunta hacia arriba: x = d6 y y = -d4"""
        x, y, z = fk.fk([0.0] * 6)
        self.assertAlmostEqual(x, 50.0, places=1)
        self.assertAlmostEqual(y, -63.4, places=1)
        self.assertAlmostEqual(z, 416.3, places=1)

    def test_predicciones_del_diseno_previo(self):
        """Las predicciones congeladas en docs/tabla_dh.md salen del código actual"""
        esperado = {'ready': ([0, -0.5, 0.5, 0, 0.5, 0], (96.6, -39.4, 402.8)),
                    'girada': ([0.6, -0.4, 0.4, 0, 0.3, 0], (102.2, 11.0, 407.6)),
                    'baja': ([0, -1.2, 1.2, 0, 0, 0], (152.5, -63.4, 346.2))}
        for nombre, (q, xyz) in esperado.items():
            for a, b in zip(fk.fk(q), xyz):
                self.assertAlmostEqual(a, b, places=1, msg=nombre)

    def test_calculo_del_error(self):
        """Error de posición = distancia euclídea entre la FK y lo medido; criterio ≤ 10 mm"""
        self.assertEqual(error_cartesiano((0, 0, 0), (3, 4, 0)), 5.0)
        predicho = fk.fk([0, -0.5, 0.5, 0, 0.5, 0])
        medido = (predicho[0] + 6.0, predicho[1] - 8.0, predicho[2])
        self.assertAlmostEqual(error_cartesiano(predicho, medido), 10.0)


# Ítem 2 — Broker
@unittest.skipIf(HAY_ROS, 'con ROS 2 real estas pruebas simuladas se omiten')
class TestItem2Broker(Base):
    def ultimo_aviso(self, br):
        return br.get_logger().lineas[-1][1]

    def test_goal_valido_accept(self):
        br = self.broker()
        self.assertEqual(enviar(br, 'a', OK)[0], GoalResponse.ACCEPT)

    def test_limite_articular_reject_con_motivo(self):
        br = self.broker()
        self.assertEqual(enviar(br, 'a', [3.5, 0, 0, 0, 0, 0])[0], GoalResponse.REJECT)
        self.assertIn('(limite)', self.ultimo_aviso(br))
        self.assertIn('fuera de rango', self.ultimo_aviso(br))

    def test_workspace_reject_con_motivo(self):
        random.seed(3)
        fuera = next(q for q in ([random.uniform(lo, hi) for lo, hi in fk.JOINT_LIMITS]
                                 for _ in range(200000)) if not fk.dentro_del_workspace(q)[0])
        br = self.broker()
        br.q_actual = list(fuera)           # Que el paso articular no sea la causa del rechazo
        self.assertEqual(enviar(br, 'a', fuera)[0], GoalResponse.REJECT)
        self.assertIn('(workspace)', self.ultimo_aviso(br))

    def test_paso_excesivo_reject_con_motivo(self):
        br = self.broker()
        self.assertEqual(enviar(br, 'a', [2.5, 0, 0, 0, 0, 0])[0], GoalResponse.REJECT)
        self.assertIn('(paso)', self.ultimo_aviso(br))
        self.assertIn('rad', self.ultimo_aviso(br))

    def test_handle_accepted_solo_encola(self):
        """Encolar ≠ ejecutar: tras aceptar hay un pedido pendiente, ninguno en ejecución y
        nada publicado en /joint_states"""
        br = self.broker()
        enviar(br, 'a', OK)
        self.assertEqual(len(br.pendientes), 1)
        self.assertIsNone(br.ejecutando)
        self.assertEqual(br.pub_joint.msgs, [])

    def test_maximo_un_goal_ejecutandose(self):
        """Exclusión mutua con 6 goals de 3 clientes a la vez"""
        br = self.broker()
        envios = [enviar(br, c, pose(0.1 * i))[1] for c in 'ABC' for i in (1, 2)]
        arrancar_worker(br)
        for gh in envios:
            self.assertTrue(gh.terminado.wait(10))
        self.assertEqual(br.max_simultaneos, 1)
        self.assertNotIn(None, br.trace)    # Nada en /joint_states sin un goal ejecutando

    def test_solo_el_broker_publica_joint_states(self):
        """Regla de oro: el único create_publisher de /joint_states del paquete está en broker.py"""
        import ast
        import glob
        publican = []
        for ruta in glob.glob(os.path.join(tb.RAIZ, 'arm_broker_g3', '*.py')):
            with open(ruta, encoding='utf-8') as f:
                for n in ast.walk(ast.parse(f.read())):
                    if (isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'create_publisher'
                            and any(isinstance(a, ast.Constant) and a.value == '/joint_states'
                                    for a in n.args)):
                        publican.append(os.path.basename(ruta))
        self.assertEqual(publican, ['broker.py'])

    def correr(self, politica):
        br = self.broker(politica)
        envios = []
        for c, n in (('A', 3), ('B', 2), ('C', 2)):
            for i in range(1, n + 1):
                envios.append((f'{c}{i}', enviar(br, c, pose(0.05 * i))[1]))
                time.sleep(0.002)
        arrancar_worker(br)
        for _, gh in envios:
            self.assertTrue(gh.terminado.wait(15))
        nombres = self.etiquetas(br, envios)
        return [nombres[g] for g in br.orden_ejecucion]

    def test_fifo_y_round_robin_generan_el_orden_esperado(self):
        self.assertEqual(self.correr('fifo'),
                         ['A1', 'A2', 'A3', 'B1', 'B2', 'C1', 'C2'])
        self.assertEqual(self.correr('round_robin'),
                         ['A1', 'B1', 'C1', 'A2', 'B2', 'C2', 'A3'])


if __name__ == '__main__':
    unittest.main()

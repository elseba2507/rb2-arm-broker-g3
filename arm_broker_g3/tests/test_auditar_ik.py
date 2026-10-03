""" Pruebas de herramientas/auditar_ik.py (ítem 4) — sin robot """

"""Se prueban la matemática del error, la conversión grados/radianes, el registro de evidencia y
el flujo completo con un brazo simulado que se comporta como un MyCobot

    cd src/arm_broker_g3 && python3 -m unittest discover -s test -v
"""
import math
import os
import sys
import unittest

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
sys.path.insert(0, os.path.join(RAIZ, 'src', 'arm_broker_g3'))
sys.path.insert(0, os.path.join(RAIZ, 'herramientas'))

import auditar_ik as A                                    # noqa: E402
from arm_broker_g3 import fk                                 # noqa: E402


class BrazoFalso:
    """Se comporta como un MyCobot: send_coords mueve a una pose q, y get_angles/get_coords la leen"""

    def __init__(self, q_deg, coords=None):
        self.q_deg = list(q_deg) if isinstance(q_deg, (list, tuple)) else q_deg
        self.coords = coords
        self.enviados = []

    def send_coords(self, coords, velocidad, modo):
        self.enviados.append((list(coords), velocidad, modo))

    def get_angles(self):
        return self.q_deg

    def get_coords(self):
        if self.coords is not None:
            return self.coords
        return list(fk.fk([math.radians(v) for v in self.q_deg])) + [0.0, 0.0, 0.0]


Q_DEG = [10.0, -25.0, 30.0, 5.0, 20.0, 0.0]


class TestMatematica(unittest.TestCase):
    def test_error_345(self):
        self.assertEqual(A.error_cartesiano((0, 0, 0), (3, 4, 0)), 5.0)


    def test_grados_a_radianes(self):
        q = A.grados_a_radianes([0, 90, 180, -90, 45, 360])
        esperado = [0, math.pi / 2, math.pi, -math.pi / 2, math.pi / 4, 2 * math.pi]
        for a, b in zip(q, esperado):
            self.assertAlmostEqual(a, b)


class TestFlujoConBrazoSimulado(unittest.TestCase):
    def test_envia_send_coords_y_audita_el_q_leido(self):
        brazo = BrazoFalso(Q_DEG)
        xyz = fk.fk(A.grados_a_radianes(Q_DEG))
        objetivo = (xyz[0] + 2.0, xyz[1] + 3.0, xyz[2] + 6.0)
        r = A.ejecutar(brazo, objetivo, (0.0, 90.0, 0.0), velocidad=25, modo=0, espera_s=0.0)
        self.assertEqual(brazo.enviados, [(list(objetivo) + [0.0, 90.0, 0.0], 25, 0)])
        self.assertAlmostEqual(r['error_mm'], 7.0)                  # sqrt(4 + 9 + 36)


    def test_main_no_mueve_sin_objetivo_ni_confirmacion(self):
        self.assertEqual(A.main([]), 2)
        self.assertEqual(A.main(['--x', '200', '--y', '50', '--z', '180',
                                 '--rx', '0', '--ry', '0', '--rz', '0']), 2)


if __name__ == '__main__':
    unittest.main()

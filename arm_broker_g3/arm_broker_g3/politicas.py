"""Políticas de cola — ítems 2 y 3."""

import threading
import time


class Pedido:
    def __init__(self, goal_handle, client_id, priority, joint_positions):
        self.goal_handle = goal_handle
        self.goal_id = bytes(goal_handle.goal_id.uuid).hex()[:12]
        self.client_id = client_id
        self.priority = int(priority)
        self.joint_positions = list(joint_positions)
        self.t_llegada = time.time()
        self.t_inicio_ejec = None
        self.fin = threading.Event()
        self.resultado = None

    @property
    def espera_s(self):
        fin = self.t_inicio_ejec if self.t_inicio_ejec else time.time()
        return fin - self.t_llegada

    def __repr__(self):
        return f'<{self.client_id} p{self.priority} {self.goal_id}>'


class Politica:
    nombre = 'base'

    def siguiente(self, pendientes):
        raise NotImplementedError

    def atendido(self, pedido):
        pass


class FIFO(Politica):
    nombre = 'fifo'

    def siguiente(self, pendientes):
        """FIFO: el primero que llega es el primero en salir (índice 0)."""
        if not pendientes:
            return None
        return 0


class SegundaPolitica(Politica):
    nombre = 'prioridad'

    def __init__(self, tau_envejecimiento_s=8.0):
        super().__init__()
        self.tau_envejecimiento = tau_envejecimiento_s

    def siguiente(self, pendientes):
        """
        Prioridad con envejecimiento: prioridad base + 1 punto por cada
        'tau' segundos de espera. Con empate gana el que llegó antes.
        Devuelve el ÍNDICE del pedido elegido.
        """
        if not pendientes:
            return None

        mejor_idx = 0
        mejor_ef = None
        for i, pedido in enumerate(pendientes):
            ef = pedido.priority + pedido.espera_s / self.tau_envejecimiento
            if mejor_ef is None or ef > mejor_ef:
                mejor_idx = i
                mejor_ef = ef
        return mejor_idx


POLITICAS = {
    'fifo': FIFO,
    'prioridad': SegundaPolitica,
}

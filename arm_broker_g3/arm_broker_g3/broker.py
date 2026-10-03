"""arm_broker_g3 — el único nodo que publica en /joint_states.

Andamiaje entregado por el curso. Los bloques IMPLEMENTAR son lo que evalúa el
reto; el resto es instrumentación y se usa tal cual.
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_broker_interfaces_g3.action import MoveArm
from arm_broker_interfaces_g3.msg import QueueState

from arm_broker_g3 import fk
from arm_broker_g3.politicas import POLITICAS, Pedido


class ArmBroker(Node):

    def __init__(self):
        super().__init__('arm_broker_g3')

        self.declare_parameter('politica', 'fifo')
        self.declare_parameter('tau_envejecimiento_s', 8.0)
        self.declare_parameter('cola_max', 20)
        self.declare_parameter('paso_max_rad', 1.2)
        self.declare_parameter('duracion_movimiento_s', 3.0)
        self.declare_parameter('pasos_interpolacion', 10)

        nombre = self.get_parameter('politica').value
        if nombre not in POLITICAS:
            raise RuntimeError(f'política desconocida: {nombre}. Hay {list(POLITICAS)}')
        clase = POLITICAS[nombre]
        if nombre == 'prioridad':
            self.politica = clase(self.get_parameter('tau_envejecimiento_s').value)
        else:
            self.politica = clase()

        self.cola_max = int(self.get_parameter('cola_max').value)
        self.paso_max = float(self.get_parameter('paso_max_rad').value)
        self.duracion = float(self.get_parameter('duracion_movimiento_s').value)
        self.pasos = max(1, int(self.get_parameter('pasos_interpolacion').value))

        self.grupo_entrada = ReentrantCallbackGroup()
        self.grupo_worker = MutuallyExclusiveCallbackGroup()

        self.lock = threading.Lock()
        self.pendientes = []
        self.por_goal_id = {}
        self.ejecutando = None
        self.q_actual = [0.0] * 6
        self.n_aceptados = 0
        self.n_rechazados = 0
        self.n_completados = 0
        self._parar = threading.Event()

        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_cola = self.create_publisher(QueueState, '/arm_g3/queue_state', 10)

        self.servidor = ActionServer(
            self,
            MoveArm,
            'move_arm_g3',
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.grupo_entrada,
        )

        self.create_timer(0.2, self.publicar_estado_cola,
                          callback_group=self.grupo_worker)

        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'arm_broker_g3 listo · política={self.politica.nombre} · '
            f'cola_max={self.cola_max} · único publicador de /joint_states')

    # ========================= IMPLEMENTAR · ítem 2 ==========================
    def goal_callback(self, goal_request):
        """Admisión. Barata e inmediata: acepta o rechaza, nunca ejecuta."""
        q_dest = list(goal_request.joint_positions)
        quien = goal_request.client_id

        # 1. ¿Está dentro de los límites de los motores?
        ok_limites, msg_limites = fk.dentro_de_limites(q_dest)
        if not ok_limites:
            with self.lock:
                self.n_rechazados += 1
            self.get_logger().info(f'Rechazo [{quien}]: {msg_limites}')
            return GoalResponse.REJECT

        # 2. ¿Chocará con la mesa o su propia base (workspace)?
        ok_ws, msg_ws = fk.dentro_del_workspace(q_dest)
        if not ok_ws:
            with self.lock:
                self.n_rechazados += 1
            self.get_logger().info(f'Rechazo [{quien}]: {msg_ws}')
            return GoalResponse.REJECT

        # 3. ¿El movimiento es demasiado brusco desde donde está ahora?
        with self.lock:
            q_act = list(self.q_actual)
        paso = fk.paso_articular(q_act, q_dest)
        if paso > self.paso_max:
            with self.lock:
                self.n_rechazados += 1
            self.get_logger().info(
                f'Rechazo [{quien}]: paso articular {paso:.2f} rad '
                f'excede el máximo {self.paso_max:.2f} rad')
            return GoalResponse.REJECT

        with self.lock:
            self.n_aceptados += 1
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Encolar. AQUÍ NO SE EJECUTA NADA, y no se publica en /joint_states."""
        req = goal_handle.request
        pedido = Pedido(goal_handle, req.client_id, req.priority, req.joint_positions)
        with self.lock:
            self.pendientes.append(pedido)
            self.por_goal_id[bytes(goal_handle.goal_id.uuid)] = pedido

    def _worker(self):
        """El único que decide a quién le toca. Corre en su propio hilo."""
        while not self._parar.is_set():
            pedido = None

            # Bloqueamos la cola solo el instante necesario para sacar un pedido
            with self.lock:
                if not self.ejecutando and self.pendientes:
                    idx = self.politica.siguiente(self.pendientes)
                    if idx is not None:
                        pedido = self.pendientes.pop(idx)

            if pedido is None:
                time.sleep(0.05)
                continue

            if pedido.goal_handle.is_cancel_requested:
                pedido.goal_handle.canceled()
                continue

            with self.lock:
                self.ejecutando = pedido
                pedido.t_inicio_ejec = time.time()

            # Esto dispara execute_callback
            pedido.goal_handle.execute()

            # Exclusión mutua: el worker espera hasta que el brazo termine
            pedido.fin.wait()

            with self.lock:
                self.ejecutando = None
                self.n_completados += 1
            self.politica.atendido(pedido)

    def execute_callback(self, goal_handle):
        """Ejecutar UN pedido. Lo llama el worker, nunca handle_accepted."""
        llave = bytes(goal_handle.goal_id.uuid)
        pedido = self.por_goal_id[llave]
        try:
            q_dest = list(goal_handle.request.joint_positions)
            with self.lock:
                q_ini = list(self.q_actual)

            pasos = self.pasos
            t_espera = self.duracion / pasos

            # Interpolación: partir el viaje en pasos chiquitos para que sea suave
            for i in range(1, pasos + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return MoveArm.Result()

                q_inter = [q_ini[j] + (q_dest[j] - q_ini[j]) * (i / pasos)
                           for j in range(6)]
                self.mover(q_inter)

                fb = MoveArm.Feedback()
                fb.state = 'EXECUTING'
                fb.queue_position = 0
                fb.elapsed_s = time.time() - pedido.t_inicio_ejec
                goal_handle.publish_feedback(fb)

                time.sleep(t_espera)

            goal_handle.succeed()
            res = MoveArm.Result()
            res.success = True
            res.message = 'Movimiento completado'
            res.wait_time_s = pedido.espera_s
            res.exec_time_s = time.time() - pedido.t_inicio_ejec
            return res

        except Exception as e:
            self.get_logger().error(f'Fallo ejecutando {pedido}: {e}')
            goal_handle.abort()
            res = MoveArm.Result()
            res.success = False
            res.message = f'Error interno: {e}'
            return res

        finally:
            with self.lock:
                self.por_goal_id.pop(llave, None)
            pedido.fin.set()

    # =========================================================================

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    # ----------------------------------------------------------- publicar
    def mover(self, q):
        import math
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = fk.JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.pub_joint.publish(msg)
        with self.lock:
            self.q_actual = list(q)

    def publicar_estado_cola(self):
        msg = QueueState()
        msg.stamp = self.get_clock().now().to_msg()
        with self.lock:
            ej = self.ejecutando
            msg.executing_client = ej.client_id if ej else ''
            msg.executing_goal_id = ej.goal_id if ej else ''
            msg.executing_elapsed_s = (time.time() - ej.t_inicio_ejec) if ej and ej.t_inicio_ejec else 0.0
            msg.queue_length = len(self.pendientes)
            msg.queued_goal_ids = [p.goal_id for p in self.pendientes]
            msg.queued_clients = [p.client_id for p in self.pendientes]
            msg.queued_priorities = [min(255, max(0, p.priority)) for p in self.pendientes]
            msg.queued_wait_s = [p.espera_s for p in self.pendientes]
            msg.total_accepted = self.n_aceptados
            msg.total_rejected = self.n_rechazados
            msg.total_completed = self.n_completados
            cola = list(self.pendientes)
        self.pub_cola.publish(msg)

        for posicion, p in enumerate(cola, start=1):
            try:
                fb = MoveArm.Feedback()
                fb.state = 'QUEUED'
                fb.queue_position = posicion
                fb.elapsed_s = p.espera_s
                p.goal_handle.publish_feedback(fb)
            except Exception:
                pass

    def destroy_node(self):
        self._parar.set()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = ArmBroker()
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

""" Simulación de ROS 2 para las pruebas esenciales — sin ROS 2 ni robot """

"""No contiene pruebas: solo los dobles de rclpy y de las interfaces, el doble del
ServerGoalHandle (con la máquina de estados de rclpy) y utilidades para armar un broker
instrumentado. Las pruebas están en test_esenciales.py"""
import enum
import os
import sys
import threading
import time
import types
import unittest
import uuid

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, RAIZ)


# 1. Dobles de rclpy y de las interfaces
class Fut:
    """Future mínimo: result(), done(), add_done_callback()"""

    def __init__(self):
        self._ev = threading.Event()
        self._val = None
        self._cbs = []

    def set_result(self, v):
        self._val = v
        self._ev.set()
        for cb in self._cbs:
            cb(self)

    def done(self):
        return self._ev.is_set()

    def result(self):
        return self._val

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self._ev.is_set():
            cb(self)


class Param:
    def __init__(self, v):
        self.value = v


class Log:
    def __init__(self):
        self.lineas = []

    def _add(self, nivel, m):
        self.lineas.append((nivel, m))

    def info(self, m, **k):
        self._add('I', m)

    def warn(self, m, **k):
        self._add('W', m)

    def error(self, m, **k):
        self._add('E', m)


class Node:
    PARAMS = {}     # Valores que reemplazan a los declarados por defecto

    def __init__(self, name):
        self._p = {}
        self._log = Log()

    def declare_parameter(self, k, v):
        self._p[k] = Param(Node.PARAMS.get(k, v))

    def get_parameter(self, k):
        return self._p[k]

    def get_logger(self):
        return self._log

    def create_publisher(self, tipo, nombre, cola):
        return Pub()

    def create_timer(self, *a, **k):
        pass

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: None))

    def destroy_node(self):
        pass


class Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class GoalResponse(enum.Enum):
    ACCEPT = 1
    REJECT = 2


class CancelResponse(enum.Enum):
    ACCEPT = 1
    REJECT = 2


class Obj:
    def __init__(self, **k):
        self.__dict__.update(k)


class Goal(Obj):
    def __init__(self):
        self.joint_positions = []
        self.client_id = ''
        self.priority = 0


class Result(Obj):
    def __init__(self):
        self.success = False
        self.message = ''
        self.wait_time_s = 0.0
        self.exec_time_s = 0.0


class Feedback(Obj):
    def __init__(self):
        self.state = ''
        self.queue_position = 0
        self.elapsed_s = 0.0


class MoveArm:
    Goal = Goal
    Result = Result
    Feedback = Feedback


class JointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.name = []
        self.position = []


def _modulo(nombre, **atributos):
    m = types.ModuleType(nombre)
    m.__dict__.update(atributos)
    sys.modules[nombre] = m
    return m


def instalar_dobles():
    """Se instalan los dobles solo si ROS 2 no está: con ROS real estas pruebas se omiten"""
    _modulo('rclpy',
            ok=lambda: True,
            spin_once=lambda nodo, timeout_sec=0.0: time.sleep(min(timeout_sec, 0.01)),
            spin_until_future_complete=lambda nodo, fut: fut._ev.wait(10))
    _modulo('rclpy.action', ActionServer=lambda *a, **k: None, ActionClient=lambda *a, **k: None,
            CancelResponse=CancelResponse, GoalResponse=GoalResponse)
    _modulo('rclpy.callback_groups', MutuallyExclusiveCallbackGroup=object,
            ReentrantCallbackGroup=object)
    _modulo('rclpy.executors', MultiThreadedExecutor=object)
    _modulo('rclpy.node', Node=Node)
    _modulo('sensor_msgs')
    _modulo('sensor_msgs.msg', JointState=JointState)
    _modulo('arm_broker_interfaces_g3')
    _modulo('arm_broker_interfaces_g3.action', MoveArm=MoveArm)
    _modulo('arm_broker_interfaces_g3.msg', QueueState=Obj)


try:
    import rclpy  # noqa: F401
    HAY_ROS = True
except ImportError:
    instalar_dobles()
    HAY_ROS = False

if not HAY_ROS:
    from arm_broker_g3 import broker as B          # noqa: E402


# 2. Doble del ServerGoalHandle con la máquina de estados de rclpy
class GH:
    def __init__(self, br, goal):
        self.request = goal
        self.goal_id = types.SimpleNamespace(uuid=uuid.uuid4().bytes)
        self.state = 'accepted'
        self.br = br
        self.feedbacks = []
        self.result_future = Fut()       # Solo lo completa execute(): igual que rclpy
        self.terminado = threading.Event()

    @property
    def is_cancel_requested(self):
        return self.state == 'canceling'

    def cancelar(self):
        """Lo que hace rclpy cuando el cliente pide cancelar y cancel_callback acepta"""
        if self.state in ('accepted', 'executing'):
            self.state = 'canceling'

    def execute(self):
        # Como rclpy: con cancelación pedida no hay transición a EXECUTING, pero sí se llama
        # a execute_callback
        if self.state == 'accepted':
            self.state = 'executing'
        elif self.state != 'canceling':
            raise RuntimeError(f'execute() inválido desde {self.state}')

        def correr():
            try:
                res = self.br.execute_callback(self)
            except Exception:
                res = Result()
            if self.state in ('executing', 'canceling'):
                self.state = 'aborted'      # rclpy: «Goal state not set, assuming aborted»
            self.terminado.set()
            self.result_future.set_result(types.SimpleNamespace(result=res))
        threading.Thread(target=correr, daemon=True).start()

    def succeed(self):
        if self.state not in ('executing', 'canceling'):
            raise RuntimeError(f'succeed() inválido desde {self.state}')
        self.state = 'succeeded'

    def abort(self):
        if self.state not in ('executing', 'canceling'):
            raise RuntimeError(f'abort() inválido desde {self.state}')
        self.state = 'aborted'

    def canceled(self):
        if self.state != 'canceling':
            raise RuntimeError(f'canceled() inválido desde {self.state}')
        self.state = 'canceled'

    def publish_feedback(self, f):
        self.feedbacks.append(f.state)


# 3. Utilidades
OK = [0.3, -0.4, 0.4, 0.0, 0.3, 0.0]     # Pose válida y cercana al origen


def pose(q1):
    return [q1] + OK[1:]


def nuevo_broker(politica='fifo', **params):
    """Se crea un broker con movimientos muy cortos y se instrumenta la exclusión mutua"""
    Node.PARAMS = dict(politica=politica, duracion_movimiento_s=0.06, pasos_interpolacion=3,
                       archivo_rechazos='')
    Node.PARAMS.update(params)
    br = B.ArmBroker()

    br.trace = []              # goal_id con el que se publicó cada mensaje de /joint_states
    br.max_simultaneos = 0     # Máximo de execute_callback corriendo a la vez
    br.orden_ejecucion = []    # goal_id en el orden en que empezaron
    br._activos = 0
    mover, execute = br.mover, br.execute_callback

    def mover_instrumentado(q):
        with br.lock:
            ej = br.ejecutando
        br.trace.append(ej.goal_id if ej else None)
        mover(q)

    def execute_instrumentado(gh):
        with br.lock:
            br._activos += 1
            br.max_simultaneos = max(br.max_simultaneos, br._activos)
            br.orden_ejecucion.append(bytes(gh.goal_id.uuid).hex())
        try:
            return execute(gh)
        finally:
            with br.lock:
                br._activos -= 1

    br.mover, br.execute_callback = mover_instrumentado, execute_instrumentado
    return br


def arrancar_worker(br):
    """El worker es un timer de 20 ms en ROS 2; aquí lo emula un hilo que lo invoca igual"""
    def bucle():
        while not br._parar.is_set():
            br._worker()
            time.sleep(0.02)
    t = threading.Thread(target=bucle, daemon=True)
    t.start()
    return t


def enviar(br, cliente, q, prioridad=1):
    g = Goal()
    g.joint_positions = q
    g.client_id = cliente
    g.priority = prioridad
    r = br.goal_callback(g)
    if r != GoalResponse.ACCEPT:
        return r, None
    gh = GH(br, g)
    br.handle_accepted_callback(gh)
    return r, gh


def esperar(cond, timeout=10.0):
    fin = time.time() + timeout
    while time.time() < fin:
        if cond():
            return True
        time.sleep(0.005)
    return False


class Base(unittest.TestCase):
    def setUp(self):
        self.brokers = []

    def tearDown(self):
        for br in self.brokers:
            br._parar.set()

    def broker(self, *a, **k):
        br = nuevo_broker(*a, **k)
        self.brokers.append(br)
        return br

    def etiquetas(self, br, envios):
        """goal_id completo -> etiqueta legible (A1, B2...)"""
        return {bytes(gh.goal_id.uuid).hex(): e for e, gh in envios}

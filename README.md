# RB-2 · El Turno del Brazo · Grupo 3

Broker con cola de prioridad para el acceso concurrente a un JetCobot (MyCobot 280) con ROS 2 Humble.
Curso Robótica (08079), ESAN, semestre 2026-2.

**Integrantes:** Nicolas Figueroa · Nahim Patiño · Sebastian Patazca

## Qué hace

Cuatro clientes mandan movimientos al mismo brazo. Ningún cliente publica en `/joint_states`:
solo el nodo `arm_broker_g3`, que corre en la Jetson, habla con el driver.

1. **Admisión (`goal_callback`):** rechaza, con motivo escrito en el log, todo objetivo fuera de
   límites articulares, fuera del workspace (calculado con la cinemática directa) o con un paso
   articular mayor a `paso_max_rad`.
2. **Encolado (`handle_accepted_callback`):** solo encola, no ejecuta.
3. **Ejecución:** un único worker elige el siguiente pedido según la política y lo ejecuta de
   forma exclusiva, interpolando en pasos.
4. **Estado de la cola:** se publica a 5 Hz en `/arm_g3/queue_state`.

Políticas implementadas: `fifo` y `prioridad` (prioridad con envejecimiento: prioridad efectiva
= `p + espera / tau`, con `tau = 8 s`; mayor número = más urgente).

## Estructura del repositorio

```
arm_broker_g3/              paquete del broker, políticas, FK y cliente
arm_broker_interfaces_g3/   acción MoveArm y mensaje QueueState
codigo_analisis/            exportar_csv.py y metricas.py (entregados por el curso)
bags/                       bag_fifo_final y bag_prioridad_final (ros2 bag)
resultados/                 CSV exportados, figura comparativa y metricas_resultado.txt
docs/                       diseño, predicción analítica del p95 y cierre reflexivo
```

## Requisitos

- Ubuntu 22.04 con ROS 2 Humble en la Jetson del JetCobot
- Driver del robot: paquete `jetcobot_driver`, nodo `sync_plan_nx` (usa `pymycobot`)
- `matplotlib` para la figura (`pip3 install matplotlib`)

## Compilar

Copia los dos paquetes dentro de `src/` de tu workspace y compila:

```bash
cd ~/rb2_ws
colcon build --symlink-install --packages-select arm_broker_interfaces_g3 arm_broker_g3
source install/setup.bash
```

## Ejecutar

Cada terminal necesita `source ~/rb2_ws/install/setup.bash`.

**1. Driver** (terminal A):

```bash
ros2 run jetcobot_driver sync_plan_nx
```

**2. Broker** (terminal B), con la política `fifo` o `prioridad`:

```bash
ros2 run arm_broker_g3 broker --ros-args -p politica:=prioridad
```

Antes de lanzar clientes, comprueba que hay un solo publicador y un suscriptor:

```bash
ros2 topic info /joint_states -v     # Publisher count: 1, Subscription count: 1
```

**3. Clientes** (una terminal por cliente, lanzados a la vez con la misma traza):

```bash
ros2 run arm_broker_g3 cliente --ros-args -p client_id:=Nicolas   -p priority:=1 -p traza:=carga.csv
ros2 run arm_broker_g3 cliente --ros-args -p client_id:=Nahim     -p priority:=2 -p traza:=carga.csv
ros2 run arm_broker_g3 cliente --ros-args -p client_id:=Sebastian -p priority:=3 -p traza:=carga.csv
ros2 run arm_broker_g3 cliente --ros-args -p client_id:=Vania     -p priority:=4 -p traza:=carga.csv
```

### Parámetros del broker

| Parámetro | Valor por defecto |
|---|---|
| `politica` | `fifo` (o `prioridad`) |
| `tau_envejecimiento_s` | 8.0 |
| `cola_max` | 20 |
| `paso_max_rad` | 1.2 |
| `duracion_movimiento_s` | 3.0 |
| `pasos_interpolacion` | 10 |

## Medición (ítem 3)

Se graba un bag por política, con el brazo en pose cero y un solo broker corriendo:

```bash
ros2 bag record -o bag_fifo /joint_states /arm_g3/queue_state
```

Luego se exporta a CSV y se calculan las métricas:

```bash
python3 codigo_analisis/exportar_csv.py bags/bag_fifo_final --salida salida/fifo
python3 codigo_analisis/exportar_csv.py bags/bag_prioridad_final --salida salida/prioridad
python3 codigo_analisis/metricas.py salida/fifo/queue_state.csv salida/prioridad/queue_state.csv
```

Los CSV ya exportados de nuestras corridas están en `resultados/` (`fifo_*.csv` y `prioridad_*.csv`).

`metricas.py` imprime espera media y p95 por prioridad, índice de inanición y equidad de Jain,
y genera `comparacion_politicas.png`.

### Resultados

Misma traza de unos 150 goals por política, cuatro clientes en contención.

| | Espera media | p95 | Máx. |
|---|---|---|---|
| FIFO, prioridad 1 | 8.14 s | 8.52 s | 8.53 s |
| FIFO, prioridad 4 | 8.34 s | 8.51 s | 8.53 s |
| Prioridad, prioridad 1 | 9.72 s | 20.57 s | 26.64 s |
| Prioridad, prioridad 2 | 7.61 s | 20.55 s | 20.62 s |
| Prioridad, prioridad 3 | 5.31 s | 8.51 s | 8.53 s |
| Prioridad, prioridad 4 | 2.40 s | 2.49 s | 2.50 s |

Equidad de Jain: 1.000 (FIFO) y 0.996 (prioridad). Índice de inanición (espera máxima de la
prioridad 1): 8.53 s en FIFO y 26.64 s en prioridad.

Resultados completos en `resultados/metricas_resultado.txt`.

## Notas importantes

- **Nombres `_g3`:** los paquetes, la acción (`move_arm_g3`) y el topic (`/arm_g3/queue_state`) llevan
  el sufijo `_g3` porque la Jetson se compartió con otro grupo y sus archivos se pisaban.
  El código es el del kit del curso con los nombres cambiados.
- **Unidades:** el driver convierte `/joint_states` de radianes a grados con `math.degrees`, así que
  el broker publica en **radianes**. Se corrigió `mover()`, que en el andamiaje publicaba en grados
  (el doble conversión desbordaba el límite de `pymycobot` y tumbaba el driver).
- **Bags:** el archivo `.db3` dentro de cada carpeta de `bags/` conserva el nombre original de la
  grabación (los nombres quedaron cruzados al grabar). El contenido se verificó por el orden de
  atención de los clientes: `bag_fifo_final` atiende por llegada y `bag_prioridad_final` atiende
  primero a los de mayor prioridad.
- **Corridas descartadas:** dos grabaciones se descartaron porque se detectaron dos brokers
  publicando a la vez. Las incluidas se hicieron con un solo broker y `Publisher count: 1`.
- **Limitaciones:** una corrida por política; el estado de la cola se publica a 5 Hz, así que las
  esperas se observan con un error de ±0.2 s.

## Otros ítems

- Ítem 1 (cinemática directa): tabla DH y código en `arm_broker_g3/arm_broker_g3/fk.py`; verificación en `docs/`.
- Ítem 4 (cinemática inversa auditada con la FK propia): ver `docs/`.

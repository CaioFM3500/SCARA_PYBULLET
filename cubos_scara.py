import pybullet as p
import pybullet_data
import time
import math
import random

# ---- Gráfico em tempo real com PyQtGraph ----
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import threading

pg.setConfigOption('background', 'w')  # fundo branco
pg.setConfigOption('foreground', 'k')  # texto e eixos em preto

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Trajetórias e Ângulos SCARA em Tempo Real")

# Gráfico 1: XY da garra + blocos
plot_xy = win.addPlot(title="Posição XY da Garra e Cubos")
curve_xy = plot_xy.plot(pen=pg.mkPen(color='r', width=2))
scatter_cubos = pg.ScatterPlotItem(size=10, brush='b')
plot_xy.addItem(scatter_cubos)
plot_xy.setXRange(-0.5, 0.5)
plot_xy.setYRange(-0.5, 0.5)
plot_xy.setLabel('left', 'Y [m]')
plot_xy.setLabel('bottom', 'X [m]')
x_data, y_data = [], []

# Gráfico 2: Junta 1
win.nextRow()
plot_j1 = win.addPlot(title="Junta 1: Ângulo Alvo (Vermelho) vs Atual (Azul)")
curve_j1_target = plot_j1.plot(pen=pg.mkPen(color='r', width=2))
curve_j1_actual = plot_j1.plot(pen=pg.mkPen(color='b', width=2))
plot_j1.setLabel('left', 'Ângulo [rad]')
plot_j1.setLabel('bottom', 'Tempo [s]')
t_data, j1_target_data, j1_actual_data = [], [], []

# Gráfico 3: Junta 2
win.nextRow()
plot_j2 = win.addPlot(title="Junta 2: Ângulo Alvo (Vermelho) vs Atual (verde)")
curve_j2_target = plot_j2.plot(pen=pg.mkPen(color='r', width=2))
curve_j2_actual = plot_j2.plot(pen=pg.mkPen(color='g', width=2))
plot_j2.setLabel('left', 'Ângulo [rad]')
plot_j2.setLabel('bottom', 'Tempo [s]')
j2_target_data, j2_actual_data = [], []

win.show()

start_time = time.time()

# Função de atualização dos gráficos
def update_graphs():
    if len(t_data) > 0:
        curve_xy.setData(x=x_data, y=y_data)
        curve_j1_target.setData(x=t_data, y=j1_target_data)
        curve_j1_actual.setData(x=t_data, y=j1_actual_data)
        curve_j2_target.setData(x=t_data, y=j2_target_data)
        curve_j2_actual.setData(x=t_data, y=j2_actual_data)

        # Atualiza posições dos cubos em verde no gráfico
        pos_cubos = []
        for cubo in cubos:
            pos, _ = p.getBasePositionAndOrientation(cubo['id'])
            pos_cubos.append({'pos': (pos[0], pos[1])})
        scatter_cubos.setData([p['pos'][0] for p in pos_cubos], [p['pos'][1] for p in pos_cubos])

timer = QtCore.QTimer()
timer.timeout.connect(update_graphs)
timer.start(30)

# Função para adicionar dados sincronizados
def append_data(t, x, y, j1_tgt, j1_act, j2_tgt, j2_act):
    t_data.append(t)
    x_data.append(x)
    y_data.append(y)
    j1_target_data.append(j1_tgt)
    j1_actual_data.append(j1_act)
    j2_target_data.append(j2_tgt)
    j2_actual_data.append(j2_act)
    if len(t_data) > 400:
        x_data.pop(0)
        y_data.pop(0)
    if len(t_data) > 1000:
        t_data.pop(0)
        j1_target_data.pop(0)
        j1_actual_data.pop(0)
        j2_target_data.pop(0)
        j2_actual_data.pop(0)

# ---- SCARA e PyBullet Setup ----
L1, L2 = 0.249, 0.24847
R_max = L1 + L2
dt = 1 / 240.0
z_mesa = 0.775
z_baixo = z_mesa + 0.025
z_topo_cubo = z_baixo + 0.05

def random_target():
    while True:
        θ = random.uniform(0, 2 * math.pi)
        r = math.sqrt(random.uniform(0, 1)) * R_max
        x = r * math.cos(θ)
        y = r * math.sin(θ)
        in_area_1 = (-0.46 <= x <= 0.46) and (0.06 <= y <= 0.36)
        in_area_2 = (0.08 <= x <= 0.48) and (-0.4 <= y <= 0.06)
        in_area_3 = (-0.48 <= x <= -0.08) and (-0.4 <= y <= 0.06)
        if in_area_1 or in_area_2 or in_area_3:
            return x, y, z_baixo

def scara_ik_2d(x, y, L1=0.249, L2=0.24847):
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(D) > 1:
        return None
    θ2a = math.atan2(math.sqrt(1 - D**2), D)
    θ2b = math.atan2(-math.sqrt(1 - D**2), D)
    θ1a = math.atan2(y, x) - math.atan2(L2 * math.sin(θ2a), L1 + L2 * math.cos(θ2a))
    θ1b = math.atan2(y, x) - math.atan2(L2 * math.sin(θ2b), L1 + L2 * math.cos(θ2b))
    return (θ1a, θ2a), (θ1b, θ2b)

def create_cube(pos):
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025]*3, rgbaColor=[0, 0, 1, 1])
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025]*3)
    ori = p.getQuaternionFromEuler([0, math.radians(270), 0])
    cube_id = p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=col,baseVisualShapeIndex=vis, basePosition=pos)
    p.resetBasePositionAndOrientation(cube_id, pos, ori)
    return cube_id

# ---- PyBullet initialization ----
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(r'_internal/pybullet_data')
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(1.2, 120, -30, [0, 0, 0.8])

# ---- Mesa com pernas ----
p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.6, 0.6, 0.025]),
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.6, 0.6, 0.025], rgbaColor=[0.8, 0.6, 0.4, 1]),
    basePosition=[0, 0, z_mesa]
)
for dx in [-0.55, 0.55]:
    for dy in [-0.55, 0.55]:
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.375]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.375], rgbaColor=[0.4, 0.4, 0.4, 1]),
            basePosition=[dx, dy, 0.375]
        )

base_ori = p.getQuaternionFromEuler([0, math.radians(270), 0])
robot = p.loadURDF("scara1.urdf", [0, 0, z_mesa + 0.07], base_ori, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
j1, j2, j3 = 0, 1, 2
ee_link = 2

cubos = []
for _ in range(6):
    x, y, z = random_target()
    cubos.append({'id': create_cube([x, y, z]), 'pos': (x, y, z)})

linha_destinos = [(i * 0.1 - 0.24, 0.4, z_baixo) for i in range(6)]

# ---- Linha vermelha sobre a mesa no eixo X (referência visual)
x_ini = linha_destinos[0][0] - 0.05  # pequeno excesso à esquerda
x_fim = linha_destinos[-1][0] + 0.05  # pequeno excesso à direita
y_mesa = linha_destinos[0][1]
z_linha = z_baixo + 0.052  # ligeiramente acima do cubo

p.addUserDebugLine([x_ini, y_mesa, z_linha],
                   [x_fim, y_mesa, z_linha],
                   [1, 0, 0],  # cor vermelha RGB
                   lineWidth=2.0)

# ---- Marcadores verticais em cada ponto de destino
for destino in linha_destinos:
    x, y, z = destino
    p.addUserDebugLine([x, y - 0.01, z_linha],  
                       [x, y + 0.01, z_linha], 
                       [1, 0, 0], 
                       lineWidth=1.5)

# --- Função de movimentação dos cubos e coleta de dados ---
def move_cube_to(robot, cube_id, origem, destino):
    pegou = grudado = subiu = chegou = desceu = terminou = False
    constraint_id = None
    frames_dentro = 0
    frames_parado = 0
    pos_anterior = [0, 0]
    tentativas_travamento = 0

    while not terminou:
        ee = p.getLinkState(robot, ee_link)[4]
        tempo = time.time() - start_time
        theta1_actual = p.getJointState(robot, j1)[0]
        theta2_actual = p.getJointState(robot, j2)[0]
        theta1_cmd, theta2_cmd = theta1_actual, theta2_actual

        if not pegou:
            if tentativas_travamento < 2:
                target_pos = origem
            elif tentativas_travamento == 2:
                target_pos = (0, 0.5)
            else:
                print("Falha persistente. Pulando cubo.")
                return

            ik_sols = scara_ik_2d(target_pos[0], target_pos[1])
            if ik_sols is None:
                print("Posição fora do alcance, pulando cubo.")
                return
            (theta1a, theta2a), (theta1b, theta2b) = ik_sols
            if target_pos[0] < 0 and target_pos[1] < 0:
                theta1a += 2 * math.pi
                theta1b += 2 * math.pi
            if tentativas_travamento in [0, 2]:
                theta1_cmd, theta2_cmd = theta1a, theta2a
            else:
                theta1_cmd, theta2_cmd = theta1b, theta2b
            if theta1_cmd < -0.75 or theta1_cmd > 3.9:
                theta1_cmd, theta2_cmd = theta1b, theta2b

            p.setJointMotorControl2(robot, j1, p.POSITION_CONTROL, targetPosition=theta1_cmd, force=5)
            p.setJointMotorControl2(robot, j2, p.POSITION_CONTROL, targetPosition=theta2_cmd, force=5)

            desloc = math.sqrt((ee[0] - pos_anterior[0])**2 + (ee[1] - pos_anterior[1])**2)
            if desloc < 0.0005:
                frames_parado += 1
            else:
                frames_parado = 0
            pos_anterior = ee

            if frames_parado > 120:
                tentativas_travamento += 1
                print(f"Travado! Tentativa #{tentativas_travamento}")
                frames_parado = 0
                continue

            dx = abs(ee[0] - origem[0])
            dy = abs(ee[1] - origem[1])
            if dx < 0.01 and dy < 0.01:
                frames_dentro += 1
                if frames_dentro > 10:
                    pegou = True
                    frames_dentro = 0
            else:
                frames_dentro = 0

        elif not grudado:
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=-0.032)
            if abs(ee[2]-0.062 - z_topo_cubo) < 0.005:
                ee_pos, ee_ori = p.getLinkState(robot, ee_link)[4:6]
                cube_pos, _ = p.getBasePositionAndOrientation(cube_id)
                p.resetBasePositionAndOrientation(cube_id, cube_pos, ee_ori)
                constraint_id = p.createConstraint(robot, ee_link, cube_id, -1, p.JOINT_FIXED, [0,0,0],[0,0,0],[0.08,0,0])  
                grudado = True

        elif not subiu:
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=0.02)
            if ee[2]-0.062 > 0.9:
                subiu = True

        elif not chegou:
            ik_sols = scara_ik_2d(destino[0], destino[1])
            if ik_sols is None:
                print("Destino fora do alcance. Pulando cubo.")
                return
            (theta1a, theta2a), (theta1b, theta2b) = ik_sols
            theta1_cmd, theta2_cmd = theta1b, theta2b
            if not (-0.75 <= theta1_cmd <= 3.9):
                theta1_cmd, theta2_cmd = theta1a, theta2a

            p.setJointMotorControl2(robot, j1, p.POSITION_CONTROL, targetPosition=theta1_cmd, force=5)
            p.setJointMotorControl2(robot, j2, p.POSITION_CONTROL, targetPosition=theta2_cmd, force=5)
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=0.02)

            dx = abs(ee[0] - destino[0])
            dy = abs(ee[1] - destino[1])
            if dx < 0.02 and dy < 0.02:
                frames_dentro += 1
                if frames_dentro > 20:
                    chegou = True
                    frames_dentro = 0
            else:
                frames_dentro = 0

        elif not desceu:
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=-0.032, force=5)
            if abs(ee[2]-0.062 - z_topo_cubo) < 0.01:
                p.removeConstraint(constraint_id)
                desceu = True

        else:
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=0, force=5)
            if ee[2]-0.062 > z_topo_cubo + 0.03:
                terminou = True

        append_data(tempo, ee[0], ee[1], theta1_cmd, theta1_actual, theta2_cmd, theta2_actual)

        p.stepSimulation()
        time.sleep(dt)

# Thread de execução principal
time.sleep(3)
def sim_loop():
    for i in range(6):
        print(f"\nMovendo cubo {i+1}...")
        move_cube_to(robot, cubos[i]['id'], cubos[i]['pos'], linha_destinos[i])
    print("\nTodos os cubos foram posicionados.")
    while True:
        ee = p.getLinkState(robot, ee_link)[4]
        tempo = time.time() - start_time
        theta1 = p.getJointState(robot, j1)[0]
        theta2 = p.getJointState(robot, j2)[0]
        append_data(tempo, ee[0], ee[1], theta1, theta1, theta2, theta2)
        p.stepSimulation()
        time.sleep(dt)

threading.Thread(target=sim_loop, daemon=True).start()

if __name__ == '__main__':
    app.exec_()

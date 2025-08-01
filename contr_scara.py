import pybullet as p
import time
import math
import random
 
# ----- Setup Inicial -----
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

p.setAdditionalSearchPath(r'_internal/pybullet_data')
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
dt = 1 / 240.0

# ---- Parâmetros da cena ----
L1, L2 = 0.249, 0.24847
R_max = L1 + L2
z_mesa = 0.775
z_cubo = z_mesa + 0.025
z_topo_cubo = z_cubo + 0.05
z_offset = 0.062  # Offset da ponta da ventosa
z_alto = 0.100
z_baixo = z_cubo
# ---- Gera um ponto aleatório sobre a mesa ----
def random_target():
    while True:
        # Gera valores dentro do alcance máximo do SCARA
        θ = random.uniform(0, 2 * math.pi)
        r = math.sqrt(random.uniform(0, 1)) * R_max
        x = r * math.cos(θ)
        y = r * math.sin(θ)

        # Checa se está em uma das áreas:
        in_area_1 = (-0.48 <= x <= 0.48) and (0.06 <= y <= 0.48)
        in_area_2 = (0.08 <= x <= 0.48) and (-0.4 <= y <= 0.06)
        in_area_3 = (-0.48 <= x <= -0.08) and (-0.4 <= y <= 0.06)

        if in_area_1 or in_area_2 or in_area_3:
            return x, y, z_baixo
        
# ---- Cin. inversa analítica 2D para SCARA ----
def scara_ik_2d(x, y, L1=0.249, L2=0.24847):
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(D) > 1:
        return None
    θ2a = math.atan2(math.sqrt(1 - D**2), D)
    θ2b = math.atan2(-math.sqrt(1 - D**2), D)
    θ1a = math.atan2(y, x) - math.atan2(L2 * math.sin(θ2a), L1 + L2 * math.cos(θ2a))
    θ1b = math.atan2(y, x) - math.atan2(L2 * math.sin(θ2b), L1 + L2 * math.cos(θ2b))
    return (θ1a, θ2a), (θ1b, θ2b)


# ---- Cria cubo azul de 5cm ----
def create_cube(pos):
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025]*3, rgbaColor=[0, 0, 1, 1])
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025]*3)
    ori = p.getQuaternionFromEuler([0, math.radians(270), 0])
    cube_id = p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=col,baseVisualShapeIndex=vis, basePosition=pos)
    p.resetBasePositionAndOrientation(cube_id, pos, ori)
    return cube_id

# Posicionar câmera
p.resetDebugVisualizerCamera(
    cameraDistance=1.4,
    cameraYaw=180,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0.8]
)
# ---- Mesa com 4 pernas ----
p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.8, 0.8, 0.025]),
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.8, 0.8, 0.025], rgbaColor=[0.8, 0.6, 0.4, 1]),
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

# ---- Robô SCARA ----
base_ori = p.getQuaternionFromEuler([0, math.radians(270), 0])
robot = p.loadURDF("scara1.urdf",
                basePosition=[0, 0, z_mesa + 0.07],
                baseOrientation=base_ori,
                useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION)

j1, j2, j3 = 0, 1, 2
ee_link = 2

# ---- Cria cubo ----
x_w, y_w, z_w = random_target()
cube_id = create_cube([x_w, y_w, z_w])
print(f"Cubo(world)= X={x_w:.3f}, Y={y_w:.3f}, Z={z_w:.3f}")
p.addUserDebugText(f"Cubo: X={x_w:.3f}, Y={y_w:.3f}", [0.6, 0.3, 1.4], textColorRGB=[1, 0, 0], textSize=1.2)

sols=scara_ik_2d(x_w, y_w, L1=0.249, L2=0.24847)
print(sols)

# ---- Sliders de controle das juntas ----
slider_j1 = p.addUserDebugParameter("Junta 1", -0.75, 3.9, 1.57)
slider_j2 = p.addUserDebugParameter("Junta 2", -2.9, 2.9, 0)


# ---- Botões ----
botao_pegar = p.addUserDebugParameter("Pegar", 1, 0, 0)
botao_soltar = p.addUserDebugParameter("Soltar", 1, 0, 0)

# ---- Estado de agarre ----
grudado = False
constraint_id = None
pegando = False
soltando = False
pegou_descendo = False
soltou_descendo = False
btn_pegar_old = 0
btn_soltar_old = 0
# ---- Loop principal ----
while True:
    # Leitura dos sliders
    a1 = p.readUserDebugParameter(slider_j1)
    a2 = p.readUserDebugParameter(slider_j2)


    # Controle das juntas
    p.setJointMotorControl2(robot, j1, p.POSITION_CONTROL, targetPosition=a1, force=5)
    p.setJointMotorControl2(robot, j2, p.POSITION_CONTROL, targetPosition=a2, force=5)
    p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=0)

    # Comando de pegar
    pegar_val=p.readUserDebugParameter(botao_pegar)
    if pegar_val > 0.5 and btn_pegar_old <= 0.5 and not grudado and not pegando:
        pegando = True
        pegou_descendo = True
        print("Iniciando movimento de pegar...")

    if pegando:
        if pegou_descendo:
            # Desce a ventosa
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=-0.032)
            ee = p.getLinkState(robot, ee_link)[4]
            z_ventosa = ee[2]-0.062
            ee = p.getLinkState(robot, ee_link)[4]
            dx = abs(ee[0] - x_w)
            dy = abs(ee[1] - y_w)
            
            if dx < 0.1 and dy < 0.1:
                if abs(z_ventosa - z_topo_cubo) < 0.005:
                    ee_pos, ee_ori = p.getLinkState(robot, ee_link)[4:6]
                    cube_pos, _ = p.getBasePositionAndOrientation(cube_id)
                    p.resetBasePositionAndOrientation(cube_id, cube_pos, ee_ori)
                    print(cube_pos[1],ee_pos[1],cube_pos[0],ee_pos[0])
                    # Gruda
                    constraint_id = p.createConstraint(
                        parentBodyUniqueId=robot,
                        parentLinkIndex=ee_link,
                        childBodyUniqueId=cube_id,
                        childLinkIndex=-1,
                        jointType=p.JOINT_FIXED,
                        jointAxis=[0, 0, 0],
                        parentFramePosition=[0, 0, 0],
                        childFramePosition=[0.08,cube_pos[0] - ee_pos[0],cube_pos[1] - ee_pos[1]]
                    )
                    grudado = True
                    
                    pegou_descendo = False
                    print("Cubo agarrado. Subindo...")
            else:
                print("fora")
        else:
            # Sobe novamente
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=0)
            ee = p.getLinkState(robot, ee_link)[4]
            if ee[2] > z_topo_cubo + 0.03:
                pegando = False
                pegar_val = 0
                print("Subiu após pegar.")

    # Comando de soltar
    soltar_val=p.readUserDebugParameter(botao_soltar)
    if soltar_val > 0.5 and btn_soltar_old <= 0.5 and grudado and not soltando:
        soltando = True
        soltou_descendo = True
        print("Iniciando movimento de soltar...")

    if soltando:
        if soltou_descendo:
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=0, force=5)
            ee = p.getLinkState(robot, ee_link)[4]
            z_ventosa = ee[2] - 0.062
            if abs(z_ventosa - z_topo_cubo) < 0.005:
                # Solta
                p.removeConstraint(constraint_id)
                grudado = False
                soltou_descendo = False
                print("Cubo solto. Subindo...")
        else:
            p.setJointMotorControl2(robot, j3, p.POSITION_CONTROL, targetPosition=0, force=5)
            ee = p.getLinkState(robot, ee_link)[4]
            z_ventosa = ee[2] - 0.062
            if z_ventosa > z_topo_cubo + 0.03:
                soltando = False
                print("Subiu após soltar.")
                time.sleep(1)
                p.disconnect()
                break
    
    btn_pegar_old = pegar_val
    btn_soltar_old = soltar_val
    p.stepSimulation()
    time.sleep(dt)

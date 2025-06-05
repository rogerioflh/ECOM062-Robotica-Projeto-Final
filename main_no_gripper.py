import sim # CoppeliaSim remote API client
import time
import math
# import numpy as np # Não estritamente necessário para esta versão simplificada

# --- Simulation Configuration ---
COPPELIASIM_IP = '127.0.0.1'
COPPELIASIM_PORT = 19997 # Porta padrão

# --- Robot Configuration ---
# Nomes das juntas do robô (baseado na sua imagem da hierarquia da cena)
ROBOT_JOINT_NAMES = [
    'joint1', 'joint2', 'joint3',
    'joint4', 'joint5', 'joint6'
]
# Nome do objeto que representa o "atuador final" ou ponta da ferramenta (opcional para este script, mas bom ter)
# Baseado na sua imagem, parece que você tem um objeto 'end_effector'
END_EFFECTOR_NAME = 'end_effector' # Verifique este nome na sua cena

# --- Predefined Robot Configurations (Joint Angles in Degrees) ---
# Estes valores SÃO ILUSTRATIVOS e PRECISAM ser ajustados para sua cena e tarefa.
# Ponto A: Repouso
Q_REST = [0, 0, 0, 0, 0, 0]

# Ponto B (Local do "pegar")
Q_APPROACH_B = [0, 30, 60, 0, -90, 0]  # Aproximação do Ponto B
Q_AT_POINT_B = [0, 45, 75, 0, -75, 0]   # Posição no Ponto B

# Ponto C (Retornar para trás)
Q_RETREAT_C = [0, 30, 60, 0, -90, 0] # Posição de recuo (pode ser igual a Q_APPROACH_B)

# Ponto D (Rotacionar base à direita)
ALPHA_ROTATION_DEG = -90 # Rotação de -90 graus para a direita (ajuste conforme necessário)
Q_ROTATE_BASE_D = [ALPHA_ROTATION_DEG, 30, 60, 0, -90, 0] # q1 alterado, outros de Q_RETREAT_C

# Ponto E (Local do "soltar")
Q_APPROACH_E = [ALPHA_ROTATION_DEG, 30, 60, 0, -90, 0] # Aproximação do Ponto E
Q_AT_POINT_E = [ALPHA_ROTATION_DEG, 45, 75, 0, -75, 0] # Posição no Ponto E

# --- Helper Functions ---

def connect_to_coppeliasim():
    """Estabelece conexão com CoppeliaSim."""
    sim.simxFinish(-1) # Fecha conexões abertas
    client_id = sim.simxStart(COPPELIASIM_IP, COPPELIASIM_PORT, True, True, 5000, 5)
    if client_id != -1:
        print("Connected to CoppeliaSim")
    else:
        print("Failed to connect to CoppeliaSim")
        exit()
    return client_id

def get_object_handles(client_id):
    """Obtém handles para as juntas do robô e o efetuador final."""
    handles = {'joints': [], 'end_effector': None}
    for name in ROBOT_JOINT_NAMES:
        err_code, handle = sim.simxGetObjectHandle(client_id, name, sim.simx_opmode_blocking)
        if err_code == sim.simx_return_ok:
            handles['joints'].append(handle)
        else:
            print(f"Error getting handle for joint: {name}")
            return None # Se uma junta não for encontrada, não podemos continuar

    if END_EFFECTOR_NAME: # Tenta obter o handle do efetuador final se um nome for fornecido
        err_code, handles['end_effector'] = sim.simxGetObjectHandle(client_id, END_EFFECTOR_NAME, sim.simx_opmode_blocking)
        if err_code != sim.simx_return_ok:
            print(f"Warning: Could not get handle for end_effector: {END_EFFECTOR_NAME}. Continuing without it.")
            handles['end_effector'] = None # Não é crítico para este script sem garra
    
    if not handles['joints'] or len(handles['joints']) != len(ROBOT_JOINT_NAMES):
        print("Could not get all robot joint handles.")
        return None
        
    return handles

def set_joint_target_positions(client_id, joint_handles, target_q_deg):
    """Define as posições alvo para as juntas e espera o movimento."""
    print(f"Moving to joint configuration: {target_q_deg}")
    
    for i, handle in enumerate(joint_handles):
        target_rad = math.radians(target_q_deg[i])
        sim.simxSetJointTargetPosition(client_id, handle, target_rad, sim.simx_opmode_oneshot)

    # Espera simples para o movimento (pode ser melhorado)
    time.sleep(0.5) # Delay inicial
    while True:
        all_settled = True
        current_q_rad_temp = []
        for i, handle in enumerate(joint_handles):
            # Assegura que o streaming está ativo para leitura
            if sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_streaming) == sim.simx_return_novalue_flag:
                time.sleep(0.05) # Espera o streaming iniciar

            err_code, q_rad = sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_buffer)
            if err_code == sim.simx_return_ok:
                current_q_rad_temp.append(q_rad)
                if abs(q_rad - math.radians(target_q_deg[i])) > math.radians(2.5): # Tolerância de 2.5 graus
                    all_settled = False
                    break # Sai do loop interno se uma junta não estiver no lugar
            else:
                all_settled = False # Assume que não está estável se não conseguir ler
                break 
        
        if not current_q_rad_temp and len(joint_handles) > 0:
            sim.simxGetPingTime(client_id) # Processa comandos pendentes
            time.sleep(0.1)
            continue

        if all_settled:
            break
        time.sleep(0.1) # Intervalo de verificação
    print("Movement complete.")


# --- Main Motion Sequence Logic ---
def perform_motion_sequence(client_id, handles):
    """Executa a sequência de movimentos do robô."""

    # Ponto A: Mover para a posição de repouso inicial
    print("\nStep A: Moving to REST position.")
    set_joint_target_positions(client_id, handles['joints'], Q_REST)
    
    # Ponto B: Ir para frente (local do "pegar")
    print("\nStep B: Moving to Point B (pick location).")
    set_joint_target_positions(client_id, handles['joints'], Q_APPROACH_B)
    set_joint_target_positions(client_id, handles['joints'], Q_AT_POINT_B)
    print("At Point B.")
    time.sleep(1.0) # Pausa no ponto B

    # Ponto C: Voltar para trás
    print("\nStep C: Retreating to Point C.")
    set_joint_

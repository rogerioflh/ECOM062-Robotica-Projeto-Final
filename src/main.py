import sim # CoppeliaSim remote API client
# import simConst # simConstants are usually available under sim.simx_opmode_blocking, etc.
import time
import math

# --- Simulation Configuration ---
COPPELIASIM_IP = '127.0.0.1'
COPPELIASIM_PORT = 19997 # Default port

# --- Robot and Gripper Configuration (VERIFIQUE E AJUSTE ESTES VALORES!) ---
# Nomes das juntas do robô (confirmado pela hierarquia da cena)
ROBOT_JOINT_NAMES = [
    'joint1', 'joint2', 'joint3',
    'joint4', 'joint5', 'joint6'
]
# Nome da junta da garra (identificado na hierarquia da cena como 'Revolute_joint' sob 'end_effector')
GRIPPER_JOINT_NAME = 'Revolute_joint'
# !!! IMPORTANTE: Determine estes ângulos manualmente no CoppeliaSim !!!
# Mova 'Revolute_joint' para encontrar os valores corretos para abrir/fechar a garra.
GRIPPER_OPEN_POS_DEG = 0    # Ângulo da junta da garra para estado aberto (graus) - AJUSTE ESTE VALOR
GRIPPER_CLOSED_POS_DEG = 35 # Ângulo da junta da garra para estado fechado (graus) - AJUSTE ESTE VALOR (para segurar 'goal')

# Nome do objeto alvo (identificado nas propriedades do objeto como 'goal')
TARGET_OBJECT_NAME = 'goal'
# Nome do TCP (Tool Center Point - Ponto Central da Ferramenta)
# Usando 'end_effector' que é um dummy na ponta do robô, conforme hierarquia.
GRIPPER_TCP_NAME = 'end_effector'

# --- Predefined Robot Configurations (Joint Angles in Degrees) ---
# !!! CRÍTICO: Estes valores de Q (ângulos das juntas) SÃO ILUSTRATIVOS e DEVEM SER AJUSTADOS
# para o seu robô específico, a posição do objeto 'goal' (x:0.625, y:0.050, z:0.200 conforme imagem),
# e os locais desejados para pegar e soltar.
# Use o CoppeliaSim para encontrar estes ângulos movendo o robô manualmente.

# Ponto A: Repouso (Exemplo: uma posição inicial segura)
Q_REST = [0, 0, 0, 0, 0, 0] # AJUSTE ESTE VALOR

# Ponto B (Pegar Objeto - 'goal' em x:0.625, y:0.050, z:0.200)
# Estes Qs devem levar o 'end_effector' (TCP) até o objeto 'goal'.
Q_APPROACH_PICK = [0, 15, 45, 0, -60, 0]  # AJUSTE: Aproximação acima do 'goal'
Q_GRASP_OBJECT = [0, 25, 55, 0, -70, 0]   # AJUSTE: Posição de agarre para o 'goal'

# Ponto C (Recuar/subir com o objeto após pegar)
Q_RETREAT_AFTER_PICK = [0, 15, 45, 0, -60, 0] # AJUSTE: Mover para cima/trás após pegar

# Ponto D (Local de Soltar - Exemplo: Rotacionar a base e mover para um novo local)
# Defina uma rotação para q1 se necessário, ex: ALPHA_ROTATION_DEG = 90
Q_APPROACH_RELEASE = [-90, 15, 45, 0, -60, 0] # AJUSTE: Aproximação acima do ponto de soltar
Q_RELEASE_OBJECT = [-90, 25, 55, 0, -70, 0]   # AJUSTE: Posição para soltar

# Ponto E (Recuar para uma posição segura ou repouso após soltar)
Q_POST_RELEASE_RETREAT = [-90, 0, 0, 0, 0, 0] # AJUSTE

# --- Helper Functions ---

def connect_to_coppeliasim():
    """Estabelece conexão com CoppeliaSim."""
    sim.simxFinish(-1) # Fecha todas as conexões abertas, por precaução
    client_id = sim.simxStart(COPPELIASIM_IP, COPPELIASIM_PORT, True, True, 5000, 5)
    if client_id != -1:
        print("Conectado ao CoppeliaSim")
    else:
        print("Falha ao conectar ao CoppeliaSim. Verifique se o CoppeliaSim está rodando e o servidor da API remota está iniciado.")
        exit()
    return client_id

def get_object_handles(client_id):
    """Obtém handles para as juntas do robô, garra, objeto alvo e TCP."""
    handles = {'joints': [], 'gripper_joint': None, 'target_object': None, 'tcp': None}
    print("Obtendo handles dos objetos...")
    for name in ROBOT_JOINT_NAMES:
        err_code, handle = sim.simxGetObjectHandle(client_id, name, sim.simx_opmode_blocking)
        if err_code == sim.simx_return_ok:
            handles['joints'].append(handle)
            # print(f"  Handle obtido para junta: {name}")
        else:
            print(f"  Erro ao obter handle para junta {name}: {err_code}")
            return None

    err_code, handles['gripper_joint'] = sim.simxGetObjectHandle(client_id, GRIPPER_JOINT_NAME, sim.simx_opmode_blocking)
    if err_code == sim.simx_return_ok:
        # print(f"  Handle obtido para junta da garra: {GRIPPER_JOINT_NAME}")
        pass
    else:
        print(f"  Erro ao obter handle para junta da garra {GRIPPER_JOINT_NAME}: {err_code}")
        return None

    err_code, handles['target_object'] = sim.simxGetObjectHandle(client_id, TARGET_OBJECT_NAME, sim.simx_opmode_blocking)
    if err_code == sim.simx_return_ok:
        # print(f"  Handle obtido para objeto alvo: {TARGET_OBJECT_NAME}")
        pass
    else:
        print(f"  Erro ao obter handle para objeto alvo {TARGET_OBJECT_NAME}: {err_code}")
        return None

    err_code, handles['tcp'] = sim.simxGetObjectHandle(client_id, GRIPPER_TCP_NAME, sim.simx_opmode_blocking)
    if err_code == sim.simx_return_ok:
        # print(f"  Handle obtido para TCP: {GRIPPER_TCP_NAME}")
        pass
    else:
        print(f"  Erro ao obter handle para TCP {GRIPPER_TCP_NAME}: {err_code}. Verifique o nome.")
        return None
        
    if len(handles['joints']) != len(ROBOT_JOINT_NAMES):
        print("Não foi possível obter todos os handles das juntas do robô.")
        return None
    print("Todos os handles necessários foram obtidos.")
    return handles

def set_joint_target_positions(client_id, joint_handles, target_q_deg, wait_time=2.5):
    """Define as posições alvo para múltiplas juntas e espera um tempo fixo."""
    print(f"Movendo para configuração de juntas (graus): {target_q_deg}")
    for i, handle in enumerate(joint_handles):
        target_rad = math.radians(target_q_deg[i])
        sim.simxSetJointTargetPosition(client_id, handle, target_rad, sim.simx_opmode_oneshot)

    # Espera simples para o movimento. Para movimentos mais robustos,
    # seria ideal verificar se as juntas atingiram o alvo.
    time.sleep(wait_time) # Ajuste wait_time conforme a velocidade do robô e distância.
    # print("Movimento (presumidamente) completo.")

def control_gripper(client_id, gripper_joint_handle, action, open_pos_deg, closed_pos_deg, wait_time=1.5):
    """Controla a garra para abrir ou fechar."""
    target_pos_rad = 0
    if action == "open":
        target_pos_rad = math.radians(open_pos_deg)
        print(f"Abrindo garra para {open_pos_deg} graus.")
    elif action == "close":
        target_pos_rad = math.radians(closed_pos_deg)
        print(f"Fechando garra para {closed_pos_deg} graus.")
    else:
        print(f"Ação inválida para garra: {action}")
        return

    sim.simxSetJointTargetPosition(client_id, gripper_joint_handle, target_pos_rad, sim.simx_opmode_oneshot)
    time.sleep(wait_time) # Espera pela ação da garra
    # print(f"Ação da garra '{action}' completa.")


def attach_object_to_gripper(client_id, tcp_handle, object_handle):
    """Anexa o objeto alvo ao TCP da garra."""
    print(f"Anexando objeto (handle {object_handle}) ao TCP (handle {tcp_handle}).")
    # Define o TCP como pai do objeto, mantendo a transformação atual do objeto.
    err_code = sim.simxSetObjectParent(client_id, object_handle, tcp_handle, True, sim.simx_opmode_oneshot_wait)
    if err_code == sim.simx_return_ok or err_code == sim.simx_return_novalue_flag:
        print("Objeto anexado com sucesso.")
    else:
        print(f"Erro ao anexar objeto: {err_code}")


def detach_object_from_gripper(client_id, object_handle, scene_root_handle=-1): # -1 para a raiz da cena (mundo)
    """Desanexa o objeto da garra, tornando-o filho da raiz da cena."""
    print(f"Desanexando objeto (handle {object_handle}).")
    # Define a raiz da cena como pai, mantendo a transformação atual.
    err_code = sim.simxSetObjectParent(client_id, object_handle, scene_root_handle, True, sim.simx_opmode_oneshot_wait)
    if err_code == sim.simx_return_ok or err_code == sim.simx_return_novalue_flag:
        print("Objeto desanexado com sucesso.")
    else:
        print(f"Erro ao desanexar objeto: {err_code}")

# --- Main Pick and Place Logic ---
def perform_pick_and_place(client_id, handles):
    """Executa a sequência completa de pegar e soltar."""
    print("\n--- Iniciando Sequência de Pegar e Soltar ---")

    # 0. Garantir que a garra está aberta inicialmente
    control_gripper(client_id, handles['gripper_joint'], "open", GRIPPER_OPEN_POS_DEG, GRIPPER_CLOSED_POS_DEG)

    # 1. Mover para Posição de Repouso (Ponto A)
    print("\nPasso 1: Movendo para posição de repouso (Q_REST).")
    set_joint_target_positions(client_id, handles['joints'], Q_REST, wait_time=3)
    time.sleep(1)

    # 2. Aproximar do Local de Pegar (Ponto B - Aproximação)
    print("\nPasso 2: Aproximando do local de pegar (Q_APPROACH_PICK).")
    set_joint_target_positions(client_id, handles['joints'], Q_APPROACH_PICK, wait_time=3)
    time.sleep(0.5)

    # 3. Mover para Posição de Agarre (Ponto B - Agarre)
    print("\nPasso 3: Movendo para posição de agarre (Q_GRASP_OBJECT).")
    set_joint_target_positions(client_id, handles['joints'], Q_GRASP_OBJECT, wait_time=2)
    time.sleep(0.5)

    # 4. Fechar Garra (Pegar)
    print("\nPasso 4: Fechando garra para pegar objeto.")
    control_gripper(client_id, handles['gripper_joint'], "close", GRIPPER_OPEN_POS_DEG, GRIPPER_CLOSED_POS_DEG)
    
    # 5. Anexar Objeto à Garra
    print("\nPasso 5: Anexando objeto.")
    attach_object_to_gripper(client_id, handles['tcp'], handles['target_object'])
    time.sleep(0.5) # Pequena pausa para estabilizar a física da anexação

    # 6. Recuar Após Pegar (Ponto C)
    print("\nPasso 6: Recuando após pegar (Q_RETREAT_AFTER_PICK).")
    set_joint_target_positions(client_id, handles['joints'], Q_RETREAT_AFTER_PICK, wait_time=3)
    time.sleep(1)

    # 7. Aproximar do Local de Soltar (Ponto D - Aproximação)
    print("\nPasso 7: Aproximando do local de soltar (Q_APPROACH_RELEASE).")
    set_joint_target_positions(client_id, handles['joints'], Q_APPROACH_RELEASE, wait_time=3)
    time.sleep(0.5)

    # 8. Mover para Posição de Soltar (Ponto D - Soltar)
    print("\nPasso 8: Movendo para posição de soltar (Q_RELEASE_OBJECT).")
    set_joint_target_positions(client_id, handles['joints'], Q_RELEASE_OBJECT, wait_time=2)
    time.sleep(0.5)

    # 9. Abrir Garra (Soltar)
    print("\nPasso 9: Abrindo garra para soltar objeto.")
    control_gripper(client_id, handles['gripper_joint'], "open", GRIPPER_OPEN_POS_DEG, GRIPPER_CLOSED_POS_DEG)

    # 10. Desanexar Objeto da Garra
    print("\nPasso 10: Desanexando objeto.")
    detach_object_from_gripper(client_id, handles['target_object']) # Desanexa para o mundo
    time.sleep(0.5)

    # 11. Recuar após soltar / Mover para uma posição pós-soltar (Ponto E)
    print("\nPasso 11: Recuando após soltar (Q_POST_RELEASE_RETREAT).")
    set_joint_target_positions(client_id, handles['joints'], Q_POST_RELEASE_RETREAT, wait_time=3)
    time.sleep(1)

    # 12. Opcional: mover de volta para Repouso
    print("\nPasso 12: Movendo de volta para posição de repouso (Q_REST).")
    set_joint_target_positions(client_id, handles['joints'], Q_REST, wait_time=3)

    print("\n--- Sequência de Pegar e Soltar Completa ---")

# --- Script Execution ---
if __name__ == "__main__":
    client_id = connect_to_coppeliasim()
    if client_id != -1:
        try:
            # Iniciar simulação (importante se ainda não estiver rodando e você precisa de física/dinâmica)
            res = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot_wait)
            # res pode ser sim.simx_return_ok ou sim.simx_return_novalue_flag se já estiver rodando
            if res == sim.simx_return_ok or res == sim.simx_return_novalue_flag or res == 8: # 8 = simx_return_initialize_error_flag (mas pode significar que já está rodando)
                print("Simulação iniciada ou já estava rodando.")
            else:
                print(f"Falha ao iniciar simulação: {res}. Pode já estar rodando.")
            
            time.sleep(1) # Dar um momento para a simulação estabilizar após iniciar

            handles = get_object_handles(client_id)
            if handles:
                # Opcional: Obter posição inicial do objeto alvo para confirmação
                err_code, obj_pos = sim.simxGetObjectPosition(client_id, handles['target_object'], -1, sim.simx_opmode_blocking)
                if err_code == sim.simx_return_ok:
                    print(f"Posição inicial do objeto alvo '{TARGET_OBJECT_NAME}': {obj_pos}")
                
                input("Pressione Enter para iniciar a sequência de pegar e soltar...")
                perform_pick_and_place(client_id, handles)
            else:
                print("Não foi possível obter todos os handles necessários. Saindo.")

        except Exception as e:
            print(f"Ocorreu um erro: {e}")
        finally:
            # Parar simulação (opcional, pode querer deixar rodando)
            # sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot_wait)
            # print("Simulação parada.")
            # Fechar a conexão com o CoppeliaSim
            sim.simxFinish(client_id)
            print("Conexão com CoppeliaSim fechada.")
    else:
        print("Não foi possível conectar ao CoppeliaSim.")
import numpy as np

def compute_jacobian_numerical(fk_func, q, delta=1e-6):
    """
    Cálculo numérico da Jacobiana usando diferenças finitas.

    fk_func: função que retorna a pose do efetuador para uma dada configuração q.
    q: configuração articular atual (numpy array)
    delta: incremento pequeno
    """
    n = len(q)
    fk0 = fk_func(q)
    J = np.zeros((len(fk0), n))
    for i in range(n):
        dq = np.zeros(n)
        dq[i] = delta
        fk1 = fk_func(q + dq)
        J[:, i] = (fk1 - fk0) / delta
    return J

def control_step(fk_func, q_current, pose_goal, gain=1.0):
    """
    Retorna deslocamento articular (velocidade) baseado em erro de pose e Jacobiana.

    fk_func: função FK (q -> pose)
    q_current: configuração articular atual
    pose_goal: pose desejada (numpy array, mesmo formato de saída da FK)
    gain: ganho de correção
    """
    pose_current = fk_func(q_current)
    error = pose_goal - pose_current
    J = compute_jacobian_numerical(fk_func, q_current)
    J_pinv = np.linalg.pinv(J)
    dq = gain * J_pinv @ error
    return dq

import numpy as np

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def fk_func(q):
    # Assumindo q = [q1, q2, q3, q4, q5, q6] em radianos
    dh = [
        (q[0], 0,   0,    np.pi/2),
        (q[1], 0, 260,    0),
        (q[2], 0,  20,    np.pi/2),
        (q[3], 250, 0,   -np.pi/2),
        (q[4], 0,   0,    np.pi/2),
        (q[5], 90,  0,    0)
    ]

    T = np.eye(4)
    for (theta, d, a, alpha) in dh:
        T = T @ dh_transform(theta, d, a, alpha)

    position = T[0:3, 3]
    # extração simples de orientação em XYZ Euler
    r = np.arctan2(T[2, 1], T[2, 2])
    p = np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2))
    y = np.arctan2(T[1, 0], T[0, 0])
    orientation = np.array([r, p, y])  # em rad
    return np.concatenate((position, orientation))  # (6,)

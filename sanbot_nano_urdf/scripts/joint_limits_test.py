#!/usr/bin/env python3
import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Definição dos limites das juntas baseado no URDF
JOINT_LIMITS = {
    'wing_left': {'lower': -1.5708, 'upper': 3.1416, 'name': 'Asa Esquerda'},
    'wing_right': {'lower': -1.5708, 'upper': 3.1416, 'name': 'Asa Direita'},
    'neck': {'lower': -1.5708, 'upper': 1.5708, 'name': 'Pescoço (Pan)'},
    'head': {'lower': 0.0, 'upper': 0.656, 'name': 'Cabeça (Tilt)'}
}

def test_all_joint_limits():
    """Testa os limites de todas as 4 juntas"""
    rospy.init_node('joint_limits_test')
    
    print("=== TESTE DE LIMITES DE TODAS AS JUNTAS ===")
    print("Iniciando testes de limites...")
    
    # Teste 1: Asa Esquerda
    print("\n1. TESTE DA ASA ESQUERDA")
    test_single_joint_limits('wing_left', '/wing_left_controller/command')
    
    # Teste 2: Asa Direita
    print("\n2. TESTE DA ASA DIREITA")
    test_single_joint_limits('wing_right', '/wing_right_controller/command')
    
    # Teste 3: Pescoço (isolado)
    print("\n3. TESTE DO PESCOÇO (ISOLADO)")
    test_single_joint_limits('neck', '/head_controller/command', is_head_joint=True)
    
    # Teste 4: Cabeça (isolado)
    print("\n4. TESTE DA CABEÇA (ISOLADO)")
    test_single_joint_limits('head', '/head_controller/command', is_head_joint=True)
    
    # Teste 5: Cabeça combinada (pan + tilt nos limites)
    print("\n5. TESTE DA CABEÇA COMBINADA (LIMITES MÁXIMOS)")
    test_head_combined_limits()
    
    # Teste 6: Cabeça combinada (posições intermediárias)
    print("\n6. TESTE DA CABEÇA COMBINADA (POSIÇÕES INTERMEDIÁRIAS)")
    test_head_combined_intermediate()
    
    print("\n=== TODOS OS TESTES CONCLUÍDOS ===")

def test_single_joint_limits(joint_name, topic, is_head_joint=False):
    """Testa os limites de uma junta individual"""
    pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)
    rospy.sleep(1)
    
    limits = JOINT_LIMITS[joint_name]
    print(f"Testando {limits['name']}: {math.degrees(limits['lower']):.1f}° a {math.degrees(limits['upper']):.1f}°")
    
    # Posição neutra (meio do range)
    neutral = (limits['lower'] + limits['upper']) / 2
    move_to_position(joint_name, neutral, 3.0, pub, is_head_joint)
    rospy.sleep(4)
    
    # Limite inferior
    print(f"  → Movendo para limite inferior: {math.degrees(limits['lower']):.1f}°")
    move_to_position(joint_name, limits['lower'], 4.0, pub, is_head_joint)
    rospy.sleep(5)
    
    # Volta para neutro
    move_to_position(joint_name, neutral, 3.0, pub, is_head_joint)
    rospy.sleep(4)
    
    # Limite superior
    print(f"  → Movendo para limite superior: {math.degrees(limits['upper']):.1f}°")
    move_to_position(joint_name, limits['upper'], 4.0, pub, is_head_joint)
    rospy.sleep(5)
    
    # Volta para neutro
    move_to_position(joint_name, neutral, 3.0, pub, is_head_joint)
    rospy.sleep(4)

def test_head_combined_limits():
    """Testa a cabeça nos limites máximos combinados"""
    pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)
    
    neck_limits = JOINT_LIMITS['neck']
    head_limits = JOINT_LIMITS['head']
    
    print("Testando combinações de limites máximos:")
    
    # Teste 1: Pan máximo + Tilt máximo
    print("  → Pan máximo + Tilt máximo")
    move_head_combined(neck_limits['upper'], head_limits['upper'], 5.0, pub)
    rospy.sleep(6)
    
    # Teste 2: Pan máximo + Tilt mínimo
    print("  → Pan máximo + Tilt mínimo")
    move_head_combined(neck_limits['upper'], head_limits['lower'], 5.0, pub)
    rospy.sleep(6)
    
    # Teste 3: Pan mínimo + Tilt máximo
    print("  → Pan mínimo + Tilt máximo")
    move_head_combined(neck_limits['lower'], head_limits['upper'], 5.0, pub)
    rospy.sleep(6)
    
    # Teste 4: Pan mínimo + Tilt mínimo
    print("  → Pan mínimo + Tilt mínimo")
    move_head_combined(neck_limits['lower'], head_limits['lower'], 5.0, pub)
    rospy.sleep(6)
    
    # Volta para posição neutra
    neutral_neck = (neck_limits['lower'] + neck_limits['upper']) / 2
    neutral_head = (head_limits['lower'] + head_limits['upper']) / 2
    move_head_combined(neutral_neck, neutral_head, 4.0, pub)
    rospy.sleep(5)

def test_head_combined_intermediate():
    """Testa a cabeça em posições intermediárias"""
    pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)
    
    neck_limits = JOINT_LIMITS['neck']
    head_limits = JOINT_LIMITS['head']
    
    print("Testando posições intermediárias:")
    
    # Posições intermediárias
    neck_positions = [neck_limits['lower'] * 0.5, 0, neck_limits['upper'] * 0.5]
    head_positions = [head_limits['lower'] + 0.1, head_limits['upper'] * 0.5, head_limits['upper'] - 0.1]
    
    for i, neck_pos in enumerate(neck_positions):
        for j, head_pos in enumerate(head_positions):
            print(f"  → Posição {i*3 + j + 1}: Pan={math.degrees(neck_pos):.1f}°, Tilt={math.degrees(head_pos):.1f}°")
            move_head_combined(neck_pos, head_pos, 3.0, pub)
            rospy.sleep(4)
    
    # Volta para posição neutra
    neutral_neck = (neck_limits['lower'] + neck_limits['upper']) / 2
    neutral_head = (head_limits['lower'] + head_limits['upper']) / 2
    move_head_combined(neutral_neck, neutral_head, 3.0, pub)
    rospy.sleep(4)

def move_to_position(joint_name, position, duration, pub, is_head_joint=False):
    """Move uma junta para uma posição específica"""
    traj = JointTrajectory()
    
    if is_head_joint:
        # Para juntas da cabeça, precisa especificar ambas
        if joint_name == 'neck':
            traj.joint_names = ['neck', 'head']
            pt = JointTrajectoryPoint()
            pt.positions = [position, 0.0]  # Mantém head em 0
            pt.velocities = [0.0, 0.0]
            pt.time_from_start = rospy.Duration(duration)
        else:  # head
            traj.joint_names = ['neck', 'head']
            pt = JointTrajectoryPoint()
            pt.positions = [0.0, position]  # Mantém neck em 0
            pt.velocities = [0.0, 0.0]
            pt.time_from_start = rospy.Duration(duration)
    else:
        # Para outras juntas
        traj.joint_names = [joint_name]
        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.velocities = [0.0]
        pt.time_from_start = rospy.Duration(duration)
    
    traj.points.append(pt)
    pub.publish(traj)

def move_head_combined(neck_angle, head_angle, duration, pub):
    """Move a cabeça para uma posição combinada"""
    traj = JointTrajectory()
    traj.joint_names = ['neck', 'head']
    
    pt = JointTrajectoryPoint()
    pt.positions = [neck_angle, head_angle]
    pt.velocities = [0.0, 0.0]
    pt.time_from_start = rospy.Duration(duration)
    
    traj.points.append(pt)
    pub.publish(traj)

def print_joint_info():
    """Imprime informações sobre todas as juntas"""
    print("=== INFORMAÇÕES DAS JUNTAS ===")
    for joint, limits in JOINT_LIMITS.items():
        print(f"{limits['name']} ({joint}):")
        print(f"  Limites: {math.degrees(limits['lower']):.1f}° a {math.degrees(limits['upper']):.1f}°")
        print(f"  Range: {math.degrees(limits['upper'] - limits['lower']):.1f}°")
        print()

if __name__ == '__main__':
    try:
        print_joint_info()
        test_all_joint_limits()
    except rospy.ROSInterruptException:
        print("Teste interrompido pelo usuário") 
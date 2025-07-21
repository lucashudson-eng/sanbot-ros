# Sanbot Nano URDF – Guia Rápido de Testes

Este guia explica como publicar comandos nos tópicos ROS já configurados no *sanbot_nano_urdf* para testar rapidamente as funcionalidades disponíveis: duas juntas da cabeça, duas asas e visualização da câmera.

---

## 1. Pré-requisitos

1. Compile e fonte o seu espaço de trabalho:
   ```bash
   cd ~/catkin_ws && catkin build
   source devel/setup.bash
   ```
2. Lance o robô (simulação ou gazebo) em um novo terminal:
   ```bash
   roslaunch sanbot_nano_urdf sanbot_nano.launch
   ```
3. Em outro terminal fonte novamente o workspace antes de publicar comandos.

---

## 2. Conversão rápida de graus → radianos
A maior parte dos exemplos usa radianos. Para converter, basta lembrar:

| Graus | Radianos |
|-----:|----------:|
| 30°  | 0,524 | 
| 45°  | 0,785 |
| 90°  | 1,571 |

Use `python -c 'import math; print(math.radians(GRAUS))'` caso precise de outros valores.

---

## 3. Controle da cabeça (`neck` e `head`)

O controlador é `JointTrajectory`. Mesmo que queira mover apenas uma junta, publique as duas na mesma mensagem. Exemplo para inclinar a cabeça **+30°** (0,524 rad) mantendo o pescoço parado:

```bash
rostopic pub -1 /head_controller/command trajectory_msgs/JointTrajectory \
"header:
  stamp: {secs: 0, nsecs: 0}   # carimbo de tempo será ajustado para 'now'
  frame_id: ''
joint_names: ['neck', 'head']
points:
- positions: [0.0, 0.523599]   # [neck, head]
  velocities: [0.0, 0.0]
  time_from_start: {secs: 2, nsecs: 0}"
```

Outros testes rápidos:

| Ação | `positions` | Observação |
|------|-------------|-----------|
| Pescoço +45° | `[0.785, 0.0]` | head fica zerada |
| Pan +45° & Tilt +20° | `[0.785, 0.349]` | movimento combinado |
| Voltar para neutro | `[0.0, 0.0]` | |

> ⚠️ Respeite os limites definidos em `scripts/joint_limits_test.py`:
> * `neck`: ±90°
> * `head`: 0° a +37,6°

---

## 4. Controle das asas

### 4.1 Asa Esquerda (`wing_left`)
```bash
rostopic pub -1 /wing_left_controller/command trajectory_msgs/JointTrajectory \
"header:
  stamp: {secs: 0, nsecs: 0}
joint_names: ['wing_left']
points:
- positions: [1.0]        # ~57° para cima
  velocities: [0.0]
  time_from_start: {secs: 2, nsecs: 0}"
```

### 4.2 Asa Direita (`wing_right`)
```bash
rostopic pub -1 /wing_right_controller/command trajectory_msgs/JointTrajectory \
"header:
  stamp: {secs: 0, nsecs: 0}
joint_names: ['wing_right']
points:
- positions: [1.0]
  velocities: [0.0]
  time_from_start: {secs: 2, nsecs: 0}"
```

*Para retornar ao repouso use `positions: [0.0]`.*

---

## 5. Visualização da câmera

1. Verifique o tópico disponível (exemplo):
   ```bash
   rostopic list | grep image
   # → /camera/image_raw
   ```
2. Abra a imagem em tempo real com **rqt_image_view**:
   ```bash
   rqt_image_view /camera/image_raw
   ```
   ou via linha de comando:
   ```bash
   rosrun image_view image_view image:=/camera/image_raw
   ```

---

## 6. Dicas úteis

• Publique repetidamente com `-r 10` para comandos contínuos (10 Hz).  
• Use `rostopic echo /joint_states` para acompanhar o ângulo real das juntas.  
• Em simulação Gazebo, abra o `gzclient` para ver a movimentação física.

---

Boa diversão testando o Sanbot Nano! 🚀 
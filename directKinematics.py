"""
Controle do Braço Robótico UR3 no CoppeliaSim
---------------------------------------------

Este script foi desenvolvido como parte do trabalho final da disciplina de Fundamentos de Robótica, 
ministrada pelo professor Roberto Baptista na Universidade de Brasília (UnB), campus Gama (FGA), 
no primeiro semestre do ano de 2024.

Autores: Matheus de Sousa Luiz (170111032)
         Nathan Spinola Zeidan (180025864)
Apresentado em: 12/09/2024

Descrição:
----------
Este script fornece uma interface para controlar o braço robótico UR3 usando o ambiente de simulação CoppeliaSim.
Ele permite o controle das juntas, cálculos de cinemática direta e inversa, e a interação com o efetuador do robô.

Funcionalidades:
----------------
- Conecta-se ao CoppeliaSim usando a API remota em uma porta especificada.
- Define e recupera as posições das juntas do robô UR3.
- Calcula a cinemática direta usando os parâmetros de Denavit-Hartenberg.
- Resolve a cinemática inversa para mover o efetuador para uma posição desejada.

Bibliotecas e Dependências:
----------------------------
- API Remota do CoppeliaSim (importada como `sim`).
- NumPy (para cálculos matemáticos).
- Robotics Toolbox for Python (para cinemática inversa).
- SpatialMath (para lidar com transformações 3D).

Classes:
--------
- UR3: Uma classe que representa o braço robótico UR3, fornecendo métodos para controlar e recuperar seu estado.

Métodos:
--------
- connect_coppelia: Estabelece a conexão com o servidor CoppeliaSim e recupera os identificadores das juntas.
- set_joint_position: Define os ângulos das juntas do robô UR3.
- get_joint_position: Recupera os ângulos atuais das juntas.
- get_effector_position: Obtém a posição atual do efetuador.
- dh_transform: Calcula a matriz de transformação de Denavit-Hartenberg para uma junta.
- forward_kinematics: Calcula a posição do efetuador usando os ângulos das juntas.
- calculate_effector_position: Calcula a posição do efetuador através da cinemática direta.
- inverse_kinematics: Resolve a cinemática inversa para mover o efetuador para uma posição alvo.

Uso:
----
1. Certifique-se de que a  API remota (sim.py, simConst.py e remoteApi.dll) está presente no
   mesmo diretório deste arquivo;
2. Certifique-se de que o cenário 'cenarioUR3.ttt' está em execução no CoppeliaSim; 
3. Execute o script, que se conecta ao CoppeliaSim e controla o robô UR3 por meio das posições 
   das juntas ou comandos do efetuador.

Exemplo:
--------
if __name__ == "__main__":
    ur3 = UR3(19999)  # Conecta-se ao CoppeliaSim na porta 19999
    ur3.set_joint_position([0, 0, 0, 0, 0, 0])  # Move o robô para sua posição inicial
    print(ur3.get_effector_position())  # Imprime a posição do efetuador

"""

import sim  # Importing CoppeliaSim's remote API for controlling the simulation
import time  # For adding delays in execution
from numpy import pi  # Importing pi constant from numpy
import numpy as np  # Importing numpy for mathematical operations
import roboticstoolbox as rtb  # For robotic models and inverse kinematics
from spatialmath import SE3  # For spatial transformations and homogeneous transformations

class UR3():
    """
    Class representing a UR3 robotic arm, interfacing with the CoppeliaSim simulation environment.
    """

    def __init__(self, port=19999):
        """
        Initializes the UR3 object by connecting to the CoppeliaSim server and obtaining joint handles.
        
        :param port: The port number for connecting to the CoppeliaSim server. Default is 19999.
        """
        self.clientID, self.junta1, self.junta2, self.junta3, self.junta4, self.junta5, self.junta6, self.effector = self.connect_coppelia(port)

    def connect_coppelia(self, port):
        """
        Establishes connection with the CoppeliaSim server and retrieves the handles of the robot joints and end-effector.

        :param port: The port number for the CoppeliaSim server.
        :return: clientID and object handles for joints and end-effector.
        """
        sim.simxFinish(-1)  # Closes any previous connections
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)  # Starts a connection to CoppeliaSim
        
        if clientID == 0:
            print(f"------------------------ Conectado ao CoppeliaSim via porta: {port} ------------------------------")
            
            # Retrieve handles for each joint and the end-effector
            _, junta1 = sim.simxGetObjectHandle(clientID, 'UR3_joint1', sim.simx_opmode_blocking)
            _, junta2 = sim.simxGetObjectHandle(clientID, 'UR3_joint2', sim.simx_opmode_blocking)
            _, junta3 = sim.simxGetObjectHandle(clientID, 'UR3_joint3', sim.simx_opmode_blocking)
            _, junta4 = sim.simxGetObjectHandle(clientID, 'UR3_joint4', sim.simx_opmode_blocking)
            _, junta5 = sim.simxGetObjectHandle(clientID, 'UR3_joint5', sim.simx_opmode_blocking)
            _, junta6 = sim.simxGetObjectHandle(clientID, 'UR3_joint6', sim.simx_opmode_blocking)
            _, endEffector = sim.simxGetObjectHandle(clientID, 'UR3_link7_visible', sim.simx_opmode_blocking)
            
            return clientID, junta1, junta2, junta3, junta4, junta5, junta6, endEffector
        else:
            raise Exception("Não foi possivel conectar ao CoppeliaSim \nTente iniciar a simulacao")

    def set_joint_position(self, positions):
        """
        Sets the target position for each joint of the UR3 robot.

        :param positions: A list of target joint positions in radians [j1, j2, j3, j4, j5, j6].
        """
        sim.simxSetJointTargetPosition(self.clientID, self.junta1, positions[0], sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID, self.junta2, positions[1], sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID, self.junta3, positions[2], sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID, self.junta4, positions[3], sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID, self.junta5, positions[4], sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID, self.junta6, positions[5], sim.simx_opmode_oneshot)

    def get_joint_position(self):
        """
        Retrieves the current position of each joint.

        :return: A tuple of joint positions [j1, j2, j3, j4, j5, j6].
        """
        _, position1 = sim.simxGetJointPosition(self.clientID, self.junta1, sim.simx_opmode_streaming)
        _, position2 = sim.simxGetJointPosition(self.clientID, self.junta2, sim.simx_opmode_streaming)
        _, position3 = sim.simxGetJointPosition(self.clientID, self.junta3, sim.simx_opmode_streaming)
        _, position4 = sim.simxGetJointPosition(self.clientID, self.junta4, sim.simx_opmode_streaming)
        _, position5 = sim.simxGetJointPosition(self.clientID, self.junta5, sim.simx_opmode_streaming)
        _, position6 = sim.simxGetJointPosition(self.clientID, self.junta6, sim.simx_opmode_streaming)
        
        return position1, position2, position3, position4, position5, position6

    def get_effector_position(self, relative=-1):
        """
        Retrieves the current position of the end-effector.

        :param relative: Specifies the reference frame for the position. Default is -1 (absolute frame).
        :return: The position of the end-effector as [x, y, z].
        """
        _, position = sim.simxGetObjectPosition(self.clientID, self.effector, relative, sim.simx_opmode_streaming)
        
        # If position is [0,0,0], keep trying until valid position is obtained
        while position == [0, 0, 0]:
            _, position = sim.simxGetObjectPosition(self.clientID, self.effector, relative, sim.simx_opmode_blocking)
        
        return position

    def dh_transform(self, theta, d, a, alpha):
        """
        Computes the Denavit-Hartenberg (DH) transformation matrix for a given joint.

        :param theta: Joint angle in radians.
        :param d: Link offset along the previous z-axis.
        :param a: Link length along the previous x-axis.
        :param alpha: Twist angle between the z-axes of two consecutive links.
        :return: 4x4 transformation matrix as a numpy array.
        """
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0,             np.sin(alpha),                 np.cos(alpha),                d],
            [0,             0,                             0,                            1]
        ])

    def forward_kinematics(self, theta1, theta2, theta3, theta4, theta5, theta6):
        """
        Computes the forward kinematics of the robot using DH parameters to get the end-effector position.

        :param theta1 to theta6: Joint angles in radians.
        :return: The position of the end-effector as [x, y, z].
        """
        DH_params = [
            (theta1, 0.1519, 0, np.pi / 2),  # DH parameters for joint 1
            (theta2, 0, -0.24365, 0),       # DH parameters for joint 2
            (theta3, 0, -0.21325, 0),       # DH parameters for joint 3
            (theta4, 0.11235, 0, np.pi / 2),# DH parameters for joint 4
            (theta5, 0.08535, 0, -np.pi / 2),# DH parameters for joint 5
            (theta6, 0.0819, 0, 0)          # DH parameters for joint 6
        ]

        T = np.eye(4)  # Identity matrix to start transformation
        
        # Compute the overall transformation matrix by multiplying the transformations for each joint
        for theta, d, a, alpha in DH_params:
            T = np.dot(T, self.dh_transform(theta, d, a, alpha))

        return T[:3, 3]  # Extract and return the position [x, y, z] from the transformation matrix

    def calculate_effector_position(self):
        """
        Uses the joint angles to compute the end-effector position via forward kinematics.

        :return: The calculated end-effector position [x, y, z].
        """
        joint_angles = self.get_joint_position()  # Get the current joint angles
        effector_position = self.forward_kinematics(*joint_angles)  # Calculate the effector position
        return effector_position

    def inverse_kinematics(self, position):
        """
        Solves inverse kinematics to compute the joint configuration required to reach the specified position.

        :param position: Desired end-effector position as [x, y, z].
        """
        model = rtb.models.DH.UR3()  # Load the UR3 robot model from the Robotics Toolbox
        T = SE3(position)  # Create a homogeneous transformation matrix for the desired position
        config_IK = model.ikine_LM(T)  # Solve inverse kinematics using Levenberg-Marquardt method
        self.set_joint_position(config_IK.q)  # Set the joints to the calculated configuration


if __name__ == "__main__":
    ur3 = UR3(19999)  # Create UR3 object and connect to CoppeliaSim on port 19999

    # Set initial joint positions to zero
    ur3.set_joint_position([0, 0, 0, 0, 0, 0])
    time.sleep(1)

    # Retrieve and print the current joint positions
    ur3.get_joint_position()
    
    # Set a new joint configuration and wait for the robot to move
    ur3.set_joint_position([pi/2, pi/4, pi/3, pi, 0, 0])
    time.sleep(1)
    
    # Print the joint positions after movement
    print(ur3.get_joint_position())

    # Set another joint configuration
    ur3.set_joint_position([pi, pi/2, -pi/2, pi, pi/2, pi/2])
    time.sleep(1)
    
    # Move to another position and print the effector's absolute position
    ur3.set_joint_position([pi, pi/2, -pi/2, pi, pi/2, pi])
    print(ur3.get_effector_position())

    # Calculate and print the effector position using forward kinematics
    print(ur3.calculate_effector_position())

    # Perform inverse kinematics to reach a new position
    ur3.inverse_kinematics([0.1, 0.2, 0.5])
    time.sleep(1)

    # Print the calculated effector position after performing inverse kinematics
    print(ur3.calculate_effector_position())

from __future__ import annotations

import math
import time
from pathlib import Path

# === LIBRERÍAS ROS 2 COMENTADAS ===
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import mujoco
import mujoco.viewer
import numpy as np


# Cambiamos "Node" por una clase normal de Python para hacer la prueba offline
class SimController: # (Node):
    def __init__(self):
        # super().__init__('sim2real_controller')
        
        # === CONFIGURACIÓN Y PUBLISHER ROS COMENTADOS ===
        # qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        # self.pub = self.create_publisher(JointState, '/joint_commands', qos)
        
        # Poses extraídas de tu script original (Down y Up)
        self.d = np.array([0.7, 1.6, 0.8, 0.7, 0.0, 0.8, 0.7, 1.6, 0.8, 0.7, 0.0, 0.8], dtype=float)
        self.u = np.array([0.7, 0.0, 1.6, 0.7, 1.6, 0.0, 0.7, 0.0, 1.6, 0.7, 1.6, 0.0], dtype=float)
        
        self.start_time = time.time()
        self.current_ctrl = self.d.copy()
        
        # === TIMER ROS COMENTADO ===
        # self.create_timer(0.05, self.publish_ros)

    def update_math(self):
        # Calculamos la interpolación basada en tiempo (funciona sin ROS)
        t = time.time() - self.start_time
        # 10 segundos ciclo total (5 subida, 5 bajada)
        a = (math.sin(2 * math.pi * t / 10.0 - math.pi / 2) + 1) / 2
        self.current_ctrl = self.d + (self.u - self.d) * a

    # === FUNCIÓN DE ENVÍO ROS COMENTADA ===
    # def publish_ros(self):
    #     m = JointState()
    #     m.header.stamp = self.get_clock().now().to_msg()
    #     m.name =['lf_haa','lf_hfe','lf_kfe', 'rf_haa','rf_hfe','rf_kfe', 'lh_haa','lh_hfe','lh_kfe', 'rh_haa','rh_hfe','rh_kfe']
    #     m.position = self.current_ctrl.tolist()
    #     self.pub.publish(m)


def main():
    # === INICIALIZACIÓN ROS COMENTADA ===
    # rclpy.init(args=None)
    
    # Instanciamos nuestro controlador offline
    controller = SimController()

    # 1. Inicializamos MuJoCo
    model_path = "xmls/loquro.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    lower_limits = model.actuator_ctrlrange[:, 0]
    upper_limits = model.actuator_ctrlrange[:, 1]

    # 2. Arrancamos el visor pasivo
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # BUCLE PRINCIPAL: Solo depende de MuJoCo
        while viewer.is_running(): # and rclpy.ok():
            
            # A. Actualizar las matemáticas de la interpolación
            controller.update_math()
            
            # B. Aplicar al modelo de MuJoCo protegiendo los límites
            if model.nu == len(controller.current_ctrl):
                data.ctrl[:] = np.clip(controller.current_ctrl, lower_limits, upper_limits)
            else:
                # Si esto salta, significa que tus arrays 'd' y 'u' no tienen 12 números
                # o tu XML no tiene 12 etiquetas <actuator>
                print(f"Alerta: El XML tiene {model.nu} motores pero el script manda {len(controller.current_ctrl)}.")
            
            # C. Avanzar la física 1 paso de simulación
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # === LLAMADA A ROS COMENTADA ===
            # rclpy.spin_once(controller, timeout_sec=0.0)
            
            # D. Respetar el tiempo real
            time.sleep(model.opt.timestep)

    # === LIMPIEZA ROS COMENTADA ===
    # controller.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()

























# python3 -c 
# import rclpy, math, time
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# class P(Node):
#     def __init__(self):
#         super().__init__('p')
#         # Configuración de QoS para tiempo real (sin buffer acumulativo)
#         qos = QoSProfile(
#         reliability=ReliabilityPolicy.BEST_EFFORT,
#         history=HistoryPolicy.KEEP_LAST,
#         depth=1
#         )
#         self.pub = self.create_publisher(JointState, '/joint_commands', qos)
#         self.d = [0.7, 1.6, 0.8, 0.7, 0.0, 0.8, 0.7, 1.6, 0.8, 0.7, 0.0, 0.8]
#         self.u = [0.7, 0.0, 1.6, 0.7, 1.6, 0.0, 0.7, 0.0, 1.6, 0.7, 1.6, 0.0]
#         self.s = time.time()
#         # Bajamos a 20Hz (0.05s) para no saturar el ancho de banda
#         self.create_timer(0.05, self.cb)

#     def cb(self):
#         # 10 segundos ciclo total (5 subida, 5 bajada)
#         a = (math.sin(2*math.pi*(time.time()-self.s)/10 - math.pi/2)+1)/2
#         m = JointState()
#         m.header.stamp = self.get_clock().now().to_msg()
#         m.name = ['lf_haa','lf_hfe','lf_kfe','rf_haa','rf_hfe','rf_kfe','lh_haa','lh_hfe','lh_kfe','rh_haa','rh_hfe','rh_kfe']
#         m.position = [float(d+(u-d)*a) for d,u in zip(self.d, self.u)]
#         self.pub.publish(m)

# rclpy.init()
# try:
#     rclpy.spin(P())
# except KeyboardInterrupt:
#     pass

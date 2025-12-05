import os
import threading
import time
from ament_index_python.packages import get_package_share_path

import rclpy
from rclpy.node import Node

import mujoco
import mujoco.viewer

from std_msgs.msg import Float64MultiArray
import numpy as np

URDF_NAME = 'rm1_4.xml'

DESIRED_FREQ = 500       

lock = threading.Lock()   #线程锁
class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')
    
        self.publisher = self.create_publisher(Float64MultiArray, 'sim_robot_state', 10)
    
        self.timer = self.create_timer(0.01, self.publish_state)
    
        self.subscription = self.create_subscription(
        Float64MultiArray,
        'sim_robot_cmd',
        self.control_callback,
        10
        )

        self.current_state = np.zeros(14)   
        self.package_name = 'mujoco_sim'
        self.package_share_path = get_package_share_path(self.package_name)
        self.urdf_path = os.path.join(self.package_share_path, 'mjcf', URDF_NAME)
        self.model = mujoco.MjModel.from_xml_path(self.urdf_path)
        self.data = mujoco.MjData(self.model)

        self.body_name = "base_link"
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.body_name)

        # 获取传感器 ID
        self.gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro")
        self.vel_id  = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "vel")        

        self.get_logger().info(f"Number of joints: {self.model.njnt}")
        self.get_logger().info(f"Number of actuators: {self.model.nu}")

        self.ground_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.wheel1_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "Link1_wheel_visual")
        self.wheel4_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "Link4_wheel_visual")
        self.controlL = 0.00
        self.controlR = 0.00
        self.sim_thread = threading.Thread(target=self.run_sim)
        self.sim_thread.daemon = True
        self.sim_thread.start()

    def control_callback(self, msg: Float64MultiArray):             #  获得控制消息
        self.control_cmd = np.array(msg.data, dtype=float)
        with lock:
            self.controlL = self.control_cmd[0]
            self.controlR = self.control_cmd[1]
         
        #print("收到控制命令:", self.control_cmd, flush=True)

    def run_sim(self):                                              #  运行渲染画面
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:  
            target_dt = 1.0 / DESIRED_FREQ
            last_time = time.time()
            while viewer.is_running():     

                current_time = time.time()

                elapsed = current_time - last_time

                if elapsed < target_dt:
                    time.sleep(max(0, target_dt - elapsed - 0.001))  # 稍微提前醒来

                last_time = time.time()

                with lock:
                    self.data.ctrl[2]=self.controlL
                    self.data.ctrl[3]=self.controlR
                    mujoco.mj_step(self.model, self.data) 

                viewer.sync()

    def publish_state(self):   # 定时更新数据

        contact1 = 0
        contact4 = 0
        contactALL = 0  # 默认无碰撞

        with lock:
            pos = self.data.xpos[self.body_id]
            quat = self.data.xquat[self.body_id]

            gyro_data = self.data.sensordata[self.model.sensor_adr[self.gyro_id]:self.model.sensor_adr[self.gyro_id] + 3]  # rad/s
            vel_data = self.data.sensordata[self.model.sensor_adr[self.vel_id]:self.model.sensor_adr[self.vel_id] + 3]     # m/s

                #print(f"Position of {body_name}: {pos}")
                #print(f"Orientation of {body_name} (quat): {quat}")
            for i in range(self.data.ncon):
                contact = self.data.contact[i]

                if ((contact.geom1 == self.ground_id and contact.geom2 == self.wheel1_id) or
                    (contact.geom1 ==self. wheel1_id and contact.geom2 == self.ground_id)):
                    contact1=1 
    
                if ((contact.geom1 == self.ground_id and contact.geom2 == self.wheel4_id) or
                    (contact.geom1 == self.wheel4_id and contact.geom2 == self.ground_id)):
                    contact4=1 

                if (contact1==1 and contact4 == 1):
                    contactALL  =  1
                    #  print("⚠️ ground 和 wheel_visual 发生碰撞")
                    self.data.ctrl[2] = 0.0   
                    self.data.ctrl[3] = 0.0    
                        
        
        self.current_state[0:3] =  pos
        self.current_state[3:7] =  quat
        self.current_state[7]   =  contactALL
        self.current_state[8]   =  gyro_data[0]
        self.current_state[9]   =  gyro_data[1]
        self.current_state[10]  =  gyro_data[2]
        self.current_state[11]  =  vel_data[0]
        self.current_state[12]  =  vel_data[1]
        self.current_state[13]  =  vel_data[2]
        #print(f"Position of {self.current_state[0]}", flush=True)

        state_msg = Float64MultiArray()
        state_msg.data = self.current_state.tolist()
        self.publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    



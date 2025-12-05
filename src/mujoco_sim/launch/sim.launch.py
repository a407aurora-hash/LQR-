from launch import LaunchDescription # launch文件的描述类
from launch_ros.actions import Node # 节点启动的描述类
def generate_launch_description(): # 自动生成launch文件的函数
    return LaunchDescription(
        [# 返回launch文件的描述信息
            
            Node( # 配置一个节点的启动
                package='mujoco_control', # 节点所在的功能包
                executable='mujoco_control_node', # 节点的可执行文件
                prefix=['stdbuf -o L'],
                output="screen",
            ),

            Node( 
                package='mujoco_sim', 
                executable='mujoco_sim_node', 
                prefix=['stdbuf -o L'],
                output="screen",
            ),
        
        ]
    )

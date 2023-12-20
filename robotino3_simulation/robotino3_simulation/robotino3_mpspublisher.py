
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os
from ament_index_python.packages import get_package_share_directory

class Robotino3MpsSpawner(Node):

    def __init__(self):
        super().__init__('robotino3_mpspublisher')
        self.Define_Parameters()
        self.SpawnMps_cb()
        
    def Define_Parameters(self):
        
        # Position of Team-Cyan mps machines(cartesian- x,y,z)
        self.declare_parameter('C-CS2-I_position', '0.40 2.0 0.4')
        self.declare_parameter('C-BS-O_position', '1.5 2.0 0.4')
        self.declare_parameter('C-DS-I_position', '3.5 0.5 0.4')
        self.declare_parameter('C-RS2-O_position', '3.5 -0.5 0.4')
        self.declare_parameter('C-RS1-O_position', '1.5 -2.0 0.4')
        self.declare_parameter('C-CS1-O_position', '0.40 -2.0 0.4')
        self.declare_parameter('C-SS-I_position', '5.5 0.0 0.4')
        
        # Orientation of Team-Cyan mps machines(quaternion- x,y,z,w)
        self.declare_parameter('C-CS2-I_orientation', '0 0 1 1.57')
        self.declare_parameter('C-BS-O_orientation', '0 0 1 1.57')
        self.declare_parameter('C-DS-I_orientation', '0 0 1 0')
        self.declare_parameter('C-RS2-O_orientation', '0 0 1 0')
        self.declare_parameter('C-RS1-O_orientation', '0 0 1 1.57')
        self.declare_parameter('C-CS1-O_orientation', '0 0 1 1.57')
        self.declare_parameter('C-SS-I_orientation', '0 0 0 1')
        
        # Position of Team-Cyan mps machines(cartesian- x,y,z)
        self.declare_parameter('M-CS2-I_position', '-0.40 2.0 0.4')
        self.declare_parameter('M-BS-I_position', '-1.5 2.0 0.4')
        self.declare_parameter('M-DS-O_position', '-3.5 0.5 0.4')
        self.declare_parameter('M-RS2-O_position', '-3.5 -0.5 0.4')
        self.declare_parameter('M-RS1-I_position', '-1.5 -2.0 0.4')
        self.declare_parameter('M-CS1-I_position', '-0.40 -2.0 0.4')
        self.declare_parameter('M-SS-I_position', '-5.5 0.0 0.4')
        
        # Orientation of Team-Cyan mps machines(quaternion- x,y,z,w)
        self.declare_parameter('M-CS2-I_orientation', '0 0 1 1.57')
        self.declare_parameter('M-BS-I_orientation', '0 0 1 1.57')
        self.declare_parameter('M-DS-O_orientation', '0 0 1 0')
        self.declare_parameter('M-RS2-O_orientation', '0 0 1 0')
        self.declare_parameter('M-RS1-I_orientation', '0 0 1 1.57')
        self.declare_parameter('M-CS1-I_orientation', '0 0 1 1.57')  
        self.declare_parameter('M-SS-I_orientation', '0 0 0 1')  
          
    # callback function to publish data over cmd_vel topic based on joy_pad inputs
    def SpawnMps_cb(self):
        
        self.machine_locations = \
        {self.get_parameter('C-CS2-I_position').name:self.get_parameter('C-CS2-I_position').get_parameter_value().string_value,
        self.get_parameter('C-BS-O_position').name:self.get_parameter('C-BS-O_position').get_parameter_value().string_value,
        self.get_parameter('C-DS-I_position').name:self.get_parameter('C-DS-I_position').get_parameter_value().string_value,
        self.get_parameter('C-RS2-O_position').name:self.get_parameter('C-RS2-O_position').get_parameter_value().string_value,
        self.get_parameter('C-RS1-O_position').name:self.get_parameter('C-RS1-O_position').get_parameter_value().string_value,
        self.get_parameter('C-CS1-O_position').name:self.get_parameter('C-CS1-O_position').get_parameter_value().string_value,
        self.get_parameter('C-SS-I_position').name:self.get_parameter('C-SS-I_position').get_parameter_value().string_value,
        self.get_parameter('M-CS2-I_position').name:self.get_parameter('M-CS2-I_position').get_parameter_value().string_value,
        self.get_parameter('M-BS-I_position').name:self.get_parameter('M-BS-I_position').get_parameter_value().string_value,
        self.get_parameter('M-DS-O_position').name:self.get_parameter('M-DS-O_position').get_parameter_value().string_value,
        self.get_parameter('M-RS2-O_position').name:self.get_parameter('M-RS2-O_position').get_parameter_value().string_value,
        self.get_parameter('M-RS1-I_position').name:self.get_parameter('M-RS1-I_position').get_parameter_value().string_value,
        self.get_parameter('M-CS1-I_position').name:self.get_parameter('M-CS1-I_position').get_parameter_value().string_value,
        self.get_parameter('M-SS-I_position').name:self.get_parameter('M-SS-I_position').get_parameter_value().string_value,
        }
        
        self.machine_orientation = \
        {self.get_parameter('C-CS2-I_orientation').name:self.get_parameter('C-CS2-I_orientation').get_parameter_value().string_value,
        self.get_parameter('C-BS-O_orientation').name:self.get_parameter('C-BS-O_orientation').get_parameter_value().string_value,
        self.get_parameter('C-DS-I_orientation').name:self.get_parameter('C-DS-I_orientation').get_parameter_value().string_value,
        self.get_parameter('C-RS2-O_orientation').name:self.get_parameter('C-RS2-O_orientation').get_parameter_value().string_value,
        self.get_parameter('C-RS1-O_orientation').name:self.get_parameter('C-RS1-O_orientation').get_parameter_value().string_value,
        self.get_parameter('C-CS1-O_orientation').name:self.get_parameter('C-CS1-O_orientation').get_parameter_value().string_value,
        self.get_parameter('C-SS-I_orientation').name:self.get_parameter('C-SS-I_orientation').get_parameter_value().string_value,
        self.get_parameter('M-CS2-I_orientation').name:self.get_parameter('M-CS2-I_orientation').get_parameter_value().string_value,
        self.get_parameter('M-BS-I_orientation').name:self.get_parameter('M-BS-I_orientation').get_parameter_value().string_value,
        self.get_parameter('M-DS-O_orientation').name:self.get_parameter('M-DS-O_orientation').get_parameter_value().string_value,
        self.get_parameter('M-RS2-O_orientation').name:self.get_parameter('M-RS2-O_orientation').get_parameter_value().string_value,
        self.get_parameter('M-RS1-I_orientation').name:self.get_parameter('M-RS1-I_orientation').get_parameter_value().string_value,
        self.get_parameter('M-CS1-I_orientation').name:self.get_parameter('M-CS1-I_orientation').get_parameter_value().string_value,
        self.get_parameter('M-SS-I_orientation').name:self.get_parameter('M-SS-I_orientation').get_parameter_value().string_value,
        }
        
        package_dir = get_package_share_directory('robotino3_simulation')
        
        world = os.path.join(package_dir, 'worlds', 'webots_robotinosim_world_plugin.wbt')
        with open(world, 'r') as f:
            world_txt = f.read()
            
        modified_world = os.path.join(package_dir, 'worlds', 'modified_world.wbt')
        with open(modified_world, 'w') as f:
            externproto_txt = ''
            #externproto_txt = externproto_txt + 'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/factory/containers/protos/WoodenBox.proto"\n'
            externproto_txt = externproto_txt + 'EXTERNPROTO "../protos/Machine_base.proto"\n'
            externproto_txt = externproto_txt + 'EXTERNPROTO "../protos/Machine_cap.proto"\n'
            externproto_txt = externproto_txt + 'EXTERNPROTO "../protos/Machine_delivery.proto"\n'
            externproto_txt = externproto_txt + 'EXTERNPROTO "../protos/Machine_ring.proto"\n'
            externproto_txt = externproto_txt + 'EXTERNPROTO "../protos/Machine_storage.proto"\n'
            world_txt = world_txt.split('\n')
            world_txt[1] = externproto_txt
            world_txt = '\n'.join(world_txt)
            
            machine = ''
            idx = 0 
            for machine_position, machine_orientation in \
                zip(self.machine_locations.keys(), self.machine_orientation.keys()):
                
                idx += 1
                if machine_position in ['C-CS2-I_position','C-CS1-O_position','M-CS1-I_position','M-CS2-I_position']:
                    machine += \
"""
Machine_ring {
translation """ + self.machine_locations[machine_position] + """
rotation """+ self.machine_orientation[machine_orientation] +"""
name \"machine""" + str(idx) + """\"
}"""
                elif machine_position in ['C-BS-O_position','M-BS-I_position']:
                    machine += \
"""
Machine_base {
translation """ + self.machine_locations[machine_position] + """
rotation """+ self.machine_orientation[machine_orientation] +"""
name \"machine""" + str(idx) + """\"
}"""
                elif machine_position in ['C-RS2-O_position','C-RS1-O_position','M-RS1-I_position','M-RS2-O_position']:
                    machine += \
"""
Machine_ring {
translation """ + self.machine_locations[machine_position] + """
rotation """+ self.machine_orientation[machine_orientation] +"""
name \"machine""" + str(idx) + """\"
}"""
                elif machine_position in ['M-SS-I_position','C-SS-I_position']:
                    machine += \
"""
Machine_storage {
translation """ + self.machine_locations[machine_position] + """
rotation """+ self.machine_orientation[machine_orientation] +"""
name \"machine""" + str(idx) + """\"
}"""
                elif machine_position in ['M-DS-O_position','C-DS-I_position']:
                    machine += \
"""
Machine_delivery {
translation """ + self.machine_locations[machine_position] + """
rotation """+ self.machine_orientation[machine_orientation] +"""
name \"machine""" + str(idx) + """\"
}"""
            f.write(world_txt+machine)
            
def main():
    rclpy.init()
    mpsspawnwer_node = Robotino3MpsSpawner()
    try:
        rclpy.spin_once(mpsspawnwer_node)
    except KeyboardInterrupt:
        pass
    mpsspawnwer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import os
import fnmatch
import subprocess
import rclpy
from rclpy.node import Node

class OpenFoxgloveStudio(Node):
    def __init__(self):
        super().__init__('open_foxglove_studio')
        self.open_dashboard()
        self.websocket_string = '"foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765"'

    def find_folder(self,folder_name, search_path='/'):
        matches = []
        for root, dirnames, filenames in os.walk(search_path):
            for dirname in fnmatch.filter(dirnames, folder_name):
                matches.append(os.path.join(root, dirname))
        return matches
    
    def open_dashboard(self):
        folder_to_search = "Agri-Robot"
        found_paths = self.find_folder(folder_to_search)
    
        if found_paths:
            print("Found folder at the following paths:")
            for path in found_paths:
                print(path)
    
            # Open the found folder, source the workspace, and run foxglove-studio
            found_folder = found_paths[0]
            # Assuming the workspace setup file is in the 'devel' or 'install' directory
            setup_file = os.path.join(found_folder, "devel", "setup.bash")
            if not os.path.exists(setup_file):
                setup_file = os.path.join(found_folder, "install", "setup.bash")
    
            if os.path.exists(setup_file):
                # Use Bash explicitly to run the command
                command = f"cd {found_folder} && source {setup_file} && export ROS_PACKAGE_PATH={found_folder}/ && foxglove-studio "
                print(f"Executing command: {command}")
                subprocess.Popen(["bash", "-c", command])
            else:
                print(f"Error: Workspace setup file not found in {found_folder}")
        else:
            print(f"No folder named '{folder_to_search}' found.")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = OpenFoxgloveStudio()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
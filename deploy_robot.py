#!/usr/bin/env python3

import sys
import os
import subprocess
import time
import signal
import argparse

class RobotDeployer:
    def __init__(self, environment):
        self.environment = environment.upper()
        self.processes = []
        
    def cleanup(self, signum=None, frame=None):
        print("\nTerminating processes...")
        for proc in self.processes:
            if proc.poll() is None:
                proc.terminate()
        
        # Kill specific processes
        subprocess.run(['killall', '-9', 'gzclient', 'gzserver', 'rviz'], 
                      stderr=subprocess.DEVNULL)
        
        for proc in self.processes:
            proc.wait()
        print("Deployment stopped.")
        sys.exit(0)
    
    def setup_ros_network(self, limo_ip):
        """Setup ROS network for real robot"""
        os.environ['ROS_MASTER_URI'] = f'http://{limo_ip}:11311'
        # Get local IP automatically
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
        local_ip = result.stdout.strip().split()[0]
        os.environ['ROS_IP'] = local_ip
        print(f"ROS Network configured: Master={limo_ip}, Local={local_ip}")
    
    def run_simulation(self):
        """Run simulation using start_simulation.sh"""
        print("Starting simulation environment...")
        script_path = os.path.join(os.path.dirname(__file__), 'start_simulation.sh')
        
        # Make script executable
        subprocess.run(['chmod', '+x', script_path])
        
        # Run the simulation script
        proc = subprocess.Popen([script_path], preexec_fn=os.setsid)
        self.processes.append(proc)
        
        try:
            proc.wait()
        except KeyboardInterrupt:
            self.cleanup()
    
    def run_real_robot(self, limo_ip):
        """Run on real robot"""
        print(f"Connecting to real LIMO robot at {limo_ip}...")
        
        # Setup ROS network
        self.setup_ros_network(limo_ip)
        
        # Source workspace
        source_cmd = 'source ~/catkin_ws/devel/setup.bash'
        
        try:
            # Test connection
            print("Testing ROS connection...")
            result = subprocess.run(['rostopic', 'list'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                print("Error: Cannot connect to robot. Ensure robot is running:")
                print("  ssh agilex@{} 'roslaunch limo_base limo_base.launch'".format(limo_ip))
                return
            
            print("Connection successful. Starting navigation...")
            
            # Launch gmapping
            print("Starting gmapping...")
            proc1 = subprocess.Popen(['roslaunch', 'limo_bringup', 'limo_gmapping.launch'])
            self.processes.append(proc1)
            time.sleep(3)
            
            # Launch navigation
            print("Starting navigation...")
            proc2 = subprocess.Popen(['rosrun', 'limo_base', 'wander.py'])
            self.processes.append(proc2)
            time.sleep(2)
            
            # Launch visual search
            print("Starting visual search...")
            proc3 = subprocess.Popen(['rosrun', 'limo_base', 'visual_search_node.py'])
            self.processes.append(proc3)
            
            # Wait for processes
            for proc in self.processes:
                proc.wait()
                
        except subprocess.TimeoutExpired:
            print("Error: Connection timeout. Check network and robot status.")
        except KeyboardInterrupt:
            self.cleanup()
        except Exception as e:
            print(f"Error: {e}")
    
    def deploy(self, limo_ip=None):
        # Setup signal handler
        signal.signal(signal.SIGINT, self.cleanup)
        
        if self.environment == 'SIM':
            self.run_simulation()
        elif self.environment == 'REAL':
            if not limo_ip:
                limo_ip = input("Enter LIMO robot IP address: ")
            self.run_real_robot(limo_ip)
        else:
            print("Error: Environment must be 'SIM' or 'REAL'")
            sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description='Deploy robot navigation system')
    parser.add_argument('environment', choices=['SIM', 'REAL'], 
                       help='Testing environment: SIM for simulation, REAL for real robot')
    parser.add_argument('--ip', help='LIMO robot IP address (required for REAL environment)')
    
    args = parser.parse_args()
    
    if args.environment == 'REAL' and not args.ip:
        print("Error: --ip required for REAL environment")
        sys.exit(1)
    
    deployer = RobotDeployer(args.environment)
    deployer.deploy(args.ip)

if __name__ == '__main__':
    main()
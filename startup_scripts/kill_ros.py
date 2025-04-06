import psutil
import subprocess
import time
import os
import signal

def kill_matching_processes():
    for proc in psutil.process_iter(['name', 'cmdline']):
        try:
            name = proc.info['name']
            cmdline = proc.info['cmdline']

            if not cmdline:
                continue  # Skip processes with no command line info

            # Skip processes that include 'planning_controller_node'
            if any('planning_controller_node' in arg for arg in cmdline):
                print(f"Skipping process: {proc.info['name']} ({proc.pid}) as it is the planning controller node.")
                continue  # Do not kill this process
                
            if cmdline:
                if (cmdline[0].startswith('/opt/ros/') or '/opt/ros/' in cmdline[0] or
                        'ws/install/' in cmdline[0]):
                    # proc.kill()
                    proc.send_signal(signal.SIGINT)  # Graceful shutdown
                
                    # Wait for process to terminate
                    try:
                        proc.wait(5)  # Wait 5 seconds for the process to stop
                    except psutil.TimeoutExpired:
                        print(f"Force killing process: {proc.info['name']} ({proc.pid})")
                        proc.kill()  # Force stop if it didn't exit
                    print(f"Process name: {name}")
                    print(f"Command line: {cmdline}")
                    print("Matches condition 1")

                if len(cmdline) > 1 and 'ws/install/' in cmdline[1]:
                    # proc.kill()

                    proc.send_signal(signal.SIGINT)  # Graceful shutdown
                
                    # Wait for process to terminate
                    try:
                        proc.wait(5)  # Wait 5 seconds for the process to stop
                    except psutil.TimeoutExpired:
                        print(f"Force killing process: {proc.info['name']} ({proc.pid})")
                        proc.kill()  # Force stop if it didn't exit

                    print(f"Process name: {name}")
                    print(f"Command line: {cmdline}")
                    print("Matches condition 2")

                if 'home/install/' in cmdline[0]:
                    # proc.kill()
                    proc.send_signal(signal.SIGINT)  # Graceful shutdown
                
                    # Wait for process to terminate
                    try:
                        proc.wait(5)  # Wait 5 seconds for the process to stop
                    except psutil.TimeoutExpired:
                        print(f"Force killing process: {proc.info['name']} ({proc.pid})")
                        proc.kill()  # Force stop if it didn't exit

                    print(f"Process name: {name}")
                    print(f"Command line: {cmdline}")
                    print("Matches condition 3")

                if len(cmdline) > 1 and ('home/install/' in cmdline[1] or 'planner_monitor' in cmdline[1]):
                    # proc.kill()
                    proc.send_signal(signal.SIGINT)  # Graceful shutdown
                
                    # Wait for process to terminate
                    try:
                        proc.wait(5)  # Wait 5 seconds for the process to stop
                    except psutil.TimeoutExpired:
                        print(f"Force killing process: {proc.info['name']} ({proc.pid})")
                        proc.kill()  # Force stop if it didn't exit
                    print(f"Process name: {name}")
                    print(f"Command line: {cmdline}")
                    print("Matches condition 4")

        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess) as e:
            print(f"Error with process {proc}: {e}")

def run_and_terminate_script():
    # Start the Python script
    process = subprocess.Popen(['python3', '/usr/bin/hello_robot_lrf_off.py'])

    # Wait for 5 seconds
    time.sleep(5)

    # Terminate the process
    try:
        os.kill(process.pid, signal.SIGTERM)
        process.wait()  # Ensure the process is cleaned up
        print("Script terminated successfully.")
    except Exception as e:
        print(f"Error terminating script: {e}")

if __name__ == "__main__":
    kill_matching_processes()
    #run_and_terminate_script()

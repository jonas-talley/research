import socket
import os
from config_manager import load_config

class AerotechDriver:
    def __init__(self):
        config = load_config()
        self.ip = config.get("aerotech_ip", "127.0.0.1")
        self.port = config.get("aerotech_port", 8000)
        self.user_files_path = config.get("aerotech_path", r"C:\Users\Public\Public Documents\Aerotech\A3200\User Files")

    def _send(self, cmd):
        """Internal single-shot sender with no persistent connection hang-ups"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2)
                s.connect((self.ip, self.port))
                s.sendall(f"{cmd}\n".encode('ascii'))
                return s.recv(1024).decode('ascii').strip()
        except Exception as e:
            return f"Error: {e}"

    def run_pgm(self, filename, task_index=1):
        # Construct Full Path
        full_path = os.path.join(self.user_files_path, filename)
        
        # Send Run Command
        # Uses quotes to handle spaces in Windows paths
        cmd = f'PROGRAM {task_index} RUN "{full_path}"'
        response = self._send(cmd)
        
        if response == "#":
            # If failed, return the server error reason
            return False, self._send("~LASTERROR")
        return True, response

    def reset_task(self, task_index=1):
        """Aborts the specified task on the Aerotech controller"""
        self._send(f"PROGRAM {task_index} STOP")
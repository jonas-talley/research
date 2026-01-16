# DIW_Control_System/Host/aerotech.py
import socket
import os

class AerotechDriver:
    def __init__(self, ip='127.0.0.1', port=8000):
        self.ip = ip
        self.port = port
        self.user_files_path = r"C:\Users\Public\Public Documents\Aerotech\A3200\User Files"

    def _send(self, cmd):
        #"""Internal single-shot sender with no persistent connection hang-ups"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2)
                s.connect((self.ip, self.port))
                s.sendall(f"{cmd}\n".encode('ascii'))
                return s.recv(1024).decode('ascii').strip()
        except Exception as e:
            return f"Error: {e}"

    def run_pgm(self, filename, task_index=1):

        
        # 2. Construct Full Path
        full_path = os.path.join(self.user_files_path, filename)
        
        # 3. Send Run Command
        # Uses quotes to handle spaces in Windows paths
        cmd = f'PROGRAM {task_index} RUN "{full_path}"'
        response = self._send(cmd)
        
        if response == "#":
            # If failed, return the server error reason
            return False, self._send("~LASTERROR")
        return True, response
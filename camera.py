from serial import Serial
import threading
from socket import socket, AF_INET, SOCK_STREAM

class Camera:

    def __init__(self, logger):
        self.logger = logger
        self.camera_values = ""

        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.connect(('localhost', 9788))

    def get_data(self):
        data = self.socket.recv(8192)
        fields = data.split(" ")
        try:
            values = (int(fields[0]), int(fields[1]))
        except:
            self.logger.write("bad camera data", data)

    def close(self):
        self.socket.close()

    def get_camera_values(self):
        return self.camera_values

class CameraThread(threading.Thread):

    def __init__(self, camera, logger):
        super(CameraThread, self).__init__()
        self.stoprequest = threading.Event()
        self.camera = camera
        self.logger = logger

    def run(self):
        self.logger.write("Starting camera thread")
        while not self.stoprequest.isSet():
            self.camera.read_data()
        self.logger.write("Terminating camera thread")
        self.camera.close()

    def join(self, timeout=None):
        self.stoprequest.set()
        super(CameraThread, self).join(timeout)

import serial

def string50(arg_str):
    my_str = arg_str
    while len(my_str) < 50:
        my_str += ' '
    
    return my_str

class Gimbal:
    def __init__(self, port):
        self.connection = serial.Serial(port, 115200, timeout = 100)

    def connect(self):
        if self.connection.is_open :
            self.connection.close()
            self.connection.open()

    def pause(self):
        self.connection.write(string50('pause\n'))

    def resume(self):
        self.connection.write(string50('resume\n'))

    

import communication as c
import sys

gim = c.Gimbal(sys.argv[1])
gim.connect()

while True:
    option = raw_input('Type \"pause\" or \"resume\": ')
    if option == 'pause':
        gim.pause()
    elif option == 'resume':
        gim.resume()
    else:
        print('invalid option')
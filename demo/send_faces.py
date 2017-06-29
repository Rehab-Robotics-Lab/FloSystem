import json, math
from pprint import pprint
from serial_coms import SerialCom # https://github.com/Rehab-Robotics-Lab/serial_coms/tree/master/computer/python/serial_coms

def data_handler(received, *data):
    print("received as ints:")
    print(data)
    print("received as string (may be nonsense):")
    print("".join(map(chr, data)))

flatten = lambda l: [item for sublist in l for item in sublist]

def bytize(data):
    flat_data = flatten(data)
    data_bytes = [0]*math.ceil(len(flat_data)/8)
    for i in range(len(data_bytes)):
        for j in range(8):
            data_bytes[i] = data_bytes[i] | (flat_data[i*8 + j] << (7-j))
    return data_bytes

coms = SerialCom('COM3', data_handler)

filename = "faces_demo.json"

json_data=open(filename)
faces = json.load(json_data)

coms.sendData([0] + bytize(faces['mouths']['happy']['on']))
coms.sendData([1] + bytize(faces['eyes']['left']['on']))
coms.sendData([2] + bytize(faces['eyes']['left']['on']))

mouths = faces['mouths'].items()

print("Commands (followed by enter key):")
print('q: quit')
print('\tEyes:')
print("\t\tj: move eyes left")
print("\t\tl: move eyes right")
print("\t\tm: move eyes down")
print("\t\ti: move eyes up")
print("\t\tk: move eyes center")
print('\tMouths:')
mouth_keys = list(faces['mouths'].keys())
i = 1
for key in mouth_keys:
    print("\t\t"+str(i)+": set to "+key)
    i+=1

keep_running = True
while keep_running:
    command = input('>>')
    # command = 1
    if command=='q':
        keep_running = False
    elif command=='j':
        coms.sendData([1] + bytize(faces['eyes']['left']['on']))
        coms.sendData([2] + bytize(faces['eyes']['left']['on']))
    elif command=='l':
        coms.sendData([1] + bytize(faces['eyes']['right']['on']))
        coms.sendData([2] + bytize(faces['eyes']['right']['on']))
    elif command=='m':
        coms.sendData([1] + bytize(faces['eyes']['down']['on']))
        coms.sendData([2] + bytize(faces['eyes']['down']['on']))
    elif command=='i':
        coms.sendData([1] + bytize(faces['eyes']['up']['on']))
        coms.sendData([2] + bytize(faces['eyes']['up']['on']))
    elif command=='k':
        coms.sendData([1] + bytize(faces['eyes']['center']['on']))
        coms.sendData([2] + bytize(faces['eyes']['center']['on']))
    else:
        try:
            val = int(command)
            coms.sendData([0] + bytize(faces['mouths'][mouth_keys[val-1]]['on']))
        except Exception:
            print("I did not understand your input") 
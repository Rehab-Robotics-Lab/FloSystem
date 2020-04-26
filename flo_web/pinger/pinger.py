import requests
import time
import os
import socket

if __name__ == "__main__":
    server_addr = os.environ['FLO_SERVER_IP']
    name = os.environ['ROBOT_NAME']
    password = os.environ['ROBOT_PASSWORD']

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_addr = s.getsockname()[0]
        s.close()
        print('setting ip addr to {}'.format(ip_addr))
        requests.put('https://'+server_addr + '/api/robots/ipaddr',
                     {'name': name, 'password': password, 'ipaddr': ip_addr})
        time.sleep(60)
    except Exception as e:
        print(e)

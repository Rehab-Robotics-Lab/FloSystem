import requests
import time
import os
import socket
import logging
import logging.handlers

if __name__ == "__main__":
    SERVER_ADDR = os.environ['FLO_SERVER_IP']
    NAME = os.environ['ROBOT_NAME']
    PASSWORD = os.environ['ROBOT_PASSWORD']
    LOG_PATH = os.path.expanduser('~/logs')
    if not os.path.exists(LOG_PATH):
        os.makedirs(LOG_PATH)
    LOG_FILE = os.path.join(LOG_PATH, 'pinger.log')

    LOG = logging.getLogger('pinger')
    LOG.setLevel(logging.DEBUG)
    FORMATTER = logging.Formatter('%(asctime)s %(levelname)-8s %(message)s')
    FH = logging.handlers.RotatingFileHandler(
        LOG_FILE, maxBytes=1024*5, backupCount=5)
    FH.setLevel(logging.DEBUG)
    FH.setFormatter(FORMATTER)
    LOG.addHandler(FH)

    try:
        S = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        S.connect(("8.8.8.8", 80))
        IP_ADDR = S.getsockname()[0]
        S.close()
        LOG.info('setting ip addr to %s', IP_ADDR)
        RESP = requests.put('https://'+SERVER_ADDR + '/api/robots/ipaddr',
                            {'name': NAME, 'password': PASSWORD, 'ipaddr': IP_ADDR})
        if RESP.status_code == 200:
            LOG.info('set ip address: %s', RESP)
        else:
            LOG.warning('error in response: %s', RESP)
    except Exception as ex:
        LOG.warning(ex)

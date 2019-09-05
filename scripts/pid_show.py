import socket
import threading
import time
import json
from json import JSONDecodeError

import matplotlib.pyplot as plt

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
while True:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', 9009))
        break
    except OSError:
        s.close()
        time.sleep(1)
        print("try")
s.listen(5)


def Tcplink(sock, addr):
    print('Accept new connection from %s:%s...' % addr)
    target_list = []
    input_list = []
    output_list = []
    time_list = []
    i = 0
    while True:
        i += 1
        try:
            data = sock.recv(1024)
        except OSError:
            return None
        if not data:
            break
        data = str(data, 'utf-8')
        try:
            data_receive = json.loads(data)
        except JSONDecodeError:
            print("json parse error, data: %s" % data)
            continue
        if 'header' in data_receive:
            print(data_receive['header'])
            data = data_receive['data']
        else:
            continue
        res = json.loads(data)
        # sock.send("w".encode('utf-8'))
        target_list.append(res[u'target'])
        input_list.append(res[u'input'])
        output_list.append(res[u'output'])
        time_list.append(res[u'time'])
        if res[u'end']:
            plt.plot(time_list, input_list)
            plt.plot(time_list, target_list)
            plt.show()
            break
    sock.close()
    print('Connection from %s:%s closed.' % addr)


if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        while True:
            sock, addr = s.accept()
            t = threading.Thread(target=Tcplink, args=(sock, addr))
            t.start()
    except KeyboardInterrupt:
        sock.close()
        s.close()
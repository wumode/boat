import socket
import threading
import time
import json
import matplotlib.pyplot as plt

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('127.0.0.1', 6800))
s.listen(5)


def Tcplink(sock, addr):
    print('Accept new connection from %s:%s...' % addr)
    target_list = []
    input_list = []
    output_list = []
    time_list = []
    while True:
        data = sock.recv(1024)
        data = str(data, 'utf-8')
        print(data)
        res = json.loads(data)
        sock.send("w".encode('utf-8'))
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
    try:
        while True:
            sock, addr = s.accept()
            t = threading.Thread(target=Tcplink, args=(sock, addr))
            t.start()
    except KeyboardInterrupt:
        s.close()

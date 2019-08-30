import socket
import threading
import time
import json
from json import JSONDecodeError

kHandShake1 = 1
kHandShake2 = 2
kHandShake3 = 3
kNormalData = 4
client_num = 0
client_id_list = []
SocketErrorType = []
kErrorFree = 0
kMissHandShake = 1
SOCKET_SIZE = 2048
SocketErrorType.append(kErrorFree)
SocketErrorType.append(kMissHandShake)
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


def data_encapsulation(data, header):
    data_str = json.dumps(data)
    data_send = {'header': header, 'time_stamp': int(1000*time.time()),
                 'data': data_str}
    data_send_str = json.dumps(data_send)
    return data_send_str


def data_parse(data):
    try:
        data_receive = json.loads(data)
    except JSONDecodeError:
        print("==>json parse error, data: %s" % data)
        return None
    return data_receive


def client_send(sock, addr, client_id):
    while client_id in client_id_list:
        print("send thead")
        time.sleep(0.05)


def tcp_link(sock, addr):
    print('Accept new connection from %s:%s...' % addr)
    hand_shake_num = 0
    identity = 0
    state = 0
    global client_id_list
    start = 0
    end = 0
    state_err_num = 0
    client_id = 0

    while True:
        try:
            end = time.time()
            data = sock.recv(SOCKET_SIZE)
            print("time used: %f" % (end - start))
            start = time.time()
        except OSError:
            return None
        if not data:
            print('not data')
            break
        data = str(data, 'utf-8')
        # print("==>parse raw data: %s" % data)
        data_receive = data_parse(data)
        if data_receive is None:
            continue
        if 'header' in data_receive:
            print("==>header: %d" % data_receive['header'])
            print("==>data: %s" % data_receive['data'])
            if 'time_stamp' not in data_receive:
                continue
            if state == 0:
                if data_receive['header'] == kHandShake1:
                    if 'data' not in data_receive:
                        continue
                    # print(data_receive['data'])
                    print("==>parse data data: %s" % data_receive['data'])
                    handshake_data = data_parse(data_receive['data'])
                    if handshake_data is None:
                        continue
                    if 'identity' in handshake_data:
                        state_err_num = 0
                        identity = handshake_data['identity']
                        if identity == 1:
                            if time.time()*1000 - data_receive['time_stamp'] < 200:
                                if 'id' in handshake_data:
                                    if handshake_data['id'] not in client_id_list:
                                        client_id = handshake_data['id']
                                        client_id_list.append(handshake_data['id'])
                                        client_send_thread = threading.Thread(target=client_send, args=(sock, addr,
                                                                                                        client_id))
                                        client_send_thread.start()
                                    socket_handshake = {'identity': 2, 'ok': 1, 'id': 1}
                                    sock.send(data_encapsulation(socket_handshake, kHandShake1).encode('utf-8'))
                                    print("==>hand shake success!")
                                    state = 1
                            else:
                                hand_shake_num += 1
                                if hand_shake_num > 10:
                                    socket_handshake = {'identity': 2, 'ok': 0}
                                    sock.send(data_encapsulation(socket_handshake, kHandShake1))
                        elif identity == 3:
                            socket_handshake = {'identity': 2, 'ok': 1, 'id': 1}
                            sock.send(data_encapsulation(socket_handshake, kHandShake1).encode('utf-8'))
                            print("==>hand shake success!")
                            state = 1
                else:
                    state_err_num += 1
                    if state_err_num > 5:
                        state = 0
                        print("==>send error")
                        socket_handshake2 = {'identity': 2, 'error_type': SocketErrorType[kMissHandShake]}
                        sock.send(data_encapsulation(socket_handshake2, kHandShake2).encode('utf-8'))
            elif state == 1:
                if data_receive['header'] == kNormalData:
                    state_err_num = 0
                    if 'data' not in data_receive:
                        continue
                else:
                    state_err_num += 1
                    if state_err_num > 5:
                        state = 0
                # print(data_receive['data'])
            print("==>state: %d" % state)
            # print(time.time()*1000 - data_receive['time_stamp'])
            # print(data_receive['data'])
        else:
            continue
    if client_id in client_id_list:
        client_id_list.remove(client_id)


if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        while True:
            sock, addr = s.accept()
            t = threading.Thread(target=tcp_link, args=(sock, addr))
            t.start()
    except KeyboardInterrupt:
        sock.close()
        s.close()
        client_id_list.clear()

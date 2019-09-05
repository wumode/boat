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
user_id_list = []
SocketErrorType = []
kErrorFree = 0
kMissHandShake = 1
SOCKET_SIZE = 2048
SocketErrorType.append(kErrorFree)
SocketErrorType.append(kMissHandShake)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

client_socket = {}
user_socket = {}
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
s.listen(20)


def data_encapsulation(data, header):
    data_str = json.dumps(data)
    data_send = {'header': header, 'time_stamp': int(1000*time.time()),
                 'data': data_str}
    data_send_str = json.dumps(data_send)
    return data_send_str.encode('utf-8')


def data_parse(data):
    try:
        data_receive = json.loads(data)
    except JSONDecodeError:
        print("==>json parse error, data: %s" % data)
        return None
    return data_receive


def socket_send(id, identity, data, header):
    global user_id_list
    data = data_encapsulation(data, header)
    if identity == 3:
        if (id in user_id_list) and (id in user_socket):
            sock = user_socket[id]
            print("==>send data: %s" % data)
            sock.send(data)
    if identity == 1:
        print("id in client_id_list: ", id in client_id_list)
        print("id in client_socket: ", id in client_socket)
        print("id: ", id)
        if (id in client_id_list) and (id in client_socket):
            sock = client_socket[id]
            print("==>send data: %s" % data)
            sock.send(data)


def tcp_link(sock, addr):
    print('Accept new connection from %s:%s...' % addr)
    hand_shake_num = 0
    identity = 0
    state = 0
    global client_id_list
    global user_id_list
    global user_socket
    start = 0
    end = 0
    state_err_num = 0
    socket_id = 0
    while True:
        try:
            data = sock.recv(SOCKET_SIZE)
        except OSError:
            return None
        if not data:
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
            if 'data' not in data_receive:
                continue
            delay = time.time()*1000 - data_receive['time_stamp']
            print("==>delay: %dms" % delay)
            if state == 0:
                if data_receive['header'] == kHandShake1:
                    if 'data' not in data_receive:
                        continue
                    # print(data_receive['data'])

                    handshake_data = data_parse(data_receive['data'])
                    if handshake_data is None:
                        continue
                    if 'id' not in handshake_data:
                        continue
                    socket_id = handshake_data['id']
                    if 'identity' in handshake_data:
                        state_err_num = 0
                        identity = handshake_data['identity']
                        if identity == 1:
                            if delay < 300:
                                if socket_id not in client_id_list:
                                    client_id_list.append(socket_id)
                                client_socket[socket_id] = sock
                                socket_handshake = {'identity': 2, 'ok': 1, 'id': 1}
                                # sock.send(data_encapsulation(socket_handshake, kHandShake1))
                                socket_send(socket_id, 1, socket_handshake, kHandShake1)
                                print("==>hand shake success!")
                                state = 1

                            else:
                                hand_shake_num += 1
                                if hand_shake_num > 10:
                                    socket_handshake = {'identity': 2, 'ok': 0}
                                    # sock.send(data_encapsulation(socket_handshake, kHandShake1))
                                    socket_send(socket_id, 1, socket_handshake, kHandShake1)
                        elif identity == 3:
                            if 'id' not in handshake_data:
                                continue

                            if handshake_data['id'] not in user_id_list:
                                user_id_list.append(handshake_data['id'])
                            user_socket[socket_id] = sock
                            socket_handshake = {'identity': 2, 'ok': 1, 'id': 1}
                            # sock.send(data_encapsulation(socket_handshake, kHandShake1))
                            socket_send(socket_id, 3, socket_handshake, kHandShake1)
                            print("==>hand shake success!")
                            state = 1
                else:
                    if identity == 1:
                        state_err_num += 1
                        if state_err_num > 5:
                            state = 0
                            print("==>send error")
                            socket_handshake2 = {'identity': 2, 'error_type': SocketErrorType[kMissHandShake]}
                            # sock.send(data_encapsulation(socket_handshake2, kHandShake2))
                            socket_send(socket_id, 1, socket_handshake2, kHandShake2)
            elif state == 1:
                if identity == 1:
                    if data_receive['header'] == kNormalData:
                        state_err_num = 0
                        normal_data = data_parse(data_receive['data'])
                        for user_online_id in user_id_list:
                            socket_send(user_online_id, 3, normal_data, kNormalData)
                    else:
                        state_err_num += 1
                        if state_err_num > 5:
                            state = 0
                if identity == 3:
                    if data_receive['header'] == kHandShake3:
                        socket_handshake3 = {
                            "identity": 2,
                            "id_list": client_id_list
                        }
                        socket_send(socket_id, 3, socket_handshake3, kHandShake3)
                    if data_receive['header'] == kNormalData:
                        normal_data = data_parse(data_receive['data'])
                        if 'receiver_id' not in normal_data:
                            continue
                        for id_client_receiver in normal_data['receiver_id']:
                            socket_send(id_client_receiver, 1, normal_data, kNormalData)
            print("==>state: %d" % state)
            # print(time.time()*1000 - data_receive['time_stamp'])
            # print(data_receive['data'])
        else:
            continue
    if identity == 1:
        if socket_id in client_id_list:
            client_id_list.remove(socket_id)
        if socket_id in client_socket:
            del client_socket[socket_id]
        print("socket client num: %d" % len(client_id_list))
    elif identity == 3:
        if socket_id in user_id_list:
            user_id_list.remove(socket_id)
        if socket_id in user_socket:
            del user_socket[socket_id]
        print("socket user num: %d" % len(user_id_list))


if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        while True:
            sock, addr = s.accept()
            t = threading.Thread(target=tcp_link, args=(sock, addr))
            t.start()
    except KeyboardInterrupt:
        sock.shutdown(socket.SHUT_RDWR)
        s.shutdown(socket.SHUT_RDWR)
        client_id_list.clear()

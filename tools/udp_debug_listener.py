import socket


HOST = "0.0.0.0"
PORT = 5005


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    print("listening udp %s:%d" % (HOST, PORT))
    while True:
        data, addr = sock.recvfrom(2048)
        print("%s:%d %s" % (addr[0], addr[1], data.decode("ascii", "ignore").strip()))


if __name__ == "__main__":
    main()

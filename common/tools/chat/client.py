import select
import socket
import sys


# Helper function (formatting)
def display():
    you = "\33[33m\33[1m" + " You: " + "\33[0m"
    sys.stdout.write(you)
    sys.stdout.flush()


def main():
    host = input("Enter host ip address: ") if len(sys.argv) < 2 else sys.argv[1]
    port = 5001

    # asks for user name
    name = input("\33[34m\33[1m CREATING NEW ID:\n Enter username: \33[0m")
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.settimeout(2)

    # connecting host
    try:
        soc.connect((host, port))
    except ConnectionRefusedError:
        print("\33[31m\33[1m Can't connect to the server \33[0m")
        sys.exit()

    # if connected
    output = name.encode('utf-8')
    soc.send(output)
    display()
    while 1:
        socket_list = [sys.stdin, soc]

        # Get the list of sockets which are readable
        red_list, _, _ = select.select(socket_list, [], [])

        for sock in red_list:
            # incoming message from server
            if sock == soc:
                data = sock.recv(4096)
                data = data.decode("utf-8")
                if not data:
                    print('\33[31m\33[1m \rDISCONNECTED!!\n \33[0m')
                    sys.exit()
                else:
                    sys.stdout.write(data)
                    display()

            # user entered a message
            else:
                msg = sys.stdin.readline()
                soc.send(msg.encode('utf-8'))
                display()


if __name__ == "__main__":
    main()

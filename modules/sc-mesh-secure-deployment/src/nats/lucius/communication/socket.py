import socket

PORT = 5884
NUM_CLIENTS = 2

class Socket:
  def __init__(self, host: str = '', port: int = PORT) -> None:
    self.host = host
    self.port = port

  def serve(self, response_callback):

    server_socket = socket.socket()  # get instance
    server_socket.bind((self.host, self.port))  # bind host address and port together

    # configure how many client the server can listen simultaneously
    server_socket.listen(NUM_CLIENTS)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    while True:
      try:
        conn, address = server_socket.accept()  # accept new connection
        print("Connection from: " + str(address))
        data = conn.recv(1024).decode()
        if not data:
          # if data is not received break
          continue
        print("from connected user: " + str(data))

        response_callback(data, lambda response: conn.send(response.encode()))
      except socket.error as err:
          print(f"socket error: {err}")
          continue
      finally:
        try:
          conn.close()
        except:
          pass

  def send(self, message: str):
    client_socket = socket.socket()  # instantiate
    client_socket.connect((self.host, self.port))  # connect to the server

    client_socket.send(message.encode())  # send message
    data = client_socket.recv(1024).decode()  # receive response

    print('Received from server: ' + data)  # show in terminal

    client_socket.close()  # close the connection

    return data
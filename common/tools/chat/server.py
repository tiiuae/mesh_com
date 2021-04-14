#python3 server.py <ip address>
import select
import socket
import sys


IP = sys.argv[1]

# Function to send message to all connected clients
def send_to_all(sock, message):
	# Message not forwarded to server and sender itself
	for socket in connected_list:
		if socket not in [server_socket, sock]:
			try:
				socket.send(message)
			except:
				# if connection not available
				socket.close()
				connected_list.remove(socket)


if __name__ == "__main__":
	name = ""
	# dictionary to store address corresponding to username
	record = {}
	# List to keep track of socket descriptors
	connected_list = []
	buffer = 4096
	port = 5001

	server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	#server_socket.bind(("localhost", port))
	server_socket.bind((IP, port))
	server_socket.listen(10)  # listen almost 10 connection at one time

	# Add server socket to the list of readable connections
	connected_list.append(server_socket)

	print("\33[32m \t\t\t\tSERVER WORKING \33[0m")

	while 1:
		# Get the list sockets which are ready to be read through select
		rList, wList, error_sockets = select.select(connected_list, [], [])

		for sock in rList:
			# New connection
			if sock == server_socket:
				# Handle the case in which there is a new connection received through server_socket
				sockfd, addr = server_socket.accept()
				name = sockfd.recv(buffer)
				name = name.decode('utf-8')
				connected_list.append(sockfd)
				record[addr] = ""
				# print(record and conn list ",record,connected_list)

				# if repeated username
				if name in record.values():
					sockfd.send("\r\33[31m\33[1m Username already taken!\n\33[0m")
					del record[addr]
					connected_list.remove(sockfd)
					sockfd.close()
					continue
				else:
					# add name and address
					record[addr] = name
					print("Client (%s, %s) connected" % addr, " [", record[addr], "]")
					sockfd.send(b"\33[32m\r\33[1m Welcome to chat room. Enter 'tata' anytime to exit\n\33[0m")
					msg = "\33[32m\33[1m\r " + name + " joined the conversation \n\33[0m"
					send_to_all(sockfd, msg.encode('utf-8'))

			# Some incoming message from a client
			else:
				# Data from client
				try:
					data1 = sock.recv(buffer)
					data = data1.decode("utf-8")
					i, p = sock.getpeername()
					if data == "tata":
						msg = "\r\33[1m" + "\33[31m " + record[(i, p)] + " left the conversation \33[0m\n"
						send_to_all(sock, msg.encode('utf-8'))
						print("Client (%s, %s) is offline" % (i, p), " [", record[(i, p)], "]")
						del record[(i, p)]
						connected_list.remove(sock)
						sock.close()
						continue

					else:
						msg = "\r\33[1m" + "\33[35m " + record[(i, p)] + ": " + "\33[0m" + data + "\n"
						send_to_all(sock, msg.encode('utf-8'))

				# abrupt user exit
				except:
					(i, p) = sock.getpeername()
					msg = "\r\33[31m \33[1m" + record[(i, p)] + " left the conversation unexpectedly\33[0m\n"
					send_to_all(sock,msg.encode('utf-8'))
					print("Client (%s, %s) is offline (error)" % (i, p), " [", record[(i, p)], "]\n")
					del record[(i, p)]
					connected_list.remove(sock)
					sock.close()
					continue

	server_socket.close()

import socket 

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("192.168.2.127",8080))
s.listen(5)

while True: 
    client_socket, address = s.accept()
    print(f"Connected from {address}")
    client_socket.send(bytes("HELLO", "utf-8"))
    client_socket.close()
import socket
import csv

def send_tcp_packet(server_ip, server_port, message):
    try:
        # Create a TCP socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the server
        client_socket.connect((server_ip, server_port))
        print(f"Connected to {server_ip}:{server_port}")

        # Send the message
        client_socket.sendall(message.encode('utf-8'))
        print(f"Sent: {message}")

        # Optionally receive a response (if server sends one)
        response = client_socket.recv(1024).decode('utf-8')
        print(f"Received: {response}")

    except socket.error as e:
        print(f"Error: {e}")

    finally:
        # Close the connection
        client_socket.close()
        print("Connection closed.")

def read_csv_file():
    SERVER_IP = "192.168.1.159"
    SERVER_PORT = 5001
    with open('angles.txt', mode='r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            print(row)
            # MESSAGE = f"""
            #     set_angles({','.join(row)})
            #     wait_command_done()
            # """
            # send_tcp_packet(SERVER_IP, SERVER_PORT, MESSAGE)

if __name__ == "__main__":
    read_csv_file()
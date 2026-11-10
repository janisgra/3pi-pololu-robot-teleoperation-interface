import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.sendto(b'{"cmd":"buzzer"}', ("192.168.7.210", 5006))  # bridge IP; robot beeps
while b'"ack"' not in (reply := s.recv(512)):
    pass  # skip bridge_status etc.
print(reply.decode())

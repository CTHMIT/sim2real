from pymavlink import mavutil
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 14540))
print("listening on 127.0.0.1:14540 ...")
master = mavutil.mavlink_connection("udpout:127.0.0.1:14540")
master.wait_heartbeat()
print("Connected!")
while True:
    data, addr = s.recvfrom(4096)
    msg = master.recv_match(type=["MANUAL_CONTROL", "RC_CHANNELS"], blocking=True)
    print(msg)

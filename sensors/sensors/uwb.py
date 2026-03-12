import socket

import struct

UDP_IP = "192.168.1.100"

UDP_PORT = 9998

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

#Pulse repetition interval in us(1ms .. 1s)

cirinterval=4000

#Number of CIR taps (do not change for now)

cirtaps=64

#Tx frequency in KHz (6.4GHz - 8GHz)

frequency=6500000

#CIR offset in taps (0..1015)

ciroffset=0

#Mode:

# Bit0  0=Disable correlated data mode (not supported)

#       1=Enable correlated data mode (default)

# Bit 1 Reserved for future use (set to 0)

# Bit 2 0=Disable DC compensation mode (default)

#       1=Enable DC compensation mode

mode=1

#DC compensation filter 0x01 k=0.01 ... 0x63 k=0.99

dccompfiltercoef=0x01

#Assemble the data packet

packetdata = struct.pack('<IIIHBBBBBBBBBHHBBBBBBBBBBBBBBBB',cirinterval,cirtaps,frequency,ciroffset,0,0,0,0,0,0,0,0,0,mode,dccompfiltercoef,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)

sock.sendto(packetdata, (UDP_IP,UDP_PORT))

print("Sent configuration packet")
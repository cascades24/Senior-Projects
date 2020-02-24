import hashlib, binascii
import hmac
import base64
import os
import sys
import socket
import time
import struct
from subprocess import call,Popen
from bitarray import bitarray


rx_port = 35777
devIP = sys.argv[1]
satIP = sys.argv[2]
aes256_pass = sys.argv[3]

#Setting up the socket
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
try:
    sock.bind((devIP, rx_port))
except socket.error as e:
    print(e, flush=True)
sock.settimeout(3)
downlink_dir = sys.argv[4]

while True:

    print("Waiting for file info", flush=True)
    try:
      fileinfo, addr = sock.recvfrom(2048)
    except socket.timeout:
      continue

    print(fileinfo[0])

    msg_type = fileinfo[0]
    if msg_type == 1:
      print(fileinfo, flush=True)

      blocks = struct.unpack("<H", fileinfo[2:4])[0]
      filename = struct.unpack("32s", fileinfo[4:])[0].split(b'\0', 1)[0].decode("utf-8")

      r_blocks = blocks * bitarray('0')

      print("FileInfo received %s %d" % (filename, blocks), flush=True)

      missing_retry_counter = 0

      encr_filename = downlink_dir + "/" + "encr_" + filename
      filename = downlink_dir + "/" + filename
      with open(encr_filename, "wb") as f:

        while True:
          try:
            fileblock, addr = sock.recvfrom(2048)
          except socket.timeout:
            missing = []
            for i in range(len(r_blocks)):
              if not r_blocks[i]:
                missing.append(i)

            if len(missing) == 0:
              print("File RXed decrypting", flush=True)

              done = []
              done.append(struct.pack("B", int(5)))
              sock.sendto(b"".join(done), (satIP, rx_port))
              sock.sendto(b"".join(done), (satIP, rx_port))
              sock.sendto(b"".join(done), (satIP, rx_port))

              f.flush()
              dec_command = "openssl aes-256-cbc -d -md sha256 -pass pass:" + aes256_pass + " -in " + encr_filename + " -out " + filename
              Popen(dec_command.split(" "))
              break

            # Give up after 3 rerequests
            elif missing_retry_counter == 3:
              break

            else:
              if(len(missing) > 98):
                missing = missing[:97]
              block_request = []
              block_request.append(struct.pack("BB%uH"%len(missing), int(3),len(missing), *missing))
              sock.sendto(b"".join(block_request), (satIP, rx_port))
              print("Missing blocks: ", missing, flush=True)

              missing_retry_counter = missing_retry_counter + 1

              continue


          msg_type = fileblock[0]
          if msg_type == 2:

            missing_retry_counter = 0

            blen = fileblock[1]
            bnum = struct.unpack("<H", fileblock[2:4])[0]

            data = struct.unpack("%us" % blen, fileblock[4:4+blen])[0]

            print("Got block %d" % bnum, flush=True)

            r_blocks[bnum] = True;

            f.seek(196*bnum)
            f.write(data)

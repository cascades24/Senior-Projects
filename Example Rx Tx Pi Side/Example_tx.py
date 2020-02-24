import hashlib, binascii
import hmac
import base64
from subprocess import call
import subprocess
import os
import sys
import socket
import time
import struct
from collections import OrderedDict
from pprint import pprint
import random

sys.stdout.flush()
tx_port = 35778
rx_port = 5006

IP_dev = sys.argv[1]
IP_sat = sys.argv[2]
filepath = sys.argv[3]
aes256_pass= sys.argv[4]

def chunks(l, n):
  chunks = []
  pos = 0
  while pos < len(l):
    length = n
    if(pos + n > len(l)):
      length = len(l) - (pos + n)
    if length < 0:
        length = length + n
    chunks.append(l[pos:pos+length])
    pos = pos + n
  return chunks
file_list = ["SOT.txt", "SCH.bin", "ConfigUpACS","ConfigUpGPS","ConfigUpRADMotor","ConfigUpRADData","ConfigUpGND","ConfigUpSCH","ConfigUpFMG","IEF.txt"]

for ordered_filename in file_list:
    for filename in os.listdir(filepath):
        if filename != ordered_filename:
            continue
        #Encrypting the file with AES256
        filename = filepath  + '/' + filename
        aes_filename = filename[:-4] + '.aes'
        enc_command = "openssl enc -aes-256-cbc -in " + filename + " -md sha256 -pass pass:" + aes256_pass + ' -out ' + aes_filename
        call(enc_command.split(" "))

        fileinfo = []
        #Getting number of blocks in a file

        with open(aes_filename, 'rb') as f:
            chunked_file = chunks(f.read(), 196)
        #fileinfo.append(struct.pack("<i",len(chunked_file)))

        #fileinfo.append(struct.pack("32s", aes_filename.split('/')[-1].encode("utf-8"))) #+ b"\x00"*(32-len(filename)))

        fileinfo.append(struct.pack("BH32s", int(1), len(chunked_file), filename.split('/')[-1].encode("utf-8"))) # type Pack

        #Getting file size
        statinfo = os.stat(filename)
        #fileinfo.append(struct.pack("<i",statinfo.st_size))

        #pprint(fileinfo)

        fileinfo_packed = b"".join(fileinfo)

        #Setting up the socket
        sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        sock.bind((IP_dev, rx_port))
        sock.settimeout(10)

        # ditched response, just send 3 times
        #pprint(binascii.hexlify(fileinfo_packed), flush=True)

        for i in range(6):
          print("Sending Fileinfo of", aes_filename+"...", flush=True)
          sock.sendto(fileinfo_packed, (IP_sat, tx_port))

          time.sleep(1.5)

        chunked_file_tmp = chunked_file
        for idx, chunk in enumerate(chunked_file_tmp):
            chunked_file_tmp[idx] = (struct.pack("<"+str(len(chunk))+"B", *chunk), idx)


        while True:
            print("Sending blocks:", [x[1] for x in chunked_file_tmp], flush=True)
            for ch_tuple in chunked_file_tmp:
                chunk_num = ch_tuple[1]
                chunk = ch_tuple[0]
                # r = random.random()
                # if r < 0.1:
                #     print("Block " + str(chunk_num)  + " skipped")
                #     continue #skip half of chunk
                chunk = struct.pack("<B", int(2)) + struct.pack("<B", int(len(chunk))) + struct.pack("<H", int(chunk_num)) + chunk
                sock.sendto(chunk, (IP_sat, tx_port))
                time.sleep(0.01)
            missing_blocks = []
            mblock_type = None
            while True:
                try:
                    mblock_request, addr = sock.recvfrom(2048)
                except socket.timeout:
                    break
                mblock_type = struct.unpack("<B", mblock_request[0:1])[0]

                if(mblock_type == 3):

                    mblock_num = struct.unpack("<B", mblock_request[1:2])[0]
                    missing_blocks = struct.unpack("<%uH" % mblock_num, mblock_request[2:2+mblock_num*2])
                    print("Request for missing blocks was received:",missing_blocks, flush=True)

                    time.sleep(1)

                    chunked_file_tmp = [chunked_file[x] for x in missing_blocks]
                    break
            if (mblock_type != 3) or mblock_type is None:
                break

        print("Finished sending file " + aes_filename, flush=True)
        remove_cmd = "rm " + aes_filename
        subprocess.Popen(remove_cmd.split(" "))

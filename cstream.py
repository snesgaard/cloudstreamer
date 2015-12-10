import zmq # Make sure this is 4.0+
import numpy as np
import sys
import time

class CloudStream:
    def __init__(self, c, s):
        self.context = c
        self.socket = s

# Create object to hold the connection instead of tuple
def connect(identity = None, host = "ipc:///tmp/datastream", context = None):
    icontext = context
    if icontext is None:
        context = zmq.Context()
    sock = context.socket(zmq.DEALER)
    if identity is None:
        identity = str(np.random.uniform(0, 0xffffffff))

    sock.setsockopt(zmq.IDENTITY, identity.encode('ascii'))
    sock.connect("ipc:///tmp/datastream")
    # This trickery is to indicate whether we created the context or not
    if context is not None:
        icontext = None
    return CloudStream(icontext, sock)

# Data must be formatted as a matrix of row vectors
# [x1 y1 z1]
# [x2 y2 z2]
# [:  :  : ]
# [xn yn zn]
def send(cloudstream, data, timeout = 0.25):
    sock = cloudstream.socket
    data = data.reshape(-1)
    sock.send("upload")
    alive = time.time()
    while time.time() - alive < timeout:
        try:
            msg = sock.recv_multipart(flags = zmq.DONTWAIT)
            pstart, size = msg
            pstart = np.fromstring(pstart, "int32", 1)[0] * 3
            size = np.fromstring(size, "int32", 1)[0] * 3
            pend = pstart + size
            #print "sending", pstart, pend
            #print("sending", pstart, pend, data.shape[0])
            if pstart < 0:
                return True
            elif pstart <= data.shape[0]:
                chunk = data[pstart:min(data.shape[0], pend)].tostring()
                sock.send(chunk)
            alive = time.time()
        except zmq.error.Again:
            continue
    print "Connection timeout"
    return False

def close(cloudstream):
    cloudstream.socket.close()
    if cloudstream.context is not None:
        cloudstream.context.term()

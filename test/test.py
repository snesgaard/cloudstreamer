import zmq
import time
import numpy as np
import sys

# ZeroMQ Context
context = zmq.Context()

identity = u'yolo'
if len(sys.argv) >= 2:
    identity = sys.argv[1]

print("binding as", identity)
# Define the socket using the "Context"
sock = context.socket(zmq.DEALER)
#sock.identity = identity.encode('ascii')
sock.setsockopt(zmq.IDENTITY, identity.encode('ascii'))
sock.connect("ipc:///tmp/datastream")

id = 0

def createdata(pcount):
    x = np.random.uniform(0, 10, pcount)
    y = np.random.uniform(0, 10, pcount)
    z = np.random.uniform(0, 10, pcount)
    p = np.array([x, y, z]).T.astype("float32")
    return p.reshape(-1)

offz = np.random.uniform(-10, 10)

def sinedata(t, pcount):
    x, y = np.meshgrid(np.arange(pcount), np.arange(pcount))
    x = x.reshape(-1).astype("float32") * 10 / pcount
    y = y.reshape(-1).astype("float32") * 10 / pcount
    z = np.sin(x + t * 100) + offz
    #x = np.arange(0, pcount).astype("float32") * 10.0 / pcount
    #y = np.arange(0, pcount).astype("float32") * 10.0 / pcount
    #z = np.zeros(pcount)

    p = np.array([x, y, z]).T.astype("float32")
    return p.reshape(-1)
    """
    # Flatten array
    p = p.reshape(-1)
    # split into chunks
    # Size is defined in points real size will be s x 3 x sizeof(float32)
    chunksize = 5
    size = chunksize * 3
    data = []
    for i in xrange((p.shape[0] - 1) / size + 1):
        minit = i * size
        mend = min(minit + size, p.shape[0])
        data.append(p[minit:mend].tostring())
    return data
    """

#print "sending x",x,"as",x.tostring()
phase = 0
last = time.time()
try:
    data = sinedata(phase, 1000)
    while True:
        print "Uploading", time.time() - last
        last = time.time()
        #data = sinedata(phase, 1000)
        phase = phase + 0.002
        #print(data.shape)
        #print("requesting")
        sock.send("upload")
        while True:
            msg = sock.recv_multipart()
            pstart, size = msg
            pstart = np.fromstring(pstart, "int32", 1)[0] * 3
            size = np.fromstring(size, "int32", 1)[0] * 3
            pend = pstart + size
            #print("sending", pstart, pend, data.shape[0])
            if pstart < 0:
                break
            elif pstart <= data.shape[0]:
                chunk = data[pstart:min(data.shape[0], pend)].tostring()
                sock.send(chunk)
finally:
    sock.close()
    print "done"

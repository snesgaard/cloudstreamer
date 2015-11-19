import zmq

# ZeroMQ Context
context = zmq.Context()

# Define the socket using the "Context"
sock = context.socket(zmq.SUB)

# Define subscription and messages with prefix to accept.
sock.setsockopt(zmq.SUBSCRIBE, "1")
sock.connect("ipc:///tmp/datastream")

while True:
    message= sock.recv()
    print message

import zmq

pub = "ipc:///tmp/datastream"

cont = zmq.Context()
socket = cont.socket(zmq.PUB)
socket.bind(pub)
socket.send_multipart(["key", "lololol"])
socket.close()

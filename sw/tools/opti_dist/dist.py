import time
import sys
import math
import matplotlib.pyplot as plt
from NatNetClient import NatNetClient

pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
plt.axis([-6, 6, -6, 6])

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    # print( "Received frame", frameNumber )
	pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    # print( "Received frame for rigid body", id )
	if id==13:
		global pos_x, pos_y, pos_z
		pos_x = position[0]
		pos_y = position[1]
		pos_z = position[2]
		# precision = 1
		# pos_x = round(pos_x, precision)
		# pos_y = round(pos_y, precision)
		# pos_z = round(pos_z, precision)
	else:
		pass

# This will create a new NatNet client
streamingClient = NatNetClient()
# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()

time.sleep(2)
print('Start tracking')
# file = open('opti.csv', 'w')
# file.write('timestamp, x, y, z\n')

old_z = pos_z
old_x = pos_x
distance = 0
start_time = time.time()
pre_time = time.time()

while True:
	# data = '{}, {}, {}, {}\n'.format(int((time.time() - start_time) * 1000), pos_x, pos_y, pos_z)
	# file.write(data)
	# distance = distance + math.hypot(pos_z-old_z, pos_x-old_x)
	h = math.hypot(pos_z-old_z, pos_x-old_x)
	if h > 0.20:
		distance += h
		old_z = pos_z
		old_x = pos_x
	if time.time()-pre_time>0.5:
		print("distance:%3.4f m; time_step:%d" % (distance, int((time.time() - start_time) * 2)))
		pre_time = time.time()
	plt.plot(pos_z, pos_x, 'ro')
	plt.draw()
	plt.pause(0.001)
	time.sleep(0.01)

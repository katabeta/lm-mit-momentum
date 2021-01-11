import time # For the example only
import asyncio
import pygazebo

# What you import here depends on the message type you are subscribing to
import pygazebo.msg.v11.laserscan_stamped_pb2


import numpy as np # for transposing across coordinate systems

import sys

# This is the gazebo master from PX4 message `[Msg] Connected to gazebo master @ http://127.0.0.1:11345`
HOST, PORT = "127.0.0.1", 11345


class GazeboMessageSubscriber: 

    def __init__(self, host, port, timeout=30):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout

    async def connect(self):
        connected = False
        for i in range(self.timeout):
            try:
                self.manager = await pygazebo.connect((self.host, self.port))
                connected = True
                break
            except Exception as e:
                print(e)
            await asyncio.sleep(1)

        if connected: 
            # info from gz topic -l, gz topic -i arg goes here
            self.gps_subscriber = self.manager.subscribe('/gazebo/default/iris_lmlidar/lmlidar/link/lmlidar/scan', 'gazebo.msgs.LaserScanStamped', self.LaserScanStamped_callback)

            await self.gps_subscriber.wait_for_connection()
            self.running = True
            while self.running:
                await asyncio.sleep(0.1)
        else:
            raise Exception("Timeout connecting to Gazebo.")

    def LaserScanStamped_callback(self, data):
        # What *_pb2 you use here depends on the message type you are subscribing to
        self.LaserScanStamped = pygazebo.msg.v11.laserscan_stamped_pb2.LaserScanStamped()
        self.LaserScanStamped.ParseFromString(data)
    
    async def get_LaserScanStamped(self):
        for i in range(self.timeout):
            try:
                return self.LaserScanStamped
            except Exception as e:
                pass
            await asyncio.sleep(1)


def quat_mult(quaternion1, quaternion0):
  # Hamilton Product
  w0 = quaternion0[0]
  x0 = quaternion0[1]
  y0 = quaternion0[2]
  z0 = quaternion0[3]
  w1 = quaternion1[0]
  x1 = quaternion1[1]
  y1 = quaternion1[2]
  z1 = quaternion1[3]
  return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                    x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                    -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                    x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)



def get_world_obst(lsr_val):
  # Laser returns are returned in a 1D array ordered as [v1h1 .. v20h1 v1h2 .. v20h2 v1h3 .. v20h3 ...], where v is the vertical index, and h is the horizontal index.
  # https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion    
  # All quaternions expressed in the order [w, i, j, k]

  # Rotation quat
  R       = np.array([lsr_val.scan.world_pose.orientation.w,  lsr_val.scan.world_pose.orientation.x,  lsr_val.scan.world_pose.orientation.y,  lsr_val.scan.world_pose.orientation.z])
  
  # Conjugate of rotation quat
  R_prime = np.array([lsr_val.scan.world_pose.orientation.w, -lsr_val.scan.world_pose.orientation.x, -lsr_val.scan.world_pose.orientation.y, -lsr_val.scan.world_pose.orientation.z])
  
  # Linear displacement vector in pure quat form
  disp = np.array([[0], [lsr_val.scan.world_pose.position.x], [lsr_val.scan.world_pose.position.y], [lsr_val.scan.world_pose.position.z]])
  
  # Linearly space vector of describing horizontal range in radians
  h_range = np.linspace(lsr_val.scan.angle_min, lsr_val.scan.angle_max, lsr_val.scan.count, dtype=np.float64)
  
  # Linearly space vector of describing vertical range in radians
  v_range = np.linspace(lsr_val.scan.vertical_angle_min, lsr_val.scan.vertical_angle_max, lsr_val.scan.vertical_count, dtype=np.float64)

  # Tile and repeat variables to allow vectorized operations
  R       = np.tile(R,          lsr_val.scan.count * lsr_val.scan.vertical_count)
  R_prime = np.tile(R_prime,    lsr_val.scan.count * lsr_val.scan.vertical_count)  
  disp    = np.tile(disp,       lsr_val.scan.count * lsr_val.scan.vertical_count)
  h_range = np.tile(h_range,    lsr_val.scan.vertical_count)
  v_range = np.repeat(v_range,  lsr_val.scan.count)

  # Get x, y, z components of the range readings in sensor frame and pad to assume pure quat form
  P = np.array([lsr_val.scan.ranges * np.sin(h_range), lsr_val.scan.ranges * np.sin(v_range), lsr_val.scan.ranges * np.cos(v_range)])
  P = np.concatenate((np.array([np.zeros(P.shape[1])]), P), axis=0)

  # Multiply out the rotation (RPR')
  P_prime = quat_mult(quat_mult(R, P), R_prime) + disp

  # Remove the pad and transpose for easy reading 
  result = P_prime[1:4].T

  # Access using the index of the return you'd like (in the format[v1h1 .. v20h1 v1h2 .. v20h2 v1h3 .. v20h3 ...]) and index of axis x=0, y=1, z=2
  # e.g. result[-60:-40, 0:3] gives 20 x-y-z values starting from 60 from the end and ending at 40 from the end

  return result
    

async def run():
    gz_sub = GazeboMessageSubscriber(HOST, PORT)
    asyncio.ensure_future(gz_sub.connect())
    lsr_val = await gz_sub.get_LaserScanStamped()
    # Simulate doing stuff and polling for the gps values only when needed
    start = time.time()
    current_time = 0
    last_time = 0
    while (current_time < 3):
        current_time = round(time.time() - start)
        if(current_time % 1 == 0 and last_time < current_time):
            lsr_val = await gz_sub.get_LaserScanStamped()
            print(get_world_obst(lsr_val))

            # # Saving the objects:
            # with open('objs.pkl', 'wb') as f:  # Python 3: open(..., 'wb')
            #     f.write(lsr_val.SerializeToString())


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
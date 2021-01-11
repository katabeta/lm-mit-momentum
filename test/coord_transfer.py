
import sys
import math
import time
import numpy as np
from dual_quaternions import DualQuaternion
import pygazebo.msg.v11.laserscan_stamped_pb2



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



def get_world_obst_dual_quat(lsr_val):
  # Make a qual quat
  dq = DualQuaternion.from_quat_pose_array([lsr_val.scan.world_pose.orientation.w,
                                            lsr_val.scan.world_pose.orientation.x,  
                                            lsr_val.scan.world_pose.orientation.y,  
                                            lsr_val.scan.world_pose.orientation.z,
                                            lsr_val.scan.world_pose.position.x,
                                            lsr_val.scan.world_pose.position.y,
                                            lsr_val.scan.world_pose.position.z])
  # Linearly space vector of describing horizontal range in radians
  h_range = np.linspace(lsr_val.scan.angle_min, lsr_val.scan.angle_max, lsr_val.scan.count, dtype=np.float64)
  # Linearly space vector of describing vertical range in radians
  v_range = np.linspace(lsr_val.scan.vertical_angle_min, lsr_val.scan.vertical_angle_max, lsr_val.scan.vertical_count, dtype=np.float64)

  # Tile and repeat variables to allow vectorized operations
  h_range = np.tile(h_range, lsr_val.scan.vertical_count)
  v_range = np.repeat(v_range, lsr_val.scan.count)

  # Get x, y, z components of the range readings in sensor frame
  P = np.array([lsr_val.scan.ranges * np.sin(h_range), lsr_val.scan.ranges * np.sin(v_range), lsr_val.scan.ranges * np.cos(v_range)])

  result = []

  # Do the transform on each return
  for i in range(lsr_val.scan.count * lsr_val.scan.vertical_count):
    result.append(dq.transform_point(P[0:3, i]))

  # Access using the index of the return you'd like (in the format[v1h1 .. v20h1 v1h2 .. v20h2 v1h3 .. v20h3 ...]) and index of axis x=0, y=1, z=2
  # e.g. result[-60:-40, 0:3] gives 20 x-y-z values starting from 60 from the end and ending at 40 from the end
  return np.array(result)



# Get saved message from file
lsr_val = pygazebo.msg.v11.laserscan_stamped_pb2.LaserScanStamped()
f = open('objs.pkl', "rb")
lsr_val.ParseFromString(f.read())
f.close()

print (lsr_val)

# Some of the values are inf and numpy complains when doing math with them, so turn off warnings
np.seterr(all='ignore') 

# Manual approach
start_time = time.time()
result = get_world_obst(lsr_val)
vec_time = time.time() - start_time
print(vec_time)
print(result)
print(result.shape)
print(result[-60:-40, 0:3])

# Dual quat approach using library to double check
start_time = time.time()
result = get_world_obst_dual_quat(lsr_val)
dual_time = time.time() - start_time
print(dual_time)
print(result)
print(result.shape)
print(result[-60:-40, 0:3])
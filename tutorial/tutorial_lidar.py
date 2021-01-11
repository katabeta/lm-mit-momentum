import time # For the example only
import asyncio
import pygazebo

# What you import here depends on the message type you are subscribing to
# HINT: The following import looks important and probably shouldn't be commented out
# import pygazebo.msg.v11.laserscan_stamped_pb2

# This is the gazebo master from PX4 message `[Msg] Connected to gazebo master @ http://127.0.0.1:11345`
HOST, PORT = "127.0.0.1", 11345

# If you would like a better understanding, move through the comments 
# and search for any notable 'EXERCISE's
class GazeboMessageSubscriber: 

    def __init__(self, host, port, timeout=30):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout

    # PX4 SITL will communicate with the simulated drone firmware
    # While running localhost (127.0.0.1), PX4 will use the default port 11345
    # ========================================================================
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
            # While running PX4 and Gazebo, then use command-line tools to take a peek at what Gazebo is publishing:
            # Find our topic using: 
            #       gz topic -l(--list)
            # Then, once we know who to listen to, then find out what type of data they are trying to send:
            #       gz topic -i(--info) /topic/goes/here
            
            # EXERCISE: Then fill out the following variables with the correct information:
            laser_scan_topic = "/this/is/not/the/data/you're/looking/for"
            laser_scan_topic_msg = "gazebo.msgs.Ignorance"

            self.ls_subscriber = self.manager.subscribe(laser_scan_topic, laser_scan_topic_msg, self.LaserScanStamped_callback)

            await self.ls_subscriber.wait_for_connection()
            self.running = True
            while self.running:
                await asyncio.sleep(0.1)
        else:
            raise Exception("Timeout connecting to Gazebo.")

    # Incoming messages aren't to be stored or interpreted directly from Gazebo
    # Instead, we follow the approach most applications use when subscribing to multiple sets of data: callbacks
    # ==========================================================================================================
    def LaserScanStamped_callback(self, data):
        # What *_pb2 you use here depends on the message type you are subscribing to
        # NOTE: If you want to subscribe to other messages, then please import responsibly!
        self.LaserScanStamped = pygazebo.msg.v11.laserscan_stamped_pb2.LaserScanStamped()
        self.LaserScanStamped.ParseFromString(data)
    
    # This is a nice way to verify for any errors while copying the last known LaserScan
    # ==================================================================================
    async def get_LaserScanStamped(self):
        for i in range(self.timeout):
            try:
                return self.LaserScanStamped
            except Exception as e:
                # print(e)
                pass
            await asyncio.sleep(1)

    # NOTE: This is your PLAYGROUND
    # The message has been subscribed and saved. Iterate through the LaserScan
    # To take a 'snapshot' of the published LaserScan data then,
    #       gz topic -e(--echo) /laser/scan/topic
    # ========================================================================
    async def display_laserScan()
        # Last known timestamps, use at your own will
        sec, nsec = self.LaserScanStamped.time.sec, self.LaserScanStamped.time.nsec

        # Get the vertical and horizontal bounds for the LiDAR
        # PLEASE fill in the correct values (based on Gazebo's published message)
        # NOTE: PX4 adores radians and will interpret your values as such. Prepare!
        horz_angle_min = -180   
        horz_angle_max = 180
        horz_angle_step = 0.01 
        horz_sz = self.LaserScanStamped.scan.count  # Number of horizontal scan lines  

        vert_angle_min = -180
        vert_angle_max = 180
        vert_angle_step = 0.01 
        vert_sz = self.LaserScanStamped.scan.vertical_count; # Number of vertical scan lines

        min_range = self.LaserScanStamped.scan.range_min    # in meters
        max_range = self.LaserScanStamped.scan.range_max    # in meters

        # NOOOOW, we have a 2D array to go over, but it's been compressed into 1D
        # NOTE: How can you use the values above to iterate through the flattened-2D array?
        #       Row-major or Col-major? Find out, after testing!
        first_range = self.LaserScanStamped.scan.ranges[0]
        last_range  = self.LaserScanStamped.scan.ranges[horz_sz*vert_sz - 1]

        # EXERCISE: Use the drone's position and orientation to find the index of the laser scan directly below the drone
        # NOTE: Only when the drone's Yaw, Pitch, or Roll are 0 (zero), 
        #       will the laserScan pointing directly below the drone (last_range) actually represent the height above ground.
        #       Otherwise, you have to approximate, or use kinematics to find the range index you want       
        #       Draw it out if you don't believe me.

async def run():
    # Start up PX4_SITL, begin drone hardware simulation, and Gazebo publishers
    gz_sub = GazeboMessageSubscriber(HOST, PORT)
    asyncio.ensure_future(gz_sub.connect())

    # Simulate doing stuff and polling for the gps values only when needed
    start = time.time()
    current_time = 0
    last_time = 0

    # Simulate for a certain, hardcoded amount of time (20 seconds or so)...
    while (current_time < 20):
        current_time = round(time.time() - start)

        # Simply put, at every interval (5 seconds) use our subscriber to receive and copy the last known LaserScan topic 
        if(current_time % 5 == 0 and last_time < current_time):
            gps_val = await gz_sub.get_LaserScanStamped() # Stamped messages contain a timestamp!
            
            # EXERCISE: USER CAN NOW MANIPULATE LASER SCAN!
            display_laserScan()

        # Else, print out the elapsed time to let the user know that the code is still working, so wait!
        # NOTE: Grabbing the stamped LaserScan at every 0.001 milliseconds can hurt more than help
        #       After all, how much did the drone move in that elapsed time?
        #       Will the readings be drastically  from before?
        if(current_time % 1  == 0 and last_time < current_time):
            print(current_time)

        # Update known time at the end of the loop
        last_time = current_time


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

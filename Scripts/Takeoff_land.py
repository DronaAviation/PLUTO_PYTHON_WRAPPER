from Utils import path  
path()

from src.drone import drone
drone=drone()
drone.connect()
#trim(Roll,Pitch,Throttle,Yaw)
drone.trim(0,0,0,0)
drone.disarm()
drone.takeoff()
drone.throttle_speed(7,1) #(value, Duration)
drone.land()

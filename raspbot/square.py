import roslibpy
import time
from math import pi

if __name__ == '__main__':
  client = roslibpy.Ros('144.39.133.25', 9090)
  client.run()
  talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
  
  for i in range(4):
    # forward
    talker.publish({ 'linear': { 'x': 0.025, 'y': 0.0, 'z': 0.0 }, 'angular': { 'x': 0.0, 'y': 0.0, 'z': 0.0 } })
    time.sleep(1.5)
    talker.publish({ 'linear': { 'x': 0.0, 'y': 0.0, 'z': 0.0 }, 'angular': { 'x': 0.0, 'y': 0.0, 'z': 0.1 * pi } })
    time.sleep(1.25)
  talker.publish({ 'linear': { 'x': 0.0, 'y': 0.0, 'z': 0.0 }, 'angular': { 'x': 0.0, 'y': 0.0, 'z': 0.0 } })
  time.sleep(1)
  client.close()


from pynput.keyboard import Key, KeyCode, Listener
import roslibpy
import time

class KeyboardTeleop:
  def __init__(self):
    self.client = roslibpy.Ros('144.39.133.25', 9090)
    self.client.run()
    self.talker = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')
    self.x_vel = 0.0
    self.z_angular = 0.0
    self.last_time = time.time()
    self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
    self.listener.start()
    
    while self.client.is_connected:
      self.talker.publish({ 'linear': { 'x': self.x_vel, 'y': 0.0, 'z': 0.0 }, 'angular': { 'x': 0.0, 'y': 0.0, 'z': self.z_angular } })
      time.sleep(0.05)

  def on_press(self, key: KeyCode):
    if key.char == 'w' or key == Key.up:
      self.x_vel = 0.025
    if key.char == 's' or key == Key.down:
      self.x_vel = -0.025
    if key.char == 'a' or key == Key.left:
      self.z_angular = 0.25
    if key.char == 'd' or key == Key.right:
      self.z_angular = -0.25

  def on_release(self, key: KeyCode):
    if key.char == 'w' or key == Key.up:
      self.x_vel = 0.0
    if key.char == 's' or key == Key.down:
      self.x_vel = 0.0
    if key.char == 'a' or key == Key.left:
      self.z_angular = 0.0
    if key.char == 'd' or key == Key.right:
      self.z_angular = 0.0

if __name__ == '__main__':
  teleop = KeyboardTeleop()
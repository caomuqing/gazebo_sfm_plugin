#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature, Humidity
import threading


class EnvMonitor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('env_monitor', anonymous=True)
        # # Define the publisher to publish environment data (e.g., temperature)
        self_pub = rospy.Publisher('/env/temperature', Float32, queue_size=10)
        # self.humidity_pub = rospy.Publisher('/env/humidity', Float32, queue_size=10)

        # # Define the subscriber to subscribe to temperature and humidity sensor data
        # rospy.Subscriber('/sensor/temperature', Temperature, self.temp_callback)
        # rospy.Subscriber('/sensor/humidity', Humidity, self.humidity_callback)

        # # Variables to store sensor data
        # self.current_temp = 0.0
        # self.current_humidity = 0.0

        # Mutex lock for thread-safe access to shared variables
        self.lock = threading.Lock()


if __name__ == '__main__':
    try:
        monitor = EnvMonitor()
        monitor.start_monitoring()
    except rospy.ROSInterruptException:
        pass
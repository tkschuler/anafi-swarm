import socket
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

class ROSClient:

    def __init__(self, IP, Port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((IP, Port))
        self.client.setblocking(0)

    def callback(self,data):
        rospy.loginfo("Received Vicon Data")
        #Send data to server
        x = data.twist.linear.x
        y = data.twist.linear.x
        z = data.twist.linear.x
        roll = data.twist.angular.x
        pitch = data.twist.angular.y
        yaw = data.twist.angular.z
        output = str(x) +","  + str(y) +"," + str(z) +"," + str(roll) +"," + str(pitch) +"," + str(yaw) + "\n"
        self.client.send(output)

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("vicon/vicon/anafi_1", TwistStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

IP = "127.0.0.1"
Port = 50001

c = ROSClient(IP,Port)
c.listener()

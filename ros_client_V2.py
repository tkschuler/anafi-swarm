import socket
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class ROSClient:

    def __init__(self, IP, Port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((IP, Port))
        self.client.setblocking(0)

    def callback(self,data):
        rospy.loginfo("Received Vicon Data")
        #Send data to server
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        qx= data.pose.orientation.x
        qy= data.pose.orientation.y
        qz= data.pose.orientation.z
        qw= data.pose.orientation.w
        output = str(x) +","  + str(y) +"," + str(z) +"," + str(qx) +"," + str(qy) +"," + str(qz) + "," + str(qw) + "\n"
        self.client.send(output)

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("vicon/anafi_1/pose", PoseStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

IP = "127.0.0.1"
Port = 50006

c = ROSClient(IP,Port)
c.listener()

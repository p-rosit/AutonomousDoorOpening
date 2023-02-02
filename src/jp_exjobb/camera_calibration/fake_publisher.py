import rospy
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

if __name__ == '__main__':
    topic = '/real_sense/rgb/image_raw'

    # load images from check and convert with cv_bridge
    images = []

    rospy.init_node('fake_image_publisher')
    ind = 0
    pub = rospy.Publisher(topic, Image, queue_size=1)

    def publish_callback(_):
        global ind
        pub.publish(images[ind])
        ind = (ind + 1) % len(images)

    sub = rospy.Subscriber('/camera_calibration/take_picture', String, callback=publish_callback)
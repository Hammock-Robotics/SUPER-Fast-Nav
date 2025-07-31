import rospy
from std_msgs.msg import String


def talker():
    rospy.init_node("talker", anonymous=False)

    pub = rospy.Publisher("chatter", String, queue_size=3)
    rate = rospy.Rate(1)
    msg_text = rospy.get_param("~message", "default message is being sent")

    while not rospy.is_shutdown():
        if rospy.has_param("~message"):
            msg_text= rospy.get_param("~message")
        
        # Build and send the message
        msg = String(data=msg_text)
        rospy.loginfo(f"Publishing: {msg.data}")
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
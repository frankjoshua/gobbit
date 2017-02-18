
            cmd.linear.x = 0
        lastError = error
    #Publish the message
    pub.publish(cmd)

def intersection(msg):
    global state
    #update state
#    state = msg.data
#    print(str(msg.data))

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('user_code_node', anonymous=False)
	rospy.Subscriber('/line/filtered', Int32, line, queue_size=1)

    print "running uploaded blockly code"

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

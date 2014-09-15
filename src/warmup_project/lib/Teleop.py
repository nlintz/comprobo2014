from geometry_msgs.msg import Twist, Vector3
import rospy

def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def charToCommand(char):
    if char == 'w':
        return Twist(linear=Vector3(x=1.0))

    if char == 's':
        return Twist(linear=Vector3(x=-1.0))

    if char == 'a':
        return Twist(angular=Vector3(z=.5))

    if char == 'd':
        return Twist(angular=Vector3(z=-.5))

    if char == 'p':
        exit()

    return Twist()

def teleop():
    telopPublisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
    rospy.init_node('teleop', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        char = getch()
        msg = charToCommand(char)
        print msg
        telopPublisher.publish(msg)
        r.sleep()
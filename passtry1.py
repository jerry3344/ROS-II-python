import rospy
import time
import simple_tello
from geomatry_msgs.msg import Twist
from std_msgs.msg import Empty

def tello_pass(t1):

    #control loop
    check = False
    while not rospy.is_shutdown():
        #wait
        while t1.state.target_x == -1 and t1.state.target_y == -1:
            pass
        print(t1.state.target_x, t1.state.target_y)

        dx = t1.state.target_x - 480
        dy = t1.state.target_y - 200

        if t1.state.canPass == 1:
            msg = Twist()
            msg.linear.y = 0.4
            msg.linear.z = 0.1
            t1.controler.move(msg, 3)

            msg = Twist()
            msg.linear.y = 0.5
            t1.controler.move(msg, 3)

            msg = Twist()
            t1.controler.move(msg, 1)
            break

        elif t1.state.canPss == 0:
            if abs(dx) < 22 and abs(dy) < 22:
                check = True
                msg = Twist()
                msg.linear.y = 0.3
                t1.controler.move(msg, 0.5)
            else:
                msg = Twist()
                if dx != 0:
                    msg.linear.x = dx / abs(dx) * 0.2
                if dy != 0:
                    msg.linear.z = -dy / abs(dy) * 0.3
                
                t1.controler.move(msg, 0.5)

        else:
            if abs(dx) >= 63 or abs(dy) >= 33:
                check = False
                msg = Twist()
                if dx !=0:
                    msg.linear.x = dx / abs(dx) * 0.2
                if dy != 0:
                    msg.linear.z = -dy / abs(dy) * 0.3

                t1.controler.move(msg, 0.5)
            
            else:
                msg = Twist()
                msg.linear.y = 0.3
                t1.controler.move(msg, 0.5)

def main():
    t1 = simple_tello.Tello_drone()

    while t1.state.is_flying == False:
        t1.controler.takeoff()
    
    while t1.state.fly_mode != 6:
        print("wait")
    
    tello_pass(t1)

    while t1.state.fly_mode != 6:
        print("wait")
    
    while t1.state.is_flying == True:
        t1.controler.land()

if __name__ == "__main__":
    rospy.init_node("pass_example", anonymous = True)
    main()
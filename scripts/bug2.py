#!/usr/bin/env python3
import rospy
import math
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion

DESIRED_DISTANCE_LEFT =1
DESIRED_DISTANCE_RIGHT =1

current_orientation = 1
current_x = 1
current_y = 1
theta = 1
incoming_range = []
goal_orientation = 0.7216485

wall_follow_flag = False
line_follow_flag = False

state = "LINE_FOLOW"

for i in range(361):
    incoming_range.append(3)

def laser_output_callback(message):

    global incoming_range


    incoming_range = message.ranges

    #print(incoming_range)

def forced_turn():

    run = Twist()

    run.linear.x = 0
    run.angular.z = -2.7 # -5.6

    pub.publish(run)

def odom_callback(msg):
    global current_orientation
    global theta
    global current_x , current_y 

    current_x=msg.pose.pose.position.x
    current_y=msg.pose.pose.position.y

    current_orientation = msg.pose.pose.orientation 
    [R ,P, theta] = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
    
def stop():
    run=Twist()
    run.linear.x =0

    pub.publish(run) 

def orientation():
    
    global goal_orientation

    run = Twist()
    
    goal_orientation = 0.7216485 # slope of line

    Kp = 1.5

    run.angular.z = Kp * (goal_orientation-theta)

    #print("theta",theta)
    #print("g-t: ", (goal_orientation-theta))

    pub.publish(run)

def move_forward():

    #print("in move_forward")

    run = Twist()
    
    run.linear.x = 4

    pub.publish(run)

def obstacle_detection():

    #print(incoming_range[180])

    average = sum(incoming_range[178:182])/len(incoming_range[178:182])
    

    if (average < 1.2):
        print("object_detected")
        return True
    else:
        return False
    
def orientation_check():
    
    if(round(goal_orientation,3) == round(theta,3)):
        return True
    else:
        return False

def set_robot():
    orientation_flag = orientation_check()

    print("orientation set ? :" ,orientation_flag)

    while( not orientation_flag):
        orientation()
        
        orientation_flag = orientation_check()
        if(orientation_flag == True):
            break

def get_distance():

    x1 = -8.0
    y1 = -2.0
    x2 = 4.5
    y2 = 9.0

    A=y1-y2
    B=x2-x1
    C=x1*y2-y1*x2

    distance = abs( (A*current_x + B*current_y+ C)/math.sqrt(A*A + B*B))

    return distance

def distance_to_goal():
    x2=4.5
    y2=9.0

    distance = math.sqrt(abs((x2-current_x)**2 + (y2-current_y)**2))

    return distance


def bug2():

    state = "LINE_FOLLOW"

    while not rospy.is_shutdown():
        global line_follow_flag

        
        if(state == "LINE_FOLLOW"):
            set_robot()
            while True:
                move_forward()
                print(state)
                goal_distance = distance_to_goal()
                print("Distance to goal: ", goal_distance)
                if(goal_distance< 0.8):
                    print("Close to goal")
                    state="UNKNOWN"
                    break
                object_detected =obstacle_detection()
                if(object_detected):
                    state = "WALL_FOLLOW"
                    wall_follow_flag = True
                    stop()
                    break

        if(state == "WALL_FOLLOW"):
            while True:
                wallfollow()
                print("in wall follow 1")

                if(distance_to_goal() < 0.8):
                    print("Goal distance :", distance_to_goal())
                    print("Close to goal")
                    stop()
                    state="UNKNOWN"
                    break

                if(get_distance()<0.2 and line_follow_flag==True):
                    while True:
                        wallfollow()
                        print("in wall follow 2")
                        
                        if(get_distance()> 1):
                           line_follow_flag = False
                           print(line_follow_flag)

                           break



                if(get_distance() < 0.3 and line_follow_flag == False):
                    print("distance to line: ", get_distance())
                    stop()
                    #forced_turn()
                    set_robot()
                    state = "LINE_FOLLOW"
                    line_follow_flag = True
                    break

                goal_distance = distance_to_goal()

                
        
        if(state == "UNKNOWN"):
            
            while True:
                stop()
                #forced_turn()
                #set_robot()
                #state = "Line_FOLLOW"
                break
                
                
        






    





    


        

def wallfollow():

    #print("wall following")

    run = Twist()
    global DESIRED_DISTANCE_RIGHT , DESIRED_DISTANCE_LEFT

    a = incoming_range[-120]  #ray of 180degree
    b= incoming_range[-1]  # ray of 120 degree
    theta = 60

    alpha_numerator = a*math.cos(math.radians(theta)) - b
    alpha_denominator = a*math.sin(math.radians(theta))
    if(alpha_denominator == 0):
        pass
    alpha = math.atan(alpha_numerator/alpha_denominator) #already in randians - no need to convert again

    AB=b*math.cos(alpha)
    AC=2  # distance = speed * time # assuming a constant speed of 2 /s
    CD = AB + AC*math.sin(alpha)

    error = DESIRED_DISTANCE_LEFT - CD

    Kp_Wallfollow = 2

    angle_to_rotate = Kp_Wallfollow * error

    run.angular.z -= angle_to_rotate
    run.linear.x = 1

    #print("error = ", error)

    pub.publish(run)




if __name__ == '__main__':

    rospy.init_node('bug2')
    #rate = rospy.Rate(5)
    sub = rospy.Subscriber('/base_scan', LaserScan, laser_output_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

    pub = rospy.Publisher('/cmd_vel' , Twist, queue_size= 1)
    bug2()


   

    
    rospy.spin()
from logging import disable
from typing import final
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point 

angle_array = []
angle_increment = 0.5

for j in range(361):
    angle_array.append(j*angle_increment)




def rviz(final_list):

    x1 = final_list[0][0]
    y1 = final_list[0][1]
    x2 = final_list[1][0]
    y2 = final_list[1][1]
    marker1 = Marker()
    marker1.type = Marker.LINE_STRIP
    marker1.header.frame_id = "base_link"
    marker1.scale.x = 0.1
    marker1.color.b = 1.0
    marker1.color.a = 1.0

    Point_1 = Point()
    Point_2 = Point()


    Point_1.x=x1
    Point_1.y=y1
    Point_1.z=0

    Point_2.x=x2
    Point_2.y=y2
    Point_2.z=0

    marker1.points.append(Point_1)
    marker1.points.append(Point_2)

    marker_pub.publish(marker1)





def laser_output_callback(message):

    incoming_range =  message.ranges
    coordinates_list = []
    
    k = 100
    threshold = 0.3


    for i in range(len(incoming_range)):
        r=incoming_range[i]
        theta=angle_array[i]
        x=r*math.sin(math.radians(theta))
        y=r*math.cos(math.radians(theta))
        coordinates_list.append([x,y])
        
    # print(angle_array[0])
    # print("co-ordintes 0" , coordinates_list[0])
    # print(angle_array[180])
    # print("co-ordinates 90", coordinates_list[180])
    # print(angle_array[360])
    # print("coordinates 180", coordinates_list[360])
    # print("message ends")

    max_inliers = []
    final_list =[]

    

    for iteration in range(0,250):

        inliers = []
        outliers = []

        index_1 = random.randint(0, len(coordinates_list)-1)
        index_2 = random.randint( 0, len(coordinates_list)-1)

        if(index_1 == index_2):
            index_2 = random.randint( 0, len(coordinates_list)-1)

        

        [x1,y1] = coordinates_list[index_1]
        [x2,y2] = coordinates_list[index_2]


        #putting those points in equation f line and finding A,B & C
        A=y1-y2
        B=x2-x1
        C=x1*y2-y1*x2

        #finding distance of every other point from this line and comparing with threshold
        

        for i in range(len(coordinates_list)):
            x0 = coordinates_list[i][0]
            y0 = coordinates_list[i][1]
            if(x1==x2 and y1==y2):
                pass
            else:

                distance = abs( (A *x0 + B*y0+ C)/math.sqrt(A*A + B*B))

                if (distance< threshold):
                    inliers.append([x0, y0])
                else:
                    outliers.append([x0, y0])

        if (len(inliers)> len(max_inliers)):
                max_inliers = inliers

                max_index_1 = index_1
                max_index_2 = index_2

                line = [x1, y1, x2, y2]
                

    #print(line)
    print( "best line after k iterations :", coordinates_list[max_index_1] , coordinates_list[max_index_2])
    final_list.append(coordinates_list[max_index_1])
    final_list.append(coordinates_list[max_index_2])

    rviz(final_list)



    


if __name__=='__main__':
    rospy.init_node('test')
    
    sub = rospy.Subscriber('/base_scan', LaserScan, laser_output_callback)

    evader_topic = '/rviz_publish'
    marker_pub = rospy.Publisher(evader_topic, Marker, queue_size= 10)
    rospy.spin()
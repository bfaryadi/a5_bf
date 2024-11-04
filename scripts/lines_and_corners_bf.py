#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data
from a5_bf.msg import CornerMsg, DepthFeatures, LineMsg
import math

id = 0

class LinesAndCorners(Node):
    def __init__(self):
        super().__init__('lines_and_corners_bf')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.time = self.get_clock().now().to_msg()

        # l1 = Line(0.0, 0.0, 0.0, Point(x=1.0,y=2.0,z=0.0), Point(x=4.0,y=2.0,z=0.0), 0)
        # l2 = Line(0.0, 0.0, 0.0, Point(x=4.0,y=2.0,z=0.0), Point(x=4.0,y=6.0,z=0.0), 0)

        # c1 = Corner(p=Point(x=4.0,y=2.0,z=0.0), psi=0.0, id=0, l_a=l1.msg, l_b=l2.msg)
        # c2 = Corner(p=Point(x=4.0,y=6.0,z=0.0), psi=0.0, id=0, l_a=l1.msg, l_b=l2.msg)

        # line_marker = buildRvizLineList([l1, l2], self.time)
        # corner_marker = buildRvizCorners([c1, c2], self.time)

        # self.publisher.publish(line_marker)
        # self.publisher.publish(corner_marker)
        # self.get_logger().info('Publishing lines and corners')

    def listener_callback(self, msg):
        points = []
        for i in range(len(msg.ranges)):
            L = msg.ranges[i]
            if(not math.isinf(L)):
                theta = msg.angle_min + i*msg.angle_increment
                points.append(Point(x=L*math.cos(theta),y=L*math.sin(theta),z=0.0))
        
        if points:
            self.time = self.get_clock().now().to_msg()
            lines = getAllLines(points)
            line_list = buildRvizLineList(lines, self.time)
            self.publisher.publish(line_list)

            corners = getCornersFromLines(lines)
            corner_list = buildRvizCorners(corners, self.time)
            self.publisher.publish(corner_list)

class Line():
    def __init__(self, a=0.0, b=0.0, c=0.0, p_a=Point(), p_b=Point(), id=-1):
        self.a = a
        self.b = b
        self.c = c
        self.p_a = p_a
        self.p_b = p_b
        self.id = id

        run = p_b.x - p_a.x
        rise = p_b.y - p_a.y

        if(run != 0):
            self.slope = rise/run
        else:
            self.slope = math.inf
        self.length = math.sqrt(rise**2 + run**2)

        self.msg = LineMsg(a=a,b=b,c=c,p_a=p_a,p_b=p_b,id=id)

class Corner():
    def __init__(self, p, psi, id, l_a, l_b):
        self.p = p
        self.psi = psi
        self.id = id
        self.l_a = l_a
        self.l_b = l_b

        self.msg = CornerMsg(p=p,psi=psi,id=id,l_a=l_a,l_b=l_b)

def getCornersFromLines(lines):
    global id
    corners = []

    for i, l1 in enumerate(lines):
        for j, l2 in enumerate(lines):
            if j <= i:
                continue

            a1 = math.degrees(math.atan(l1.slope))
            a2 = math.degrees(math.atan(l2.slope))
            if a1 < 0:
                a1 += 180
            if a2 < 0:
                a2 += 180
            
            angle_threshold = 45
            if abs(a1 - a2) > angle_threshold:
                endpoints = []
                endpoints.append(l1.p_a)
                endpoints.append(l1.p_b)
                endpoints.append(l2.p_a)
                endpoints.append(l2.p_b)

                dist_threshold = 0.2
                for k, p1 in enumerate(endpoints):
                    for l, p2 in enumerate(endpoints):
                        if k <= l:
                            continue
                        d = getDistBetwPoints(p1, p2)
                        if d < dist_threshold:
                            # finding point of intersection between lines
                            x = (l1.b*l2.c - l2.b*l1.c)/(l1.a*l2.b - l2.a*l1.b)
                            y = (l1.c*l2.a - l2.c*l1.a)/(l1.a*l2.b - l2.a*l1.b)
                            corners.append(CornerMsg(p=Point(x=x,y=y,z=0.0),psi=0.0,id=id,l_a=l1.msg,l_b=l2.msg))
                            id += 1
                            break
    return corners


def getLineBetweenPoints(p1, p2):
    global id
    a = p1.y - p2.y
    b = p2.x - p1.x
    c = p1.x*p2.y - p2.x*p1.y
    line = Line(a=a,b=b,c=c,p_a=p1,p_b=p2,id=id)

    id += 1

    return line

def getDistanceToLine(p, l):
    return abs(l.a*p.x + l.b*p.y + l.c)/math.sqrt(l.a**2 + l.b**2)

def buildRvizLineList(lines, time):
 
    line_list = Marker()
    line_list.header.frame_id = 'laser_link'
    line_list.header.stamp = time
    line_list.ns = ''
 
    line_list.id = 0
    line_list.type = 5
    line_list.action = 0
    
    line_list.scale.x = 0.02
 
    line_list.color.g = 1.0
    line_list.color.a = 1.0
 
    # Add the line endpoints to list of points
    for l in lines:
        line_list.points.append(l.p_a)
        line_list.points.append(l.p_b)
    
    return line_list

def buildRvizCorners(corners, time):
 
    pointMarker = Marker()
    pointMarker.header.frame_id = 'laser_link'
    pointMarker.header.stamp = time


    pointMarker.ns = ''
 
    pointMarker.id = 10
    pointMarker.type = 8
    pointMarker.action = 0
    
    pointMarker.scale.x = 0.2
    pointMarker.scale.y = 0.2
    pointMarker.scale.z = 0.2
 
    pointMarker.color.b = 1.0
    pointMarker.color.a = 1.0
    pointMarker.colors.append(pointMarker.color)
 
    
    for c in corners:
        pointMarker.points.append(c.p)
 
    #pub_rviz.publish(pointMarker)
    return pointMarker

def getDistBetwPoints(p_a, p_b):
    return math.sqrt( math.pow(p_a.x - p_b.x,2) + math.pow(p_a.y - p_b.y,2) )

def getLine(points, first=False):
    '''
    Given a list of geometry_msgs/Point objects, return the line that fits over them.
 
    Parameters:
        points (list): list of Point objects
        first (bool): True if the first time this is called, false otherwise.
                      This is needed so that we can use the point furthest away from the first Point object in the list
                      when this function is called for the first time.
 
    Returns:
        Line or -1: the Line obect that fits over the points, or -1 indicating that a line could not be found to fit the points.
        float: the max distance found between the line and any point in the points parameter
    '''
    #print('In getLine')
    
    # Get index of last point in the list
    l = len(points)-1
 
    # If it's the first set of points, then don't use first and last points to find the line
    # Instead, use the first point and the point with furthest distance from the first point
    if first:
        # Loop through and find the point furthest away from the first point
        max_d = 0.0
        i_max_d = 0
        for i_p,p in enumerate(points):
            d = getDistBetwPoints(p, points[0])
            #print('points[0]: %s\n p: %s\n d: %s' % (points[0],p,d))
            if d > max_d:
                max_d = d
                i_max_d = i_p
        # Set 
        l = i_max_d
    
    #*****************************
    # Continue writing code below
    #***************************** 

    line = getLineBetweenPoints(points[0], points[l])

    max_d = 0.0
    i_max_d = 0
    for i in range(0, l):
        d = getDistanceToLine(points[i], line)
        if d > max_d:
            max_d = d
            i_max_d = i

    
    threshold = 0.1
    if max_d < threshold:
        return line, max_d
    else:
        return Line(), i_max_d

def getAllLines(points):
    '''
    Given a list of geometry_msgs/Point objects, return all the lines 
    that can be fit over them
 
    Parameters:
        points (list): list of Point objects
 
    Returns:
        list: list of Line objects that were fit over the points parameter
    '''
 
    #print('In getAllLines')
    #print('points:')
    #print('%s,\n%s\n...\n%s' % (points[0], points[1], points[-1]))
 
    # Initialize an empty list to hold all the lines
    result = []
 
    # Get the first line over all the points
    # Do this outside the loop because the endpoints are different the first time we do this
    l, i_max_dist = getLine(points, True)
    result.append(l)
    
    #print('First line:')
    #result[0].printLine()
 
    # Maintain a list of point sets to process (create and evaluate a line for all sets of points in toProcess)
    toProcess = []
    
    # If the first line wasn't good then split the data and add the sets to toProcess
    if result[0].id == -1:
        #print('Line not good, adding to toProcess')
 
        # Split data on the point with max distance from the line
        p_a = points[0:i_max_dist]
        p_b = points[i_max_dist:]
 
        # Add new point sets to toProcess
        toProcess.append(p_a)
        toProcess.append(p_b)
 
        # Remove the bad line
        result = result[:-1]
    
    # Send points to be displayed in rviz (optional, debugging) 
    #highlightPointSet(p_a,1)
    #highlightPointSet(p_b,2)
    #i_marker = 3
 
 
    # while there are still point sets to find lines for
    while len(toProcess) > 0:
 
        # Get next set of points to find a line for
        points = toProcess[0]
 
        # Send points to be displayed in rviz (optional, debugging) 
        #highlightPointSet(points, i_marker)
        #i_marker += 1
 
        # Check that we have more than 2 points. If we only have endpoints then no line should be valid.
        if len(toProcess[0]) > 2:
 
            #**************************************************
            # Calling getLine
            #**************************************************
            l, i_max_dist = getLine(toProcess[0])
            result.append(l)
            
            #print('New line:')
            #result[-1].printLine()
            
            # If the line is invalid, then remove it and split the data
            if result[-1].id == -1:
                # Split data
                p_a = points[0:i_max_dist]
                p_b = points[i_max_dist:]
 
                # Add new sets to toProcess list
                toProcess.append(p_a)
                toProcess.append(p_b)
 
                # Remove the bad line
                result = result[:-1]
            
        # Remove the first list in toProcess (the one we just processed)
        toProcess = toProcess[1:]
 
    #print('Exiting getAllLines')
    return result

def main(args=None):
    

    rclpy.init(args=args)
    node = LinesAndCorners()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
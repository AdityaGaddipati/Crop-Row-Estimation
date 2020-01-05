#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
import tf
import cv2
import numpy as np
from cv_bridge import CvBridge
import math
import time

class CropRowEstimator():
    def __init__(self):

        self.costmap_width = 0
        self.costmap_height = 0
        self.costmap_resolution = 0

        self.crop_width = np.array([])
        self.crop_row_width = 10

        self.mapX=0;self.mapY=0
        self.botBinX=0;self.botBinY=0

        self.bin1 = {}; self.bin2 = {}
        self.bin1_max = {}; self.bin2_max = {}

        self.estimate_lines = True

        self.count_callback=0
        self.count_no_line=0
        self.count_single_line=0
        self.count_parallel_lines=0
        self.count_nonparallel_lines=0
        self.count_intersecting_lines=0

        self.blank_img = np.zeros(
        (
        	720,
        	1280,
        	1
        ),
        dtype=np.uint8,
        )
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("lines",Image, queue_size=10)
    	self.image_pub2 = rospy.Publisher("crop_row",Image, queue_size=10)

    def costmap_cb(self,msg):
        if self.estimate_lines==False:
                return
        # position of cell (0,0) in odom
        self.mapX = msg.info.origin.position.x
    	self.mapY = msg.info.origin.position.y
        #print msg.info.width,msg.info.height

        # costmap width in cells, height in cells, resolution in meters/cell
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_resolution = msg.info.resolution
        self.count_callback += 1

        bot_pos_x = int(round(self.mapX/self.costmap_resolution + self.costmap_width/2))
        bot_pos_y = int(round(self.mapY/self.costmap_resolution + self.costmap_height/2))
        self.botBinX = bot_pos_x - (bot_pos_x%3-1)
        self.botBinY = bot_pos_y - (bot_pos_y%3-1)

        #convert costmap into image for applying image processing techniques
        costmap_image = self.OccupancyGrid_to_Image(msg.data)

        #Estimate lines in unstructured costmap_image
        lines,vis_image = self.detect_lines(costmap_image)
        #print vis_image.shape
        if lines is None:
            self.count_no_line += 1
            return

        #Extract useful lines from the detected lines
        filtered_lines = self.filter_lines(lines)

        #Visulaize the crop lines #Use only for debugging purpose
        img = self.draw_lines(vis_image,filtered_lines)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))

        if len(filtered_lines) == 0:
    		return

        #The estimated crop lines undergo disection and binning
        self.disect_and_bin(filtered_lines)

    def OccupancyGrid_to_Image(self,occ_grid_data):

        occ_grid_array_1d = np.array(occ_grid_data)
        #reshape 1d OccupancyGrid into 2d image
    	occ_grid_array_2d = np.reshape(occ_grid_array_1d,(self.costmap_height,self.costmap_width))
        #convert datatype to uint8 as required by cv2 functions
    	occ_grid_dtype = occ_grid_array_2d.astype(np.uint8)

        #When converting OccupancyGrid data structure into corresponding cv2 image you need to account for the difference in coordinate frames
        #For OccupancyGrid origin is at bottom right, X axis UP and Y axis LEFT
        #For cv2 image origin is at top left, X axis RIGHT and Y axis DOWN
        '''
        ROS costmap                   CV2 image
        ********40   X(width)         0----------->X
        ********30     ^              |
        ********20     |              |
        ********10     |              |
        9876543210     |              |
        Y(height)<-----0              Y

        self.input_costmap.data = [0,1,2,3,4,5,6,7,8,9,10,,,,,20,,,,,30,,,,,40]
        '''
    	return cv2.flip(occ_grid_dtype.transpose(),-1)
    	#cropped = d[:d.shape[0]/2,:]
    	#cropped2 = d[d.shape[0]/4:3*d.shape[0]/4,:]

    def detect_lines(self,costmap_image):
        #Black in OccupancyGrid is 100 and white is 0.
        #Black in Image is 0 and white is 255.
        #Account for that by inverting the values
    	ret,thresh = cv2.threshold(costmap_image,80,255,cv2.THRESH_BINARY_INV)

        #Perform canny edge detection
    	cannyed_image = cv2.Canny(thresh, 0, 255)

        #Fit straight lines using hough transform
    	#lines = cv2.HoughLinesP(cannyed_image,rho=6,theta=np.pi/60,threshold=60,lines=np.array([]),minLineLength=40,maxLineGap=100)
    	#lines = cv2.HoughLinesP(cannyed_image,rho=6,theta=np.pi/60,threshold=60,lines=np.array([]),minLineLength=40,maxLineGap=30)
    	lines = cv2.HoughLinesP(cannyed_image,rho=6,theta=np.pi/60,threshold=60,lines=np.array([]),minLineLength=50,maxLineGap=30)
    	#lines = cv2.HoughLinesP(cannyed_image,rho=6,theta=np.pi/60,threshold=60,lines=np.array([]),minLineLength=20,maxLineGap=30)

        #lines contains the pixel coordinates (x1,y1),(x2,y2) of each line detected
        # [line1,line2,line3,...lineN] where lineN = [x1,y1,x2,y2]
        return lines,cannyed_image

    def filter_lines(self,lines):
        distance_neg = []
    	distance_pos = []
    	originX = int(self.costmap_height/2)		# costmap height/2
    	originY = int(self.costmap_width/2)			# costmap width/2
    	line_dict = {}

        #First step is to divide the detected lines into left and right side of the robot
        #For this we find the perpendicular distance from the robot (center of the image) to each detected line.
        #All lines having positive perpendicular distance fall to the left side of the robot.
        #All lines having negative perpendicular distance fall to the right side of the robot.
        #Note that the coordinate system followed is of an image because we have the pixel positions of lines and not the odom positions of lines
    	for l in lines:
    		try:
    			m = float(l[0][1] - l[0][3])/float(l[0][0] - l[0][2])
    			c = -m*l[0][0] + l[0][1]
    			d = (m*originX - originY + c)/math.sqrt(math.pow(m,2)+1)
    			if m<0:
    				d = -1*d
    			if d<0:
    					distance_neg.append(d)
    			else:
    					distance_pos.append(d)
    			line_dict[d] = [l,m,c]
    		except ZeroDivisionError:
    			d = originX - l[0][0]
    			m = 1000
    			c = -m*l[0][0] + l[0][1]
    			if d<0:
    					distance_neg.append(d)
    			else:
    					distance_pos.append(d)
    			line_dict[d] = [l,m,c]

        #Now we find minimum perpendicular distance on each side of the robot
        #Lines corresponding to those min distance will undergo furrther filtering and remaining lines are ignored
        #After this step we are left with minimum perpendicular distance of two lines, one on each side of the robot. Sometimes only one.
    	min_d1 = -100; min_d2 = 100
    	for d in distance_neg:
    		if d > min_d1:
    			min_d1 = d

    	for d in distance_pos:
    		if d < min_d2:
    			min_d2 = d

        #The lines corresponding to the pair of minimum distances found above are retrieved from the line_dictionary
        #These lines are checked for intersection. If they intersect then they are rejected.
        #Otherwise they are considered as the pair of crop lines detected in the received costmap
        #Sometimes only one line is detected. That case is also handeled.
    	filtered_lines = []
    	if min_d1 != -100 and min_d2 != 100:

    		m1 = line_dict[min_d1][1]
    		m2 = line_dict[min_d2][1]
    		c1 = line_dict[min_d1][2]
    		c2 = line_dict[min_d2][2]

    		if m1!=m2:
    			x_int = (c2-c1)/(m1-m2)
    			y_int = (m1*c2 - m2*c1)/(m1-m2)
    			# 2m thick border around actual area of costmap added for checking intersecting lines
    			if not (x_int>-40 and x_int<self.costmap_height+40 and y_int>-40 and y_int<self.costmap_width+40):
    				filtered_lines.append(line_dict[min_d1][0][0])
    				filtered_lines.append(line_dict[min_d2][0][0])
    				self.count_nonparallel_lines += 1
    			else:
    				self.count_intersecting_lines += 1
    				return filtered_lines
    		else:
    			filtered_lines.append(line_dict[min_d1][0][0])
    			filtered_lines.append(line_dict[min_d2][0][0])
    			self.count_parallel_lines += 1

        elif min_d1 != -100 and min_d2 == 100:
                filtered_lines.append(line_dict[min_d1][0][0])
                filtered_lines.append([])
                self.count_single_line += 1
        else:
                filtered_lines.append([])
                filtered_lines.append(line_dict[min_d2][0][0])
                self.count_single_line += 1

        # print "count_callback: "+str(self.count_callback)
    	# print "count_no_line: "+str(self.count_no_line)
    	# print "count_single_line: "+str(self.count_single_line)
    	# print "count_parallel_lines: "+str(self.count_parallel_lines)
    	# print "count_nonparallel_lines: "+str(self.count_nonparallel_lines)
    	# print "count_intersecting_lines: "+str(self.count_intersecting_lines)
    	# print "*************************"

        self.visulaize_raw_crop_lines(filtered_lines)
        return filtered_lines

    def disect_and_bin(self,filtered_lines):

        #Imagine you have divided the ground palne (odom frame) into 15cm by 15cm square areas called bins.
        #Each estimated crop line will be occupying a set of bins.
        #The estimated crop lines are disected and placed in these bins.
        crop_line_coordinates = [] #For visualizing crop rows
        for i in range(2):
            if len(filtered_lines[i])==0:
                continue
            if i==0:
                bin = self.bin1
            else:
                bin = self.bin2
            xx1 = int(round(self.mapX/self.costmap_resolution) + (self.costmap_width - filtered_lines[i][1]))
            yy1 = int(round(self.mapY/self.costmap_resolution) + (self.costmap_height - filtered_lines[i][0]))
            xx2 = int(round(self.mapX/self.costmap_resolution) + (self.costmap_width - filtered_lines[i][3]))
            yy2 = int(round(self.mapY/self.costmap_resolution) + (self.costmap_height - filtered_lines[i][2]))

            if xx1>xx2:
        		temp = xx1
        		xx1 = xx2
        		xx2 = temp

        		temp = yy1
        		yy1 = yy2
        		yy2 = temp

            if xx1%3 != 0:
                if bin.has_key(xx1 - (xx1%3-1)):
                    if bin[xx1 - (xx1%3-1)].has_key(yy1 - (yy1%3-1)):
                        bin[xx1 - (xx1%3-1)][yy1 - (yy1%3-1)]  += 1
                    else:
                        bin[xx1 - (xx1%3-1)][yy1 - (yy1%3-1)] = 1
                else:
                    bin[xx1 - (xx1%3-1)] = {}
                    bin[xx1 - (xx1%3-1)][yy1 - (yy1%3-1)] = 1

        	for itr_x in range(xx1 + 3 - (xx1-1)%3, xx2 - (xx2-1)%3 + 1, 3):
        		itr_y = yy1 + int(round((yy2-yy1)*(itr_x-xx1)/(xx2-xx1)*1.0))
        		if bin.has_key(itr_x):
        			if bin[itr_x].has_key(itr_y - (itr_y%3-1)):
        				bin[itr_x][itr_y - (itr_y%3-1)]  += 1
        			else:
        				bin[itr_x][itr_y - (itr_y%3-1)] = 1
        		else:
        			bin[itr_x] = {}
        			bin[itr_x][itr_y - (itr_y%3-1)] = 1

        	if xx2%3 == 0:
        		if bin.has_key(xx2 - (xx2%3-1)):
        			if bin[xx2 - (xx2%3-1)].has_key(yy2 - (yy2%3-1)):
        				bin[xx2 - (xx2%3-1)][yy2 - (yy2%3-1)]  += 1
        			else:
        				bin[xx2 - (xx2%3-1)][yy2 - (yy2%3-1)] = 1
        		else:
        			bin[xx2 - (xx2%3-1)] = {}
        			bin[xx2 - (xx2%3-1)][yy2 - (yy2%3-1)] = 1

            temp_array = [] #For visualizing crop rows
            if i==0:
                self.bin1 = bin
                for itr in sorted(self.bin1.keys()):
                    # self.bin1_max[itr] = self.bin1[itr].keys()[self.bin1[itr].values().index(sorted(self.bin1[itr].values())[-1])]
                    tmp = sorted(self.bin1[itr].values())
                    if len(tmp)>1:
                        if (tmp[-2]*1.0)/tmp[-1]>=0.5:
                            if tmp[-1]!=tmp[-2]:
                                max_bin1 = self.bin1[itr].keys()[self.bin1[itr].values().index(tmp[-1])]
                                max_bin2 = self.bin1[itr].keys()[self.bin1[itr].values().index(tmp[-2])]
                            else:
                                index1 = self.bin1[itr].values().index(tmp[-1])
                                index2 = (index1+1) + self.bin1[itr].values()[index1+1:].index(tmp[-2])
                                max_bin1 = self.bin1[itr].keys()[index1]
                                max_bin2 = self.bin1[itr].keys()[index2]
                            if max_bin1>max_bin2:
                                max_bin = max_bin1
                            else:
                                max_bin = max_bin2
                            self.bin1_max[itr] = max_bin
                        else:
                            self.bin1_max[itr] = self.bin1[itr].keys()[self.bin1[itr].values().index(tmp[-1])]
                    else:
                        self.bin1_max[itr] = self.bin1[itr].keys()[self.bin1[itr].values().index(tmp[-1])]

                    #For visualizing the crop rows #Use only for debugging
                    temp_array.append([410-self.bin1_max[itr],640-itr])
                    if len(temp_array)>1:
                        crop_line_coordinates.append(temp_array[-2]+temp_array[-1])
            else:
                self.bin2 = bin
                for itr in sorted(self.bin2.keys()):
                    # self.bin2_max[itr] = self.bin2[itr].keys()[self.bin2[itr].values().index(sorted(self.bin2[itr].values())[-1])]
                    tmp = sorted(self.bin2[itr].values())
                    if len(tmp)>1:
                        if (tmp[-2]*1.0)/tmp[-1]>=0.5:
                            if tmp[-1]!=tmp[-2]:
                                max_bin1 = self.bin2[itr].keys()[self.bin2[itr].values().index(tmp[-1])]
                                max_bin2 = self.bin2[itr].keys()[self.bin2[itr].values().index(tmp[-2])]
                            else:
                                index1 = self.bin2[itr].values().index(tmp[-1])
                                index2 = (index1+1) + self.bin2[itr].values()[index1+1:].index(tmp[-2])
                                max_bin1 = self.bin2[itr].keys()[index1]
                                max_bin2 = self.bin2[itr].keys()[index2]

                            if max_bin1>max_bin2:
                                max_bin = max_bin2
                            else:
                                max_bin = max_bin1
                            self.bin2_max[itr] = max_bin
                        else:
                            self.bin2_max[itr] = self.bin2[itr].keys()[self.bin2[itr].values().index(tmp[-1])]
                    else:
                        self.bin2_max[itr] = self.bin2[itr].keys()[self.bin2[itr].values().index(tmp[-1])]

                    #For visualizing the crop rows #Use only for debugging
                    temp_array.append([410-self.bin2_max[itr],640-itr])
                    if len(temp_array)>1:
                        crop_line_coordinates.append(temp_array[-2]+temp_array[-1])

        #Use for debugging only
        self.visualize_refined_crop_rows(crop_line_coordinates)

    def visulaize_raw_crop_lines(self,filtered_lines):
        temp_array = []
    	for f in filtered_lines:
            if len(f)==0:
                continue
            else:
                a = int(360 - round(self.mapY/self.costmap_resolution) - (self.costmap_height - f[0]))
                b = int(640 - round(self.mapX/self.costmap_resolution) - (self.costmap_width - f[1]))
                c = int(360 - round(self.mapY/self.costmap_resolution) - (self.costmap_height - f[2]))
                d = int(640 - round(self.mapX/self.costmap_resolution) - (self.costmap_width - f[3]))
                temp_array.append([a,b,c,d])

    	self.blank_img = self.draw_lines(self.blank_img,temp_array)

    def visualize_refined_crop_rows(self,crop_line_coordinates):
    	blank_img1 = self.draw_lines(self.blank_img,crop_line_coordinates)
    	self.image_pub2.publish(self.bridge.cv2_to_imgmsg(blank_img1, "mono8"))

    def draw_lines(self, img, lines, color=[255, 255, 255], thickness=1):
    	# Make a copy of the original image.
    	img = np.copy(img)

    	# Create a blank image that matches the original in size.
    	line_img = np.zeros(
    	(
    		img.shape[0],
    		img.shape[1],
    		1
    	),
    	dtype=np.uint8,
    	)
    	# Loop over all lines and draw them on the blank image.

    	for l in lines:
            if len(l)==0:
                continue
            else:
                cv2.line(line_img, (l[0], l[1]), (l[2], l[3]), color, thickness)

    	# Merge the image with the lines onto the original.
    	img = cv2.addWeighted(img, 1, line_img, 1.0, 0.0)
    	# Return the modified image.
    	return img

    def restart(self):
        self.costmap_width = 0
        self.costmap_height = 0
        self.costmap_resolution = 0

        self.crop_width = np.array([])
        self.crop_row_width = 10

        self.mapX=0;self.mapY=0
        self.botBinX=0;self.botBinY=0

        self.bin1 = {}; self.bin2 = {}
        self.bin1_max = {}; self.bin2_max = {}

        self.count_callback=0
        self.count_no_line=0
        self.count_single_line=0
        self.count_parallel_lines=0
        self.count_nonparallel_lines=0
        self.count_intersecting_lines=0

        self.blank_img = np.zeros(
        (
        	720,
        	1280,
        	1
        ),
        dtype=np.uint8,
        )

def main():
    rospy.init_node("crop_lines")
    crop_rows = CropRowEstimator()
    rospy.Subscriber("/costmap_node/costmap/costmap", OccupancyGrid, crop_rows.costmap_cb)
    rospy.spin()
    
if __name__=="__main__":
    main()

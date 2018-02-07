import spidev
import time
import cv2
import io
import numpy as np
import PID
import scipy
import serial
import struct
from scipy import signal
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from picamera.array import PiRGBArray
from picamera import PiCamera

ser = serial.Serial('/dev/ttyACM0', 9600) #define serial comm to Arduino
motorPWM = 80;

""" calculate line intersection """
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)

    if div == 0:
       raise Exception('lines do not intersect')
    
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

""" calculate slope """
def line_slope(x1,y1,x2,y2):
    den = x2-x1
    if den == 0:
        den = 0.000001
        k = (y2-y1)/den
    else:
        k = (y2-y1)/den
    return k

""" write pot """
def write_pot(input):
    msb = input >> 8
    lsb = input & 0xFF
    spi.xfer([lsb])

def signed(byte):
    if byte > 127:
        return (256-byte)*(-1)
    else:
        return byte

""" initialize the camera and grab a reference to the raw camera capture """
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 24
camera.rotation = 90
camera.zoom = (0.0, 0.0, 1.0, 1.0) # (x, y, w, h) 
rawCapture = PiRGBArray(camera, size=(320, 240))

""" init spi """
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 1000000

""" allow the camera to warmup """
time.sleep(0.1)

""" init once """
once = 0

""" capture frames from the camera """
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    """ grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text """
    image = frame.array
    h,w,c = image.shape
    image = image[0:h, 0:w]
    #gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(image,225,240,apertureSize=3)

    """ construct initial coordinates """
    if once == 0:
        pt1_r = (int(w/2-1),int(h/2-1))
        pt1_l = (int(w/2-1),int(h/2+1))
        pt2_r = (int(w/2+1),int(h/2+1))
        pt2_l = (int(w/2+1),int(h/2-1))
        pt1_r_old = (int(w/2-1),int(h/2-1))
        pt1_l_old = (int(w/2-1),int(h/2+1))
        pt2_r_old = (int(w/2+1),int(h/2+1))
        pt2_l_old = (int(w/2+1),int(h/2-1))
        """ make center reference """
        x_ref = float(w/2)
        """ define the minimum absolute value of slope tolerance """
        slope_min = 0.1
        list_A_output = []
        once = 1

    """ houghline parameters """
    min_line_length = image.shape[1]-300
    max_line_gap = 10
    threshold = 70
    rho = 1
    theta = np.pi/560

    """ construct houghlines """
    lines = cv2.HoughLines(edges,rho,theta,threshold,min_line_length)

    """ create empty variables """
    x1_l = []
    x2_l = []
    y1_l = []
    y2_l = []
    x1_r = []
    x2_r = []
    y1_r = []
    y2_r = []
    
    try:
        a,b,c = lines.shape

        """ loop through the number of lines found and construct list of coordinates """
        for i in range(a):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0, y0, = a*rho, b*rho
            x1 = x0+1000*(-b)
            x2 = x0-1000*(-b)
            y1 = y0+1000*(a)
            y2 = y0-1000*(a)
            slope = line_slope(x1,y1,x2,y2)

            if slope >= slope_min and slope <= 1/slope_min:
                """ append lines with slope greater than slope_min """
                x1_l.append(x1)
                x2_l.append(x2)
                y1_l.append(y1)
                y2_l.append(y2)

##                """ This shows all lines found """
##                pt1_l = (int(x1),int(y1))
##                pt2_l = (int(x2),int(y2))
##                cv2.line(frame,pt1_l,pt2_l,(255,0,0),1)

            elif slope <= -slope_min and slope >= -1/slope_min:
                """ append lines with slope less than -slope_min """
                x1_r.append(x1)
                x2_r.append(x2)
                y1_r.append(y1)
                y2_r.append(y2)

##                """ This shows all lines found """
##                pt1_r = (int(x1),int(y1))
##                pt2_r = (int(x2),int(y2))
##                cv2.line(frame,pt1_r,pt2_r,(0,0,255),1)
            else:
                pass

    except:
        """ if no lines found, use initial or previous values """
        #print("No houghlines found. Using old.")

        cv2.line(edges,pt1_r_old,pt2_r_old,(255,0,0),2)
        cv2.line(edges,pt1_l_old,pt2_l_old,(0,0,255),2)

    """ sort line after smallest '(x1+x2)/2'-value"""
    try:
        x_s = x1+x2
        x_s = np.array(x_s)
        x1_l = np.array(x1_l)
        x2_l = np.array(x2_l)
        y1_l = np.array(y1_l)
        y2_l = np.array(y2_l)
        indx = x_s.argsort()
        x1_l = x1_l[indx]
        x2_l = x2_l[indx]
        y1_l = y1_l[indx]
        y2_l = y2_l[indx]
        
        """ use smallest values to construct coordinates """
        pt1_l = int(x1_l[0]),int(y1_l[0])
        pt2_l = int(x2_l[0]),int(y2_l[0])

        """ save found lines for later use """
        pt1_l_old = pt1_l
        pt2_l_old = pt2_l

        """ construct line from coordinates """
        cv2.line(image,pt1_l,pt2_l,(255,0,0),2,4)

    except:
        """ if no lines found, use initial or old values """
        #print('No "left" line. Using old.')
        cv2.line(image,pt1_l_old,pt2_l_old,(255,0,0),2,4)

    """ sort line after smallest '(x1+x2)/2'-value"""
    try:
        x_s = x1+x2
        x_s = np.array(x_s)
        x1_r = np.array(x1_r)
        x2_r = np.array(x2_r)
        y1_r = np.array(y1_r)
        y2_r = np.array(y2_r)
        indx = x_s.argsort()
        x1_r = x1_r[indx]
        x2_r = x2_r[indx]
        y1_r = y1_r[indx]
        y2_r = y2_r[indx]

        """ use smallest values to construct coordinates """       
        pt1_r = int(x1_r[0]),int(y1_r[0])
        pt2_r = int(x2_r[0]),int(y2_r[0])

        """ save found lines for later use """
        pt1_r_old = pt1_r
        pt2_r_old = pt2_r

        """ construct line from coordinates """ 
        cv2.line(image,pt1_r,pt2_r,(0,0,255),2,4)

    except:
        """ if no lines found, use initial or old values """
        #print('No "right" line. Using old.')
        cv2.line(image,pt1_r_old,pt2_r_old,(0,0,255),2,4)

    """ try to construct intersection point between lines """
    try:
        x_pt = line_intersection((pt1_l, pt2_l), (pt1_r, pt2_r))[0]

        if x_pt < 0:
            x_pt = 0
        elif x_pt > w:
            x_pt = w
        else:
            pass

        """ make normalized amplitude value of what direction to drive """
        A_input = (x_pt-x_ref)/x_ref
        #cv2.line(image,(int(x_pt),int(h/2)),(int(w/2),int(h/2)),(0,255,0),(0,255,0),1,8,0,0.1)
        cv2.arrowedLine(image,(int(w/2),int(h/2)),(int(x_pt),int(h/2)),(0,255,0),(0,255,0),1,8,0,0.1)
      
    except:
        """ if no intersection found, use old values for intersection """ 
        #print("No intersection. Using old.")
        x_pt = line_intersection((pt1_l_old, pt2_l_old), (pt1_r_old, pt2_r_old))[0]
        if x_pt < 0:
            x_pt = 0
        elif x_pt > w:
            x_pt = w
        else:
            pass

        """ make normalized amplitude value of what direction to drive """        
        A_input = (x_pt-x_ref)/x_ref
        cv2.arrowedLine(image,(int(w/2),int(h/2)),(int(x_pt),int(h/2)),(0,255,0),1,8,0,0.1)
    
    """ PID-controller input """
    P = 0.5
    I = 3.0
    D = 0.0
    feedback = 0
    pid = PID.PID(P,I,D)
    pid.SetPoint = A_input

    """ PID-output """
    pid.update(feedback)
    A_output = pid.output
    if A_output < -1:
        A_output = -1
    elif A_output > 1:
        A_output = 1
    else:
        pass

    """ create list of inputs for filtering """
    if len(list_A_output) < 5:
        list_A_output.append(A_input)
    else:
        list_A_output.pop(0)
        list_A_output.append(A_input)
    #print(list_A_output)

    b, a = signal.butter(3,1)
    A_output_filtered = signal.lfilter(b, a, list_A_output)

    if len(list_A_output) > 4:
        ave_output = 0
        ave_output = sum(list_A_output)/len(list_A_output)
        #print("filtered: ", A_output_filtered[2])
        #print("Average: ", ave_output)
        
        pass
    else:
        pass

    """ Arduino communication """
    tx_spi_turn = int(A_output*90) #turn output

    tx_spi_turn = np.clip((tx_spi_turn+90), 0, 180)
    ser.write(struct.pack('<BB', tx_spi_turn, motorPWM)) #send turn command to Ard
    
    """ plot controller input and output """
    print("Turn signal: ",tx_spi_turn)

##    """ send data via spi """
##    write_pot(tx_spi_turn)
##    rx_spi = spi.readbytes(1)
##    rx_spi = signed(rx_spi[0])
##    #print("Rx: ",rx_spi)
##    #print("")
##
##    """ ask for ir via spi """
##    """ left sensor """
##    write_pot(124)
##    rx_spi_124 = spi.readbytes(1)
##    #print("IR124: ",rx_spi_124)
##    write_pot(124)
##
##    """ forward sensor """
##    rx_spi_125 = spi.readbytes(1)
##    #print("IR125: ",rx_spi_125)
##
##    """ right sensor """
##    write_pot(126)
##    rx_spi_126 = spi.readbytes(1)
##    #print("IR126: ",rx_spi_126)
##
##    if rx_spi_125[0] > 80:  
##        write_pot(100)
##    elif rx_spi_125[0] > 70:
##        write_pot(90)
##    elif rx_spi_125[0] > 60:
##        write_pot(85)
##    elif rx_spi_125[0] > 50:
##        write_pot(80)
##    elif rx_spi_125[0] > 40:
##        write_pot(75)
##    elif rx_spi_125[0] > 30:
##        write_pot(65)
##    elif rx_spi_125[0] > 20:
##        write_pot(60)
##    elif rx_spi_125[0] > 10:
##        write_pot(55)
##    else:
##        write_pot(50)

    """ construct image with lines """
    cv2.imshow('frame',image)

    """ construct edges-image """
    cv2.imshow('edges',edges)

    key = cv2.waitKey(1) & 0xFF
    
    """ clear the stream in preparation for the next frame """
    rawCapture.truncate(0)
    
    """ if the `q` key was pressed, break from the loop """
    if key == ord("q"):
            break

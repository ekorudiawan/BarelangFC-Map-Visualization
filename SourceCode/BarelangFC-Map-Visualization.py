from flask import Flask, render_template, Response, request
from scipy.spatial import distance
from numpy.random import uniform, normal
import time
import socket
import sys
import numpy as np
import scipy.stats
import math
import cv2
import webbrowser
import os

R1_IP = "127.0.0.1"
R2_IP = "127.0.0.1"
R3_IP = "127.0.0.1"
R4_IP = "127.0.0.1"
R5_IP = "127.0.0.1"
R1_IN_PORT = 8000
R2_IN_PORT = 8000
R3_IN_PORT = 8000
R4_IN_PORT = 8000
R5_IN_PORT = 8000

# Configuration in Cm
fieldLength = 900
fieldWidth = 600

mapImage = np.zeros((800,1100,3), np.uint8)

# Variabel posisi robot
robot1Position = np.zeros((3))
robot2Position = np.zeros((3))
robot3Position = np.zeros((3))
robot4Position = np.zeros((3))
robot5Position = np.zeros((3))

ballRobot1Position = np.zeros((2))
ballRobot2Position = np.zeros((2))
ballRobot3Position = np.zeros((2))
ballRobot4Position = np.zeros((2))
ballRobot5Position = np.zeros((2))

ballMeanPosition = np.zeros((2))

deltaTime = 1

robot1Color = (0,127,127)
robot2Color = (0,127,255)
robot3Color = (0,255,127)
robot4Color = (0,255,255)
robot5Color = (255,0,127)
ballColor = (0,0,255)

app = Flask(__name__)

# http://mattrichardson.com/Raspberry-Pi-Flask/
@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(main(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def worldCoorToImageCoor(x, y):
    x = x + 100
    y = 800 - (y + 100)
    return x, y

def main():
    simulationMode = True
    if simulationMode == False:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
            sock.bind((R1_IP, R1_IN_PORT))
            sock.bind((R2_IP, R2_IN_PORT))
            sock.bind((R3_IP, R3_IN_PORT))
            sock.bind((R4_IP, R4_IN_PORT))
            sock.bind((R5_IP, R5_IN_PORT))
        except socket.error:
            print 'Failed to create socket'
            sys.exit()

    # Timing value
    nowTime = 0
    lastTime = 0
    loop = 0

    while True:
        nowTime = time.clock()
        timer = nowTime - lastTime
        halfDeltaTime = deltaTime / 2.00
        # Update every 0.5 * deltatime
        if timer > halfDeltaTime:
            lastTime = nowTime
            loop += 1
            print 'Runtime : {} s'.format(deltaTime*loop) 

            mapFromFile = False
            if mapFromFile == True:
                # image tidak clear
                mapImage[:] = cv2.imread('mapImage.jpg')
            else:
                mapImage[:] = (0, 255, 0)
                cv2.rectangle(mapImage, (100,100), (1000,700), (255,255,255), 3) # Garis Luar
                cv2.rectangle(mapImage, (40,530), (100,270), (255,0,0), 10) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage, (1000,530), (1060,270), (0,0,255), 10) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage, (100,650), (200,150), (255,255,255), 3) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage, (900,650), (1000,150), (255,255,255), 3) # Garis Luar Gawang Kiri
                cv2.line(mapImage, (550,100), (550,700), (255,255,255), 3) # Garis Tengah
                cv2.circle(mapImage, (550,400), 75, (255,255,255), 3) # Lingkaran Tengah
                cv2.circle(mapImage, (310,400), 3, (255,255,255), 5)
                cv2.circle(mapImage, (790,400), 3, (255,255,255), 5)

                textLine = "(0,0)"
                x, y = worldCoorToImageCoor(0,0)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

                textLine = "(0,600)"
                x, y = worldCoorToImageCoor(0,600)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

                textLine = "(900,600)"
                x, y = worldCoorToImageCoor(900,600)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

                textLine = "(900,0)"
                x, y = worldCoorToImageCoor(900,0)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

                # cv2.imwrite("mapImage.jpg", mapImage)
                # break

            if simulationMode == True:
                robot1Position[0] = uniform(0, fieldLength)
                robot1Position[1] = uniform(0, fieldWidth)
                robot1Position[2] = uniform(0, 360)
                robot2Position[0] = uniform(0, fieldLength)
                robot2Position[1] = uniform(0, fieldWidth)
                robot2Position[2] = uniform(0, 360)
                robot3Position[0] = uniform(0, fieldLength)
                robot3Position[1] = uniform(0, fieldWidth)
                robot3Position[2] = uniform(0, 360)
                robot4Position[0] = uniform(0, fieldLength)
                robot4Position[1] = uniform(0, fieldWidth)
                robot4Position[2] = uniform(0, 360)
                robot5Position[0] = uniform(0, fieldLength)
                robot5Position[1] = uniform(0, fieldWidth)
                robot5Position[2] = uniform(0, 360)
                ballMeanPosition[0] = uniform(0, fieldLength)
                ballMeanPosition[1] = uniform(0, fieldWidth)

            drawRobot = True
            if drawRobot == True:
                x, y = worldCoorToImageCoor(int(robot1Position[0]), int(robot1Position[1]))
                cv2.circle(mapImage,(x, y), 20, robot1Color, -1)
                textLine = "R1"
                cv2.putText(mapImage, textLine, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                x, y = worldCoorToImageCoor(int(robot2Position[0]), int(robot2Position[1]))
                cv2.circle(mapImage,(x, y), 20, robot2Color, -1)
                textLine = "R2"
                cv2.putText(mapImage, textLine, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                x, y = worldCoorToImageCoor(int(robot3Position[0]), int(robot3Position[1]))
                cv2.circle(mapImage,(x, y), 20, robot3Color, -1)
                textLine = "R3"
                cv2.putText(mapImage, textLine, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                x, y = worldCoorToImageCoor(int(robot4Position[0]), int(robot4Position[1]))
                cv2.circle(mapImage,(x, y), 20, robot4Color, -1)
                textLine = "R4"
                cv2.putText(mapImage, textLine, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                x, y = worldCoorToImageCoor(int(robot5Position[0]), int(robot5Position[1]))
                cv2.circle(mapImage,(x, y), 20, robot5Color, -1)
                textLine = "R5"
                cv2.putText(mapImage, textLine, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                

            drawBall = True
            if drawBall == True:
                x, y = worldCoorToImageCoor(int(ballMeanPosition[0]), int(ballMeanPosition[1]))
                cv2.circle(mapImage,(x, y), 15, ballColor, -1)
                textLine = "B"
                cv2.putText(mapImage, textLine, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                

            textLine = "R1 Position : ({}, {}, {})".format(int(robot1Position[0]), int(robot1Position[1]), int(robot1Position[2]))
            cv2.putText(mapImage, textLine, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "R2 Position : ({}, {}, {})".format(int(robot2Position[0]), int(robot2Position[1]), int(robot2Position[2]))
            cv2.putText(mapImage, textLine, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "R3 Position : ({}, {}, {})".format(int(robot3Position[0]), int(robot3Position[1]), int(robot3Position[2]))
            cv2.putText(mapImage, textLine, (300,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "R4 Position : ({}, {}, {})".format(int(robot4Position[0]), int(robot4Position[1]), int(robot4Position[2]))
            cv2.putText(mapImage, textLine, (300,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "R5 Position : ({}, {}, {})".format(int(robot5Position[0]), int(robot5Position[1]), int(robot5Position[2]))
            cv2.putText(mapImage, textLine, (590,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "Ball Position : ({}, {})".format(int(ballMeanPosition[0]), int(ballMeanPosition[1]))
            cv2.putText(mapImage, textLine, (590,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            # Enable GUI Streaming
            showGUI = True
            if showGUI:
                cv2.imshow("Barelang FC - Map Visualization", mapImage)
            # Enable URL Streaming
            streamUrl = False
            if streamUrl == True:
                smallMapImage = cv2.resize(mapImage, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
                cv2.imwrite('stream.jpg', smallMapImage)
                yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + open('stream.jpg', 'rb').read() + b'\r\n')
                    
            if showGUI:
                k = cv2.waitKey(1)
                if k == ord('x'):
                    cv2.destroyAllWindows()
                    break

if __name__ == "__main__":
    url = "http://0.0.0.0:9999"
    if (os.name == "nt"):
        chromedir= 'C:/Program Files (x86)/Google/Chrome/Application/chrome.exe %s'
        webbrowser.get(chromedir).open(url)
    else:
        webbrowser.get(using='firefox').open_new_tab(url)
    app.run(host='0.0.0.0', port=9999, debug=False, threaded=False)
    
try:
    import sim
    import numpy as np
    import cv2
    import time
    from array import array
except:
    print ('--------------------------------------------------------------')
    print ('Library loading failed!')
    print ('')

first_red = False
last_red = False


#Get the red pixel 
def detect_red(img):
    red1 = cv2.inRange(img, (0, 220, 100), (5, 255, 255))
    red2 = cv2.inRange(img, (175, 220, 100), (180, 255, 255))
    return red1+red2



#Get the green pixel 
def detect_green(img):
    green = cv2.inRange(img, (55, 220, 100), (65, 255, 255))
    return green


def sensor_color(img):
    sensor_array = np.sum(img, 0, dtype=np.uint16)
    sensor_array_shrunken = [0.0]*16
    for i in range(16):
        sensor_array_shrunken[i] = np.sum(sensor_array[i*16:(i+1)*16], dtype=np.uint32)*1.0
        if(sensor_array_shrunken[i] > 300000):
            sensor_array_shrunken[i] = 1.0
        elif(sensor_array_shrunken[i] < 60000):
            sensor_array_shrunken[i] = 0.0
        else:
            sensor_array_shrunken[i] = (sensor_array_shrunken[i]-60000)/(300000.0-60000.0)

    # print sensor_array_shrunken
    return sensor_array_shrunken




def move(sonar_readings, img, ID, left_motor, right_motor):
    red_list = detect_red(img)
    green_list = detect_green(img)
    green_array = sensor_color(green_list)
    red_array = sensor_color(red_list)
    global first_red
    global last_red
    
    v_left = 1.0
    v_right = 1.0
    for i in range(len(sonar_readings)):
        v_left += sonar_readings[i]*sonar_braitenberg_L[i]
        v_right += sonar_readings[i]*sonar_braitenberg_R[i]

    print(v_left, "  ", v_right)
    

    for i in range(len(green_array)):
        v_left += green_array[i] * green_braitenberg_L[i]
        v_right += green_array[i] * green_braitenberg_R[i]

    for i in range(len(red_array)):
        v_left += red_array[i] * red_braitenberg_L[i]
        v_right += red_array[i] * red_braitenberg_R[i]
        

    result = all(element == 0.0 for element in red_array)
    print("result ", result)
    print("First ", first_red)
    if(not result):
        if(not first_red):
            first_red = True
            print("First 1", first_red)
    
    if(result):
        if(last_red):
            first_red = False
            v_left = 0
            v_right = 0
            print
    
    print(v_left, "  ", v_right)

    sim.simxSetJointTargetVelocity(ID, left_motor, v_left, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(ID, right_motor, v_right, sim.simx_opmode_oneshot)


def read_image(image_ready, ID, handler):
    res, resolution, image = sim.simxGetVisionSensorImage(ID, handler, 0, sim.simx_opmode_buffer)

    if(res == sim.simx_return_ok):
        if(not image_ready):
            print("image OK!!!")
            image_ready = True
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        return img, image_ready
    elif(res == sim.simx_return_novalue_flag):
        if(image_ready):
            print("no image")
            image_ready = False
        return np.array([], dtype=np.uint8), image_ready
    else:
        print("error: " + str(res))
        return np.array([], dtype=np.uint8), image_ready


def read_sonar(ID, handlers, sonar_ready):
    points = [None]*8
    states = [False]*8
    for i in range(8):
        res, states[i], points[i], _, normal_vec = sim.simxReadProximitySensor(ID, handlers[i], sim.simx_opmode_buffer)

    dists = [i[2] for i in points]
    if(sonar_ready):
        for i in range(len(dists)):
            if(states[i] and dists[i] < 0.5):
                if(dists[i] < 0.2):
                    dists[i] = 0.2

                # map how close an obstacle is to the robot to [0, 1]
                dists[i] = 1.0 - (dists[i] - 0.2) / (0.5 - 0.2)
            else:
                dists[i] = 0.0
        return dists, sonar_ready
    else:
        flag = True
        for i in range(len(dists)):
            if(dists[i] == 0.0):
                flag = False
                break
        if(flag):
            sonar_ready = True
        return None, sonar_ready


if __name__ == "__main__":
    # global first_red
    # global last_red

    
    print ('Program started')
    sim.simxFinish(-1)  # just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1', 19998, True, True, 5000, 5)  # Connect to CoppeliaSim

    if(clientID != -1):
        print ('Connected to remote API server')

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print ('Number of objects in the scene: ', len(objs))
        else:
            print ('Remote API function call returned with error code: ', res)
        time.sleep(2)

        # get vision sensor handler
        print('Vision Sensor object handling')
        res, veh_camera = sim.simxGetObjectHandle(clientID, 'veh_camera#0', sim.simx_opmode_oneshot_wait)
        # get sonor handler
        print('Sonar object handling')
        veh_sonar = [None]*8
        for i in range(8):
            res, veh_sonar[i] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+'{}'.format(i+1)+"#0",
                                                        sim.simx_opmode_oneshot_wait)

        # get left motor handler
        res, veh_left_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0', sim.simx_opmode_oneshot_wait)

        # get right motor handler
        res, veh_right_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0', sim.simx_opmode_oneshot_wait)

        # let the server prepare the first image
        print('Getting first image')
        res, resolution, image = sim.simxGetVisionSensorImage(clientID, veh_camera, 0, sim.simx_opmode_streaming)
        image_ready_flag = False
        
        err_code, handler = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx#0', sim.simx_opmode_blocking)
        err_code, object_h = sim.simxGetObjectHandle(clientID, 'Plane22', sim.simx_opmode_blocking)
        
        # let the server prepare the first sonar reading
        points = [None] * 8
        for i in range(8):
            res, state, points[i], _, normal_vec = sim.simxReadProximitySensor(clientID, veh_sonar[i],
                                                                               sim.simx_opmode_streaming)
        sonar_braitenberg_L = [0.25, 0.03, -0.22, -0.45, -0.64, -0.88, -1.6, -1.5]
        sonar_braitenberg_R = [-1.2, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2]


        red_braitenberg_L = [0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
        red_braitenberg_R = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]



        green_braitenberg_L = [0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
        green_braitenberg_R = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]



        first_red =False
        last_red = False
        # print [j[2] for j in points]
        sonar_ready_flag = False
        # keep running until the server shuts down

        res, relative_position = sim.simxGetObjectPosition(clientID, object_h, handler, sim.simx_opmode_blocking)
        print("relative_position: ",relative_position)
        while sim.simxGetConnectionId(clientID) != -1:
            image, image_ready_flag = read_image(image_ready_flag, clientID, veh_camera)
            detections, sonar_ready_flag = read_sonar(clientID, veh_sonar, sonar_ready_flag)

            if(first_red):
                res, relative_position = sim.simxGetObjectPosition(clientID, object_h, handler, sim.simx_opmode_blocking)

                if(relative_position[0] < 0.09 and relative_position[1]<0.09):
                    last_red = True 
                    print("last_red ",last_red)

            else:
                print("First one,",first_red)
            
            

            if(image_ready_flag and sonar_ready_flag and not detections is None):
                move(detections, image, clientID, veh_left_motor, veh_right_motor)

            if(last_red):
                break


        cv2.destroyAllWindows()

        sim.simxGetPingTime(clientID)

        sim.simxFinish(clientID)
    else:
        print('Failed connecting to remote API server')
    print('Program ended')

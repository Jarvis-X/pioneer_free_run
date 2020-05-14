try:
    import sim
    import numpy as np
    import cv2
    import time
except:
    print ('--------------------------------------------------------------')
    print ('Library loading failed!')
    print ('')


def filter_red(img):
    red1 = cv2.inRange(img, (0, 220, 100), (5, 255, 255))
    red2 = cv2.inRange(img, (175, 220, 100), (180, 255, 255))
    return red1+red2


def filter_green(img):
    green = cv2.inRange(img, (55, 220, 100), (65, 255, 255))
    return green


def sensor_color(img):
    sensor_array = np.sum(img, 0, dtype=np.uint16)
    sensor_array_shrunken = [0.0]*16
    for i in range(16):
        sensor_array_shrunken[i] = np.sum(sensor_array[i*16:(i+1)*16], dtype=np.uint32)*1.0
        if sensor_array_shrunken[i] > 300000:
            sensor_array_shrunken[i] = 1.0
        elif sensor_array_shrunken[i] < 60000:
            sensor_array_shrunken[i] = 0.0
        else:
            sensor_array_shrunken[i] = (sensor_array_shrunken[i]-60000)/(300000.0-60000.0)

    # print sensor_array_shrunken
    return sensor_array_shrunken


def free_running(sonar_readings, img, ID, left_motor, right_motor):
    red_threshold = filter_red(img)
    green_threshold = filter_green(img)
    green_sensor_array = sensor_color(green_threshold)
    red_sensor_array = sensor_color(red_threshold)

    # cv2.imshow("red", red_threshold)
    # cv2.imshow("green", green_threshold)
    # cv2.waitKey(1)
    v_left = 1.0
    v_right = 1.0
    for i in range(len(sonar_readings)):
        v_left += sonar_readings[i]*braitenberg_sonar_L[i]
        v_right += sonar_readings[i]*braitenberg_sonar_R[i]

    print v_left, "  ", v_right
    # TODO: add red cube avoidance terms
    for i in range(len(red_sensor_array)):
        v_left += red_sensor_array[i] * braitenberg_red_L[i]
        v_right += red_sensor_array[i] * braitenberg_red_R[i]
        
    for i in range(len(green_sensor_array)):
        v_left += green_sensor_array[i] * braitenberg_green_L[i]
        v_right += green_sensor_array[i] * braitenberg_green_R[i]

    print v_left, "  ", v_right

    # print v_left, "  ", v_right
    sim.simxSetJointTargetVelocity(ID, left_motor, v_left, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(ID, right_motor, v_right, sim.simx_opmode_oneshot)


def read_image(image_ready, ID, handler):
    res, resolution, image = sim.simxGetVisionSensorImage(ID, handler, 0, sim.simx_opmode_buffer)

    if res == sim.simx_return_ok:
        if not image_ready:
            print "image OK!!!"
            image_ready = True
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        return img, image_ready
    elif res == sim.simx_return_novalue_flag:
        if image_ready:
            print "no image"
            image_ready = False
        return np.array([], dtype=np.uint8), image_ready
    else:
        print "error: " + str(res)
        return np.array([], dtype=np.uint8), image_ready


def read_sonar(ID, handlers, sonar_ready):
    points = [None]*8
    states = [False]*8
    for i in range(8):
        res, states[i], points[i], _, normal_vec = sim.simxReadProximitySensor(ID, handlers[i], sim.simx_opmode_buffer)

    dists = [i[2] for i in points]
    if sonar_ready:
        for i in range(len(dists)):
            if states[i] and dists[i] < 0.5:
                if dists[i] < 0.2:
                    dists[i] = 0.2

                # map how close an obstacle is to the robot to [0, 1]
                dists[i] = 1.0 - (dists[i] - 0.2) / (0.5 - 0.2)
            else:
                dists[i] = 0.0
        return dists, sonar_ready
    else:
        flag = True
        for i in range(len(dists)):
            if dists[i] == 0.0:
                flag = False
                break
        if flag:
            sonar_ready = True
        return None, sonar_ready


if __name__ == "__main__":
    print ('Program started')
    sim.simxFinish(-1)  # just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

    if clientID != -1:
        print ('Connected to remote API server')

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print ('Number of objects in the scene: ', len(objs))
        else:
            print ('Remote API function call returned with error code: ', res)
        time.sleep(2)

        # get vision sensor handler
        print 'Vision Sensor object handling'
        res, veh_camera = sim.simxGetObjectHandle(clientID, 'veh_camera', sim.simx_opmode_oneshot_wait)
        # get sonor handler
        print 'Sonar object handling'
        veh_sonar = [None]*8
        for i in range(8):
            res, veh_sonar[i] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+'{}'.format(i+1),
                                                        sim.simx_opmode_oneshot_wait)
            # print res == sim.simx_return_ok
        # get left motor handler
        res, veh_left_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
        # print res == sim.simx_return_ok
        # get right motor handler
        res, veh_right_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
        # print res == sim.simx_return_ok

        # let the server prepare the first image
        print 'Getting first image'
        res, resolution, image = sim.simxGetVisionSensorImage(clientID, veh_camera, 0, sim.simx_opmode_streaming)
        image_ready_flag = False

        # let the server prepare the first sonar reading
        points = [None] * 8
        for i in range(8):
            res, state, points[i], _, normal_vec = sim.simxReadProximitySensor(clientID, veh_sonar[i],
                                                                               sim.simx_opmode_streaming)
        braitenberg_sonar_L = [0.2, 0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.2]
        braitenberg_sonar_R = [-1.2, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2]

        # TODO: fix the red cube avoidance
        braitenberg_red_L = [-0.65, -0.6, -0.55, -0.5, -0.45, -0.4, -0.35, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.10]
        braitenberg_red_R = [0.10, 0.05, 0.0, -0.05, -0.1, -0.15, -0.2, -0.25, -0.3, -0.35, -0.4, -0.45, -0.5, -0.55, -0.6, -0.65]

        braitenberg_green_L = [0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
        braitenberg_green_R = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]

        # braitenberg_sonar_L = [0.6, 0.8, 0.6, 0.4, -0.4, -0.6, -0.8, -0.6]
        # braitenberg_sonar_R = [-0.6, -0.8, -0.6, -0.4, 0.4, 0.6, 0.8, 0.6]

        # print [j[2] for j in points]
        sonar_ready_flag = False

        # keep running until the server shuts down
        while sim.simxGetConnectionId(clientID) != -1:
            image, image_ready_flag = read_image(image_ready_flag, clientID, veh_camera)
            detections, sonar_ready_flag = read_sonar(clientID, veh_sonar, sonar_ready_flag)
            # print detections
            # if image_ready_flag:
            #     cv2.imshow("image", image)
            #     cv2.waitKey(1)
            if image_ready_flag and sonar_ready_flag and not detections is None:
                free_running(detections, image, clientID, veh_left_motor, veh_right_motor)

        cv2.destroyAllWindows()
        # Now send some data to CoppeliaSim in a non-blocking fashion:
        # sim.simxAddStatusbarMessage(clientID, 'Hello CoppeliaSim!', sim.simx_opmode_oneshot)

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You
        # can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print 'Failed connecting to remote API server'
    print 'Program ended'

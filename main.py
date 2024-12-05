from djitellopy import Tello
import time
from threading import Thread, Event
import cv2
from ultralytics import YOLO
from time import time, sleep

DRON_H = 150
BASE_H = 30
BASE_WIDTH = 30
BASE_HEIGHT = 22.5

CUBE_SIDE = 250
MINI_CUBE_SIDE = CUBE_SIDE // 2

model = YOLO('./best.pt')

dron = Tello()
dron.connect()
print(dron.get_battery())
dron.streamon()
steam = dron.get_frame_read()

mahine_detect = Event()
running = Event()
running.set()

xy = (0, 0)

mahine_centr = (0, 0)

svframe = None

def dron_thread():
    global xy
    # dron.takeoff()

    dh = dron.get_distance_tof()
    to_go = 150 - dh
    # if to_go >= 20:med(0, 0, to_go, 30)
    dh = dron.get_distance_tof()
    while running.is_set():
        print('start')
        # dgo = (int(0.2 * CUBE_SIDE) - xy[0], -int(0.2 * CUBE_SIDE) - xy[1])
        # dron.go_xyz_speed(dgo[0], dgo[1], 0, 30)
        # xy = (xy[0] + dgo[0], xy[1] + dgo[1])
        print('cube')
        while not mahine_detect.is_set():
            sleep(2)
            # dgo = (MINI_CUBE_SIDE, 0)
            # dron.go_xyz_speed(dgo[0], dgo[1], 0, 30)
            # xy = (xy[0] + dgo[0], xy[1] + dgo[1])
            # if mahine_detect.is_set():
            #     break
            #
            # dgo = (0, -MINI_CUBE_SIDE)
            # dron.go_xyz_speed(dgo[0], dgo[1], 0, 30)
            # xy = (xy[0] + dgo[0], xy[1] + dgo[1])
            # if mahine_detect.is_set():
            #     break
            #
            # dgo = (-MINI_CUBE_SIDE, 0)
            # dron.go_xyz_speed(dgo[0], dgo[1], 0, 30)
            # xy = (xy[0] + dgo[0], xy[1] + dgo[1])
            # if mahine_detect.is_set():
            #     break
            #
            # dgo = (0, MINI_CUBE_SIDE)
            # dron.go_xyz_speed(dgo[0], dgo[1], 0, 30)
            # xy = (xy[0] + dgo[0], xy[1] + dgo[1])
            # if mahine_detect.is_set():
            #     break

        print('go machine')
        out = cv2.VideoWriter('out.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))
        while mahine_detect.is_set():
            print()
            out.write(svframe)
            dgo = (-(mahine_centr[0] - 320) * 0.23,
                   -(mahine_centr[1] - 240) * 0.23)
            dgo = (int(dgo[1]) if abs(dgo[1]) >= 20 else 0,
                   int(dgo[0]) if abs(dgo[0]) >= 20 else 0)
            print(mahine_centr)
            print(dgo)
            print(xy)
            # dron.go_xyz_speed(dgo[0], dgo[1], 0, 30)
            xy = (xy[0] + dgo[0], xy[1] + dgo[1])
            sleep(1)
        out.release()
    time.sleep(5)
    dron.land()


def cam_thread():
    global steam, mahine_centr, svframe
    t_detect = 0
    while True:
        frame = steam.frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (640, 480))

        model_out = model(frame, verbose=False)[0]

        for mahine in model_out.boxes:
            # print()
            # print(mahine.xyxy)
            # print(mahine.cls)
            # print(mahine.conf)

            if mahine.conf > 0.8:
                rect = [int(x) for x in mahine.xyxy[0]]

                mahine_centr = [int(x) for x in mahine.xywh[0][:2]]

                cv2.rectangle(frame, rect[:2], rect[2:], (0, 0, 255), 2)
                cv2.circle(frame, mahine_centr, 4, (0, 0, 255), -1)
                mahine_detect.set()
                t_detect = time()
                break

        if time() - t_detect > 20:
            mahine_detect.clear()

        cv2.imshow('s', frame)
        svframe = frame.copy()
        if cv2.waitKey(1) == 27:
            break
    cv2.destroyAllWindows()


Thread(target=dron_thread).start()
Thread(target=cam_thread).start()

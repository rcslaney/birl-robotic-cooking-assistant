from pyzbar.pyzbar import decode
import cv2
from pyzbar.pyzbar import ZBarSymbol
import numpy as np

from dt_apriltags import Detector

import waypoints as wp
import kg_robot as kgr

print("------------Configuring Burt-------------\r\n")
#burt = kgr.kg_robot(port=30010, db_host="192.168.1.6")
print("----------------Hi Burt!-----------------\r\n\r\n")



#initial_pose = burt.getl()

#x_av = [initial_pose[0]] * 5
#y_av = [initial_pose[1]] * 5

#print("Initial pose: ", initial_pose)

# define a video capture object
vid = cv2.VideoCapture(2, cv2.CAP_V4L)

tag_locations = np.zeros((100, 3))
detections = [None] * 100

while True:

    # Capture the video frame
    # by frame
    ret, frame = vid.read()

    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    at_detector = Detector(searchpath=['apriltags'],
                           families='tag36h11',
                           nthreads=1,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

    curr_detections = at_detector.detect(grayFrame, estimate_tag_pose=True, camera_params=[1402, 1402, 642, 470], tag_size=0.091)

    for i in range(len(curr_detections)):
        detections[curr_detections[i].tag_id] = curr_detections[i]
        print(i)

    blur = cv2.GaussianBlur(grayFrame, (5, 5), 0)
    ret, bw_im = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # zbar
    qr = decode(bw_im, symbols=[ZBarSymbol.QRCODE])

    # print(qr)

    for j in range(len(qr)):
        poly = qr[j].polygon

        for i in range(0, 4):
            frame = cv2.line(frame, (poly[i].x, poly[i].y), (poly[(i + 1) % 4].x, poly[(i + 1) % 4].y), (255, 0, 0), 8)

    camera_matrix = np.eye(3)

    for d in detections:
        if d is None:
            continue
        poly = d.corners
        for i in range(0, 4):
            frame = cv2.line(frame, (round(poly[i][0]), round(poly[i][1])), (round(poly[(i + 1) % 4][0]), round(poly[(i + 1) % 4][1])), (255, 0, 0), 8)
    font = cv2.FONT_HERSHEY_SIMPLEX

    for d in curr_detections:
        if d is None:
            continue
        # print(detections[i].pose_R)
        pt = d.pose_t
        pt_f = np.array([pt[0][0], pt[1][0], pt[2][0]])
        pt_t = np.linalg.inv(detections[0].pose_R) @ pt_f

        # cv2.putText(frame, "Tag " + str(detections[i].tag_id) + ": " + str(round(pt[0][0] * 100)) + ", " + str(round(pt[1][0] * 100)) + ", " + str(round(pt[2][0] * 100)), (10, 40 + i * 40), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Tag " + str(d.tag_id) + ": " + str(round(pt_t[0] * 100)) + ", " + str(round(pt_t[1] * 100)) + ", " + str(round(pt_t[2] * 100)), (10, 40 + d.tag_id * 40), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        tag_locations[d.tag_id] = pt_t

    try:
        robot_loc = np.copy(tag_locations[0])
        robot_loc[0] -= .28
        robot_loc[1] -= .53
        tag1_loc_relative = tag_locations[1] - robot_loc
        np.set_printoptions(precision=2)
        np.set_printoptions(suppress=True)
        print(tag1_loc_relative)

        #new_pose = initial_pose

        #x_av.pop(0)
        #x_av.append(-tag1_loc_relative[0])

        #y_av.pop(0)
        #y_av.append(tag1_loc_relative[1])

        #new_pose[0] = sum(x_av)/5
        #new_pose[1] = sum(y_av)/5

        #print(new_pose[0], new_pose[1])

        # burt.movejl(new_pose, wait=False, acc=1, min_time=0, radius=0.005)
    except:
        pass


    # Display the resulting frame
    cv2.imshow('frame', frame)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()

"""
  tag36h11
  tag25h9
  tag16h5
  tagCircle21h7
  tagCircle49h12
  tagStandard41h12
  tagStandard52h13
  tagCustom48h12
"""
from pyzbar.pyzbar import decode
import cv2
from pyzbar.pyzbar import ZBarSymbol
import numpy as np

from dt_apriltags import Detector

# define a video capture object
vid = cv2.VideoCapture(3)

width = 640
height = 480
vid.set(cv2.CAP_PROP_FRAME_WIDTH, width)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
vid.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
vid.set(cv2.CAP_PROP_EXPOSURE, -6)
tag_locations = np.zeros((10, 3))

while True:

    # Capture the video frame
    # by frame
    ret, frame = vid.read()

    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    at_detector = Detector(searchpath=['apriltags'],
                           families='tag36h11',
                           nthreads=4,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

    detections = at_detector.detect(grayFrame, estimate_tag_pose=True, camera_params=[1402, 1402, 642, 470], tag_size=0.091)



    blur = cv2.GaussianBlur(grayFrame, (5, 5), 0)
    ret, bw_im = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # zbar
    #qr = decode(bw_im, symbols=[ZBarSymbol.QRCODE])

    # print(qr)

    # for j in range(len(qr)):
    #     poly = qr[j].polygon
    #
    #     for i in range(0, 4):
    #         frame = cv2.line(frame, (poly[i].x, poly[i].y), (poly[(i + 1) % 4].x, poly[(i + 1) % 4].y), (255, 0, 0), 8)

    for j in range(len(detections)):
        poly = detections[j].corners
        for i in range(0, 4):
            frame = cv2.line(frame, (round(poly[i][0]), round(poly[i][1])), (round(poly[(i + 1) % 4][0]), round(poly[(i + 1) % 4][1])), (255, 0, 0), 8)
    font = cv2.FONT_HERSHEY_SIMPLEX

    for i in range(len(detections)):
        print(i, detections[i].tag_id)
        pt = detections[i].pose_t
        pt_b = detections[0].pose_t
        pt_f = np.array([pt[0][0], pt[1][0], pt[2][0]])
        pt_b = np.array([pt_b[0][0], pt_b[1][0], pt_b[2][0]])
        pt_t = (pt_f - pt_b)

        # cv2.putText(frame, "Tag " + str(detections[i].tag_id) + ": " + str(round(pt[0][0] * 100)) + ", " + str(round(pt[1][0] * 100)) + ", " + str(round(pt[2][0] * 100)), (10, 40 + i * 40), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Tag " + str(detections[i].tag_id) + ": " + str(round(pt_t[0] * 100)) + ", " + str(round(pt_t[1] * 100)) + ", " + str(round(pt_t[2] * 100)), (10, 40 + i * 40), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        tag_locations[detections[i].tag_id] = pt_t




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
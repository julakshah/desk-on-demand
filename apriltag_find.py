import cv2 
import sys
import numpy as np

# Parameters for Realsense Color channel, 
RS_INTRINSIC_COLOR_640 = np.array([
    [615.21,0,310.90],[0,614.45,243.97],[0,0,1]
])

RS_DIST_COLOR_640 = np.array([0,0,0,0,0])

def main(channel):
    cap = cv2.VideoCapture(int(channel))
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    #w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    #print(f"w: {w}")
    if not cap.isOpened():
        print("Could not open video")
        sys.exit()

    while 1:
        ret, frame = cap.read()
        if not ret:
            print("Could not read frame")
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        detect_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict,detectorParams=detect_params)
        corners = []
        ids = cv2.UMat()
        corners, ids, rejected = detector.detectMarkers(frame)
        #print(f"corners: {corners}, ids: {ids}")

        marker_length = 100 # 100 mm
        obj_pt_arr = np.asarray([
            [-marker_length/2.0,marker_length/2.0,0],
            [marker_length/2.0,marker_length/2.0,0],
            [marker_length/2.0,-marker_length/2.0,0],
            [-marker_length/2.0,-marker_length/2.0,0]
        ])
        obj_points = cv2.Mat(obj_pt_arr)
        camMatrix = RS_INTRINSIC_COLOR_640
        distCoeffs = RS_DIST_COLOR_640

        img_clone = frame
        has_found_tag = False
        if len(corners) > 0: # if we found a marker
            has_found_tag = True
            n_markers = len(corners[0])
            #print(n_markers)

            for i in range(n_markers):
                #print("solving pnp")
                _, rvecs, tvecs = cv2.solvePnP(
                    objectPoints=obj_pt_arr,
                    imagePoints=corners[i],
                    cameraMatrix=camMatrix,
                    distCoeffs=distCoeffs,
                )

            img_clone = cv2.aruco.drawDetectedMarkers(img_clone,corners,ids)
            #print(f"rvec: {rvecs}")
            #print(f"X: {tvecs[0]*0.001} m, \nY: {tvecs[1]*0.001} m, \nZ: {tvecs[2]*0.001} m")

            for i in range(len(ids)):
                cv2.drawFrameAxes(image=img_clone,
                    cameraMatrix=camMatrix,
                    distCoeffs=distCoeffs,
                    rvec=rvecs,
                    tvec=tvecs,
                    length=100.0,
                    thickness=10
                )

        cv2.imshow("Camera Feed",img_clone)
        if cv2.waitKey(1) & 0xFF == ord('p'):
            # Print pose relative to camera
            if has_found_tag:
                print(f"X: {tvecs[0]*0.001} m, \nY: {tvecs[1]*0.001} m, \nZ: {tvecs[2]*0.001} m")
                # X right, Y down, Z forward

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    if len(sys.argv)> 1:
        channel = sys.argv[1]
    else:
        channel = 8 # realsense RGB color channel by default on my computer
    main(channel=channel)
import rospy
import numpy as np
import cv2
import matplotlib.pyplot as plt

plt.ion()
cap = cv2.VideoCapture(0)
fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows=False,history=30) #circa 1 second

fig, ax = plt.subplots()	
orb = cv2.ORB_create()

data = list()

cv2.waitKey(1)

for i in range(10):
    retvale, o_frame = cap.read()
    #o_frame = fgbg.apply(o_frame) #THis produces noise
    o_frame = cv2.cvtColor(o_frame, cv2.COLOR_BGR2GRAY)

# find the keypoints with ORB
kp0 = orb.detect(o_frame,None)
# compute the descriptors with ORB
kp0, des0 = orb.compute(o_frame, kp0)
# draw only keypoints location,not size and orientation
#img2 = cv2.drawKeypoints(o_frame, kp0, None, color=(0,255,0), flags=0)
#cv2.imshow("original", img2)
MIN_MATCH_COUNT = 4

ret,thresh = cv2.threshold(o_frame,127,200,0)
img,o_contours,hierarchy = cv2.findContours(thresh, 1, 2)

cnt = max(o_contours, key = cv2.contourArea)
original_area = cv2.contourArea(cnt)


for i in range (1,1000):
    scale = -1
    #FIRST APPROACH POINTS AREA MATCHING
    retvale, frame = cap.read()
    #not so sure if needed... noise
    #fgmask = fgbg.apply(frame)
    #instead 
    fgmask = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #find the keypoints with ORB
    kp = orb.detect(fgmask,None)
    # compute the descriptors with ORB
    kp, des = orb.compute(fgmask, kp)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 30)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(np.asarray(des0,np.float32),np.asarray(des,np.float32), 2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)


    #COUNTOURS APPROACH
    ret,thresh = cv2.threshold(fgmask,127,200,0)
    img,contours,hierarchy = cv2.findContours(thresh, 1, 2)

    defects = None
    if len(contours) >2:
        countour_frame = frame
        c = max(contours, key = cv2.contourArea)
	area = cv2.contourArea(c)
        hull = cv2.convexHull(cnt,returnPoints = False)
        defects = cv2.convexityDefects(cnt,hull)
        complete_hull=[cv2.convexHull(c) for c in contours]
        hull_cnt_frame = cv2.drawContours(countour_frame, complete_hull, -1, (255,0,0))
        #cv2.imshow("ALL Contours", hull_cnt_frame)

        countour_frame = frame
        # draw in blue the contours that were founded
        #cv2.drawContours(frame, contours, -1, 255, 3)
        x,y,w,h = cv2.boundingRect(c)
        # draw the biggest contour (in green)
        cv2.rectangle(countour_frame,(x,y),(x+w,y+h),(0,255,0),2)
        # show the images
        cv2.imshow("Biggest Contour Found", countour_frame)

    #print "SCALE " , scale
    match_frame = cv2.drawMatches(o_frame, kp0, fgmask, kp, good, None) 
    cv2.imshow("MATCHES", match_frame)

    #scale = original_area/area

    #THIRD
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp0[m.queryIdx].pt for m in good ])#.reshape(-1,1,2)
        dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ])#.reshape(-1,1,2)
	area = cv2.contourArea(src_pts)
	f_area = cv2.contourArea(dst_pts)
        #print "OAREA", area
        #print "FAREA", f_area
        scale = f_area/area
 
    """
    #FOURTH COMPUTING PERSPECGTIVE TRANSFORMATION IN DEVELOPMENT
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp0[m.queryIdx].pt for m in good ])#.reshape(-1,1,2)
        dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ])#.reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
 
        try:
            dst = cv2.perspectiveTransform(src_pts,dst_pts)
            #print "PERS" , dst
            #img2 = cv2.polylines(fgmask,[np.int32(dst)],True,255,3, cv2.LINE_AA)
            #cv2.drawKeypoints(fgmask, kp, img2, color=(255,255,0), flags=0)
            #cv2.imshow("process", img2)
        except:
            print "soomething fails"
    #cv2.imshow("CURRENT", fgmask)
    """
    x = np.arange(i)
    data.append(scale)
    plt.scatter(x, data)
    fig.canvas.draw_idle()
    plt.pause(0.1)
    cv2.waitKey(10)
cap.release()
cv2.destroyAllWindows()

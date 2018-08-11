import rospy
import numpy as np
import cv2

cap = cv2.VideoCapture(0)
fgbg = cv2.createBackgroundSubtractorMOG2()

orb = cv2.ORB_create()

cv2.waitKey(1)

for i in range(10):
    retvale, o_frame = cap.read()
    fgmask = fgbg.apply(o_frame)

# find the keypoints with ORB
kp0 = orb.detect(fgmask,None)
# compute the descriptors with ORB
kp0, des0 = orb.compute(fgmask, kp0)
# draw only keypoints location,not size and orientation
img2 = cv2.drawKeypoints(fgmask, kp0, None, color=(0,255,0), flags=0)
cv2.imshow("original", img2)
MIN_MATCH_COUNT = 4

ret,thresh = cv2.threshold(fgmask,127,255,0)
img,o_contours,hierarchy = cv2.findContours(thresh, 1, 2)

largest_area = -1

for i in range(0, len(o_contours)):
    area = cv2.contourArea(o_contours[i])
    if (area>largest_area):
        largest_area=area
        largest_contour_index=i
cnt = o_contours[largest_contour_index]
original_area = cv2.contourArea(cnt)
print "ORIGINAL AREA ", original_area

for i in range (1,1000):

    #FIRST APPROACH POINTS AREA MATCHING
    retvale, frame = cap.read()
    #not so sure if needed... noise
    #fgmask = fgbg.apply(frame)
    #instead 
    fgmask = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # find the keypoints with ORB
    kp = orb.detect(fgmask,None)
    # compute the descriptors with ORB
    kp, des = orb.compute(fgmask, kp)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(np.asarray(des0,np.float32),np.asarray(des,np.float32), 2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)


    #COUNTOURS APPROACH
    ret,thresh = cv2.threshold(fgmask,127,255,0)
    img,contours,hierarchy = cv2.findContours(thresh, 1, 2)

    defects = None
    if len(contours) >2:
        c = max(contours, key = cv2.contourArea)
	area = cv2.contourArea(c)
        hull = cv2.convexHull(cnt,returnPoints = False)
        defects = cv2.convexityDefects(cnt,hull)
        complete_hull=[cv2.convexHull(c) for c in contours]
        hull_cnt_frame = cv2.drawContours(frame, complete_hull, -1, (255,0,0))
        cv2.imshow("MAX AREA", hull_cnt_frame)

        # draw in blue the contours that were founded
        cv2.drawContours(frame, contours, -1, 255, 3)
        #find the biggest area
        c = max(contours, key = cv2.contourArea)

        countour_frame = frame
        x,y,w,h = cv2.boundingRect(c)
        # draw the book contour (in green)
        cv2.rectangle(countour_frame,(x,y),(x+w,y+h),(0,255,0),2)
        # show the images
        cv2.imshow("Result", countour_frame)

    scale = original_area/area
    #print "SCALE " , scale

    #THIRD
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp0[m.queryIdx].pt for m in good ])#.reshape(-1,1,2)
        dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ])#.reshape(-1,1,2)
	area = cv2.contourArea(src_pts)
	f_area = cv2.contourArea(dst_pts)
        print "OAREA", area
        print "FAREA", f_area
        print "SCALE", f_area/area
 

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
    cv2.waitKey(10)
cap.release()
cv2.destroyAllWindows()

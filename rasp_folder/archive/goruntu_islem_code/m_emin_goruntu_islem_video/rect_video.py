import cv2          
import numpy as np      #importing library
        
cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
size = (640,480)
out = cv2.VideoWriter('output_rect.mp4', fourcc, 24, size)
#frame = cv2.imread('kirmizi.png')

while True:
    ret, frame = cap.read() 
    
    if not ret:
        break
    
    else:
        
        frame = cv2.resize(frame, size)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # converting BGR to HSV 

        lower1 = [0, 180, 20]                            
        upper1 = [10, 255, 255]                       # choosing wanted color boundry 
            # converting color array 'int' to 'unsigned int' 
            
        lower1 = np.array(lower1, dtype="uint8")      
        upper1 = np.array(upper1, dtype="uint8")  

        lower2 = [170, 180, 20]                            
        upper2 = [179, 255, 255]                       # choosing wanted color boundry 

        lower2 = np.array(lower2, dtype="uint8")      
        upper2 = np.array(upper2, dtype="uint8")  

        mask1 = cv2.inRange(hsv, lower1, upper1)        # mask can be thinking as big binary array that contains
                                                        # 0 or 1 pixel value that can be considered black or white


        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask_f = mask1 + mask2

        mask_f = cv2.GaussianBlur(mask_f, (21, 21), 0)  # blurring the frame for reducing noise
        edge = cv2.Canny(mask_f, 125, 175)

        frame_cont = frame.copy()

        contours, hier = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            contour = max(contours, key = cv2.contourArea)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            mean_x = (box[0][0] + box[2][0])/2
            mean_x = np.int64(mean_x)
            mean_y = (box[0][1] + box[2][1])/2
            mean_y = np.int64(mean_y)

            cv2.circle(frame_cont,(mean_x, mean_y),3,(255,0,0),-1)

            cv2.drawContours(frame_cont,[box],0,(0,255,0),2)


        (way_point_x, way_point_y) = (-1,-1)
        masked_output = cv2.bitwise_and(hsv, hsv, mask=mask_f)  # stacking togather mask and original frame with bitwise
                                                                # and operator. 'bitwise_and' operator filter just only 
                                                                # if both of frame have nonzero values in particular pixel.

        #out.write(frame_cont)
        cv2.imshow("Masked Frame", masked_output)       # showing masked frame
        #cv2.imshow("Original Frame", frame)            # showing original frame
        cv2.imshow('Countours', frame_cont)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

print('zoooort')
cap.release()
out.release()

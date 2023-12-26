import cv2          
import numpy as np      #importing library
from time import sleep

class camera_class():

    def __init__(self,video,fourcc,out):  
        
        self.video = video
        self.fourcc = fourcc
        self.out = out

    def detect_x_y(self,size,show,record_detect):
        
        (grabbed, frame) = self.video.read()   
        if not grabbed:
            return print("No frame detected.")# if frame does not captured kill the loop
            
        frame = cv2.resize(frame, (size[0],size[1]))
        self.frame_orig = frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # converting BGR to HSV 

        lower1 = [5, 90, 50]                            
        upper1 = [20, 255, 255]                   # choosing wanted color boundry 
        lower1 = np.array(lower1, dtype="uint8")      
        upper1 = np.array(upper1, dtype="uint8")  

        #lower2 = [160, 100, 20]                            
        #upper2 = [179, 255, 255]                       # choosing wanted color boundry 
        #lower2 = np.array(lower2, dtype="uint8")      
        #upper2 = np.array(upper2, dtype="uint8")  

        mask1 = cv2.inRange(hsv, lower1, upper1)        # mask can be thinking as big binary array that contains
        #mask2 = cv2.inRange(hsv, lower2, upper2)        # 0 or 1 pixel value that can be considered black or whit

        #mask_f = mask1 + mask2
        mask_f = mask1

        mask_f = cv2.GaussianBlur(mask_f, (21, 21), 0)  # blurring the frame for reducing noise
        
        edge = cv2.Canny(mask_f, 125, 175)              # edge detection 

        frame_cont = frame.copy()

        contours, hier = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        masked_output = cv2.bitwise_and(hsv, hsv, mask=mask_f)  # stacking togather mask and original frame with bitwise
                                                                    # and operator. 'bitwise_and' operator filter just only 
                                                                    # if both of frame have nonzero values in particular pixel.
        if len(contours) != 0:
            contour = max(contours, key = cv2.contourArea)
            if cv2.contourArea(contour) < 4:

                #print(cv2.contourArea(contour))
                
                if record_detect: 
                    self.out.write(frame_cont)
                
                if show:
                    #cv2.imshow("Masked Frame", masked_output)       # showing masked frame
                    cv2.imshow('Frame', frame_cont)
                    #cv2.imshow("Original Frame", frame)            # showing original frame
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        return None            
                return None

            else:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                mean_x = (box[0][0] + box[2][0])/2
                mean_x = np.int64(mean_x)
                mean_y = (box[0][1] + box[2][1])/2
                mean_y = np.int64(mean_y)

                cv2.circle(frame_cont,(mean_x, mean_y),3,(255,0,0),-1)
                cv2.drawContours(frame_cont,[box],0,(0,255,0),2)
                
                if record_detect: 
                    self.out.write(frame_cont)
                
                if show:
                    #cv2.imshow("Masked Frame", masked_output)       # showing masked frame
                    cv2.imshow('Frame', frame_cont)
                    #cv2.imshow("Original Frame", frame)            # showing original frame
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        return None

                (self.wp_x_px, self.wp_y_px) = (mean_x, mean_y)
                return (self.wp_x_px, self.wp_y_px)
        
        else:
            #print('no contor')
            if record_detect: 
                self.out.write(self.frame_orig)
                
            if show:
                #cv2.imshow("Masked Frame", masked_output)       # showing masked frame
                #cv2.imshow('Countours', frame_cont)            
                cv2.imshow("Frame", self.frame_orig)            # showing original frame
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return None

            (self.wp_x_px, self.wp_y_px) = (None, None)
            return None
    
    def only_record_orig(self,size):
        
        (grabbed, frame) = self.video.read()   
        if not grabbed:
            return print("No frame detected.")# if frame does not captured kill the loop
            
        frame = cv2.resize(frame, (size[0],size[1]))
        self.frame_orig = frame
        return self.out.write(self.frame_orig)
    
# video = cv2.VideoCapture('red_rect.mp4')
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# out = cv2.VideoWriter('output.mp4', fourcc, 24, (1920,1080))

# while True:
#     a  = camera_class(video,fourcc,out)
#     z = a.detect_x_y((1920,1080),False,True)
    
#     if z != None:
#         if z[0] != None:
#             print(z)
# video.release()
# out.release()

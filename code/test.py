import cv2 as cv
import numpy as np
import serial # giao tiep ardunio
import time 
# nguong so sanh
CONFIDENCE_THRESHOLD = 0.4
NMS_THRESHOLD = 0.3
KNOWN_DISTANCE = 202 #cm
PERSON_WIDTH = 163 #cm

# mau cho doi tuong
COLORS = [(255,0,0),(255,0,255),(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]
GREEN =(0,255,0)
BLACK =(0,0,0)
YELLOW =(0,255,255)
PERPEL = (255,0,255)
WHITE = (255,255,255)
FONTS = cv.FONT_HERSHEY_COMPLEX
# doc class tu file txt
class_names = []
with open("classes.txt", "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]
yoloNet = cv.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
#su dung gpu
yoloNet.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
yoloNet.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA)

model = cv.dnn_DetectionModel(yoloNet)
model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)

# cau hinh camera


def object_detector(image):
    classes, scores, boxes = model.detect(image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    # tao tep du lieu doi tương
    data_list = []
    for (classid, score, box) in zip(classes, scores, boxes):
     if classid == 0: # id cua class nguoi la 0
        # mau cho doi tuong
        color = COLORS[int(classid) % len(COLORS)]

        label = "%s : %f" % (class_names[classid[0]], score)
 
        # ve hcn tren lable
        cv.rectangle(image, box, color, 2)

        cv.putText(image, label, (box[0], box[1] - 14), FONTS, 0.5, color, 2)

        # getting the data
        # 0: ten class  1: chieu cao cua nguoi trong anh, 2: vi tri ve text , 3 tao do x1 cua box, 4 toa do x2 cua box
      # person class id
        data_list.append([class_names[classid[0]], box[3], (box[0], box[1] - 2),box[0],box[2]])  
        break
    return data_list
# ham tim noi suy  
def focal_length_finder (measured_distance, real_width, width_in_rf):
    focal_length = (width_in_rf * measured_distance) / real_width

    return focal_length

# ham tim kc
def distance_finder(focal_length, real_object_width, width_in_frame):
    distance = (real_object_width * focal_length) / width_in_frame
    return distance
# tim chieu cao trong anh tu cac anh mau

# anh mau 1
ref_person_1 = cv.imread('anhmau/image200.png')
person_data_1 = object_detector(ref_person_1)
person_width_in_rf_1 = person_data_1[0][1]
# anh mau 2
ref_person_2 = cv.imread('anhmau/image240.png')
person_data_2 = object_detector(ref_person_2)
person_width_in_rf_2 = person_data_2[0][1]
# anh mau 3
ref_person_3 = cv.imread('anhmau/image280.png')
person_data_3 = object_detector(ref_person_3)
person_width_in_rf_3 = person_data_3[0][1]
# anh mau 4
ref_person_4 = cv.imread('anhmau/image320.png')
person_data_4 = object_detector(ref_person_4)
person_width_in_rf_4 = person_data_4[0][1]


# tim focal length
focal_person_1 = focal_length_finder(KNOWN_DISTANCE, PERSON_WIDTH, person_width_in_rf_1)
focal_person_2 = focal_length_finder(KNOWN_DISTANCE+40, PERSON_WIDTH, person_width_in_rf_2)
focal_person_3 = focal_length_finder(KNOWN_DISTANCE+40, PERSON_WIDTH, person_width_in_rf_3)
focal_person_4 = focal_length_finder(KNOWN_DISTANCE+40, PERSON_WIDTH, person_width_in_rf_4)
focal_person=(focal_person_1+focal_person_2+focal_person_3+focal_person_4)/4
print(f"tieu cu cua may anh : {focal_person}")
#thiet lap giao tiep ardunio
Arduino = serial.Serial(baudrate=9600, port = 'COM4')
huong =0
speed_motor1 =0 # toc do cua dong co 
speed_motor2 =0
speed_lon =90
speed_nho =100
speed_for_backward =120
# doc tu camera
camera = cv.VideoCapture(1)

while True:
    ret, frame = camera.read()

    data = object_detector(frame)
    for d in data:
        if d[0] == 'person':
            Right_Bound = 640-150
            Left_Bound =150
            box_x1=d[3]
            box_x2=d[3]+d[4]
            distance = distance_finder(focal_person, PERSON_WIDTH, d[1])
            x, y = d[2]
            cv.rectangle(frame, (x, y - 3), (x + 150, y + 23), BLACK, -1)
            cv.putText(frame, f'Distance: {round(distance, 2)} cm', (x + 5, y + 13), FONTS, 0.48, GREEN, 2)
            # dieu khien robot
            if box_x1<Left_Bound:
                 
                Motor1_Speed=0
                Motor2_Speed=speed_lon
                print("quay trai ")
                # huong di
                huong=4
                cv.line(frame, (50,65), (170, 65), (BLACK), 15)
                cv.putText(frame, f"quay trai ", (50,70), FONTS,0.4, (YELLOW),1)
            elif box_x2>Right_Bound:
                 
                Motor1_Speed=speed_lon-15
                Motor2_Speed=0
                print("quay phai ")
                # quay phai
                huong=3
                cv.line(frame, (50,65), (170, 65), (BLACK), 15)
                cv.putText(frame, f"quay phai ", (50,70), FONTS,0.4, (GREEN),1)

            elif distance >220:
                
                Motor1_Speed=speed_for_backward -15
                Motor2_Speed=speed_for_backward 
                #di thang
                huong=1
                cv.line(frame, (50,55), (200, 55), (BLACK), 15)
                cv.putText(frame, f"tien", (50,58), FONTS,0.4, (PERPEL),1)
                print("tien")
            
            elif distance >50 and distance<=180:
                 
                Motor1_Speed=speed_for_backward -15
                Motor2_Speed=speed_for_backward 
                # lui
                huong=2
                print("lui")
                cv.line(frame, (50,55), (200, 55), (BLACK), 15)
                cv.putText(frame, f"lui", (50,58), FONTS,0.4, (PERPEL),1)
            elif distance >180 and distance<=220:
               
                Motor1_Speed=0
                Motor2_Speed=0
                # dung im
                huong=0
                print(data)
                cv.line(frame, (50,55), (200, 55), (BLACK), 15)
                cv.putText(frame, f"dung im", (50,58), FONTS,0.4, (PERPEL),1)    
            else:
                time.sleep(0.5)
                Motor1_Speed=0
                Motor2_Speed=0
                # dung im
                huong=0
                print(data)
                cv.line(frame, (50,55), (200, 55), (BLACK), 15)
                cv.putText(frame, f"dung im", (50,58), FONTS,0.4, (PERPEL),1)
            
            data = f"A{Motor1_Speed}B{Motor2_Speed}D{huong}" #X180Y180D2
            print(data)
            Arduino.write(data.encode())
            time.sleep(0.002) # thoi gian gui du lieu delay
            Arduino.flushInput()


    cv.line(frame, (150, 80), (150, 480-80), (YELLOW), 2)
    cv.line(frame, (490, 80),(490, 480-80), (YELLOW), 2)
    cv.imshow('Webcam', frame)

    key = cv.waitKey(1)
    if key == ord('q'):
        break
camera.release()
cv.destroyAllWindows()
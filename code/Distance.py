import cv2 as cv
import numpy as np
# nguong so sanh
CONFIDENCE_THRESHOLD = 0.4
NMS_THRESHOLD = 0.3
KNOWN_DISTANCE = 202 #cm
PERSON_WIDTH = 163 #cm

# mau cho doi tuong
COLORS = [(255,0,0),(255,0,255),(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]
GREEN =(0,255,0)
BLACK =(0,0,0)
FONTS = cv.FONT_HERSHEY_COMPLEX
# doc class tu file txt
class_names = []
with open("classes.txt", "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]
#  cau hinh
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
       
        # mau cho doi tuong
        color = COLORS[int(classid) % len(COLORS)]

        label = "%s : %f" % (class_names[classid[0]], score)
 
        # ve hcn tren lable
        cv.rectangle(image, box, color, 2)

        cv.putText(image, label, (box[0], box[1] - 14), FONTS, 0.5, color, 2)
        # getting the data
        # 1: class name  2: object width in pixels, 3: position where have to draw text(distance)
        if classid == 0:  # person class id
            data_list.append([class_names[classid[0]], box[3], (box[0], box[1] - 2)])  
            print(f"x : {box[1]}")
            break
    return data_list

    
def focal_length_finder (measured_distance, real_width, width_in_rf):
    focal_length = (width_in_rf * measured_distance) / real_width

    return focal_length

# ham tim kc
def distance_finder(focal_length, real_object_width, width_in_frame):
    distance = (real_object_width * focal_length) / width_in_frame
    return distance


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
# doc tu camera
camera = cv.VideoCapture(0)

while True:
    ret, frame = camera.read()

    data = object_detector(frame)
    for d in data:
        if d[0] == 'person':
            distance = distance_finder(focal_person, PERSON_WIDTH, d[1])
            x, y = d[2]
        cv.rectangle(frame, (x, y - 3), (x + 150, y + 23), BLACK, -1)
        cv.putText(frame, f'Distance: {round(distance, 2)} cm', (x + 5, y + 13), FONTS, 0.48, GREEN, 2)

    cv.imshow('Webcam', frame)

    key = cv.waitKey(1)
    if key == ord('q'):
        break
camera.release()
cv.destroyAllWindows()

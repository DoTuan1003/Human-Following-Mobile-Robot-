import cv2 as cv
# nguong so sanh
CONFIDENCE_THRESHOLD = 0.4
NMS_THRESHOLD = 0.3
# mau cho doi tuong
COLORS = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]
RED = (0, 0, 255)
GREEN = (0, 255, 0)
PINK = (147, 20, 255)
ORANGE = (0, 69, 255)
fonts = cv.FONT_HERSHEY_COMPLEX
# doc class tu file txt
class_names = []
with open("classes.txt", "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]
#  cau hinh
yoloNet = cv.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
# sd gpu

yoloNet.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
yoloNet.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA)

model = cv.dnn_DetectionModel(yoloNet)
model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)




def ObjectDetector(image):
    classes, scores, boxes = model.detect(
        image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)

    for (classid, score, box) in zip(classes, scores, boxes):
        print(f"do dai nguoi trong anh pixels : {box}")
        color = COLORS[int(classid) % len(COLORS)]
        label = "%s : %f" % (class_names[classid[0]], score)
        cv.rectangle(image, box, PINK, 2)
        cv.putText(frame, label, (box[0], box[1]-10), fonts, 0.5, RED , 2)
        print(f"toa do : {box}")
camera = cv.VideoCapture(0)
while True:
    ret, frame = camera.read()
    ObjectDetector(frame)
    cv.imshow('WEBCAM', frame)
    key = cv.waitKey(1)
    if key == ord('q'):
        break
camera.release()
cv.destroyAllWindows()

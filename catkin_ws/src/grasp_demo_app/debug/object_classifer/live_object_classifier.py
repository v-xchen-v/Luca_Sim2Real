import os, sys
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)


from core.object_classifier import get_object_name_from_clip
# opencv capture image then predict and text on the image
import cv2
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite("data/debug_data/tmp_cropped_rgb_image.jpg", frame)
        # wait for a while
        cv2.waitKey(1000)
        
        # read the image and predict
        object_name = get_object_name_from_clip("data/debug_data/tmp_cropped_rgb_image.jpg")
        # got the object name
        print(object_name)
        # break
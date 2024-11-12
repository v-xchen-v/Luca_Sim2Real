import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

CHECK_CAMERA_CONNECTION = True
CHECK_CLASSIFIER = True
CHECK_REMOVEBG = True
if CHECK_CAMERA_CONNECTION:
    from pointcloud_processing.realsense_capture import realsense_instance
    import numpy as np
    rgb = realsense_instance.get_rgb_frame()
    if rgb is not None:
        print(f'Camera connected, shape: {rgb.shape}')
        
        
    pcd, rgb = realsense_instance.capture()
    if pcd is not None: 
        points = np.asarray(pcd.points)
        print(f'Point cloud captured, shape: {points.shape}') 
    
    
if CHECK_CLASSIFIER:
    import os, sys
    module_path = os.path.abspath('catkin_ws/src/grasp_demo_app')
    if module_path not in sys.path:
        sys.path.append(module_path)

    from core.object_classifier import get_object_name_from_clip
    import cv2
    cv2.imwrite('temp.jpg', rgb)
    object_name = get_object_name_from_clip('temp.jpg')
    print(f'Object Classifier Ready, object name: {object_name}')
    
if CHECK_REMOVEBG:
    from rembg import remove
    removed = remove(rgb)
    if removed is not None:
        print('Background Remover Ready')
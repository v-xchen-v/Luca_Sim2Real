import torch
import clip
from PIL import Image
import numpy as np

class ObjectClassifier:
    def __init__(self, class_names, model_name="ViT-B/16"):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load(model_name, device=self.device)
        self.class_names = class_names
        with torch.no_grad():
            self.text = clip.tokenize(class_names).to(self.device)

    def predict(self, image_path, crop=None):
        with torch.no_grad():
            if crop is None:
                image = self.preprocess(Image.open(image_path)).unsqueeze(0).to(self.device)
            else:
                image = self.preprocess(Image.open(image_path).crop(crop)).unsqueeze(0).to(self.device)
            image_features = self.model.encode_image(image)
            logits_per_image, logits_per_text = self.model(image, self.text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
        return np.argmax(probs)
    
    
    
class_names = [
    # "duck", 
    "orange", 
    "dark green tape measure", 
    "dark yellow box", 
    "toy hammer", 
    "Rubik's cube white cube",
    "milk juice",
    "coke can"]
object_classifier = ObjectClassifier(class_names=class_names, model_name='ViT-L/14')


class_mapping_dict = {
    "orange": "orange_1024",
    # "duck": "duck_1104",
    "dark green tape measure": "tape_measure_1105",
    "dark yellow box": "realsense_box_1024",
    "toy hammer": "hammer_1102",
    "Rubik's cube white cube": "cube_055_1103",
    "milk juice": "bottle_coconut_1105",
    "coke can": "coke_can_1104"
}

def get_object_name_from_clip(rgb_color_image_path):
    res = object_classifier.predict(rgb_color_image_path)
    return class_mapping_dict[class_names[res]]

if __name__ == "__main__":
    # res= classifier.predict("data/debug_data/cropped_rgb_image.jpg")
    # print(res)
    
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
            res = object_classifier.predict("data/debug_data/tmp_cropped_rgb_image.jpg")
            # got the object name
            object_name = class_mapping_dict[class_names[res]]
            print(object_name)
            break
import torch
from PIL import Image
from utils.general import (
    non_max_suppression
)


# Model
device = torch.device('cuda:0')
model = torch.hub.load('.', 'custom', path='/home/inspirationagx01/multi_cam_detection/src/multi_cam_obj_detection/multi_cam_obj_detection/best.pt', source='local', force_reload=True).to(device)

model.conf = 0.5
model.iou = 0.45

# model = DetectMultiBackend(weights="/home/chengjing/Desktop/multi_cam_detection/src/multi_cam_obj_detection/multi_cam_obj_detection/yolov5s.pt", device=device)

# Images
for f in ['zidane.jpg', 'bus.jpg']:  # download 2 images
    print(f'Downloading {f}...')
    torch.hub.download_url_to_file('https://github.com/ultralytics/yolov5/releases/download/v1.0/' + f, f)
imgs = [Image.open('bus.jpg'), Image.open('zidane.jpg')]  # batched list of imag


results = model(imgs).tolist()


for result in results:
    for i, det in enumerate(result.pred):
         for *box, conf, cls in reversed(det):
             print(box)


# results = non_max_suppression(results.pred, 0.5, 0.5, classes=None, agnostic=False, max_det=10)

# print(results.tolist())
# print(type(results))

# for result in results:

#     result = non_max_suppression(result, 0.5, 0.5, classes=None, agnostic=False, max_det=10)
    
#     print(type(result))
#     print(result)


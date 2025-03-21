from ultralytics import YOLO


model = YOLO("yolo11n.yaml")  # build a new model from YAML

results = model.train(data="path/data.yaml", epochs=300, imgsz=640, degrees=220, scale=0.75, perspective=0.0005, fliplr=0.5, mixup=0.05, copy_paste=0.1)

model.export(format="edgetpu")  # creates 'efficientdet-lite-road-signs_edgetpu.tflite'

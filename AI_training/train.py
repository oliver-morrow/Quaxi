from ultralytics import YOLO


# pathToBest="/home/chevan/.pyenv/runs/detect/train9/weights/"
model = YOLO("yolo11n.yaml")  # build a new model from YAML
# model = YOLO(pathToBest+"best.pt")

results = model.train(data="/media/DataPandemonium/chevan/dev/dataset/picar_yoloV11/data.yaml", epochs=300, imgsz=640, degrees=220, scale=0.75, perspective=0.0005, fliplr=0.5, mixup=0.05, copy_paste=0.1)

model.export(format="edgetpu")  # creates 'yolo11n_full_integer_quant_edgetpu.tflite'

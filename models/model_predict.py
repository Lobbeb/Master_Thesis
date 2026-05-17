from ultralytics import YOLO
from pathlib import Path
import argparse

IMAGES_PATH = Path("/home/ruben/halmstad_ws/datasets/baylands_leader_test/images/test/")
MODEL_PATH = Path("obb/mymodels/baylands-leader-v1-obb.pt")



parser = argparse.ArgumentParser(description="Example script with arguments")
parser.add_argument("--dataset", required=True, help="Input name of dataset")
parser.add_argument("--dataset-path", required=False, help="If specified, overrides the default dataset path")
parser.add_argument("--model", required=True, help="Input model file")

args = parser.parse_args()

images_path = Path("/home/ruben/halmstad_ws/datasets/", args.dataset, "images/test/*.jpg")
if args.dataset_path:
    images_path = Path(args.dataset_path, "images/test/")
model_path = Path("obb/mymodels/", args.model)

print("Dataset:", args.dataset)
if args.dataset_path:
    print("Dataset path:", args.dataset_path)
print("Images path:", images_path)
print("Model:", args.model, "\n")


model = YOLO(str(model_path))

results = model.predict(
    source=str(images_path),
    split="test",
    task="obb",
    project=Path("/home/ruben/halmstad_ws/models/results/",args.model),
    conf=0.65,
    save=True,
    max_det=1,
    stream=True,
    visualize=False,
)

for result in results:
    if result.obb is None or len(result.obb) == 0:
        print(f"No detections for {result.path}")
        continue

    xywhr = result.obb.xywhr
    xyxyxyxy = result.obb.xyxyxyxy
    names = [result.names[int(cls)] for cls in result.obb.cls]
    confs = result.obb.conf
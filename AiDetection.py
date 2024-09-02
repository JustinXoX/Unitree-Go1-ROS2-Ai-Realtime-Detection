import cv2
import subprocess
import time
from inference import get_model
import supervision as sv

def run_inference(image, model):
    try:
        results = model.infer(image)[0]
        return results
    except Exception as e:
        print(f"An error occurred during inference: {e}")
        return None

def main():
    # 初始化摄像头
    cap = cv2.VideoCapture(0)

    # 加载模型
    smoking_model = get_model(model_id="smoking-detection-ngim8/5")
    fight_model = get_model(model_id="violence-detection-w2xnz/4")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 吸烟检测
        smoking_results = run_inference(frame, smoking_model)
        if smoking_results:
            smoking_detections = sv.Detections.from_inference(smoking_results)
            smoking_annotator = sv.BoundingBoxAnnotator()
            frame = smoking_annotator.annotate(scene=frame, detections=smoking_detections)
            label_annotator = sv.LabelAnnotator()
            frame = label_annotator.annotate(scene=frame, detections=smoking_detections)

        # 打架检测
        fight_results = run_inference(frame, fight_model)
        if fight_results:
            fight_detections = sv.Detections.from_inference(fight_results)
            fight_annotator = sv.BoundingBoxAnnotator()
            frame = fight_annotator.annotate(scene=frame, detections=fight_detections)
            label_annotator = sv.LabelAnnotator()
            frame = label_annotator.annotate(scene=frame, detections=fight_detections)

        # 显示检测结果
        cv2.imshow("Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

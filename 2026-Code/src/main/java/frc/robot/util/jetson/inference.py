import cv2
import numpy as np
import time
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# -------------------------------
# Config
# -------------------------------
ENGINE_PATH = "fuel_v1.trt"
CONF_THRESHOLD = 0.75

MODEL_INPUT_SIZE = (640, 640)  # width, height


# -------------------------------
# TensorRT engine loader
# -------------------------------
class TRTModel:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # Allocate buffers
        self.inputs, self.outputs, self.bindings = [], [], []
        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding)) * self.engine.max_batch_size
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            dev_mem = cuda.mem_alloc(host_mem.nbytes)
            if self.engine.binding_is_input(binding):
                self.inputs.append({'host': host_mem, 'device': dev_mem})
            else:
                self.outputs.append({'host': host_mem, 'device': dev_mem})
            self.bindings.append(int(dev_mem))
        self.stream = cuda.Stream()

    def infer(self, input_image):
        # Copy input image to device
        np.copyto(self.inputs[0]['host'], input_image.ravel())
        cuda.memcpy_htod_async(self.inputs[0]['device'], self.inputs[0]['host'], self.stream)
        # Run inference
        self.context.execute_async_v2(self.bindings, self.stream.handle)
        # Copy output back
        for output in self.outputs:
            cuda.memcpy_dtoh_async(output['host'], output['device'], self.stream)
        self.stream.synchronize()
        return [output['host'] for output in self.outputs]

# -------------------------------
# Letterbox (resize + padding)
# -------------------------------
def letterbox(image, new_size=MODEL_INPUT_SIZE):
    h0, w0 = image.shape[:2]
    r = min(new_size[1]/w0, new_size[0]/h0)
    new_unpad = int(w0*r), int(h0*r)
    dw, dh = new_size[1] - new_unpad[0], new_size[0] - new_unpad[1]
    dw /= 2
    dh /= 2
    img = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)
    img = cv2.copyMakeBorder(img, int(dh), int(dh), int(dw), int(dw), cv2.BORDER_CONSTANT, value=(114,114,114))
    return img, r, dw, dh

# -------------------------------
# Scale boxes back to original frame
# -------------------------------
def scale_boxes(boxes, r, dw, dh):
    boxes[:, [0,2]] = (boxes[:, [0,2]] - dw) / r
    boxes[:, [1,3]] = (boxes[:, [1,3]] - dh) / r
    return boxes

# -------------------------------
# Draw bounding boxes
# -------------------------------
def draw_detections(frame, detections, conf_threshold=CONF_THRESHOLD):
    for det in detections:
        x1, y1, x2, y2 = det[:4]
        confs = det[4:6]
        cls_id = np.argmax(confs)
        conf = confs[cls_id]
        print(conf)
        if conf > conf_threshold:  # now works correctly with 0.8
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            label = "fuel" if cls_id == 0 else "fuels"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"{label}:{conf:.2f}", (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    return frame



# -------------------------------
# Main loop
# -------------------------------
def main():
    trt_model = TRTModel(ENGINE_PATH)

    cap = cv2.VideoCapture(0)  # 0 = default webcam, can be 1,2,... for others
    if not cap.isOpened():
        print("Cannot open webcam")
        return

    prev_time = time.time()
    fps = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h0, w0 = frame.shape[:2]  # original frame size

        # Letterbox and preprocess
        img, r, dw, dh = letterbox(frame, MODEL_INPUT_SIZE)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2,0,1))  # HWC -> CHW
        img = np.expand_dims(img, axis=0)

        # Inference
        outputs = trt_model.infer(img)
        detections = outputs[0].reshape(-1,6)

        # Scale boxes to original frame
        detections = scale_boxes(detections, r, dw, dh)

        # Draw boxes
        frame = draw_detections(frame, detections, conf_threshold=CONF_THRESHOLD)

        # FPS
        curr_time = time.time()
        fps = 0.9*fps + 0.1*(1/(curr_time - prev_time))
        prev_time = curr_time
        cv2.putText(frame, f"FPS: {fps:.1f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        cv2.imshow("TRT Webcam Inference", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


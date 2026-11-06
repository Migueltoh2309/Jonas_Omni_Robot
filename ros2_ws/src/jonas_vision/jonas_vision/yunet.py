"""Detector de caras YuNet (opencv_zoo, modelo 2023mar) usando solo cv2.dnn.

Se decodifica la salida a mano porque cv2.FaceDetectorYN de OpenCV 4.6 (el de Ubuntu 24.04)
no es compatible con el modelo 2023mar, y el modelo 2022mar ya no se distribuye.
La decodificación es la misma que hace OpenCV >= 4.8 en modules/objdetect/src/face_detect.cpp.
"""
import cv2
import numpy as np

STRIDES = (8, 16, 32)


class YuNet:
    def __init__(self, model_path, input_size=(320, 240), score_threshold=0.7, nms_threshold=0.3, top_k=50):
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.score_threshold = score_threshold
        self.nms_threshold = nms_threshold
        self.top_k = top_k
        self.set_input_size(input_size)

    def set_input_size(self, size):
        w, h = size
        self.w, self.h = w, h
        # La red necesita dimensiones múltiplos de 32: se rellena abajo / a la derecha
        self.pw = (w + 31) // 32 * 32
        self.ph = (h + 31) // 32 * 32
        self.names = ["cls_8", "cls_16", "cls_32", "obj_8", "obj_16", "obj_32", "bbox_8", "bbox_16", "bbox_32"]
        self.grid = {}
        for s in STRIDES:
            cols, rows = self.pw // s, self.ph // s
            r, c = np.divmod(np.arange(rows * cols), cols)
            self.grid[s] = (c.astype(np.float32), r.astype(np.float32))

    def detect(self, bgr):
        """bgr: imagen de tamaño input_size. Devuelve lista de (x, y, w, h, score) en píxeles."""
        if bgr.shape[1] != self.w or bgr.shape[0] != self.h:
            bgr = cv2.resize(bgr, (self.w, self.h))
        if (self.pw, self.ph) != (self.w, self.h):
            bgr = cv2.copyMakeBorder(bgr, 0, self.ph - self.h, 0, self.pw - self.w, cv2.BORDER_CONSTANT)
        self.net.setInput(cv2.dnn.blobFromImage(bgr))
        out = dict(zip(self.names, self.net.forward(self.names)))

        boxes, scores = [], []
        for s in STRIDES:
            cls = np.clip(out[f"cls_{s}"].reshape(-1), 0, 1)
            obj = np.clip(out[f"obj_{s}"].reshape(-1), 0, 1)
            score = np.sqrt(cls * obj)
            keep = score >= self.score_threshold
            if not keep.any():
                continue
            b = out[f"bbox_{s}"].reshape(-1, 4)[keep]
            c, r = self.grid[s][0][keep], self.grid[s][1][keep]
            cx = (c + b[:, 0]) * s
            cy = (r + b[:, 1]) * s
            w = np.exp(b[:, 2]) * s
            h = np.exp(b[:, 3]) * s
            boxes += np.stack([cx - w / 2, cy - h / 2, w, h], 1).tolist()
            scores += score[keep].tolist()
        if not boxes:
            return []
        idx = cv2.dnn.NMSBoxes(boxes, scores, self.score_threshold, self.nms_threshold, top_k=self.top_k)
        return [(*boxes[i], scores[i]) for i in np.array(idx).reshape(-1)]

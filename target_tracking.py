import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional
import logging

logger = logging.getLogger(__name__)

@dataclass
class Target:
    id: int
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2
    confidence: float
    class_id: int
    last_seen: float
    track_history: List[Tuple[float, float]]

class KalmanTracker:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(8, 4)  # 8 состояний (x, y, w, h, vx, vy, vw, vh), 4 измерения (x, y, w, h)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                                                [0, 1, 0, 0, 0, 0, 0, 0],
                                                [0, 0, 1, 0, 0, 0, 0, 0],
                                                [0, 0, 0, 1, 0, 0, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 0, 0, 1, 0, 0, 0],
                                               [0, 1, 0, 0, 0, 1, 0, 0],
                                               [0, 0, 1, 0, 0, 0, 1, 0],
                                               [0, 0, 0, 1, 0, 0, 0, 1],
                                               [0, 0, 0, 0, 1, 0, 0, 0],
                                               [0, 0, 0, 0, 0, 1, 0, 0],
                                               [0, 0, 0, 0, 0, 0, 1, 0],
                                               [0, 0, 0, 0, 0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(8, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * 0.1

class TargetTracker:
    def __init__(self, max_age: float = 1.0, min_hits: int = 3, iou_threshold: float = 0.3):
        self.trackers: List[KalmanTracker] = []
        self.targets: List[Target] = []
        self.next_id = 0
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.frame_count = 0
        
    def _calculate_iou(self, bbox1: Tuple[float, float, float, float], 
                      bbox2: Tuple[float, float, float, float]) -> float:
        """Вычисляет IoU между двумя ограничивающими рамками"""
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])
        
        if x2 < x1 or y2 < y1:
            return 0.0
            
        intersection = (x2 - x1) * (y2 - y1)
        area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        
        return intersection / (area1 + area2 - intersection)
        
    def _predict(self):
        """Предсказывает новые позиции для всех трекеров"""
        for tracker in self.trackers:
            prediction = tracker.kalman.predict()
            return prediction
            
    def _update(self, tracker: KalmanTracker, measurement: np.ndarray):
        """Обновляет трекер с новым измерением"""
        tracker.kalman.correct(measurement)
        
    def update(self, detections: List[Tuple[Tuple[float, float, float, float], float, int]], 
               timestamp: float) -> List[Target]:
        """Обновляет трекеры с новыми обнаружениями"""
        self.frame_count += 1
        
        # Предсказание новых позиций
        predictions = [self._predict() for _ in self.trackers]
        
        # Сопоставление обнаружений с существующими трекерами
        matched_indices = []
        unmatched_detections = list(range(len(detections)))
        
        for i, tracker in enumerate(self.trackers):
            if len(detections) == 0:
                break
                
            # Находим лучшее соответствие
            best_iou = self.iou_threshold
            best_j = -1
            
            for j in unmatched_detections:
                iou = self._calculate_iou(predictions[i][:4], detections[j][0])
                if iou > best_iou:
                    best_iou = iou
                    best_j = j
                    
            if best_j != -1:
                matched_indices.append((i, best_j))
                unmatched_detections.remove(best_j)
                
        # Обновляем существующие трекеры
        for i, j in matched_indices:
            bbox, conf, cls = detections[j]
            measurement = np.array([[bbox[0]], [bbox[1]], [bbox[2] - bbox[0]], [bbox[3] - bbox[1]]], np.float32)
            self._update(self.trackers[i], measurement)
            
            # Обновляем информацию о цели
            self.targets[i].bbox = bbox
            self.targets[i].confidence = conf
            self.targets[i].class_id = cls
            self.targets[i].last_seen = timestamp
            self.targets[i].track_history.append((bbox[0] + (bbox[2] - bbox[0])/2, 
                                                bbox[1] + (bbox[3] - bbox[1])/2))
            
        # Создаем новые трекеры для несоответствующих обнаружений
        for j in unmatched_detections:
            bbox, conf, cls = detections[j]
            tracker = KalmanTracker()
            measurement = np.array([[bbox[0]], [bbox[1]], [bbox[2] - bbox[0]], [bbox[3] - bbox[1]]], np.float32)
            tracker.kalman.statePre = measurement
            tracker.kalman.statePost = measurement
            
            self.trackers.append(tracker)
            self.targets.append(Target(
                id=self.next_id,
                bbox=bbox,
                confidence=conf,
                class_id=cls,
                last_seen=timestamp,
                track_history=[(bbox[0] + (bbox[2] - bbox[0])/2, bbox[1] + (bbox[3] - bbox[1])/2)]
            ))
            self.next_id += 1
            
        # Удаляем устаревшие трекеры
        current_time = timestamp
        active_targets = []
        active_trackers = []
        
        for i, target in enumerate(self.targets):
            if current_time - target.last_seen <= self.max_age:
                active_targets.append(target)
                active_trackers.append(self.trackers[i])
                
        self.targets = active_targets
        self.trackers = active_trackers
        
        return self.targets 
import numpy as np
import logging
from dataclasses import dataclass
from typing import List, Tuple, Optional
import json
import os

logger = logging.getLogger(__name__)

@dataclass
class CalibrationPoint:
    distance: float  # расстояние в метрах
    wind_speed: float  # скорость ветра в м/с
    wind_direction: float  # направление ветра в градусах
    temperature: float  # температура в градусах Цельсия
    humidity: float  # влажность в процентах
    elevation: float  # высота над уровнем моря в метрах
    offset_x: float  # смещение по X в пикселях
    offset_y: float  # смещение по Y в пикселях

class WeaponCalibrator:
    def __init__(self, calibration_file: str = "weapon_calibration.json"):
        self.calibration_file = calibration_file
        self.calibration_points: List[CalibrationPoint] = []
        self.load_calibration()
        
    def load_calibration(self):
        """Загружает калибровочные данные из файла"""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    data = json.load(f)
                    self.calibration_points = [
                        CalibrationPoint(**point) for point in data
                    ]
                logger.info(f"Загружено {len(self.calibration_points)} калибровочных точек")
            except Exception as e:
                logger.error(f"Ошибка при загрузке калибровки: {e}")
                
    def save_calibration(self):
        """Сохраняет калибровочные данные в файл"""
        try:
            data = [
                {
                    "distance": point.distance,
                    "wind_speed": point.wind_speed,
                    "wind_direction": point.wind_direction,
                    "temperature": point.temperature,
                    "humidity": point.humidity,
                    "elevation": point.elevation,
                    "offset_x": point.offset_x,
                    "offset_y": point.offset_y
                }
                for point in self.calibration_points
            ]
            with open(self.calibration_file, 'w') as f:
                json.dump(data, f, indent=4)
            logger.info("Калибровочные данные сохранены")
        except Exception as e:
            logger.error(f"Ошибка при сохранении калибровки: {e}")
            
    def add_calibration_point(self, point: CalibrationPoint):
        """Добавляет новую калибровочную точку"""
        self.calibration_points.append(point)
        self.save_calibration()
        
    def calculate_ballistic_correction(self, 
                                     distance: float,
                                     wind_speed: float,
                                     wind_direction: float,
                                     temperature: float,
                                     humidity: float,
                                     elevation: float) -> Tuple[float, float]:
        """Рассчитывает баллистическую поправку с учетом внешних условий"""
        if not self.calibration_points:
            logger.warning("Нет калибровочных данных")
            return 0.0, 0.0
            
        # Находим ближайшие калибровочные точки
        distances = [abs(point.distance - distance) for point in self.calibration_points]
        closest_indices = np.argsort(distances)[:3]
        
        # Интерполируем поправки
        weights = 1 / (np.array(distances)[closest_indices] + 1e-6)
        weights = weights / np.sum(weights)
        
        offset_x = sum(self.calibration_points[i].offset_x * weights[i] for i in closest_indices)
        offset_y = sum(self.calibration_points[i].offset_y * weights[i] for i in closest_indices)
        
        # Корректируем на основе внешних условий
        wind_factor = wind_speed * np.cos(np.radians(wind_direction))
        temp_factor = (temperature - 20) * 0.01  # 1% на градус от стандартной температуры
        humidity_factor = (humidity - 50) * 0.005  # 0.5% на процент влажности от стандартной
        elevation_factor = elevation * 0.0001  # 0.01% на метр высоты
        
        correction_factor = 1 + wind_factor + temp_factor + humidity_factor + elevation_factor
        
        return offset_x * correction_factor, offset_y * correction_factor
        
    def calibrate_weapon(self, 
                        target_distance: float,
                        wind_speed: float,
                        wind_direction: float,
                        temperature: float,
                        humidity: float,
                        elevation: float,
                        actual_offset_x: float,
                        actual_offset_y: float):
        """Калибрует оружие на основе реальных измерений"""
        point = CalibrationPoint(
            distance=target_distance,
            wind_speed=wind_speed,
            wind_direction=wind_direction,
            temperature=temperature,
            humidity=humidity,
            elevation=elevation,
            offset_x=actual_offset_x,
            offset_y=actual_offset_y
        )
        self.add_calibration_point(point)
        logger.info("Новая калибровочная точка добавлена") 
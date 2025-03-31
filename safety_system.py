import logging
import time
import threading
import queue
from dataclasses import dataclass
from typing import Dict, List, Optional
import json
import os
from enum import Enum

logger = logging.getLogger(__name__)

class SafetyLevel(Enum):
    NORMAL = "normal"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"

@dataclass
class SystemStatus:
    level: SafetyLevel
    message: str
    timestamp: float
    details: Dict

class SafetySystem:
    def __init__(self):
        self.safety_checks = {
            "battery": {
                "level": SafetyLevel.NORMAL,
                "last_check": 0.0,
                "check_interval": 1.0,
                "min_voltage": 11.0,  # В
                "max_voltage": 12.6,  # В
                "current_draw": 0.0,  # А
                "temperature": 0.0,   # °C
                "is_charging": False
            },
            "communication": {
                "level": SafetyLevel.NORMAL,
                "last_check": 0.0,
                "check_interval": 0.5,
                "signal_strength": 0.0,
                "latency": 0.0,
                "packet_loss": 0.0,
                "last_heartbeat": 0.0,
                "connection_status": True
            },
            "weapon": {
                "level": SafetyLevel.NORMAL,
                "last_check": 0.0,
                "check_interval": 0.2,
                "safety_engaged": True,
                "armed": False,
                "ammo_count": 0,
                "malfunction": False,
                "temperature": 0.0,
                "vibration": 0.0
            },
            "motor": {
                "level": SafetyLevel.NORMAL,
                "last_check": 0.0,
                "check_interval": 0.1,
                "current_draw": 0.0,
                "temperature": 0.0,
                "vibration": 0.0,
                "speed": 0.0,
                "acceleration": 0.0
            },
            "camera": {
                "level": SafetyLevel.NORMAL,
                "last_check": 0.0,
                "check_interval": 0.5,
                "fps": 0.0,
                "resolution": (0, 0),
                "exposure": 0.0,
                "gain": 0.0,
                "is_obstructed": False
            },
            "environment": {
                "level": SafetyLevel.NORMAL,
                "last_check": 0.0,
                "check_interval": 1.0,
                "temperature": 0.0,
                "humidity": 0.0,
                "pressure": 0.0,
                "wind_speed": 0.0,
                "visibility": 0.0
            }
        }
        
        self.emergency_stop = False
        self.safety_status = SafetyLevel.NORMAL
        self.status_history: List[SystemStatus] = []
        self.max_history_size = 1000
        
        # Очереди для асинхронных проверок
        self.check_queue = queue.Queue()
        self.alert_queue = queue.Queue()
        
        # Запуск мониторинга
        self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        
    def _monitoring_loop(self):
        """Основной цикл мониторинга безопасности"""
        while not self.emergency_stop:
            current_time = time.time()
            
            # Проверяем все системы
            for system, params in self.safety_checks.items():
                if current_time - params["last_check"] >= params["check_interval"]:
                    self._check_system(system)
                    
            # Обрабатываем предупреждения
            while not self.alert_queue.empty():
                alert = self.alert_queue.get()
                self._handle_alert(alert)
                
            time.sleep(0.1)  # Предотвращаем перегрузку CPU
            
    def _check_system(self, system: str):
        """Проверяет конкретную систему"""
        current_time = time.time()
        params = self.safety_checks[system]
        params["last_check"] = current_time
        
        try:
            if system == "battery":
                self._check_battery()
            elif system == "communication":
                self._check_communication()
            elif system == "weapon":
                self._check_weapon()
            elif system == "motor":
                self._check_motor()
            elif system == "camera":
                self._check_camera()
            elif system == "environment":
                self._check_environment()
        except Exception as e:
            logger.error(f"Ошибка при проверке системы {system}: {e}")
            self._add_alert(SafetyLevel.CRITICAL, f"Ошибка проверки {system}", {"error": str(e)})
            
    def _check_battery(self):
        """Проверка системы питания"""
        params = self.safety_checks["battery"]
        
        # Здесь должны быть реальные измерения
        voltage = 12.0  # Пример
        current = 2.0   # Пример
        temp = 35.0     # Пример
        
        params["current_draw"] = current
        params["temperature"] = temp
        
        if voltage < params["min_voltage"]:
            self._add_alert(SafetyLevel.CRITICAL, "Критически низкий заряд батареи", 
                          {"voltage": voltage, "min_voltage": params["min_voltage"]})
        elif voltage < params["min_voltage"] * 1.1:
            self._add_alert(SafetyLevel.WARNING, "Низкий заряд батареи", 
                          {"voltage": voltage, "min_voltage": params["min_voltage"]})
            
        if temp > 60.0:
            self._add_alert(SafetyLevel.CRITICAL, "Перегрев батареи", 
                          {"temperature": temp})
            
    def _check_communication(self):
        """Проверка системы связи"""
        params = self.safety_checks["communication"]
        current_time = time.time()
        
        # Проверка последнего heartbeat
        if current_time - params["last_heartbeat"] > 2.0:
            self._add_alert(SafetyLevel.CRITICAL, "Потеря связи с базой", 
                          {"time_since_last_heartbeat": current_time - params["last_heartbeat"]})
            
        # Проверка качества связи
        if params["packet_loss"] > 0.1:  # 10% потери пакетов
            self._add_alert(SafetyLevel.WARNING, "Высокая потеря пакетов", 
                          {"packet_loss": params["packet_loss"]})
            
    def _check_weapon(self):
        """Проверка оружия"""
        params = self.safety_checks["weapon"]
        
        # Проверка температуры оружия
        if params["temperature"] > 80.0:
            self._add_alert(SafetyLevel.CRITICAL, "Перегрев оружия", 
                          {"temperature": params["temperature"]})
            
        # Проверка вибрации
        if params["vibration"] > 5.0:  # мм/с
            self._add_alert(SafetyLevel.WARNING, "Высокая вибрация оружия", 
                          {"vibration": params["vibration"]})
            
        # Проверка неисправностей
        if params["malfunction"]:
            self._add_alert(SafetyLevel.CRITICAL, "Обнаружена неисправность оружия", 
                          {"malfunction": True})
            
    def _check_motor(self):
        """Проверка двигателей"""
        params = self.safety_checks["motor"]
        
        # Проверка температуры
        if params["temperature"] > 70.0:
            self._add_alert(SafetyLevel.CRITICAL, "Перегрев двигателей", 
                          {"temperature": params["temperature"]})
            
        # Проверка вибрации
        if params["vibration"] > 3.0:  # мм/с
            self._add_alert(SafetyLevel.WARNING, "Высокая вибрация двигателей", 
                          {"vibration": params["vibration"]})
            
        # Проверка ускорения
        if abs(params["acceleration"]) > 5.0:  # м/с²
            self._add_alert(SafetyLevel.WARNING, "Высокое ускорение", 
                          {"acceleration": params["acceleration"]})
            
    def _check_camera(self):
        """Проверка камеры"""
        params = self.safety_checks["camera"]
        
        # Проверка FPS
        if params["fps"] < 15.0:
            self._add_alert(SafetyLevel.WARNING, "Низкий FPS камеры", 
                          {"fps": params["fps"]})
            
        # Проверка засорения
        if params["is_obstructed"]:
            self._add_alert(SafetyLevel.CRITICAL, "Камера засорена", 
                          {"is_obstructed": True})
            
    def _check_environment(self):
        """Проверка окружающей среды"""
        params = self.safety_checks["environment"]
        
        # Проверка видимости
        if params["visibility"] < 10.0:  # метров
            self._add_alert(SafetyLevel.WARNING, "Низкая видимость", 
                          {"visibility": params["visibility"]})
            
        # Проверка ветра
        if params["wind_speed"] > 15.0:  # м/с
            self._add_alert(SafetyLevel.WARNING, "Сильный ветер", 
                          {"wind_speed": params["wind_speed"]})
            
    def _add_alert(self, level: SafetyLevel, message: str, details: Dict):
        """Добавляет предупреждение в очередь"""
        alert = SystemStatus(
            level=level,
            message=message,
            timestamp=time.time(),
            details=details
        )
        self.alert_queue.put(alert)
        
    def _handle_alert(self, alert: SystemStatus):
        """Обрабатывает предупреждение"""
        # Обновляем историю
        self.status_history.append(alert)
        if len(self.status_history) > self.max_history_size:
            self.status_history.pop(0)
            
        # Обновляем общий статус безопасности
        if alert.level.value > self.safety_status.value:
            self.safety_status = alert.level
            
        # Логируем предупреждение
        if alert.level == SafetyLevel.EMERGENCY:
            logger.critical(f"EMERGENCY: {alert.message}")
        elif alert.level == SafetyLevel.CRITICAL:
            logger.error(f"CRITICAL: {alert.message}")
        elif alert.level == SafetyLevel.WARNING:
            logger.warning(f"WARNING: {alert.message}")
        else:
            logger.info(f"INFO: {alert.message}")
            
        # Проверяем необходимость аварийной остановки
        if alert.level == SafetyLevel.EMERGENCY:
            self.emergency_stop = True
            
    def get_safety_status(self) -> Dict:
        """Возвращает текущий статус безопасности"""
        return {
            "emergency_stop": self.emergency_stop,
            "safety_level": self.safety_status.value,
            "systems": {
                system: {
                    "level": params["level"].value,
                    "last_check": params["last_check"]
                }
                for system, params in self.safety_checks.items()
            }
        }
        
    def get_status_history(self) -> List[Dict]:
        """Возвращает историю статусов"""
        return [
            {
                "level": status.level.value,
                "message": status.message,
                "timestamp": status.timestamp,
                "details": status.details
            }
            for status in self.status_history
        ]
        
    def update_system_parameters(self, system: str, parameters: Dict):
        """Обновляет параметры системы"""
        if system in self.safety_checks:
            self.safety_checks[system].update(parameters)
            
    def cleanup(self):
        """Очищает ресурсы"""
        self.emergency_stop = True
        if self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=1.0) 
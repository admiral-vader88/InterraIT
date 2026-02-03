# basic code for a smart sensor which includes temperature + calibration
from dataclasses import dataclass

# Struct-like container (like C++ struct)
@dataclass
class SensorPacket:
    sensor_id: str
    value: float
    unit: str


# Base class (Inheritance + Polymorphism)
class Sensor:
    def __init__(self, sensor_id):
        self.sensor_id = sensor_id   # public
        self.__is_active = True      # private-like

    def get_status(self):
        return self.__is_active

    def set_status(self, status: bool):
        self.__is_active = status

    # Polymorphic method (to be overridden)
    def read_value(self) -> SensorPacket:
        raise NotImplementedError("Derived class must implement read_value()")


# Derived class 1 (IoT sensor)
class TemperatureSensor(Sensor):
    def __init__(self, sensor_id, temp_c):
        super().__init__(sensor_id)   # calling base constructor
        self.__temp_c = temp_c        # private-like
        self.__calibration = 0.0      # private-like

    def set_temperature(self, temp):
        if -50 <= temp <= 150:
            self.__temp_c = temp
        else:
            raise ValueError("Temperature out of range!")

    def calibrate(self, offset):
        if -10 <= offset <= 10:
            self.__calibration = offset
        else:
            raise ValueError("Calibration offset too large!")

    # Polymorphism: overriding base method
    def read_value(self) -> SensorPacket:
        value = self.__temp_c + self.__calibration
        return SensorPacket(self.sensor_id, value, "C")


# Derived class 2 (ROS simulation / robot battery)
class BatterySensor(Sensor):
    def __init__(self, sensor_id, battery_percent):
        super().__init__(sensor_id)
        self.__battery = battery_percent

    def drain(self, amount):
        self.__battery = max(0, self.__battery - amount)

    # Polymorphism: overriding base method
    def read_value(self) -> SensorPacket:
        return SensorPacket(self.sensor_id, self.__battery, "%")

sensors = [
    TemperatureSensor("DHT11_01", 28.5),
    BatterySensor("BAT_01", 85)
]

# Polymorphism 
for s in sensors:
    packet = s.read_value()
    print(packet)

# Encapsulation 
temp_sensor = sensors[0]
temp_sensor.calibrate(1.5)
print(temp_sensor.read_value())

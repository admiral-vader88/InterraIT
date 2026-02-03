# basic code for a smart sensor which includes temperature + calibration
class SmartSensor:
    def __init__(self, sensor_id, temp_c):
        self.sensor_id = sensor_id          # public
        self.__temp_c = temp_c              # private-like
        self.__calibration_offset = 0.0     # private-like

    def get_temperature(self):
        return self.__temp_c + self.__calibration_offset

    def set_temperature(self, new_temp):
        if -50 <= new_temp <= 150:
            self.__temp_c = new_temp
        else:
            raise ValueError("Temperature out of sensor range!")

    def calibrate(self, offset):
        if -10 <= offset <= 10:
            self.__calibration_offset = offset
        else:
            raise ValueError("Calibration offset too large!")


# Example
s1 = SmartSensor("DHT11_01", 28.5)
print("Sensor:", s1.sensor_id)
print("Temp:", s1.get_temperature())

s1.calibrate(+1.2)
print("Temp after calibration:", s1.get_temperature())

s1.set_temperature(31.0)
print("Updated Temp:", s1.get_temperature())


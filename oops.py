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

# basic oops for robot
class RobotArm:
    def __init__(self, robot_name, joint_angle):
        self.robot_name = robot_name        # public
        self.__joint_angle = joint_angle    # private-like

    def get_joint_angle(self):
        return self.__joint_angle

    def set_joint_angle(self, angle):
        if 0 <= angle <= 180:
            self.__joint_angle = angle
        else:
            raise ValueError("Joint angle must be between 0 and 180 degrees!")

    def move_to(self, angle):
        self.set_joint_angle(angle)
        print(f"{self.robot_name} arm moved to {self.__joint_angle} degrees")


# Example
arm = RobotArm("ThrowerBot", 45)
print(arm.robot_name)
print("Initial Angle:", arm.get_joint_angle())

arm.move_to(90)
print("Final Angle:", arm.get_joint_angle())


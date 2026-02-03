// basic code for smart sensor ( temperature + calibration )
#include <iostream>
#include <string>
using namespace std;

class SmartSensor {
public:
    string sensor_id;   // public

    SmartSensor(string id, float temp_c) {
        sensor_id = id;
        tempC = temp_c;
        calibrationOffset = 0.0;
    }

    float getTemperature() {
        return tempC + calibrationOffset;
    }

    void setTemperature(float newTemp) {
        if (newTemp >= -50 && newTemp <= 150)
            tempC = newTemp;
        else
            throw invalid_argument("Temperature out of sensor range!");
    }

    void calibrate(float offset) {
        if (offset >= -10 && offset <= 10)
            calibrationOffset = offset;
        else
            throw invalid_argument("Calibration offset too large!");
    }

private:
    float tempC;               // private
    float calibrationOffset;   // private
};

int main() {
    SmartSensor s1("DHT11_01", 28.5);

    cout << "Sensor: " << s1.sensor_id << endl;
    cout << "Temp: " << s1.getTemperature() << endl;

    s1.calibrate(1.2);
    cout << "Temp after calibration: " << s1.getTemperature() << endl;

    s1.setTemperature(31.0);
    cout << "Updated Temp: " << s1.getTemperature() << endl;

    return 0;
}

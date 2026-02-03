// basic code for smart sensor ( temperature + calibration )
#include <iostream>
#include <string>
#include <vector>
using namespace std;

// Struct (Data Packet like ROS message)
struct SensorPacket {
    string sensor_id;
    float value;
    string unit;
};

// Base class (Inheritance + Polymorphism)
class Sensor {
public:
    string sensor_id;   // public

    // Constructor
    Sensor(string id) {
        sensor_id = id;
        isActive = true;
    }

    bool getStatus() {
        return isActive;
    }

    void setStatus(bool status) {
        isActive = status;
    }

    // Polymorphic method
    virtual SensorPacket readValue() = 0;

    // Virtual destructor (important for polymorphism)
    virtual ~Sensor() {}

private:
    bool isActive; // encapsulated private member
};

// Derived class 1: Temperature Sensor (IoT)
class TemperatureSensor : public Sensor {
public:
    TemperatureSensor(string id, float temp_c) : Sensor(id) {
        tempC = temp_c;
        calibration = 0.0;
    }

    void setTemperature(float temp) {
        if (temp >= -50 && temp <= 150)
            tempC = temp;
        else
            throw invalid_argument("Temperature out of range!");
    }

    void calibrate(float offset) {
        if (offset >= -10 && offset <= 10)
            calibration = offset;
        else
            throw invalid_argument("Calibration offset too large!");
    }

    // Polymorphism
    SensorPacket readValue() override {
        SensorPacket packet;
        packet.sensor_id = sensor_id;
        packet.value = tempC + calibration;
        packet.unit = "C";
        return packet;
    }

private:
    float tempC;
    float calibration;
};

// Derived class 2: Battery Sensor (Robotics / ROS Simulation)
class BatterySensor : public Sensor {
public:
    BatterySensor(string id, float battery_percent) : Sensor(id) {
        battery = battery_percent;
    }

    void drain(float amount) {
        battery -= amount;
        if (battery < 0) battery = 0;
    }

    // Polymorphism
    SensorPacket readValue() override {
        SensorPacket packet;
        packet.sensor_id = sensor_id;
        packet.value = battery;
        packet.unit = "%";
        return packet;
    }

private:
    float battery;
};

int main() {
    vector<Sensor*> sensors;

    sensors.push_back(new TemperatureSensor("DHT11_01", 28.5));
    sensors.push_back(new BatterySensor("BAT_01", 85));

    // Polymorphism
    for (auto s : sensors) {
        SensorPacket p = s->readValue();
        cout << "SensorPacket { id=" << p.sensor_id
             << ", value=" << p.value
             << ", unit=" << p.unit << " }" << endl;
    }

    for (auto s : sensors) delete s;

    return 0;
}



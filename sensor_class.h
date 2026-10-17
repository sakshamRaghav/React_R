
#ifndef SENSOR_CLASS_H
#define SENSOR_CLASS_H

#include <unordered_map> 
#include <string>
#include <iostream>

using namespace std;

enum class SensorDirection {
    FRONT,
    REAR,
    LEFT,
    RIGHT
};

inline string directionToString(SensorDirection dir) {
    switch (dir) {
        case SensorDirection::FRONT: return "FRONT";
        case SensorDirection::REAR:  return "REAR";
        case SensorDirection::LEFT:  return "LEFT";
        case SensorDirection::RIGHT: return "RIGHT";
        default:                     return "UNKNOWN";
    }
}

enum class SensorStatus {
    OK,            
    STUCK_VALUE,   
    OUT_OF_RANGE,  
    DISCONNECTED   
};

inline string statusToString(SensorStatus status) {
    switch (status) {
        case SensorStatus::OK:           return "OK";
        case SensorStatus::STUCK_VALUE:  return "STUCK_VALUE";
        case SensorStatus::OUT_OF_RANGE: return "OUT_OF_RANGE";
        case SensorStatus::DISCONNECTED: return "DISCONNECTED";
        default:                         return "UNKNOWN";
    }
}

struct Sensor {
    int             id;         
    SensorDirection direction; 
    double          distance;   
    double          min_dist;   
    double          max_dist;   
    SensorStatus    status;     
    long long       last_tick;  

    Sensor()
        : id(0),
          direction(SensorDirection::FRONT),
          distance(-1.0),       
          min_dist(0.0),
          max_dist(5.0),
          status(SensorStatus::DISCONNECTED),
          last_tick(0)
    {}

    Sensor(int id, SensorDirection dir, double dist,
           double min_d, double max_d, long long tick)
        : id(id),
          direction(dir),
          distance(dist),
          min_dist(min_d),
          max_dist(max_d),
          status(SensorStatus::OK),  
          last_tick(tick)
    {}
};

class SensorMonitor {
public:

    void registerSensor(int id, SensorDirection dir,
                        double min_dist, double max_dist) {
        Sensor s;
        s.id        = id;
        s.direction = dir;
        s.min_dist  = min_dist;
        s.max_dist  = max_dist;
        s.distance  = -1.0;                   
        s.status    = SensorStatus::DISCONNECTED;
        s.last_tick = 0;
        sensors_[id] = s;
    }

    void updateReading(int id, double new_distance, long long current_tick,
                       int stuck_threshold = 5) {

        auto it = sensors_.find(id);
        if (it == sensors_.end()) {
            cerr << "Warning: sensor ID " << id << " was never registered.\n";
            return;
        }

        Sensor& s = it->second;

        // -1= disconnected
        if (new_distance < 0.0) {
            s.distance  = -1.0;
            s.status    = SensorStatus::DISCONNECTED;
            s.last_tick = current_tick;
            return;
        }

        // same value for too many ticks=frozzen
        bool value_unchanged        = (new_distance == s.distance);
        long long ticks_since_change = current_tick - s.last_tick;

        if (value_unchanged && ticks_since_change >= stuck_threshold) {
            s.status = SensorStatus::STUCK_VALUE;
            return;
        }

        s.distance  = new_distance;
        s.last_tick = current_tick;

        if (new_distance < s.min_dist || new_distance > s.max_dist) {
            s.status = SensorStatus::OUT_OF_RANGE;
        } else {
            s.status = SensorStatus::OK;
        }
    }

    const Sensor* getSensor(int id) const {
        auto it = sensors_.find(id);
        if (it == sensors_.end()) return nullptr;
        return &it->second;
    }

    const unordered_map<int, Sensor>& getAllSensors() const {
        return sensors_;
    }

    void printStatus() const {
        cout << "\n--- Sensor Readings ---\n";
        for (const auto& pair : sensors_) {
            const Sensor& s = pair.second;
            cout << "  Sensor #" << s.id
                 << " | Side: "   << directionToString(s.direction)
                 << " | Dist: "   << s.distance << " m"
                 << " | Health: " << statusToString(s.status)
                 << "\n";
        }
        cout << "-----------------------\n";
    }

private:
    unordered_map<int, Sensor> sensors_;
};

#endif

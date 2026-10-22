// sensor_class.h
// 2nd-year CS implementation for storing sensor distance readings
// Uses a simple fixed array to build a circular queue buffer for noise filtering.

#ifndef SENSOR_CLASS_H
#define SENSOR_CLASS_H

#include <unordered_map>
#include <string>
#include <iostream>
#include <cmath>

using namespace std;

// Direction enum to track which side the sensor is facing
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

// Health status enum for sensor diagnostics
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

// -----------------------------------------------------------------------
// Simple Fixed-Size Circular Queue Buffer
// Built with a fixed array so we don't need dynamic memory allocation.
// -----------------------------------------------------------------------
class SensorQueue {
public:
    static const int CAPACITY = 5; // Size of sliding window

    SensorQueue() {
        head = 0;
        tail = 0;
        count = 0;
    }

    // Add a new reading; overwrite oldest reading if buffer is full
    void push(double val) {
        data[tail] = val;
        tail = (tail + 1) % CAPACITY; // Circular wrap using modulo

        if (count < CAPACITY) {
            count++;
        } else {
            head = (head + 1) % CAPACITY; // Advance head to drop oldest value
        }
    }

    // Calculate moving average of all samples in buffer
    double getAverage() const {
        if (count == 0) return -1.0;
        
        double sum = 0.0;
        for (int i = 0; i < count; i++) {
            int index = (head + i) % CAPACITY;
            sum += data[index];
        }
        return sum / count;
    }

    void reset() {
        head = 0;
        tail = 0;
        count = 0;
    }

    int getCount() const { return count; }
    int getCapacity() const { return CAPACITY; }

private:
    double data[CAPACITY];
    int head;
    int tail;
    int count;
};

// -----------------------------------------------------------------------
// Sensor Struct
// -----------------------------------------------------------------------
struct Sensor {
    int             id;
    SensorDirection direction;
    double          distance;          // Moving average distance in meters
    double          min_dist;
    double          max_dist;
    SensorStatus    status;
    long long       last_tick;
    long long       last_change_tick;

    SensorQueue     buffer;            // Fixed circular queue instance

    Sensor() {
        id = 0;
        direction = SensorDirection::FRONT;
        distance = -1.0;
        min_dist = 0.0;
        max_dist = 5.0;
        status = SensorStatus::DISCONNECTED;
        last_tick = 0;
        last_change_tick = 0;
    }
};

// -----------------------------------------------------------------------
// SensorMonitor Class
// Stores sensors in an unordered_map for O(1) lookups by ID
// -----------------------------------------------------------------------
class SensorMonitor {
public:
    static constexpr double SAME_VALUE_TOLERANCE = 0.005;

    void registerSensor(int id, SensorDirection dir, double min_dist, double max_dist) {
        Sensor s;
        s.id               = id;
        s.direction        = dir;
        s.min_dist         = min_dist;
        s.max_dist         = max_dist;
        s.distance         = -1.0;
        s.status           = SensorStatus::DISCONNECTED;
        s.last_tick        = 0;
        s.last_change_tick = 0;
        s.buffer.reset();
        
        sensors[id] = s;
    }

    void updateReading(int id, double new_distance, long long current_tick,
                       int stuck_threshold = 5, bool robot_moving = true) {

        auto it = sensors.find(id);
        if (it == sensors.end()) {
            cerr << "Warning: sensor ID " << id << " not registered.\n";
            return;
        }

        Sensor& s = it->second;

        // Check for NaN or invalid negative values
        if (isnan(new_distance) || new_distance < 0.0) {
            s.distance         = -1.0;
            s.status           = SensorStatus::DISCONNECTED;
            s.last_tick        = current_tick;
            s.last_change_tick = current_tick;
            s.buffer.reset();
            return;
        }

        // Clamp values at maximum distance limit
        bool nothing_seen = isinf(new_distance) || new_distance >= s.max_dist;
        if (nothing_seen) {
            new_distance = s.max_dist;
        }

        // Push raw reading into queue and compute moving average
        s.buffer.push(new_distance);
        double filtered_dist = s.buffer.getAverage();

        // Check if value changed beyond tolerance
        bool unchanged = fabs(filtered_dist - s.distance) < SAME_VALUE_TOLERANCE;

        if (!unchanged || !robot_moving || nothing_seen) {
            s.last_change_tick = current_tick;
        } else if (current_tick - s.last_change_tick >= stuck_threshold) {
            s.status    = SensorStatus::STUCK_VALUE;
            s.last_tick = current_tick;
            return;
        }

        s.distance  = filtered_dist;
        s.last_tick = current_tick;

        // Check distance boundaries
        if (filtered_dist < s.min_dist) {
            s.status = SensorStatus::OUT_OF_RANGE;
        } else {
            s.status = SensorStatus::OK;
        }
    }

    void updateFromLidar(const float* ranges, int n, long long current_tick,
                         int front_index, bool index_goes_left,
                         bool robot_moving = true, int stuck_threshold = 5) {

        if (ranges == nullptr || n < 8) return;

        int quarter = n / 4;
        int half    = quarter / 2;
        int turn    = index_goes_left ? quarter : -quarter;

        for (auto& pair : sensors) {
            int center = front_index;
            if (pair.second.direction == SensorDirection::LEFT)  center = front_index + turn;
            if (pair.second.direction == SensorDirection::REAR)  center = front_index + 2 * quarter;
            if (pair.second.direction == SensorDirection::RIGHT) center = front_index - turn;

            double closest   = INFINITY;
            bool   any_valid = false;

            for (int k = -half; k < half; k++) {
                int idx = ((center + k) % n + n) % n; // Modulo index wrap
                double r = ranges[idx];
                if (isnan(r)) continue;
                any_valid = true;
                if (r < closest) closest = r;
            }

            if (any_valid) {
                updateReading(pair.first, closest, current_tick, stuck_threshold, robot_moving);
            } else {
                updateReading(pair.first, -1.0, current_tick, stuck_threshold, robot_moving);
            }
        }
    }

    const Sensor* getSensor(int id) const {
        auto it = sensors.find(id);
        if (it == sensors.end()) return nullptr;
        return &it->second;
    }

    const unordered_map<int, Sensor>& getAllSensors() const {
        return sensors;
    }

    void printStatus() const {
        cout << "\n--- Sensor Readings (Filtered) ---\n";
        for (const auto& pair : sensors) {
            const Sensor& s = pair.second;
            cout << "  Sensor #" << s.id
                 << " | Side: "   << directionToString(s.direction)
                 << " | Dist: "   << s.distance << " m"
                 << " | Queue: "  << s.buffer.getCount() << "/" << s.buffer.getCapacity()
                 << " | Health: " << statusToString(s.status)
                 << "\n";
        }
        cout << "----------------------------------\n";
    }

private:
    unordered_map<int, Sensor> sensors;
};

#endif
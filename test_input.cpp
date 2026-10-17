

#include"sensor_class.h"
#include"decision_engine.h"
#include "risk_assessment.h"
#include "robot_controller.h"
#include <iostream>

using namespace std;

int main() {

    SensorMonitor   sensors;
    RiskAssessment  risk;
    DecisionEngine  decision;
    RobotController robot;

    // register the 4 sensors once
    sensors.registerSensor(1, SensorDirection::FRONT, 0.05, 5.0);
    sensors.registerSensor(2, SensorDirection::REAR,  0.05, 5.0);
    sensors.registerSensor(3, SensorDirection::LEFT,  0.05, 5.0);
    sensors.registerSensor(4, SensorDirection::RIGHT, 0.05, 5.0);

    cout << "\n=== REACT-R manual test ===\n";
    cout << "Type the distance in metres for each sensor.\n";
    cout << "  -1       = sensor disconnected\n";
    cout << "  5 or more = nothing seen (path clear)\n";

    long long tick = 0;
    char again = 'y';

    while (again == 'y' || again == 'Y') {
        tick++;

        double front, rear, left, right;

        cout << "\nTest number " << tick << "\n";
        cout << "Front distance: ";
        cin >> front;
        cout << "Rear distance : ";
        cin >> rear;
        cout << "Left distance : ";
        cin >> left;
        cout << "Right distance: ";
        cin >> right;

        // stop if the user typed something that is not a number
        if (!cin) {
            cout << "That was not a number. Exiting.\n";
            return 1;
        }

        // robot_moving is true only if the robot was moving after the last test.
        // typing the same values again and again while it is moving
        // will make the sensor look frozen (STUCK_VALUE) after 5 tests.
        bool moving = robot.getCurrentSpeed() > 0.0;

        sensors.updateReading(1, front, tick, 5, moving);
        sensors.updateReading(2, rear,  tick, 5, moving);
        sensors.updateReading(3, left,  tick, 5, moving);
        sensors.updateReading(4, right, tick, 5, moving);

        sensors.printStatus();
        risk.printReport(sensors);

        RiskLevel level = risk.evaluate(sensors);
        decision.printDecision(level);

        robot.execute(decision.decide(level));
        robot.printState();

        cout << "\nRun another test? (y/n): ";
        cin >> again;
    }

    cout << "\nDone.\n";
    return 0;
}

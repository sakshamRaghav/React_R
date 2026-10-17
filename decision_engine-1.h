// decision_engine.h
// Takes the risk level and decides what the robot should do.
// It is basically a simple table: HIGH = STOP, MEDIUM = slow down, LOW = go.

#ifndef DECISION_ENGINE_H
#define DECISION_ENGINE_H

#include "risk_assessment.h"
#include <iostream>
#include <string>

using namespace std;

enum class RobotAction {
    PROCEED,    // move forward at normal speed
    SLOW_DOWN,  // move forward slowly
    STOP        // do not move
};

inline string actionToString(RobotAction action) {
    switch (action) {
        case RobotAction::PROCEED:   return "PROCEED";
        case RobotAction::SLOW_DOWN: return "SLOW_DOWN";
        case RobotAction::STOP:      return "STOP";
        default:                     return "UNKNOWN";
    }
}

class DecisionEngine {
public:

    // look at the risk and decide what to do
    RobotAction decide(RiskLevel risk) const {
        switch (risk) {
            case RiskLevel::LOW:    return RobotAction::PROCEED;
            case RiskLevel::MEDIUM: return RobotAction::SLOW_DOWN;
            case RiskLevel::HIGH:   return RobotAction::STOP;
            default:
                // something unexpected, safest thing is to stop
                cerr << "Unknown risk level, stopping to be safe.\n";
                return RobotAction::STOP;
        }
    }

    // print the risk and the action we chose for it
    void printDecision(RiskLevel risk) const {
        cout << "\n--- Decision ---\n";
        cout << "  Risk Level : " << riskToString(risk)             << "\n";
        cout << "  Action     : " << actionToString(decide(risk))   << "\n";
        cout << "----------------\n";
    }
};

#endif

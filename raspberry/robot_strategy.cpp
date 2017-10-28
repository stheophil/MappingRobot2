#include "robot_strategy.h"


SRobotCommand CRobotStrategy::receivedSensorData(SScanLine const& scanline) {
    CFastParticleSlamBase::receivedSensorData(scanline);
    // TODO: Calculate strategy, return robot control
    return SRobotCommand::stop();
}

void CRobotStrategy::PrintHelp() {}
void CRobotStrategy::OnChar(char ch) {}

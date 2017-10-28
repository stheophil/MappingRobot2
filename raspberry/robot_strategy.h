#include "fast_particle_slam.h"
#include "scanline.h"

struct CRobotStrategy : CFastParticleSlamBase {
    SRobotCommand receivedSensorData(SScanLine const& scanline);    
    void PrintHelp();
    void OnChar(char ch);
};
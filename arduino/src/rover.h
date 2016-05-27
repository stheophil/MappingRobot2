#ifndef _rover_h
#define _rover_h

#include <math.h>
#include <assert.h>
#include <stdlib.h>

const char g_chHandshake = 'X';

enum ECommand : short {
    ecmdRESET,
    ecmdMOVE
};

// Speeds are given in same units as SRobotCommand.m_nSpeed*
// 100 is a good value so the robot does not turn too fast
// and can be stopped at the right time
const short c_nMaxFwdSpeed = 200;

struct SRobotCommand {
    ECommand m_ecmd;
    short m_nSpeedLeft;     // max is 255, but leave some room for PID
    short m_nSpeedRight;

    inline static SRobotCommand forward() { return {ecmdMOVE, c_nMaxFwdSpeed, c_nMaxFwdSpeed}; }
    inline static SRobotCommand backward() { return {ecmdMOVE, -c_nMaxFwdSpeed, -c_nMaxFwdSpeed}; }
    inline static SRobotCommand left_turn() { return {ecmdMOVE, -c_nMaxFwdSpeed, c_nMaxFwdSpeed}; }
    inline static SRobotCommand right_turn() { return {ecmdMOVE, c_nMaxFwdSpeed, -c_nMaxFwdSpeed}; }
    
    inline static SRobotCommand reset() { return {ecmdRESET, 0, 0}; }
    inline static SRobotCommand stop() { return {ecmdMOVE, 0, 0}; }
};

static_assert(sizeof(SRobotCommand)==6, "");


struct SSensorData { // must be < 64 bytes
    unsigned short m_nYaw; // in degrees (0, 359) both inclusive
    short m_nAngle; // lidar measurement angle, in lidar frame of reference
    short m_nDistance; // lidar distance in cm, in lidar frame of reference
    short m_anEncoderTicks[4]; // front left, front right, back left, back right
};

// Robot configuration
const int c_nRobotWidth = 30; // cm
const int c_nRobotHeight = 30; // cm
const int c_nWheelRadius = 6; // cm

// Offset of robot center to lidar center as x, y coordinates
// y-axis points into direction of robot front
const int c_ptLidarOffset[] = {0, 7}; 

inline double encoderTicksToCm(short nTicks) { // Note: Formula depends on wheel encoders
    return nTicks * 6.0 * M_PI * c_nWheelRadius / 1000.0;
}

#endif

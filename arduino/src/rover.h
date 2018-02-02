#pragma once

#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

enum ECommand : short {
    ecmdCONNECT = 10,
    ecmdRESET = 11,
    ecmdMOVE = 12
};

// Speeds are given in same units as SRobotCommand.m_nSpeed*
// 100 is a good value so the robot does not turn too fast
// and can be stopped at the right time
const short c_nMaxFwdSpeed = 200;
const short c_nMaxSpeed = 500;

struct SRobotCommand {
    ECommand m_ecmd;
    short m_nSpeedLeft;     // max is 255, but leave some room for PID
    short m_nSpeedRight;

    inline static SRobotCommand forward() { return {ecmdMOVE, c_nMaxFwdSpeed, c_nMaxFwdSpeed}; }
    inline static SRobotCommand forward_left() { return {ecmdMOVE, c_nMaxFwdSpeed/2, c_nMaxFwdSpeed}; }
    inline static SRobotCommand forward_right() { return {ecmdMOVE, c_nMaxFwdSpeed, c_nMaxFwdSpeed/2}; }

    inline static SRobotCommand backward() { return {ecmdMOVE, -c_nMaxFwdSpeed, -c_nMaxFwdSpeed}; }
    inline static SRobotCommand backward_left() { return {ecmdMOVE, -c_nMaxFwdSpeed/2, -c_nMaxFwdSpeed}; }
    inline static SRobotCommand backward_right() { return {ecmdMOVE, -c_nMaxFwdSpeed, -c_nMaxFwdSpeed/2}; }

    inline static SRobotCommand left_turn() { return {ecmdMOVE, -c_nMaxFwdSpeed, c_nMaxFwdSpeed}; }
    inline static SRobotCommand right_turn() { return {ecmdMOVE, c_nMaxFwdSpeed, -c_nMaxFwdSpeed}; }
    
    inline static SRobotCommand connect() { return {ecmdCONNECT, 0, 0}; }
    inline static SRobotCommand reset() { return {ecmdRESET, 0, 0}; }
    inline static SRobotCommand stop() { return {ecmdMOVE, 0, 0}; }
};

static_assert(sizeof(SRobotCommand)==6, "");

struct SOdometryData { 
    short m_nFrontLeft;
    short m_nFrontRight;
    short m_nBackLeft;
    short m_nBackRight;
};

constexpr uint8_t c_nFIRST_LIDAR_INDEX = 0xA0;
constexpr uint8_t c_nFIRST_LIDAR_BYTE = 0xFA;

struct SLidarData { // Neato XV11 Lidar packet
    uint8_t m_nReserved;
    uint8_t m_nIndex;
    uint16_t m_nSpeed; 

    struct SData {
        unsigned int m_nDistance : 14;
        unsigned int m_flagStrength : 1;
        unsigned int m_flagInvalidData : 1;
        uint16_t m_nStrength;
    } __attribute__((packed));
    static_assert(sizeof(SData)==4, "");

    SData m_adata[4];

    uint16_t m_nChecksum;

    bool ValidChecksum() const;
} __attribute__((packed));
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "");
static_assert(sizeof(SLidarData)==22, "");

constexpr size_t c_cbLIDAR_FULL_ROTATION = sizeof(SLidarData) * 90;
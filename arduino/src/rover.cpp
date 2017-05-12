#include "Arduino.h"
#include "rover.h"

#include "SPI.h"
#include "PID_v1.h"

#define countof(a) (sizeof(a)/sizeof(a[0]))

//////////////////////////////////////////////////////////////////////////////
//
// Motor and PID control
//

static const int MAX_SPEED = 500; // max encoder ticks per second

struct SMotor {
    const int POWER;
    const int DIR;
    const int CURRENT;
    
    const int ENCODER_IRQ; // Interrupt, not PIN
    
    void setup() {
        pinMode(POWER, OUTPUT);
        pinMode(DIR, OUTPUT);
        pinMode(CURRENT, INPUT);
        pinMode(ENCODER_IRQ, INPUT_PULLUP);
        
        m_nTicks = 0;
        m_bReverse = false;
        
        m_nLastCompute = 0;
        m_fSpeed = 0.0;
        m_fTicksPerSecond = 0.0;
        m_fPower = 0.0;
    }
    
    double m_fSpeed; // pid set point == desired ticks per second
    double m_fTicksPerSecond; // pid input
    double m_fPower; // pid output in [0, 255]
    
    unsigned long m_nLastCompute;
    volatile int m_nTicksPID;
    
    bool m_bReverse;
    void SetSpeed(int nSpeed) {
        m_fSpeed = abs(nSpeed);
        m_bReverse = nSpeed < 0;
        digitalWrite(DIR, m_bReverse ? LOW : HIGH);
    }
    
    void Stop(PID& pid) {
        pid.SetMode(MANUAL);
        SetSpeed(0);
        
        m_fPower = 0.0;
        m_nTicksPID = 0;
        m_nLastCompute = 0;
        
        analogWrite(POWER, 0);
        pid.SetMode(AUTOMATIC);
    }
    
    bool ComputePID(PID& pid) {
        unsigned long nNow = millis();
        unsigned long nChange = nNow - m_nLastCompute;
        m_fTicksPerSecond = m_nTicksPID / (nChange / 1000.0);
        if(pid.Compute()) { // is the sample time low enough?
            m_nTicksPID = 0;
            m_nLastCompute = nNow;
            analogWrite(POWER, (int)m_fPower);
            return true;
        }
        return false;
    }
    
    volatile int m_nTicks;
    void onInterrupt() {
        ++m_nTicks;
        ++m_nTicksPID;
    }
    
    int Pop() {
        int nTick = m_nTicks;
        m_nTicks = 0;
        return nTick * (m_bReverse ? -1 : 1);
    }
    
    float Current() const {
        return analogRead(CURRENT) * 5.0 / 1023.0;
    }
};

SMotor g_amotors[] = {
    // See pins.txt
    {14, 21, 45, /*4*/36},
    {15, 12, 44, /*5*/37},
    {16, 11, 43, /*6*/19},
    {26, 10, 42, /*7*/18}
};

// PID ctor expects double pointer, can't make it member of SMotor
// TODO: http://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
static const double c_fCoeffP = 0.2;
static const double c_fCoeffI = 0.7;
static const double c_fCoeffD = 0.0;

PID g_apid[] = {
    PID(&g_amotors[0].m_fTicksPerSecond, &g_amotors[0].m_fPower, &g_amotors[0].m_fSpeed, c_fCoeffP, c_fCoeffI, c_fCoeffD, DIRECT),
    PID(&g_amotors[1].m_fTicksPerSecond, &g_amotors[1].m_fPower, &g_amotors[1].m_fSpeed, c_fCoeffP, c_fCoeffI, c_fCoeffD, DIRECT),
    PID(&g_amotors[2].m_fTicksPerSecond, &g_amotors[2].m_fPower, &g_amotors[2].m_fSpeed, c_fCoeffP, c_fCoeffI, c_fCoeffD, DIRECT),
    PID(&g_amotors[3].m_fTicksPerSecond, &g_amotors[3].m_fPower, &g_amotors[3].m_fSpeed, c_fCoeffP, c_fCoeffI, c_fCoeffD, DIRECT)
};

void OnMotor0Interrupt() { g_amotors[0].onInterrupt(); }
void OnMotor1Interrupt() { g_amotors[1].onInterrupt(); }
void OnMotor2Interrupt() { g_amotors[2].onInterrupt(); }
void OnMotor3Interrupt() { g_amotors[3].onInterrupt(); }

void (*c_afnInterrupts[4])() = {
  OnMotor0Interrupt, OnMotor1Interrupt, OnMotor2Interrupt, OnMotor3Interrupt
};

#define RELAY_PIN 9

//////////////////////////////////////////////////////////////////////////////
//
// LED
//

#define LED_PIN 6
void blink(int nDelay, int nTimes = 1) {
    for(int i = 0; i<nTimes; ++i) {
        digitalWrite(LED_PIN, LOW);
        delay(nDelay);
        digitalWrite(LED_PIN, HIGH);
        delay(nDelay);
    }
}

void OnConnection();
void OnDisconnection();

void setup()
{    
    Serial.begin(115200);
    delay(3000);  //3 seconds delay for enabling to see the start up comments on the serial board

    // Show we are in setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Port setup
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);

    // Motor setup
    for(unsigned int i=0; i<countof(g_amotors); ++i) {
        g_amotors[i].setup();
        attachInterrupt(g_amotors[i].ENCODER_IRQ, c_afnInterrupts[i], CHANGE);
    }
    OnDisconnection();
}

void InternalHandleCommand(SRobotCommand const& cmd);

bool g_bConnected = false;
void OnConnection() {
    blink(200, 3);
    g_bConnected = true;
    // Activate motor controller
    digitalWrite(RELAY_PIN, LOW);
    
    InternalHandleCommand(SRobotCommand::stop()); // reinitializes motor state
}

void OnDisconnection() {
    InternalHandleCommand(SRobotCommand::stop()); // reinitializes motor state
    
    g_bConnected = false;
    // Deactivate motor controller
    digitalWrite(RELAY_PIN, HIGH);
}

struct SReadCommand {
    SRobotCommand m_cmd;
    bool m_bValid;
};

SReadCommand ReadCommand() {
    SReadCommand readcmd;
    char* pcmd = (char*)&readcmd.m_cmd;
    char* pcmdEnd = pcmd+sizeof(SRobotCommand);
    for(; pcmd<pcmdEnd && 0<Serial.available(); ++pcmd) {
        *pcmd = Serial.read();
    }
    readcmd.m_bValid = pcmd==pcmdEnd;
    return readcmd;
}

void InternalHandleCommand(SRobotCommand const& cmd) {
    switch(cmd.m_ecmd) {
       case ecmdCONNECT:
            OnConnection();
            break;
        case ecmdRESET: 
            blink(200, 4);
            OnDisconnection();
            break;
        case ecmdMOVE:
        {
            bool bReverse = false;
            for(unsigned int i=0; i<countof(g_amotors); ++i) {
                int const nSpeed = i%2==0 ? cmd.m_nSpeedLeft : cmd.m_nSpeedRight;
                bReverse = bReverse || ((nSpeed < 0) != g_amotors[i].m_bReverse);
            }

            if(bReverse) { // stop all motors and reset PID
                // Serial.println("STOP Motors");
                for(unsigned int i=0; i<countof(g_amotors); ++i) {
                    g_amotors[i].Stop(g_apid[i]);
                }
                delay(200);
            }

            for(unsigned int i=0; i<countof(g_amotors); ++i) {
                int const nSpeed = i%2==0 ? cmd.m_nSpeedLeft : cmd.m_nSpeedRight;
                if(0==nSpeed) {
                    g_amotors[i].Stop(g_apid[i]);
                } else {
                    g_amotors[i].SetSpeed(constrain(nSpeed, -MAX_SPEED, MAX_SPEED));
                }
            }
        }
    }
}

unsigned long g_nLastCommand = 0; // time in millis() of last command
void HandleCommand(SRobotCommand const& cmd) {
    g_nLastCommand = millis();
    InternalHandleCommand(cmd);
}

unsigned long g_nLastSensorData = 0; // time in millis() of last command
void SendSensorData() {
    unsigned long nMillis = millis();
    if(100<nMillis-g_nLastSensorData) {
        g_nLastSensorData = nMillis;
        // TODO: Transmit current or make emergency stop if motor current too high
        SOdometryData data = {
            g_amotors[0].Pop(),
            g_amotors[1].Pop(),
            g_amotors[2].Pop(),
            g_amotors[3].Pop()
        };
        Serial.write((byte*)&data, sizeof(data));
    }
}

static const unsigned long c_nTIMETOSTOP = 200; // ms
static const unsigned long c_nTIMETODISCONNECT = 60000; // ms

void loop() {
    if(g_bConnected) {        
        if(0<Serial.available()) {
            auto readcmd = ReadCommand();
            if(readcmd.m_bValid) {
                HandleCommand(readcmd.m_cmd);
                if(!g_bConnected) return;
            }
        } else if(c_nTIMETODISCONNECT < millis()-g_nLastCommand) {
            blink(100, 20);
            OnDisconnection(); 
            return;
        } else if(c_nTIMETOSTOP < millis()-g_nLastCommand) {
            InternalHandleCommand(SRobotCommand::stop());
        }
        
        for(unsigned int i=0; i<countof(g_amotors); ++i) {
            g_amotors[i].ComputePID(g_apid[i]);
        }

        SendSensorData();
    } else {
        auto readcmd = ReadCommand();
        if(readcmd.m_bValid && readcmd.m_cmd.m_ecmd==ecmdCONNECT) {
            HandleCommand(readcmd.m_cmd);
        }
    }
}

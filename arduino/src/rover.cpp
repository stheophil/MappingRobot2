#include "Arduino.h"
#include "rover.h"
#include "Adafruit_BNO055.h"

#include "SPI.h"
#include "PID_v1.h"
#include "Servo.h"

// #define PID_TEST
// #define AHRS_TEST

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
    int m_nTicksPID;
    
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
    
    int m_nTicks;
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
    {14, 10, 39, 2},
    {15, 11, 40, 3},
    {16, 12, 41, 6},
    {17, 13, 42, 7}
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
// IMU
//

Adafruit_BNO055 g_bno = Adafruit_BNO055(55);
unsigned short g_nYaw; // in degrees

//////////////////////////////////////////////////////////////////////////////
//
// LIDAR
//
struct SFeedbackServo {
    Servo m_servo;  
    int const m_nServoPin;
    int const m_nFeedbackPin;
    int const m_nMinPosition;
    int const m_nMaxPosition;

    int m_nMinFeedback;
    int m_nMaxFeedback;

    SFeedbackServo(int nServoPin, int nFeedbackPin, int nMinPosition = 0, int nMaxPosition = 180)
        : m_nServoPin(nServoPin)
        , m_nFeedbackPin(nFeedbackPin)
        , m_nMinPosition(nMinPosition)
        , m_nMaxPosition(nMaxPosition)
    {}

    void setup() {
        m_servo.attach(m_nServoPin);

        // Calibrate
        // Move to the minimum position and record the feedback value
        m_servo.write(m_nMinPosition);
        delay(3000); // make sure it has time to get there and settle
        m_nMinFeedback = analogRead(m_nFeedbackPin);

        // Move to the maximum position and record the feedback value
        m_servo.write(m_nMaxPosition);
        delay(3000); // make sure it has time to get there and settle
        m_nMaxFeedback = analogRead(m_nFeedbackPin);
    }

    int Position() const {
        return map(analogRead(m_nFeedbackPin), m_nMinFeedback, m_nMaxFeedback, m_nMinPosition, m_nMaxPosition);
    }

    void loop() {
        int nAngle = Position();
        if(nAngle <= m_nMinPosition + 2) {
            m_servo.write(m_nMaxPosition);
            delay(20);
        } else if(nAngle >= m_nMaxPosition - 2) {
            m_servo.write(m_nMinPosition);
            delay(20);
        }
    } 
} g_servo(27, 0);

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

void setup()
{
    Serial.begin(57600);
    delay(3000);  //3 seconds delay for enabling to see the start up comments on the serial board
    
    // Show we are in setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Port setup
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);

    // Servo
    g_servo.setup();
    
    // Motor setup
    for(unsigned int i=0; i<countof(g_amotors); ++i) {
        g_amotors[i].setup();
        attachInterrupt(g_amotors[i].ENCODER_IRQ, c_afnInterrupts[i], CHANGE);
    }

    /* Initialise the sensor */
    if(!g_bno.begin()) {        
        while(1) {
            blink(500);    
        }
    } else {
        blink(200, 3);
    }
    delay(1000);
    g_bno.setExtCrystalUse(true);

    Serial.print(g_chHandshake);
}

void OnConnection() {
    blink(200, 3);
    // Active motor controller
    digitalWrite(RELAY_PIN, LOW);
}

void OnDisconnection() {
    // Serial.println("Disconnected");
    // Deactivate motor controller
    digitalWrite(RELAY_PIN, HIGH);
}

#if defined(PID_TEST)
#include "pidtest.h"
#elif defined(AHRS_TEST)
#include "ahrs_test.h"
#else

unsigned long g_nLastCommand = 0; // time in millis() of last command
SRobotCommand g_cmdLastCommand;

/*
static const int c_nMinYaw = (int)(-M_PI * 1000 + 0.5); // TODO: to degrees
static const int c_nMaxYaw = (int)(M_PI * 1000 + 0.5);

int YawDifference(short nYawTarget) {  // TODO: to degrees
    int nYawDiff = nYawTarget - g_nYaw;
    if(nYawDiff<c_nMinYaw) nYawDiff+=2*c_nMaxYaw;
    if(c_nMaxYaw<=nYawDiff) nYawDiff-=2*c_nMaxYaw;
    return nYawDiff;
} // < 0 -> turn left

void HandleCommand(SRobotCommand const& cmd) {
    g_cmdLastCommand = cmd;
    g_nLastCommand = millis();
    
    switch(cmd.m_cmd) {
        case ecmdMOVE:
        case ecmdTURN360:
        case ecmdTURN:
        {
            short nSpeedLeft;
            short nSpeedRight;
            if(ecmdMOVE==cmd.m_cmd) {
                nSpeedLeft = cmd.arg.move.m_nSpeedLeft;
                nSpeedRight = cmd.arg.move.m_nSpeedRight;
            } else {
                bool const bTurnLeft = ecmdTURN360==cmd.m_cmd || YawDifference(cmd.arg.turn.m_nYawTarget)<0;
                nSpeedLeft = cmd.arg.turn.m_nSpeed * (bTurnLeft ? -1 : 1);
                nSpeedRight = cmd.arg.turn.m_nSpeed * (bTurnLeft ? 1 : -1);

                if(ecmdTURN360==cmd.m_cmd) {
                    g_cmdLastCommand.arg.turn.m_nYawTarget = g_nYaw;
                    // Serial.println("Turn 360");
                } else {
                    // Serial.print("Turn ");
                    // Serial.print(bTurnLeft ? "left " : "right ");
                    // Serial.print(g_nYaw);
                    // Serial.print(" -> ");
                    // Serial.println(cmd.arg.turn.m_nYawTarget);
                }
            }
            
            bool bReverse = false;
            for(unsigned int i=0; i<countof(g_amotors); ++i) {
                int nSpeed = i%2==0 ? nSpeedLeft : nSpeedRight;
                bReverse = bReverse || ((nSpeed < 0) != g_amotors[i].m_bReverse);
            }
            if(bReverse) { // stop all motors and reset PID
                // Serial.println("STOP Motors");
                for(unsigned int i=0; i<countof(g_amotors); ++i) {
                    g_amotors[i].Stop(g_apid[i]);
                }
                delay(200);
            }
                
            // LEFT MOTORS:
            g_amotors[0].SetSpeed(constrain(nSpeedLeft, -MAX_SPEED, MAX_SPEED));
            g_amotors[2].SetSpeed(constrain(nSpeedLeft, -MAX_SPEED, MAX_SPEED));
            // RIGHT MOTORS
            g_amotors[1].SetSpeed(constrain(nSpeedRight, -MAX_SPEED, MAX_SPEED));
            g_amotors[3].SetSpeed(constrain(nSpeedRight, -MAX_SPEED, MAX_SPEED));
            break;
        }
        case ecmdSTOP:
            for(unsigned int i=0; i<countof(g_amotors); ++i) g_amotors[i].Stop(g_apid[i]);
            break;
            
        default: ;
    }
}
struct SPerformanceCounter {
    unsigned long m_nStart;
    SPerformanceCounter() : m_nStart(micros()) {}
    unsigned long Stop() {
        unsigned long nEnd = micros();
        return nEnd - m_nStart;
    }
};

*/

unsigned long g_nLastSensor = 0;
void SendSensorData() {
    // TODO: Transmit current or make emergency stop if motor current too high
    // TODO: Limit number of transmissions, once every 50 ms?
    if(millis() - g_nLastSensor > 20) {
        g_nLastSensor = millis();

        int nDistance = 0; // in cm
        int nAngle = g_servo.Position();
        
        sensors_event_t event; 
        g_bno.getEvent(&event);
        g_nYaw = (short)round(event.orientation.x * 100);

        SSensorData data = {
            g_nYaw,
            nAngle, nDistance,
            g_amotors[0].Pop(),
            g_amotors[1].Pop(),
            g_amotors[2].Pop(),
            g_amotors[3].Pop(),
            g_cmdLastCommand.m_cmd
        };
        Serial.write((byte*)&data, sizeof(data));
    }
}

bool g_bConnected = false;
static const unsigned long c_nTIMETOSTOP = 200; // ms

void loop() {
    if(g_bConnected) {
        g_servo.loop();
/*
        if(ecmdTURN360==g_cmdLastCommand.m_cmd || ecmdTURN==g_cmdLastCommand.m_cmd) {
            const int nYawTolerance = 17; // ~ pi/180 * 1000 ie one degree
            int const nYawDiff = YawDifference(g_cmdLastCommand.arg.turn.m_nYawTarget);
            
            // wait a bit before comparing angles when rotating 360
            if(ecmdTURN360==g_cmdLastCommand.m_cmd) {
                if(1500 < millis() - g_nLastCommand
                && nYawDiff >= -nYawTolerance && nYawDiff < 3*nYawTolerance) { // Upper tolerance depends on turn speed!
                    Serial.println("Turned 360 deg. Stopping");
                    HandleCommand(c_rcmdStop);
                }
            } else {
                bool const bTurnLeft = g_amotors[0].m_bReverse;
                if(abs(nYawDiff)<=nYawTolerance) {
                    Serial.println("Turn complete. Stopping");
                    HandleCommand(c_rcmdStop);
                } else if(bTurnLeft != (nYawDiff<=0)) {
                    Serial.println("Wrong direction. Did yaw measurement change? Try again.");
                    HandleCommand(g_cmdLastCommand);
                }
            }
            
        } else if(0<Serial.available()) {
            SRobotCommand cmd;
            char* pcmd = (char*)&cmd;
            char* pcmdEnd = pcmd+sizeof(SRobotCommand);
            for(; pcmd<pcmdEnd && 0<Serial.available(); ++pcmd) {
                *pcmd = Serial.read();
            }
            if(pcmd==pcmdEnd) {
                HandleCommand(cmd);
            } else {
                Serial.print("Incomplete Command. Read ");
                Serial.print(pcmd - (char*)&cmd);
                Serial.println(" bytes");
            }
        } else if(c_nTIMETOSTOP < millis()-g_nLastCommand) {
            HandleCommand(c_rcmdStop);
        }
        for(unsigned int i=0; i<countof(g_amotors); ++i) {
            g_amotors[i].ComputePID(g_apid[i]); // effective sample time ~ 130 ms
        }
*/
        SendSensorData(); // ~ 40 ms
    } else {
        if(Serial.available() && Serial.read()==g_chHandshake) {
            g_bConnected = true;
            OnConnection();
        }
    }
}
#endif

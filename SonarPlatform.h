#ifndef _SonarPlatform_h_
#define _SonarPlatform_h_

#include <inttypes.h>
#include <Servo.h>
#include <RTL_StdLib.h>
#include <EventSource.h>
#include <SonarSensor.h>


class SonarPlatform : public IEventListener, public EventSource
{
	DECLARE_CLASSNAME;

    //**************************************************************************
    // Class variables
    //**************************************************************************

    public: static const EVENT_ID PING_EVENT          = (EventSourceID::SonarPlatform | 0x00F0);
    public: static const EVENT_ID OBSTACLE_EVENT      = (EventSourceID::SonarPlatform | EventCode::Obstacle);
    public: static const EVENT_ID SCAN_COMPLETE_EVENT = (EventSourceID::SonarPlatform | 0x00F1);


    public: typedef struct SonarPing_struct
    {
        uint8_t  Angle;
        uint16_t Ping;

        SonarPing_struct() : Angle(0), Ping(0) {  };

        SonarPing_struct(const uint8_t angle, const uint16_t  ping) : Angle(angle), Ping(ping) {  };

        void Set(const uint8_t angle, const uint16_t ping) { Angle = angle; Ping = ping; };

        void Zero() { Angle = 0; Ping = 0; };

        SonarPing_struct& operator=(const int value) { Angle=value; Ping=value; return *this; };
    }
    SonarPing;


    //**************************************************************************
    // Constructor
    //**************************************************************************
    public: SonarPlatform(SonarSensor& sonar);

    //**************************************************************************
    // Public interface
    //**************************************************************************
    public: void AttachServos(uint8_t azServoPin, uint8_t tiltServoPin);
	
    public: void SetPlatformBias(int azBias=0, int tiltBias=0) { _azimuthBias = azBias; _tiltBias = tiltBias; };

    public: void Deploy(int azAngle=90, int tiltAngle=90) { Azimuth(azAngle); Tilt(tiltAngle); };

    public: void Stow() { Azimuth(90); Tilt(180); };

    public: void Azimuth(int azAngle) { _azimuthServo.write(constrain(azAngle + _azimuthBias, 0, 180)); };

    public: void Tilt(int tiltAngle) { _tiltServo.write(constrain(tiltAngle + _tiltBias, 0, 180)); };

    public: uint16_t Ping();

    public: void StartScan(int tiltAngle=90);

    public: bool NextScanPing();

    public: SonarPing BestPing() { return _bestPing; }

    public: void StartObstacleDetection(int azAngle, int tiltAngle, uint16_t distance);

    public: void StartObstacleDetection(uint16_t distance);

    public: void StopObstacleDetection();

    public: void OnEvent(const Event* pEvent);

    public: void Poll();

    //**************************************************************************
    // Private implementation
    //**************************************************************************
    private: uint16_t ProcessPings(SonarPing pings[], int arraySize);

    private: uint16_t ScanPing();

    private: SonarSensor* _sonar;
    private: Servo _azimuthServo;
    private: Servo _tiltServo;
    private: uint16_t _detectionDistance;
    private: int8_t _azimuthBias;
    private: int8_t _tiltBias;
    private: uint8_t  _scanAngle = 0;
    private: uint8_t  _nextSlot;
    private: SonarPing _last5Pings[5];
    private: SonarPing _bestPing;
    private: bool _asyncEnabled;
};

#endif


#define DEBUG 0

#include <Arduino.h>
#include <Debug.h>
#include <Timer.h>
#include <EventBinding.h>
#include "SonarPlatform.h"


//******************************************************************************
// Class variables
//******************************************************************************
DEFINE_CLASSNAME(SonarPlatform);

static EventBinding SonarSensorBinding;

static int _triggerCount = 0;

static uint32_t _startDelayTime = 0;

static Timer _timer;


//******************************************************************************
// Constructor
//******************************************************************************
SonarPlatform::SonarPlatform(SonarSensor& sonar)
{
    _id = "SonarPlatform";
    _sonar = &sonar;
    _asyncEnabled = false;
    _azimuthBias = 0;
    _tiltBias = 0;
    SonarSensorBinding.Bind(*this, *_sonar);
}


void SonarPlatform::OnEvent(const Event* pEvent)
{
    switch (pEvent->EventID)
	{
        case SonarSensor::SONAR_EVENT:
        {
            auto distance = PingTimeToCentimeters(pEvent->Data.UnsignedLong);

            TRACE(Logger(_classname_, this) << F("SONAR_EVENT distance=") << distance << endl);

            if (0 < distance && distance < _detectionDistance)
            {
                if (++_triggerCount >= 3)
                {
                    QueueEvent(OBSTACLE_EVENT, distance);
                    //DispatchEvent(OBSTACLE_EVENT, distance);
                }
            }
            else
            {
                _triggerCount = 0;
            }
        }
		break;
        
        case TimerFiredEvent:
        {
            TRACE(Logger(_classname_, this) << F("TIMER_EVENT") << endl);

            if (NextScanPing())
            {
                _timer.Start(100); // Allow time for platform to move to next position and ping
            }
            else
            {
				Logger(_classname_, this) << F("QUEUE SCAN_COMPLETE_EVENT") << endl;
                QueueEvent(SCAN_COMPLETE_EVENT, (int16_t)_bestPing.Angle);
            }
        }
		break;
		
		default:
		break;
	}
}


void SonarPlatform::Poll()
{
    //TRACE(Logger(_classname_, this) << F("Poll") << endl);

    // Start another async ping
    if (_asyncEnabled && _sonar->Ready() && (millis() > _startDelayTime)) _sonar->PingAsync();
}


void SonarPlatform::AttachServos(uint8_t azServoPin, uint8_t tiltServoPin)
{
    _azimuthServo.attach(azServoPin);
    _tiltServo.attach(tiltServoPin);
}


uint16_t SonarPlatform::Ping()
{
    uint16_t ping = _sonar->PingCentimeters();

    return ping;
}


void SonarPlatform::StartObstacleDetection(int azAngle, int tiltAngle, uint16_t distance)
{
    Deploy(azAngle, tiltAngle);
    StartObstacleDetection(distance);
}


void SonarPlatform::StartObstacleDetection(uint16_t distance)
{
    _detectionDistance = distance;
    _asyncEnabled = true;
    _startDelayTime = millis() + 500;
    _triggerCount = 0;

    // Async pinging will start on next Poll() cycle after start delay interval
}


void SonarPlatform::StopObstacleDetection()
{
    _asyncEnabled = false;
}


void SonarPlatform::StartScan(int tiltAngle)
{
    _bestPing.Zero();
    _scanAngle = 0;
    _nextSlot = 0;
    ZERO_ARRAY(_last5Pings);
    Deploy(_scanAngle, tiltAngle);
    _timer.Attach(*this);
    _timer.Start(500);
}


bool SonarPlatform::NextScanPing()
{
	Logger(_classname_, this) << F("NextScanPing") << endl;
    ScanPing();

    if ((_scanAngle += 10) > 180) return false;

    Azimuth(_scanAngle);

    return true;
}


uint16_t SonarPlatform::ScanPing()
{
    const int lastIndex = LAST_INDEX(_last5Pings);
    
    //while (!_sonar->Ready());

    auto pingTime = _sonar->PingMedian();
    auto ping = PingTimeToCentimeters(pingTime);

    if (_nextSlot > lastIndex)
    {
        SHIFT_ARRAY_LEFT(_last5Pings);
        _last5Pings[lastIndex].Set(_scanAngle, ping);
        _nextSlot = lastIndex;
    }

    _last5Pings[_nextSlot++].Set(_scanAngle, ping);

    if (_nextSlot > lastIndex)
    {
        uint16_t computedPing = ProcessPings(_last5Pings, ARRAY_LENGTH(_last5Pings));

        if (computedPing > _bestPing.Ping)
        {
            _bestPing.Set(_last5Pings[2].Angle, computedPing);
        }
    }

    TRACE(Logger(_classname_, this) << F("ScanPing: _scanAngle=") << _scanAngle 
                                    << F(", ping=") << ping  
                                    << F(", _bestPing.Angle=") << _bestPing.Angle 
                                    << F(", _bestPing.Ping=") << _bestPing.Ping 
	                                << endl);

    return ping;
}


uint16_t SonarPlatform::ProcessPings(SonarPing pings[], int arraySize)
{
    TRACE(Logger(_classname_, this) << F("ProcessPings: arraySize=") << arraySize << endl);

    //for (int i=0; i < arraySize; i++) Debug.Log(this) << _func__ << F(": pings[") << i << F("]: Angle=") << pings[i].Angle << F("Ping=") << pings[i].Ping << endl;

    SonarPing sortedPings[arraySize];
    
    COPY_ARRAY_PTR(sortedPings, pings, arraySize);

    // Sort array elements
    // This is just a plain old-fashioned bubble sort. Its a simple algorithm
    // that is adequate for very small data sets
    for (int i = 0; i < (arraySize-1); i++)
        for (int j = i+1; j < arraySize; j++)
            if (sortedPings[j].Ping < sortedPings[i].Ping) swap(sortedPings[i], sortedPings[j]);

    // Compute statistical parameters of data set
    int median = sortedPings[arraySize / 2].Ping;       // median value
    int q1     = sortedPings[arraySize / 4].Ping;       // First quartile value
    int q3     = sortedPings[arraySize * 3 / 4].Ping;   // Third quartile value
    int iqr    = (q3 - q1)+ 1;                          // Inter-quartile range
    int upperOutlierLimit = q3 + (iqr * 2);             // limit value for detecting upper outliers

    TRACE(Logger(_classname_, this) << F("ProcessPings: median=") << median
                                    << F(", q1=") << q1 	
                                    << F(", q3=") << q3 	
                                    << F(", iqr=") << iqr 	
                                    << F(", upperOutlierLimit=") << upperOutlierLimit 	
	                                << endl);

    // Compute moving average, filtering out upper outliers as these may represent failed pings
    uint16_t avg = 0;
    uint16_t count = 0;

    for (int i = 0; i < arraySize; i++)
    {
        if (sortedPings[i].Ping <= upperOutlierLimit)
        {
            avg += sortedPings[i].Ping;
            count++;
        }
    }

    avg = (avg / count); // Count should never be zero as all data points can't be outliers

    TRACE(Logger(_classname_, this) << F("ProcessPings: Adjusted Ping=") << avg << endl);

    return avg;
}


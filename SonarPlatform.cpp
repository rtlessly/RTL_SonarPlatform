#define DEBUG 0

#include <Arduino.h>
#include <RTL_Stdlib.h>
#include <Debug.h>
#include <EventBinding.h>
#include "SonarPlatform.h"


//******************************************************************************
// Class variables
//******************************************************************************

EVENT_ID SonarPlatform::OBSTACLE_EVENT = EventSource::GenerateEventID();

static DebugHelper Debug("SonarPlatform");

static EventBinding SonarSensorBinding;

static int _triggerCount = 0;

static uint32_t _startDelayTime = 0;


//******************************************************************************
// Constructor
//******************************************************************************
SonarPlatform::SonarPlatform(const SonarSensor& sonar)
{
    _sonar = &sonar;
    _asyncEnabled = false;
    _azimuthBias = 0;
    _tiltBias = 0;
    SonarSensorBinding.Bind(*this, *_sonar);
//   sonar.Attach(*this);
}


void SonarPlatform::OnEvent(const Event* pEvent)
{
    WithEvent(pEvent)
        When(SonarSensor::SONAR_EVENT)
        {
            uint16_t distance = PingTimeToCentimeters(pEvent->Data.UnsignedLong);

            Debug.Log("%s SONAR_EVENT(%i)", __func__, distance);

            if (0 < distance && distance < _detectionDistance)
            {
                if (++_triggerCount >= 3)
                {
                    DispatchEvent(OBSTACLE_EVENT, distance);
//                    Event event(OBSTACLE_EVENT, distance);
//                    DispatchEvent(&event);
                }
            }
            else
            {
                _triggerCount = 0;
            }
        }
}


void SonarPlatform::Poll()
{
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
}


bool SonarPlatform::NextScanPing()
{
    ScanPing();

    if ((_scanAngle += 10) > 180) return false;

    Azimuth(_scanAngle);

    return true;
}


uint16_t SonarPlatform::ScanPing()
{
    while (!_sonar->Ready());

    uint16_t ping = _sonar->PingMedian();

    // If we got no echo that means the distance is greated than can be measured, so set to max distance
    // Otherwise, convert ping  time to centimeters
    ping = (ping == NO_ECHO) ? MAX_SENSOR_DISTANCE : PingTimeToCentimeters(ping);

    const int lastIndex = LAST_INDEX(_last5Pings);
    
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
            //Debug.Log("%s: _bestPing.Angle=%i, _bestPing.Ping=%i", __func__, _bestPing.Angle, _bestPing.Ping);
            //Debug.Log("%s: _last5Pings[2].Angle=%i, computedPing=%i", __func__, _last5Pings[2].Angle, computedPing);
            _bestPing.Set(_last5Pings[2].Angle, computedPing);
            //Debug.Log("%s: _bestPing.Angle=%i, _bestPing.Ping=%i", __func__, _bestPing.Angle, _bestPing.Ping);
        }
    }

    Debug.Log("%s: angle=%i, ping=%i, bestPing.Angle=%i, bestPing.Ping=%i", __func__, _scanAngle, ping, _bestPing.Angle, _bestPing.Ping);

    return ping;
}


uint16_t SonarPlatform::ProcessPings(SonarPing pings[], int arraySize)
{
    Debug.Log("%s: arraySize=%i", __func__, arraySize);

    //for (int i=0; i < arraySize; i++) Debug.Log("%s: pings[%i].Angle=%i, pings[%i].Ping=%i", __func__, i, pings[i].Angle, i, pings[i].Ping);

    SonarPing sortedPings[arraySize];
    
    COPY_ARRAY_PTR(sortedPings, pings, arraySize);

    // Sort array elements
    // This is just a plain old-fashioned bubble sort. Its a simple algorithm
    // that is adequate for very small data sets
    for (int i = 0; i < (arraySize-1); i++)
        for (int j = i+1; j < arraySize; j++)
            if (sortedPings[j].Ping < sortedPings[i].Ping) swap(sortedPings[i], sortedPings[j]);

    //for (int i=0; i < arraySize; i++) Debug.Log("%s: sortedPings[%i].Angle=%i, sortedPings[%i].Ping=%i", __func__, i, sortedPings[i].Angle, i, sortedPings[i].Ping);

    // Compute statitical parameters of data set
    int median = sortedPings[arraySize / 2].Ping;       // median value
    int q1     = sortedPings[arraySize / 4].Ping;       // First quartile value
    int q3     = sortedPings[arraySize * 3 / 4].Ping;   // Third quartile value
    int iqr    = (q3 - q1)+ 1;                          // Inter-quartile range
    int upperOutlierLimit = q3 + (iqr * 2);             // limit value for detecting upper outliers

    Debug.Log("%s: median=%i, q1=%i, q3=%i, iqr=%i, upperOutlierLimit=%i", __func__, median, q1,q3, iqr, upperOutlierLimit);

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

    Debug.Log("%s: Adjusted Ping=%i", __func__, avg);

    return avg;
}


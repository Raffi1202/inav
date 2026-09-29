/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

#include "gtest/gtest.h"
#include "unittest_macros.h"

extern "C" {
#include "platform.h"

#include "common/maths.h"
#include "common/printf.h"

#include "config/feature.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/gps.h"

#include "navigation/navigation.h"

#include "rx/crsf.h"

#include "sensors/battery.h"
#include "sensors/pitotmeter.h"
#include "sensors/sensors.h"
#include "sensors/temperature.h"

#include "telemetry/crsf.h"
#include "telemetry/msp_shared.h"
#include "telemetry/telemetry.h"

gpsSolutionData_t gpsSol;
telemetryConfig_t telemetryConfig_System;
navConfig_t navConfig_System;
attitudeEulerAngles_t attitude;
uint32_t armingFlags;
uint32_t stateFlags;
uint32_t flightModeFlags;

static float estimatedAltitudeCm;

// only the GPS frame is under test, the other frames and the receiver link just need to link
float getEstimatedActualPosition(int axis) { return axis == Z ? estimatedAltitudeCm : 0; }
float getEstimatedActualVelocity(int axis) { UNUSED(axis); return 0; }
bool feature(uint32_t mask) { UNUSED(mask); return false; }
bool sensors(uint32_t mask) { UNUSED(mask); return false; }
bool IS_RC_MODE_ACTIVE(boxId_e boxId) { UNUSED(boxId); return false; }
bool navigationRequiresAngleMode(void) { return false; }
bool isWaypointMissionRTHActive(void) { return false; }
int32_t constrain(int32_t amt, int32_t low, int32_t high) { return amt < low ? low : (amt > high ? high : amt); }
uint8_t calculateBatteryPercentage(void) { return 0; }
int16_t getAmperage(void) { return 0; }
int32_t getMAhDrawn(void) { return 0; }
uint16_t getBatteryVoltage(void) { return 0; }
uint16_t getBatteryAverageCellVoltage(void) { return 0; }
float getAirspeedEstimate(void) { return 0; }
bool getSensorTemperature(uint8_t sensorIndex, int16_t *temperature)
{
    UNUSED(sensorIndex);
    UNUSED(temperature);
    return false;
}
bool crsfRxIsActive(void) { return true; }
bool crsfRxIsTelemetryBufEmpty(void) { return true; }
void crsfRxSendTelemetryData(void) {}
void crsfRxWriteTelemetryData(const void *data, int len) { UNUSED(data); UNUSED(len); }
bool handleMspFrame(uint8_t *frameStart, int payloadLength)
{
    UNUSED(frameStart);
    UNUSED(payloadLength);
    return false;
}
bool sendMspReply(uint8_t payloadSize, mspResponseFnPtr responseFn)
{
    UNUSED(payloadSize);
    UNUSED(responseFn);
    return false;
}
int tfp_sprintf(char *s, const char *fmt, ...) { UNUSED(fmt); s[0] = 0; return 0; }
}

#define GPS_FRAME_SIZE          19  // sync, length, type, 15 byte payload, CRC
#define GPS_FRAME_TAIL_SIZE     5

static uint8_t crc8DvbS2(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t readU32BigEndian(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

class TelemetryCrsfGpsTest : public ::testing::Test {
protected:
    uint8_t frame[CRSF_FRAME_SIZE_MAX];

    void SetUp() override
    {
        memset(frame, 0, sizeof(frame));
        memset(&telemetryConfig_System, 0, sizeof(telemetryConfig_System));
        memset(&gpsSol, 0, sizeof(gpsSol));
        gpsSol.llh.lat = 470000000;
        gpsSol.llh.lon = 80000000;
        gpsSol.llh.alt = 50000;         // cm
        gpsSol.groundSpeed = 500;       // cm/s
        gpsSol.groundCourse = 900;      // decidegrees
        gpsSol.numSat = 12;
        gpsSol.fixType = GPS_FIX_3D;
        gpsSol.time.year = 2026;
        gpsSol.flags.validFixTime = true;
        gpsSol.fixTimeOfDayMs = 0x01A2B3C4;
    }

    void expectStandardPart(void)
    {
        EXPECT_EQ(CRSF_TELEMETRY_SYNC_BYTE, frame[0]);
        EXPECT_EQ(CRSF_FRAMETYPE_GPS, frame[2]);
        EXPECT_EQ(470000000u, readU32BigEndian(&frame[3]));
        EXPECT_EQ(80000000u, readU32BigEndian(&frame[7]));
        EXPECT_EQ(180, (frame[11] << 8) | frame[12]);      // km/h * 10
        EXPECT_EQ(9000, (frame[13] << 8) | frame[14]);     // centidegrees
        EXPECT_EQ(1500, (frame[15] << 8) | frame[16]);     // m + 1000
        EXPECT_EQ(12, frame[17]);
    }

    void expectCrcOverTypeAndPayload(int frameSize)
    {
        // CRC covers type and payload: everything after sync and length, before the CRC byte
        EXPECT_EQ(crc8DvbS2(&frame[2], frameSize - 3), frame[frameSize - 1]);
    }
};

TEST_F(TelemetryCrsfGpsTest, TailWithExactFixTime)
{
    const int frameSize = getCrsfFrame(frame, CRSF_FRAMETYPE_GPS);

    EXPECT_EQ(GPS_FRAME_SIZE + GPS_FRAME_TAIL_SIZE, frameSize);
    EXPECT_EQ(22, frame[1]);
    expectStandardPart();
    EXPECT_EQ(0x01, frame[18]);     // big-endian time of day
    EXPECT_EQ(0xA2, frame[19]);
    EXPECT_EQ(0xB3, frame[20]);
    EXPECT_EQ(0xC4, frame[21]);
    EXPECT_EQ(GPS_FIX_3D, frame[22]);
    expectCrcOverTypeAndPayload(frameSize);
}

TEST_F(TelemetryCrsfGpsTest, NoTailWithoutExactFixTime)
{
    gpsSol.flags.validFixTime = false;
    const int frameSize = getCrsfFrame(frame, CRSF_FRAMETYPE_GPS);

    EXPECT_EQ(GPS_FRAME_SIZE, frameSize);
    EXPECT_EQ(17, frame[1]);
    expectStandardPart();
    expectCrcOverTypeAndPayload(frameSize);
}

TEST_F(TelemetryCrsfGpsTest, NoTailWithoutYear)
{
    gpsSol.time.year = 0;
    const int frameSize = getCrsfFrame(frame, CRSF_FRAMETYPE_GPS);

    EXPECT_EQ(GPS_FRAME_SIZE, frameSize);
    EXPECT_EQ(17, frame[1]);
}

TEST_F(TelemetryCrsfGpsTest, NoTailWithLegacyBaroPacket)
{
    // the altitude field then carries the current estimate, not the altitude of this fix
    telemetryConfig_System.crsf_use_legacy_baro_packet = true;
    estimatedAltitudeCm = 12300;
    const int frameSize = getCrsfFrame(frame, CRSF_FRAMETYPE_GPS);

    EXPECT_EQ(GPS_FRAME_SIZE, frameSize);
    EXPECT_EQ(17, frame[1]);
    EXPECT_EQ(1123, (frame[15] << 8) | frame[16]);
    expectCrcOverTypeAndPayload(frameSize);
}

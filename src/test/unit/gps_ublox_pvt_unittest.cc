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

#include "build/debug.h"

#include "common/typeconversion.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/gps.h"
#include "io/gps_private.h"
#include "io/gps_ublox.h"
#include "io/serial.h"

bool gpsNewFrameUBLOX(uint8_t data);

gpsReceiverData_t gpsState;
gpsStatistics_t gpsStats;
gpsSolutionData_t gpsSolDRV;
gpsSolutionData_t gpsSol;
gpsConfig_t gpsConfig_System;
baudRate_e gpsToSerialBaudRate[GPS_BAUDRATE_COUNT];
const uint32_t baudRates[] = { 0 };

int32_t debug[DEBUG32_VALUE_COUNT];
uint8_t debugMode;

// the parser under test only needs these to link, the serial and state paths are not exercised
timeMs_t millis(void) { return 0; }
float fastA2F(const char *p) { UNUSED(p); return 0; }
int gpsBaudRateToInt(gpsBaudRate_e baudrate) { UNUSED(baudrate); return 0; }
uint16_t gpsConstrainEPE(uint32_t epe) { return epe; }
uint16_t gpsConstrainHDOP(uint32_t hdop) { return hdop; }
void gpsProcessNewDriverData(void) {}
void gpsProcessNewSolutionData(bool timeout) { UNUSED(timeout); }
void gpsSetProtocolTimeout(timeMs_t timeoutMs) { UNUSED(timeoutMs); }
void gpsSetState(gpsState_e state) { UNUSED(state); }
bool isSerialTransmitBufferEmpty(const serialPort_t *instance) { UNUSED(instance); return true; }
void serialPrint(serialPort_t *instance, const char *str) { UNUSED(instance); UNUSED(str); }
uint8_t serialRead(serialPort_t *instance) { UNUSED(instance); return 0; }
uint32_t serialRxBytesWaiting(const serialPort_t *instance) { UNUSED(instance); return 0; }
void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate) { UNUSED(instance); UNUSED(baudRate); }
void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count)
{
    UNUSED(instance);
    UNUSED(data);
    UNUSED(count);
}

char *strnstr(const char *s, const char *find, size_t slen)
{
    const size_t len = strlen(find);
    for (size_t i = 0; i + len <= slen && s[i]; i++) {
        if (strncmp(s + i, find, len) == 0) {
            return (char *)(s + i);
        }
    }
    return NULL;
}
}

#define NAV_PVT_PAYLOAD_SIZE 92
#define STABLE_EPOCHS 5

static bool feedUbx(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t payloadLength)
{
    const uint8_t header[] = { PREAMBLE1, PREAMBLE2, msgClass, msgId,
        (uint8_t)(payloadLength & 0xFF), (uint8_t)(payloadLength >> 8) };
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    bool parsed = false;

    for (unsigned i = 0; i < sizeof(header); i++) {
        if (i >= 2) {
            ckA += header[i];
            ckB += ckA;
        }
        parsed |= gpsNewFrameUBLOX(header[i]);
    }
    for (unsigned i = 0; i < payloadLength; i++) {
        ckA += payload[i];
        ckB += ckA;
        parsed |= gpsNewFrameUBLOX(payload[i]);
    }
    parsed |= gpsNewFrameUBLOX(ckA);
    parsed |= gpsNewFrameUBLOX(ckB);
    return parsed;
}

static bool feedPvt(uint8_t msgClass, uint16_t payloadLength, const ubx_nav_pvt *pvt)
{
    uint8_t payload[NAV_PVT_PAYLOAD_SIZE] = { 0 };
    memcpy(payload, pvt, sizeof(*pvt));
    return feedUbx(msgClass, MSG_PVT, payload, payloadLength);
}

static bool feedPvtEpochs(uint8_t msgClass, uint16_t payloadLength, const ubx_nav_pvt *pvt, int count)
{
    bool parsed = true;
    for (int i = 0; i < count; i++) {
        parsed &= feedPvt(msgClass, payloadLength, pvt);
    }
    return parsed;
}

static ubx_nav_pvt validPvt(uint8_t hour, uint8_t min, uint8_t sec, int32_t nano)
{
    ubx_nav_pvt pvt;
    memset(&pvt, 0, sizeof(pvt));
    pvt.year = 2026;
    pvt.month = 9;
    pvt.day = 29;
    pvt.hour = hour;
    pvt.min = min;
    pvt.sec = sec;
    pvt.valid = 0x07;   // validDate, validTime, fullyResolved
    pvt.tAcc = 500000;  // 500 us
    pvt.nano = nano;
    pvt.fix_type = FIX_3D;
    pvt.fix_status = NAV_STATUS_FIX_VALID;
    pvt.satellites = 12;
    pvt.latitude = 470000000;
    pvt.longitude = 80000000;
    return pvt;
}

static uint32_t timeOfDayMs(uint32_t hour, uint32_t min, uint32_t sec, uint32_t ms)
{
    return ((hour * 60 + min) * 60 + sec) * 1000 + ms;
}

class GpsUbloxPvtTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // one epoch without a valid time resets the driver's stability counter
        ubx_nav_pvt reset = validPvt(0, 0, 0, 0);
        reset.valid = 0;
        feedPvt(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &reset);
        memset(&gpsSolDRV, 0, sizeof(gpsSolDRV));
    }

    void expectFixTime(const ubx_nav_pvt *pvt, uint32_t expectedTimeOfDayMs)
    {
        EXPECT_TRUE(feedPvtEpochs(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, pvt, STABLE_EPOCHS));
        EXPECT_TRUE(gpsSolDRV.flags.validFixTime);
        EXPECT_EQ(expectedTimeOfDayMs, gpsSolDRV.fixTimeOfDayMs);
    }

    void expectNoFixTime(uint8_t msgClass, uint16_t payloadLength, const ubx_nav_pvt *pvt)
    {
        feedPvtEpochs(msgClass, payloadLength, pvt, STABLE_EPOCHS + 1);
        EXPECT_FALSE(gpsSolDRV.flags.validFixTime);
    }

    void expectSingleEpoch(const ubx_nav_pvt *pvt, bool validFixTime, uint32_t expectedTimeOfDayMs = 0)
    {
        EXPECT_TRUE(feedPvt(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, pvt));
        EXPECT_EQ(validFixTime, gpsSolDRV.flags.validFixTime);
        if (validFixTime) {
            EXPECT_EQ(expectedTimeOfDayMs, gpsSolDRV.fixTimeOfDayMs);
        }
    }
};

TEST_F(GpsUbloxPvtTest, PositiveNanoRoundsToNearestMs)
{
    const ubx_nav_pvt pvt = validPvt(10, 20, 30, 199999990);
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 200));
    EXPECT_EQ(470000000, gpsSolDRV.llh.lat);
    EXPECT_EQ(80000000, gpsSolDRV.llh.lon);
}

TEST_F(GpsUbloxPvtTest, SmallNegativeNanoAtSecondBoundary)
{
    const ubx_nav_pvt pvt = validPvt(10, 20, 31, -10);
    expectFixTime(&pvt, timeOfDayMs(10, 20, 31, 0));
}

TEST_F(GpsUbloxPvtTest, NegativeNanoGivesPreviousSecond)
{
    const ubx_nav_pvt pvt = validPvt(10, 20, 31, -3000000);
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 997));
}

TEST_F(GpsUbloxPvtTest, NanoRoundsHalfAwayFromZero)
{
    ubx_nav_pvt pvt = validPvt(10, 20, 31, 0);
    expectFixTime(&pvt, timeOfDayMs(10, 20, 31, 0));

    pvt.nano = -2600000;    // round toward zero would give .998
    expectSingleEpoch(&pvt, true, timeOfDayMs(10, 20, 30, 997));

    pvt.nano = -500000;
    expectSingleEpoch(&pvt, true, timeOfDayMs(10, 20, 30, 999));

    pvt.nano = -499999;
    expectSingleEpoch(&pvt, true, timeOfDayMs(10, 20, 31, 0));

    pvt.nano = 499999;
    expectSingleEpoch(&pvt, true, timeOfDayMs(10, 20, 31, 0));

    pvt.nano = 500000;
    expectSingleEpoch(&pvt, true, timeOfDayMs(10, 20, 31, 1));
}

TEST_F(GpsUbloxPvtTest, TimeBeforeTheReportedDayIsNotExact)
{
    const ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 0));

    // 23:59:59.997, or 23:59:60.997 after a leap second, of the previous day
    const ubx_nav_pvt midnight = validPvt(0, 0, 0, -3000000);
    expectSingleEpoch(&midnight, false);

    // a representation limit, not a receiver problem: the stable run continues
    expectSingleEpoch(&pvt, true, timeOfDayMs(10, 20, 30, 0));
}

TEST_F(GpsUbloxPvtTest, TimeAfterTheReportedDayIsNotExact)
{
    const ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 0));

    const ubx_nav_pvt pastMidnight = validPvt(23, 59, 59, 999600000);
    expectSingleEpoch(&pastMidnight, false);

    const ubx_nav_pvt beforeMidnight = validPvt(23, 59, 59, 999400000);
    expectSingleEpoch(&beforeMidnight, true, 86399999u);
}

TEST_F(GpsUbloxPvtTest, LeapSecond)
{
    ubx_nav_pvt pvt = validPvt(23, 59, 60, 0);
    expectFixTime(&pvt, 86400000u);

    pvt.nano = 995000000;
    expectFixTime(&pvt, 86400995u);

    pvt.nano = -3000000;
    expectFixTime(&pvt, 86399997u);
}

TEST_F(GpsUbloxPvtTest, NotFullyResolved)
{
    ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);
    pvt.valid = 0x03;
    expectNoFixTime(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt);
    EXPECT_TRUE(gpsSolDRV.flags.validTime);
}

TEST_F(GpsUbloxPvtTest, ConfirmedTime)
{
    ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);

    pvt.flags2 = 0x20;  // confirmedAvai without confirmedTime
    expectNoFixTime(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt);

    pvt.flags2 = 0xA0;  // confirmedAvai and confirmedTime
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 0));

    pvt.flags2 = 0x20;
    expectSingleEpoch(&pvt, false);

    pvt.flags2 = 0x00;  // confirmation not available
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 0));
}

TEST_F(GpsUbloxPvtTest, WrongClassIsIgnored)
{
    const ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);

    EXPECT_FALSE(feedPvtEpochs(0x02, NAV_PVT_PAYLOAD_SIZE, &pvt, STABLE_EPOCHS));   // RXM class, same id
    EXPECT_FALSE(gpsSolDRV.flags.validFixTime);
    EXPECT_FALSE(gpsSolDRV.flags.validTime);
    EXPECT_EQ(0, gpsSolDRV.llh.lat);
    EXPECT_EQ(0, gpsSolDRV.llh.lon);

    EXPECT_FALSE(feedPvtEpochs(0x29, NAV_PVT_PAYLOAD_SIZE, &pvt, STABLE_EPOCHS));   // NAV2-PVT, same id and layout
    EXPECT_FALSE(gpsSolDRV.flags.validFixTime);
    EXPECT_FALSE(gpsSolDRV.flags.validTime);
    EXPECT_EQ(0, gpsSolDRV.llh.lat);
    EXPECT_EQ(0, gpsSolDRV.llh.lon);
}

TEST_F(GpsUbloxPvtTest, ShortPayload)
{
    const ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);
    expectNoFixTime(CLASS_NAV, sizeof(ubx_nav_pvt) - 4, &pvt);
    EXPECT_TRUE(gpsSolDRV.flags.validTime);

    EXPECT_TRUE(feedPvtEpochs(CLASS_NAV, sizeof(ubx_nav_pvt), &pvt, STABLE_EPOCHS));
    EXPECT_TRUE(gpsSolDRV.flags.validFixTime);
}

TEST_F(GpsUbloxPvtTest, DateOrTimeNotValid)
{
    ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);

    pvt.valid = 0x06;   // no validDate
    expectNoFixTime(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt);
    EXPECT_FALSE(gpsSolDRV.flags.validTime);

    pvt.valid = 0x05;   // no validTime
    expectNoFixTime(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt);
    EXPECT_FALSE(gpsSolDRV.flags.validTime);
}

TEST_F(GpsUbloxPvtTest, InvalidTimeClearsFixTime)
{
    ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 0));

    pvt.valid = 0;
    expectSingleEpoch(&pvt, false);
    EXPECT_FALSE(gpsSolDRV.flags.validTime);
}

TEST_F(GpsUbloxPvtTest, PosllhClearsFixTime)
{
    const ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 0));

    ubx_nav_posllh posllh;
    memset(&posllh, 0, sizeof(posllh));
    posllh.latitude = 470001000;
    posllh.longitude = 80001000;
    ASSERT_EQ(28u, sizeof(posllh));
    feedUbx(CLASS_NAV, MSG_POSLLH, (const uint8_t *)&posllh, sizeof(posllh));
    EXPECT_EQ(470001000, gpsSolDRV.llh.lat);
    EXPECT_FALSE(gpsSolDRV.flags.validFixTime);
}

TEST_F(GpsUbloxPvtTest, Requires3dFix)
{
    ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);

    pvt.fix_type = FIX_2D;
    expectNoFixTime(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt);
    EXPECT_EQ(GPS_FIX_2D, gpsSolDRV.fixType);

    pvt.fix_type = FIX_3D;
    pvt.fix_status = 0;     // gnssFixOK not set
    expectNoFixTime(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt);
    EXPECT_EQ(GPS_NO_FIX, gpsSolDRV.fixType);
}

TEST_F(GpsUbloxPvtTest, RequiresTimeAccuracy)
{
    ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);

    pvt.tAcc = 2000000;     // 2 ms
    expectNoFixTime(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt);

    pvt.tAcc = 1000000;     // 1 ms, the limit
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 0));

    pvt.tAcc = 0xFFFFFFFF;  // largest U4, far off like unresolved leap seconds
    expectSingleEpoch(&pvt, false);

    pvt.tAcc = 500000;      // 500 us
    expectFixTime(&pvt, timeOfDayMs(10, 20, 30, 0));
}

TEST_F(GpsUbloxPvtTest, RequiresStableRunOfGoodEpochs)
{
    ubx_nav_pvt pvt = validPvt(10, 20, 30, 0);

    for (int epoch = 1; epoch <= STABLE_EPOCHS; epoch++) {
        EXPECT_TRUE(feedPvt(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt));
        EXPECT_EQ(epoch == STABLE_EPOCHS, gpsSolDRV.flags.validFixTime) << "epoch " << epoch;
    }
    // 5 + 251 = 256 good epochs: a counter without saturation would wrap to 0 on the last one
    for (int epoch = STABLE_EPOCHS + 1; epoch <= 256; epoch++) {
        EXPECT_TRUE(feedPvt(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt));
        ASSERT_TRUE(gpsSolDRV.flags.validFixTime) << "epoch " << epoch;
    }

    pvt.tAcc = 2000000;     // one bad epoch resets the run
    expectSingleEpoch(&pvt, false);

    pvt.tAcc = 500000;
    for (int epoch = 1; epoch <= STABLE_EPOCHS; epoch++) {
        EXPECT_TRUE(feedPvt(CLASS_NAV, NAV_PVT_PAYLOAD_SIZE, &pvt));
        EXPECT_EQ(epoch == STABLE_EPOCHS, gpsSolDRV.flags.validFixTime) << "epoch " << epoch;
    }
}

#include "copro/OTOS.hpp"
#include "copro/Coprocessor.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>
#include <system_error>

// getCoprocessorVersion,         // 0 // low prio
// otos::getStatus,               // 1 // done
// otos::getVersion,              // 2 // low prio
// otos::resetTracking,           // 3 // done
// otos::getPosVelAccel,          // 4 // low prio
// otos::getPosVelAccelStdDev,    // 5 // low prio
// otos::getPosVelAccelAndStdDev, // 6 // low prio
// otos::getPose,                 // 7  // done
// otos::setPose,                 // 8  // done
// otos::getPositionStdDev,       // 9  // low prio
// otos::getVelocity,             // 10 // low prio
// otos::getVelocityStdDev,       // 11 // low prio
// otos::getAcceleration,         // 12 // low prio
// otos::getAccelerationStdDev,   // 13 // low prio
// otos::getLinearUnit,           // 14 // unnecessary
// otos::setLinearUnit,           // 15 // unnecessary
// otos::getAngularUnit,          // 16 // unnecessary
// otos::setAngularUnit,          // 17 // unnecessary
// otos::getLinearScalar,         // 18 // done
// otos::setLinearScalar,         // 19 // done
// otos::getAngularScalar,        // 20 // done
// otos::setAngularScalar,        // 21 // done
// otos::getSignalProcessConfig,  // 22 // low prio
// otos::setSignalProcessConfig,  // 23 // low prio
// otos::selfTest,                // 24 // done
// otos::calibrate,               // 25 // done
// otos::isCalibrated,            // 26 // done
// otos::getOffset,               // 27 // impossible until FW bug fixed
// otos::setOffset                // 28 // done

namespace otos {

//////////////////////////////////////
// constants
/////////////////
constexpr int READ_TIMEOUT = 5;

constexpr float kRadianToDegree = 180.0 / 3.14159;
constexpr float kDegreeToRadian = 3.14159 / 180.0;
constexpr float kMeterToInch = 39.3701;
constexpr float kInchToMeter = 1.0 / kMeterToInch;

constexpr float kMeterToInt16 = 32768.0 / 10.0;
constexpr float kInt16ToMeter = 1.0 / kMeterToInt16;
constexpr float kInt16ToInch = kInt16ToMeter * kMeterToInch;
constexpr float kInchToInt16 = 1.0 / kInt16ToInch;

constexpr float kRadToInt16 = 32768.0 / 3.14159;
constexpr float kInt16ToRad = 1.0 / kRadToInt16;
constexpr float kInt16ToDeg = kInt16ToRad * kRadianToDegree;
constexpr float kDegToInt16 = 1.0 / kInt16ToDeg;

//////////////////////////////////////
// util
/////////////////

static std::vector<uint8_t> write_and_receive(uint8_t id, const std::vector<uint8_t>& data, int timeout) noexcept {
    // prepare data
    std::vector<uint8_t> out;
    out.push_back(id);
    out.insert(out.end(), data.begin(), data.end());
    // write data
    copro::write(out);
    // wait for a response
    const int start = pros::millis();
    while (true) {
        try {
            auto raw = copro::read();
            std::vector<uint8_t> rtn;
            rtn.insert(rtn.end(), raw.begin() + 1, raw.end());
            return rtn;
        } catch (std::system_error& e) {
            if (e.code().value() == ENODATA) {
                if (pros::millis() - start > timeout) {
                    std::cout << e.code() << ", " << e.what() << std::endl;
                    errno = e.code().value();
                    return {};
                } else {
                    continue;
                }
            } else {
                std::cout << e.code() << ", " << e.what() << std::endl;
                errno = e.code().value();
                return {};
            }
        }
    }
}

template <typename T> static std::vector<uint8_t> serialize(const T& data) {
    static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
    auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(data);
    std::vector<uint8_t> out(sizeof(T));
    for (int i = 0; i < sizeof(T); ++i) { out.at(i) = raw.at(i); }
    return out;
}

template <typename T, int N> static T deserialize(const std::vector<uint8_t>& data) {
    static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable");
    std::array<uint8_t, N> raw;
    for (int i = 0; i < N; ++i) { raw.at(i) = data.at(i); }
    return std::bit_cast<T>(raw);
}

//////////////////////////////////////
// OTOS
/////////////////

Status getStatus() noexcept {
    constexpr int ID = 1;

    union {
            struct {
                    uint8_t warn_tilt_angle : 1;
                    uint8_t warn_optical_tracking : 1;
                    uint8_t reserved : 4;
                    uint8_t optical_fatal : 1;
                    uint8_t imu_fatal : 1;
            };

            uint8_t value;
    } s;

    auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.empty()) {
        return {0, 0, 0, 0, 1};
    } else {
        s.value = raw.at(0);
        return {static_cast<bool>(s.warn_tilt_angle), static_cast<bool>(s.warn_optical_tracking),
                static_cast<bool>(s.optical_fatal), static_cast<bool>(s.imu_fatal), 0};
    }
}

int selfTest() noexcept {
    constexpr int ID = 24;
    const auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

int resetTracking() noexcept {
    constexpr int ID = 3;
    auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

//////////////////////////////////////
// pose
/////////////////

Pose get_pose() noexcept {
    constexpr int ID = 7;
    constexpr Pose ERROR = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::infinity()};

    // request, receive
    auto tmp = write_and_receive(ID, {}, READ_TIMEOUT);
    if (tmp.empty()) { return ERROR; }

    // error checking
    bool err = true;
    for (uint8_t b : tmp) {
        if (b != 1) { err = false; }
    }
    if (err) { return ERROR; }

    // parse raw data
    int16_t rawX = (tmp[1] << 8) | tmp[0];
    int16_t rawY = (tmp[3] << 8) | tmp[2];
    int16_t rawH = (tmp[5] << 8) | tmp[4];

    return {rawX * kInt16ToInch, rawY * kInt16ToInch, rawH * kInt16ToDeg};
}

int set_pose(Pose pose) noexcept {
    constexpr int ID = 8;
    // cast
    int16_t rawX = (pose.x * kInchToInt16);
    int16_t rawY = (pose.y * kInchToInt16);
    int16_t rawH = (pose.h * kDegToInt16);
    // init vector
    std::vector<uint8_t> out(6, 0);
    // serialize
    out[0] = rawX & 0xFF;
    out[1] = (rawX >> 8) & 0xFF;
    out[2] = rawY & 0xFF;
    out[3] = (rawY >> 8) & 0xFF;
    out[4] = rawH & 0xFF;
    out[5] = (rawH >> 8) & 0xFF;
    // write and get response
    auto raw = write_and_receive(ID, out, READ_TIMEOUT);
    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

//////////////////////////////////////
// offset
/////////////////

int set_offset(Pose pose) noexcept {
    constexpr int ID = 28;
    // cast
    int16_t rawX = (pose.x * kInchToInt16);
    int16_t rawY = (pose.y * kInchToInt16);
    int16_t rawH = (pose.h * kDegToInt16);
    // init vector
    std::vector<uint8_t> out(6, 0);
    // serialize
    out[0] = rawX & 0xFF;
    out[1] = (rawX >> 8) & 0xFF;
    out[2] = rawY & 0xFF;
    out[3] = (rawY >> 8) & 0xFF;
    out[4] = rawH & 0xFF;
    out[5] = (rawH >> 8) & 0xFF;
    // write and get response
    auto raw = write_and_receive(ID, out, READ_TIMEOUT);
    if (raw.empty()) {
        return PROS_ERR;
    } else {
        return static_cast<int>(raw.at(0));
    }
}

//////////////////////////////////////
// linear scalar
/////////////////

float get_linear_scalar() noexcept {
    constexpr int ID = 18;
    auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.empty()) {
        return std::numeric_limits<float>::infinity();
    } else {
        return 0.001f * static_cast<int8_t>(raw.at(0)) + 1.0f;
    }
}

int set_linear_scalar(float scalar) noexcept {
    constexpr int ID = 19;
    auto raw = static_cast<int8_t>((scalar - 1.0f) * 1000 + 0.5f);
    auto err = write_and_receive(ID, {raw}, READ_TIMEOUT);
    if (err.empty()) {
        return PROS_ERR;
    } else {
        return 0;
    }
}

//////////////////////////////////////
// angular scalar
/////////////////

float get_angular_scalar() noexcept {
    constexpr int ID = 20;
    auto raw = write_and_receive(ID, {}, READ_TIMEOUT);
    if (raw.empty()) {
        return std::numeric_limits<float>::infinity();
    } else {
        return 0.001f * static_cast<int8_t>(raw.at(0)) + 1.0f;
    }
}

int set_angular_scalar(float scalar) noexcept {
    constexpr int ID = 21;
    auto raw = static_cast<int8_t>((scalar - 1.0f) * 1000 + 0.5f);
    auto err = write_and_receive(ID, {raw}, READ_TIMEOUT);
    if (err.empty()) {
        return PROS_ERR;
    } else {
        return 0;
    }
}

//////////////////////////////////////
// calibrate
/////////////////

int calibrate(uint8_t samples) noexcept {
    constexpr int ID = 25;
    auto err = write_and_receive(ID, {samples}, READ_TIMEOUT);
    if (err.empty() || (err.at(0) != 0 && err.at(0) != 1)) { return PROS_ERR; }
    return err.at(0);
}

int isCalibrated() noexcept {
    constexpr int ID = 26;
    auto err = write_and_receive(ID, {}, READ_TIMEOUT);
    if (err.empty() || (err.at(0) != 0 && err.at(0) != 1)) { return PROS_ERR; }
    return err.at(0);
}

} // namespace otos
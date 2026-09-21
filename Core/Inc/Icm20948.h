/*
 * Icm20948.h
 *
 *  Created on: Sep 14, 2026
 *      Author: Tim Flannery
 */

#pragma once
#include "stm32f4xx_hal.h"

class Icm20948 {
public:
    explicit Icm20948(I2C_HandleTypeDef* hi2c);
    bool checkWhoAmI();
    void wake();
    void configureAccelGyro();
    bool readAccelGyro(float& axG, float& ayG, float& azG,
                        float& gxDps, float& gyDps, float& gzDps);
    bool enableMagnetometerBypass();
    bool configureMagnetometer();
    bool readMagnetometer(float& mxUt, float& myUt, float& mzUt);

private:
    I2C_HandleTypeDef* hi2c_;
    static constexpr uint16_t kI2cAddr = 0x68 << 1;

    // Bank 0
    static constexpr uint8_t kWhoAmIReg      = 0x00;
    static constexpr uint8_t kWhoAmIExpected = 0xEA;
    static constexpr uint8_t kPwrMgmt1Reg    = 0x06;
    static constexpr uint8_t kAccelXoutH     = 0x2D;   // start of a 12-byte accel+gyro burst
    static constexpr uint8_t kRegBankSel     = 0x7F;

    // Bank 2
    static constexpr uint8_t kGyroConfig1  = 0x01;
    static constexpr uint8_t kAccelConfig  = 0x14;

    // ICM-20948 Bank 0 registers, needed to enable bypass mode
    static constexpr uint8_t kUserCtrlReg  = 0x03;
    static constexpr uint8_t kIntPinCfgReg = 0x0F;

    // AK09916 magnetometer — a separate chip, its own register map and I2C address
    static constexpr uint16_t kMagI2cAddr    = 0x0C << 1;
    static constexpr uint8_t  kMagHxlReg     = 0x11;
    static constexpr uint8_t  kMagCntl2Reg   = 0x31;
    static constexpr float    kMagSensitivity = 0.15f;   // uT per count — fixed by the datasheet, not configurable


    void selectBank(uint8_t bank);

    float accelSensitivity_ = 16384.0f;  // LSB per g, for +/-2g
    float gyroSensitivity_  = 131.0f;    // LSB per deg/s, for +/-250 dps
};

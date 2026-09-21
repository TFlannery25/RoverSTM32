/*
 * Icm20948.cpp
 *
 *  Created on: Sep 14, 2026
 *      Author: Tim Flannery
 */


#include "Icm20948.h"

Icm20948::Icm20948(I2C_HandleTypeDef* hi2c) : hi2c_(hi2c) {}

bool Icm20948::checkWhoAmI() {
    uint8_t id = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hi2c_, kI2cAddr, kWhoAmIReg, I2C_MEMADD_SIZE_8BIT, &id, 1, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        return false;   // I2C transaction itself failed — wiring/address problem
    }
    return id == kWhoAmIExpected;  // transaction worked, but does the ID match?
}

void Icm20948::selectBank(uint8_t bank) {
    uint8_t value = static_cast<uint8_t>(bank << 4);   // bank number sits in bits [5:4]
    HAL_I2C_Mem_Write(hi2c_, kI2cAddr, kRegBankSel, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

void Icm20948::wake() {
    selectBank(0);   // PWR_MGMT_1 lives in Bank 0
    uint8_t value = 0x01;   // clear SLEEP bit, auto-select clock source
    HAL_I2C_Mem_Write(hi2c_, kI2cAddr, kPwrMgmt1Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

void Icm20948::configureAccelGyro() {
    selectBank(2);   // range settings live in Bank 2

    uint8_t accelConfigValue = 0x00;   // FS_SEL = 00 -> +/-2g
    HAL_I2C_Mem_Write(hi2c_, kI2cAddr, kAccelConfig, I2C_MEMADD_SIZE_8BIT, &accelConfigValue, 1, HAL_MAX_DELAY);
    accelSensitivity_ = 16384.0f;

    uint8_t gyroConfigValue = 0x00;    // FS_SEL = 00 -> +/-250 dps
    HAL_I2C_Mem_Write(hi2c_, kI2cAddr, kGyroConfig1, I2C_MEMADD_SIZE_8BIT, &gyroConfigValue, 1, HAL_MAX_DELAY);
    gyroSensitivity_ = 131.0f;

    selectBank(0);   // leave it on Bank 0, since that's where the data registers are read from
}

bool Icm20948::readAccelGyro(float& axG, float& ayG, float& azG,
                              float& gxDps, float& gyDps, float& gzDps) {
    uint8_t raw[12] = {0};
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hi2c_, kI2cAddr, kAccelXoutH, I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return false;
    }

    auto toInt16 = [](uint8_t hi, uint8_t lo) -> int16_t {
        return static_cast<int16_t>((hi << 8) | lo);
    };

    int16_t axRaw = toInt16(raw[0], raw[1]);
    int16_t ayRaw = toInt16(raw[2], raw[3]);
    int16_t azRaw = toInt16(raw[4], raw[5]);
    int16_t gxRaw = toInt16(raw[6], raw[7]);
    int16_t gyRaw = toInt16(raw[8], raw[9]);
    int16_t gzRaw = toInt16(raw[10], raw[11]);

    axG = axRaw / accelSensitivity_;
    ayG = ayRaw / accelSensitivity_;
    azG = azRaw / accelSensitivity_;
    gxDps = gxRaw / gyroSensitivity_;
    gyDps = gyRaw / gyroSensitivity_;
    gzDps = gzRaw / gyroSensitivity_;
    return true;
}

bool Icm20948::enableMagnetometerBypass() {
    selectBank(0);   // both registers below are Bank 0 registers on the ICM-20948 side

    uint8_t userCtrl = 0x00;   // make sure the internal I2C master is off, so bypass mode is allowed to work
    if (HAL_I2C_Mem_Write(hi2c_, kI2cAddr, kUserCtrlReg, I2C_MEMADD_SIZE_8BIT, &userCtrl, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }

    uint8_t bypassValue = 0x02;   // BYPASS_EN bit
    return HAL_I2C_Mem_Write(hi2c_, kI2cAddr, kIntPinCfgReg, I2C_MEMADD_SIZE_8BIT, &bypassValue, 1, HAL_MAX_DELAY) == HAL_OK;
}

bool Icm20948::configureMagnetometer() {
    uint8_t mode = 0x08;   // continuous measurement, 100 Hz
    return HAL_I2C_Mem_Write(hi2c_, kMagI2cAddr, kMagCntl2Reg, I2C_MEMADD_SIZE_8BIT, &mode, 1, HAL_MAX_DELAY) == HAL_OK;
}

bool Icm20948::readMagnetometer(float& mxUt, float& myUt, float& mzUt) {
    uint8_t raw[8] = {0};   // HXL,HXH,HYL,HYH,HZL,HZH,TMPS,ST2
    if (HAL_I2C_Mem_Read(hi2c_, kMagI2cAddr, kMagHxlReg, I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    // raw[7] is ST2 — reading it is required by the sensor itself to release the next sample,
    // even though we don't need its value here.

    int16_t mxRaw = static_cast<int16_t>((raw[1] << 8) | raw[0]);   // note: low byte first
    int16_t myRaw = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    int16_t mzRaw = static_cast<int16_t>((raw[5] << 8) | raw[4]);

    mxUt = mxRaw * kMagSensitivity;
    myUt = myRaw * kMagSensitivity;
    mzUt = mzRaw * kMagSensitivity;
    return true;
}

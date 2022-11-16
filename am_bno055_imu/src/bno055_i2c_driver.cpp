/* bno055_i2c_driver.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 */

#include "imu_bno055/bno055_i2c_driver.h"
#include <iostream>
#include <fstream>

namespace imu_bno055 {

BNO055I2CDriver::BNO055I2CDriver(std::string device_, int address_) {
    device = device_;
    address = address_;
}

bool BNO055I2CDriver::reset() {
    int i = 0;

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // reset
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // wait for chip to come back online
    while(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(i++ > 500) {
            throw std::runtime_error("chip did not come back online within 5 seconds of reset");
            return false;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // normal power mode
    _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    bool use_calib_file = false;
    if (use_calib_file) {
        write_calib_data();
    }

    _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
    //_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_IMUPLUS);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    return true;
}

void BNO055I2CDriver::init() {

    file = open(device.c_str(), O_RDWR);

    if(ioctl(file, I2C_SLAVE, address) < 0) {
        throw std::runtime_error("i2c device open failed");
    }

    if(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        throw std::runtime_error("incorrect chip ID");
    }

    std::cerr << "rev ids:"
      << " accel:" << _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
      << " mag:" << _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
      << " gyro:" << _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
      << " sw:" << _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
      << " bl:" << _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR) << std::endl;

    if(!reset()) {
	    throw std::runtime_error("chip init failed");
    }
}

IMURecord BNO055I2CDriver::read() {
    IMURecord record;
    unsigned char c = 0;

    // can only read a length of 0x20 at a time, so do it in 2 reads
    // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
        throw std::runtime_error("read error");
    }
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x1B, (uint8_t*)&record + 0x20) != 0x1B) {
        throw std::runtime_error("read error");
    } // Changed by Dylan on 27-Oct-2021 0x13 -> 0x1B in order to read up to register 0x42

    return record;
}

IMUCalibParam BNO055I2CDriver::read_calib_data() {
    IMUCalibParam calib_params;

    // std::cout << "Reading calibration data!\n";

    //_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    //std::this_thread::sleep_for(std::chrono::milliseconds(25));
	
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_SIC_MATRIX_0_LSB_ADDR, 0x20, (uint8_t*)&calib_params) != 0x20) {
        throw std::runtime_error("read error");
    }
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_SIC_MATRIX_0_LSB_ADDR + 0x20, 0x08, (uint8_t*)&calib_params + 0x20) != 0x08) {
        throw std::runtime_error("read error");
    }
	
    //_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
    //_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_IMUPLUS);
    //std::this_thread::sleep_for(std::chrono::milliseconds(25));
}


bool BNO055I2CDriver::write_calib_data() {
    IMUCalibParam calib_params;

    std::cout << "Writing calibration data to sensor!\n";

    // Read calibration parameters from file
    calib_params = read_calib_params_from_file();
    
    // Write calibration parameters to the IMU's registers
    _i2c_smbus_write_i2c_block_data(file, BNO055_SIC_MATRIX_0_LSB_ADDR, 0x20, (uint8_t*)&calib_params);
    _i2c_smbus_write_i2c_block_data(file, BNO055_SIC_MATRIX_0_LSB_ADDR + 0x20, 0x08, (uint8_t*)&calib_params + 0x20);

    return true;
}


IMUCalibParam BNO055I2CDriver::read_calib_params_from_file() {

    std::cout << "Reading calibration params from file!\n";

    IMUCalibParam calib_params;

    std::ifstream infile("/root/catkin_ws/calib_params_outdoor.txt", std::ifstream::binary);
    char bufferRead[sizeof(IMUCalibParam)];
    infile.read(bufferRead, sizeof(bufferRead));

    calib_params = *reinterpret_cast<IMUCalibParam*>(bufferRead);

    infile.close();

    return calib_params;
}


bool BNO055I2CDriver::write_calib_params_to_file(IMUCalibParam calib_params){
    std::cout << "Writing calibration params to file!\n";

    char* bufferOut = (char*)&calib_params;
    std::ofstream outfile("/root/catkin_ws/calib_params_outdoor.txt", std::ofstream::binary);

    if (outfile.is_open())
        outfile.write(bufferOut, sizeof(calib_params));
    else
        std::cout << "Out file is not open!\n";

    outfile.close();

    return true;
}

}

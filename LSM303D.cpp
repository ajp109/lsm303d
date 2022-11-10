#include "LSM303D.h"

static unsigned char const accel_lookup[] = {2, 4, 6, 8, 16};
static unsigned char const mag_lookup[] = {2, 4, 8, 12};

LSM303D::LSM303D(PinName sda, PinName scl, unsigned char i2c_addr) :
    i2c_(sda, scl), addr_8bit_(i2c_addr << 1),
    TEMP_OUT    (*this, 0x05),
    STATUS_M    (*this, 0x07),
    OUT_X_M     (*this, 0x08),
    OUT_Y_M     (*this, 0x0A),
    OUT_Z_M     (*this, 0x0C),
    OUT_M       (*this, 0x08),
    WHO_AM_I    (*this, 0x0F),
    INT_CTRL_M  (*this, 0x12),
    INT_SRC_M   (*this, 0x13),
    INT_THS_M   (*this, 0x14),
    OFFSET_X_M  (*this, 0x16),
    OFFSET_Y_M  (*this, 0x18),
    OFFSET_Z_M  (*this, 0x1A),
    OFFSET_M    (*this, 0x16),
    REFERENCE_X (*this, 0x1C),
    REFERENCE_Y (*this, 0x1D),
    REFERENCE_Z (*this, 0x1E),
    REFERENCE   (*this, 0x1C),
    CTRL0       (*this, 0x1F),
    CTRL1       (*this, 0x20),
    CTRL2       (*this, 0x21),
    CTRL3       (*this, 0x22),
    CTRL4       (*this, 0x23),
    CTRL5       (*this, 0x24),
    CTRL6       (*this, 0x25),
    CTRL7       (*this, 0x26),
    STATUS_A    (*this, 0x27),
    OUT_X_A     (*this, 0x28),
    OUT_Y_A     (*this, 0x2A),
    OUT_Z_A     (*this, 0x2C),
    OUT_A       (*this, 0x28),
    FIFO_CTRL   (*this, 0x2E),
    FIFO_SRC    (*this, 0x2F),
    IG_CFG1     (*this, 0x30),
    IG_SRC1     (*this, 0x31),
    IG_THS1     (*this, 0x32),
    IG_DUR1     (*this, 0x33),
    IG_CFG2     (*this, 0x34),
    IG_SRC2     (*this, 0x35),
    IG_THS2     (*this, 0x36),
    IG_DUR2     (*this, 0x37),
    CLICK_CFG   (*this, 0x38),
    CLICK_SRC   (*this, 0x39),
    CLICK_THS   (*this, 0x3A),
    TIME_LIMIT  (*this, 0x3B),
    TIME_LATENCY(*this, 0x3C),
    TIME_WINDOW (*this, 0x3D),
    Act_THS     (*this, 0x3E),
    Act_DUR     (*this, 0x3F),

    accel_enableX   (CTRL1, 0x01),
    accel_enableY   (CTRL1, 0x02),
    accel_enableZ   (CTRL1, 0x04),
    accel_enableAll (CTRL1, 0x07),

    INT1_enable_BOOT     (CTRL3, 0x80),
    INT1_enable_CLICK    (CTRL3, 0x40),
    INT1_enable_IG1      (CTRL3, 0x20),
    INT1_enable_IG2      (CTRL3, 0x10),
    INT1_enable_IGM      (CTRL3, 0x08),
    INT1_enable_DRDY_A   (CTRL3, 0x04),
    INT1_enable_DRDY_M   (CTRL3, 0x02),
    INT1_enable_EMPTY    (CTRL3, 0x01),

    temp_enable         (CTRL5, 0x80),
    LIR1_enable         (CTRL5, 0x01)

 {
    // Check device responds correctly
    MBED_ASSERT(WHO_AM_I == 0x49);

    // Grab current accelerometer and magnetometer scales from registers
    accel_scale_ = accel_lookup[(CTRL2 & 0x3E) >> 3];
    mag_scale_ = mag_lookup[(CTRL6 & 0x60) >> 5];
    
    // Useful default config
    accel_enableAll();
    setAccelRate(AccelRate::rate_50Hz);
    temp_enable();
    setMagRate(MagRate::rate_50Hz);
    setMagMode(MagMode::continuous);
}

void LSM303D::getRawAccelInto(short &x, short &y, short &z) {
    i16_3 xyz = OUT_A;
    x = xyz.x;
    y = xyz.y;
    z = xyz.z;
}

void LSM303D::getAccelInto(float &x, float &y, float &z) {
    i16_3 xyz = OUT_A;
    float scale = accel_scale_ / 32768.0;
    x = xyz.x * scale;
    y = xyz.y * scale;
    z = xyz.z * scale;
}

void LSM303D::getRawMagInto(short &x, short &y, short &z) {
    i16_3 xyz = OUT_M;
    x = xyz.x;
    y = xyz.y;
    z = xyz.z;
}

void LSM303D::getMagInto(float &x, float &y, float &z) {
    i16_3 xyz = OUT_M;
    float scale = mag_scale_ / 32768.0;
    x = xyz.x * scale;
    y = xyz.y * scale;
    z = xyz.z * scale;        
}

void LSM303D::setAccelRate(AccelRate const &rate) {
    u8 set = static_cast<u8>(rate);
    CTRL1 = (CTRL1 & ~0xF0) | set;
}

void LSM303D::setAccelFilterFreq(AccelFilter const &freq) {
    u8 set = static_cast<u8>(freq);
    CTRL2 = (CTRL2 & ~0xC0) | set;
}

void LSM303D::setAccelScale(AccelScale const &scale) {
    u8 set = static_cast<u8>(scale);
    CTRL2 = (CTRL2 & ~0x38) | set;
    accel_scale_ = accel_lookup[set >> 3];
}

void LSM303D::setMagRes(MagRes const &res) {
    u8 set = static_cast<u8>(res);
    CTRL5 = (CTRL5 & ~0x60) | set;
}

void LSM303D::setMagRate(MagRate const &rate) {
    u8 set = static_cast<u8>(rate);
    CTRL5 = (CTRL5 & ~0x1C) | set;
}

void LSM303D::setMagScale(MagScale const &scale) {
    u8 set = static_cast<u8>(scale);
    CTRL6 = (CTRL6 & ~0x60) | set;
    mag_scale_ = mag_lookup[set >> 5];
}

void LSM303D::setMagMode(MagMode const &mode) {
    u8 set = static_cast<u8>(mode);
    CTRL7 = (CTRL7 & ~0x03) | set;
}

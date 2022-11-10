#include "mbed.h"

typedef unsigned char u8;
typedef signed short i16;
typedef unsigned short u16;

#pragma pack(push)
#pragma pack(1)
typedef struct {
    u8 x;
    u8 y;
    u8 z;
} u8_3;
typedef struct {
    i16 x;
    i16 y;
    i16 z;
} i16_3;
#pragma pack(pop)

/** LSM303D class.
 * Communicates with LSM303D electronic compass over I<sup>2</sup>C.  Accelerometer,
 * magnetometer and temperature sensor supported.  Configurable sample rates
 * and full-scale ranges.  All LSM303D config registers available for raw
 * reads and writes if needed.
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "lsm303d.h"
 *
 * LSM303D lsm303d(SDA, SCL);
 * lsm303d.setAccelRate(LSM303D::AccelRate::rate_25Hz);
 * lsm303d.setAccelScale(LSM303D::AccelScale::scale_8g);
 * 
 * int main() {
 *     short x, y, z;
 *     lsm303d.getRawAccelInto(x, y, z); // 16-bit signed, unscaled
 *     float xf, yf, zf;
 *     lsm303d.getAccelInto(xf, yf, zf); // floats, in g
 * }
 * @endcode
 */
class LSM303D {
    
    friend class AccelScaleOption;
    friend class MagScaleOption;
    
    unsigned char const addr_8bit_;
    unsigned char accel_scale_;
    unsigned char mag_scale_;
    I2C i2c_;

protected:
    // These classes are protected (rather than private) for inlusion in docs.
    // Wouldn't be necessary if mbed online toolchain allowed local Doxyfiles.

    template <class T>
    class RWRegister;

    /** Read-only register class.
     * @tparam T The underlying data type of the register.
     * Provides an interface onto the read-only registers of the LSM303D.
     * Operator overloading means that it can be treated as a const variable of
     * type T for most purposes.  Every read initiates an explicit
     * I<sup>2</sup>C transfer.
     */
    template <class T>
    class Register {
        friend class LSM303D;
        friend class RWRegister<T>;
    private:
        unsigned char addr_;
        LSM303D &parent_;
        Register(LSM303D &parent, unsigned char addr) :
            parent_(parent),
            addr_(addr | (sizeof(T)>1 ? 0x80 : 0x00)) {}
        /// Explicit read method.
        /// \return The current value of the register.
        virtual T read() const {
            T result;
            parent_.i2c_.lock();
            parent_.i2c_.write(parent_.addr_8bit_, (char *)&addr_, 1);
            parent_.i2c_.read(parent_.addr_8bit_, (char *)&result, sizeof(T));
            parent_.i2c_.unlock();
            return result;
        }
        /// Explicit cast to type T
        inline operator T() const {
            return read();
        }
    };
    
    /** Read-write register class.
     * @tparam T The underlying data type of the register.
     * Provides an interface onto the writable registers of the LSM303D.
     * Operator overloading means that it can be treated as a variable of type
     * T for most purposes.  Assignment from int is also explicitly supported
     * to allow integer literals to be used without qualification or casting.
     * The register value is cached on first read and updated on write, so read
     * accesses do not use the I<sup>2</sup>C bus.
     */
    template<class T>
    class RWRegister : public Register<T> {
        friend class LSM303D;
        using Register<T>::parent_;
        using Register<T>::addr_;
    private:
        mutable T cache_;
        mutable bool cached_;
    protected:
        RWRegister(LSM303D &parent, unsigned char addr) :
            Register<T>(parent, addr), cached_(false) {}
        /// Explicit write method.
        /// @param data The new value to write to the register.
        void write(T const &data) {
            char packet[sizeof(T)+1];
            packet[0] = addr_;
            memcpy(&packet[1], &data, sizeof(T));
            cache_ = data;
            cached_ = true;
            parent_.i2c_.lock();
            parent_.i2c_.write(parent_.addr_8bit_, packet, sizeof(T)+1);
            parent_.i2c_.unlock();
        }
        /// Explicit read method with cache.
        /// @return The current value of the register, from cache if available.
        virtual T read() const {
            if (!cached_) {
                cache_ = Register<T>::read();
                cached_ = true;
            }
            return cache_;
        }
        /// Equivalent to write().
        /// @param data The new value to write to the register.
        /// @return A reference to this object.
        inline RWRegister & operator= (T const &data) {
            write(data);
            return *this;
        }
        /// Equivalent to write(), with an explicit int parameter.
        /// @param data The new value to write to the register.
        /// @return A reference to this object.
        inline RWRegister & operator= (int const &data) {
            write(T(data));
            return *this;
        }
        /// Or-assignment operator.
        /// @param data The value to OR with the register's existing contents.
        /// @return A reference to this object.
        inline RWRegister & operator|= (T const &data) {
            write(read() | data);
            return *this;
        }
        /// Or-assignment operator, with an explicit int parameter.
        /// @param data The value to OR with the register's existing contents.
        /// @return A reference to this object.
        inline RWRegister & operator|= (int const &data) {
            write(read() | T(data));
            return *this;
        }
        /// And-assignment operator.
        /// @param data The value to AND with the register's existing contents.
        /// @return A reference to this object.
        inline RWRegister & operator&= (T const &data) {
            write(read() & data);
            return *this;
        }
        /// And-assignment operator, with an explicit int parameter.
        /// @param data The value to AND with the register's existing contents.
        /// @return A reference to this object.
        inline RWRegister & operator&= (int const &data) {
            write(read() & T(data));
            return *this;
        }
    };

    /** Class representing binary (on/off) options.
     * @tparam T The underlying data type of the register used to set the option.
     * Provides a user-friendly interface onto LSM303D options.
     */
    template <class T>
    class Option {
        friend class LSM303D;
    protected:
        T const bits_;
        RWRegister<T> & reg_;
        Option(RWRegister<T> & reg, int bits) :
            reg_(reg), bits_(T(bits)) {}
    public:
        /// Set the option 'on'.
        inline void operator() () const { reg_ |= bits_; };
        /// Set the option 'on' or 'off'.
        /// @param sr Option status: set to true for 'on', false for 'off'.
        inline void operator() (bool sr) const
            { if (sr) reg_ |= bits_; else reg_ &= ~bits_; };
    };
        
    const Register   <i16_3> OUT_M;         ///< Triple magnetometer register
    const Register   <i16_3> OFFSET_M;      ///< Triple magnetometer offset register
    const Register   <u8_3>  REFERENCE;     ///< Triple reference register
    const Register   <i16_3> OUT_A;         ///< Triple accelerometer register

public:
    const Register   <i16>   TEMP_OUT;      ///< TEMP_OUT register, read-only
    const Register   <u8>    STATUS_M;      ///< STATUS_M register, read-only
    const Register   <i16>   OUT_X_M;       ///< OUT_X_M register, read-only
    const Register   <i16>   OUT_Y_M;       ///< OUT_Y_M register, read-only
    const Register   <i16>   OUT_Z_M;       ///< OUT_Z_M register, read-only
    const Register   <u8>    WHO_AM_I;      ///< WHO_AM_I register, read-only
    const Register   <u8>    INT_SRC_M;     ///< INT_SRC_M register, read-only
    const Register   <i16>   OFFSET_X_M;    ///< OFFSET_X_M register, read-only
    const Register   <i16>   OFFSET_Y_M;    ///< OFFSET_Y_M register, read-only
    const Register   <i16>   OFFSET_Z_M;    ///< OFFSET_Z_M register, read-only
    const Register   <u8>    STATUS_A;      ///< STATUS_A register, read-only
    const Register   <i16>   OUT_X_A;       ///< OUT_X_A register, read-only
    const Register   <i16>   OUT_Y_A;       ///< OUT_Y_A register, read-only
    const Register   <i16>   OUT_Z_A;       ///< OUT_Z_A register, read-only
    const Register   <u8>    FIFO_SRC;      ///< FIFO_SRC register, read-only
    const Register   <u8>    IG_SRC1;       ///< IG_SRC1 register, read-only
    const Register   <u8>    IG_SRC2;       ///< IG_SRC2 register, read-only
    const Register   <u8>    CLICK_SRC;     ///< CLICK_SRC register, read-only

    RWRegister <u8>    INT_CTRL_M;  ///< INT_CTRL_M register, read-write
    RWRegister <u16>   INT_THS_M;   ///< INT_THS_M register, read-write
    RWRegister <u8>    REFERENCE_X; ///< REFERENCE_X register, read-write
    RWRegister <u8>    REFERENCE_Y; ///< REFERENCE_Y register, read-write
    RWRegister <u8>    REFERENCE_Z; ///< REFERENCE_Z register, read-write
    RWRegister <u8>    CTRL0;       ///< CTRL0 register, read-write
    RWRegister <u8>    CTRL1;       ///< CTRL1 register, read-write
    RWRegister <u8>    CTRL2;       ///< CTRL2 register, read-write
    RWRegister <u8>    CTRL3;       ///< CTRL3 register, read-write
    RWRegister <u8>    CTRL4;       ///< CTRL4 register, read-write
    RWRegister <u8>    CTRL5;       ///< CTRL5 register, read-write
    RWRegister <u8>    CTRL6;       ///< CTRL6 register, read-write
    RWRegister <u8>    CTRL7;       ///< CTRL7 register, read-write
    RWRegister <u8>    FIFO_CTRL;   ///< FIFO_CTRL register, read-write
    RWRegister <u8>    IG_CFG1;     ///< IG_CFG1 register, read-write
    RWRegister <u8>    IG_THS1;     ///< IG_THS1 register, read-write
    RWRegister <u8>    IG_DUR1;     ///< IG_DUR1 register, read-write
    RWRegister <u8>    IG_CFG2;     ///< IG_CFG2 register, read-write
    RWRegister <u8>    IG_THS2;     ///< IG_THS2 register, read-write
    RWRegister <u8>    IG_DUR2;     ///< IG_DUR2 register, read-write
    RWRegister <u8>    CLICK_CFG;   ///< CLICK_CFG register, read-write
    RWRegister <u8>    CLICK_THS;   ///< CLICK_THS register, read-write
    RWRegister <u8>    TIME_LIMIT;  ///< TIME_LIMIT register, read-write
    RWRegister <u8>    TIME_LATENCY;///< TIME_LATENCY register, read-write
    RWRegister <u8>    TIME_WINDOW; ///< TIME_WINDOW register, read-write
    RWRegister <u8>    Act_THS;     ///< Act_THS register, read-write
    RWRegister <u8>    Act_DUR;     ///< Act_DUR register, read-write
    
    // CTRL1
    /** AccelRate enum.
     * Provides friendly names for the accelerometer update rates
     * that are provided by the device.
     */
    enum class AccelRate {
        powerdown    = 0x00,    ///< power-down mode
        rate_3_125Hz = 0x10,    ///< 3.125Hz update rate
        rate_6_25Hz  = 0x20,    ///< 6.25Hz update rate
        rate_12_5Hz  = 0x30,    ///< 12.5Hz update rate
        rate_25Hz    = 0x40,    ///< 25Hz update rate
        rate_50Hz    = 0x50,    ///< 50Hz update rate
        rate_100Hz   = 0x60,    ///< 100Hz update rate
        rate_200Hz   = 0x70,    ///< 200Hz update rate
        rate_400Hz   = 0x80,    ///< 400Hz update rate
        rate_800Hz   = 0x90,    ///< 800Hz update rate
        rate_1600Hz  = 0xA0     ///< 1600Hz update rate
    };
    /** Sets the accelerometer update rate.
     * @param rate The new update rate.
     * Available values are:
     * - AccelRate::powerdown (power-down mode)
     * - AccelRate::rate_3_125Hz (3.125Hz)
     * - AccelRate::rate_6_25Hz (6.25Hz)
     * - AccelRate::rate_12_5Hz (12.5Hz)
     * - AccelRate::rate_25Hz (25Hz)
     * - AccelRate::rate_50Hz (50Hz)
     * - AccelRate::rate_100Hz (100Hz)
     * - AccelRate::rate_200Hz (200Hz)
     * - AccelRate::rate_400Hz (400Hz)
     * - AccelRate::rate_800Hz (800Hz)
     * - AccelRate::rate_1600Hz (1600Hz)
     */
    void setAccelRate(AccelRate const &rate);

    
    const Option<u8> accel_enableX;     ///< Enable or disable x-axis accelerometer
    const Option<u8> accel_enableY;     ///< Enable or disable y-axis accelerometer
    const Option<u8> accel_enableZ;     ///< Enable or disable z-axis accelerometer
    const Option<u8> accel_enableAll;   ///< Enable or disable all accelerometers

    // CTRL2
    /** AccelFilter enum.
     * Provides friendly names for the accelerometer antialiasing filter
     * frequencies that are provided by the device.
     */
    enum class AccelFilter {
        filter_773Hz = 0x00,    ///< 773Hz corner frequency
        filter_194Hz = 0x40,    ///< 194Hz corner frequency
        filter_362Hz = 0x80,    ///< 362Hz corner frequency
        filter_50Hz  = 0xC0     ///< 50Hz corner frequency
    };
    /** Sets the accelerometer antialiasing filter frequency.
     * @param freq The new frequency.
     * Available values are:
     * - AccelFilter::filter_773Hz
     * - AccelFilter::filter_194Hz
     * - AccelFilter::filter_362Hz
     * - AccelFilter::filter_50Hz
     */
    void setAccelFilterFreq(AccelFilter const &freq);
    /** AccelScale enum.
     * Provides friendly names for the accelerometer scale ranges
     * that are provided by the device.
     */
    enum class AccelScale {
        scale_2g  = 0x00,   ///< +-2g full scale
        scale_4g  = 0x08,   ///< +-4g full scale
        scale_6g  = 0x10,   ///< +-6g full scale
        scale_8g  = 0x18,   ///< +-8g full scale
        scale_16g = 0x20    ///< +-16g full scale
    };
    /** Sets the accelerometer scale range.
     * @param scale The new scale.
     * Available values are:
     * - AccelScale::scale_2g (+-2g)
     * - AccelScale::scale_4g (+-4g)
     * - AccelScale::scale_6g (+-6g)
     * - AccelScale::scale_8g (+-8g)
     * - AccelScale::scale_16g (+-16g)
     */
    void setAccelScale(AccelScale const &scale);

    // CTRL3
    const Option<u8> INT1_enable_BOOT;  ///< Enable or disable "boot" source for INT1
    const Option<u8> INT1_enable_CLICK; ///< Enable or disable "click" source for INT1
    const Option<u8> INT1_enable_IG1;   ///< Enable or disable "inertial 1" source for INT1
    const Option<u8> INT1_enable_IG2;   ///< Enable or disable "inertial 2" source for INT1
    const Option<u8> INT1_enable_IGM;   ///< Enable or disable "magnetic" source for INT1
    const Option<u8> INT1_enable_DRDY_A;///< Enable or disable "accelerometer data ready" source for INT1
    const Option<u8> INT1_enable_DRDY_M;///< Enable or disable "magnetometer data ready" source for INT1
    const Option<u8> INT1_enable_EMPTY; ///< Enable or disable "FIFO empty" source for INT1
    
    // CTRL4 concerns INT2 which is not broken out on the Pimoroni board
    
    // CTRL5
    const Option<u8> temp_enable;   ///< Enable or disable temperature sensor
    const Option<u8> LIR1_enable;   ///< Enable or disable latching interrupt requests for INT1
    
    /** MagRes enum.
     * Provides friendly names for the magnetometer resolution options
     * that are provided by the device.
     */
    enum class MagRes {
        high = 0x60,    ///< High resolution
        low  = 0x00,    ///< Low resolution
    };
    /** Sets the magnetometer resolution.
     * @param res The new resolution.
     * Available values are:
     * - MagRes::high (high resolution)
     * - MagRes::low (low resolution)
     */
    void setMagRes(MagRes const &res);

    /** MagRate enum.
     * Provides friendly names for the magnetometer update rates
     * that are provided by the device.
     */
    enum class MagRate {
        rate_3_125Hz = 0x00,    ///< 3.125Hz update rate
        rate_6_25Hz  = 0x04,    ///< 6.25Hz update rate
        rate_12_5Hz  = 0x08,    ///< 12.5Hz update rate
        rate_25Hz    = 0x0C,    ///< 25Hz update rate
        rate_50Hz    = 0x10,    ///< 50Hz update rate
        rate_100Hz   = 0x14     ///< 100Hz update rate
    };    
    /** Sets the magnetometer update rate.
     * @param rate The new update rate.
     * Available values are:
     * - MagRate::rate_3_125Hz (3.125Hz)
     * - MagRate::rate_6_25Hz (6.25Hz)
     * - MagRate::rate_12_5Hz (12.5Hz)
     * - MagRate::rate_25Hz (25Hz)
     * - MagRate::rate_50Hz (50Hz)
     * - MagRate::rate_100Hz (100Hz)
     * Note that the 100Hz update rate is available only if the accelerometer
     * update rate is set to 50Hz or higher or the accelerometer is set to
     * power-down mode (see setAccelRate()).  Failing to meet these conditions
     * will lead to undefined behaviour.
     */
    void setMagRate(MagRate const &rate);

    // CTRL6
    /** MagScale enum.
     * Provides friendly names for the magnetometer scale ranges
     * that are provided by the device.
     */
    enum class MagScale {
        scale_2G  = 0x00,   ///< +-2 Gauss
        scale_4G  = 0x20,   ///< +-4 Gauss
        scale_8G  = 0x40,   ///< +-8 Gauss
        scale_12G = 0x60    ///< +-12 Gauss
    };
    /** Sets the magnetometer scale range.
     * @param scale The new scale.
     * Available values are:
     * - MagScale::scale_2G (+-2 Gauss)
     * - MagScale::scale_4G (+-4 Gauss)
     * - MagScale::scale_8G (+-8 Gauss)
     * - MagScale::scale_12G (+-16 Gauss)
     */
    void setMagScale(MagScale const &scale);

    // CTRL7
    /** MagScale enum.
     * Provides friendly names for the magnetometer modes that are supported
     * by the device.
     */
    enum class MagMode {
        continuous  = 0x00, ///< Continous conversion mode
        single      = 0x01, ///< Single conversion mode
        powerdown   = 0x02  ///< Magnetometer power-down mode
    };
    /** Sets the magnetometer mode.
     * @param mode The new mode.
     * Available values are:
     * - MagMode::continuous (continuous conversion mode)
     * - MagMode::single (single-conversion mode)
     * - MagMode::powerdown (power-down mode)
     */
    void setMagMode(MagMode const &mode);

    /** Constructor.
     * @param sda The pin to use for the I^^2^^C SDA signal.
     * @param scl The pin to use for the I^^2^^C SCL signal.
     * @param i2c_addr The 7-bit I<sup>2</sup>C address.  Defaults to 0x1D, but
     * this can be overridden to 0x1E with a jumper setting on the device allowing
     * two LSM303D devices to coexist on a bus.
     */
    LSM303D(PinName sda, PinName scl, unsigned char i2c_addr = 0x1D);
    
    /** Gets the raw (signed 16-bit integer) X-axis acceleration value.
     * @return The raw acceleration value.
     */
    inline i16 getRawAccelX() { return OUT_X_A; };
    /** Gets the raw (signed 16-bit integer) Y-axis acceleration value.
     * @return The raw acceleration value.
     */
    inline i16 getRawAccelY() { return OUT_Y_A; };
    /** Gets the raw (signed 16-bit integer) Z-axis acceleration value.
     * @return The raw acceleration value.
     */
    inline i16 getRawAccelZ() { return OUT_Z_A; };
    /** Gets the scaled (float) X-axis acceleration value in g.
     * @return The scaled acceleration value in g.
     */
    inline float getAccelX() { return (float)OUT_X_A / accel_scale_ / 32768; };
    /** Gets the scaled (float) Y-axis acceleration value in g.
     * @return The scaled acceleration value in g.
     */
    inline float getAccelY() { return (float)OUT_Y_A / accel_scale_ / 32768; };
    /** Gets the scaled (float) Z-axis acceleration value in g.
     * @return The scaled acceleration value in g.
     */
    inline float getAccelZ() { return (float)OUT_Z_A / accel_scale_ / 32768; };
    /** Reads the raw (signed 16-bit integer) acceleration values into the three
     * axis value containers provided.
     * @param x Container for the X value.
     * @param y Container for the Y value.
     * @param z Container for the Z value.
     */
    void getRawAccelInto(short &x, short &y, short &z);
    /** Reads the scaled (float) acceleration values in g into the three
     * axis value containers provided.
     * @param x Container for the X value.
     * @param y Container for the Y value.
     * @param z Container for the Z value.
     */
    void getAccelInto(float &x, float &y, float &z);
    
    /** Gets the raw (signed 16-bit integer) X-axis magnetometer value.
     * @return The raw magnetometer value.
     */
    inline i16 getRawMagX() { return OUT_X_M; };
    /** Gets the raw (signed 16-bit integer) Y-axis magnetometer value.
     * @return The raw magnetometer value.
     */
    inline i16 getRawMagY() { return OUT_Y_M; };
    /** Gets the raw (signed 16-bit integer) Z-axis magnetometer value.
     * @return The raw magnetometer value.
     */
    inline i16 getRawMagZ() { return OUT_Z_M; };
    /** Gets the scaled (float) X-axis magnetometer value in Gauss.
     * @return The scaled magnetometer value in Gauss.
     */
    inline float getMagX() { return (float)OUT_X_M / mag_scale_ / 32768; };
    /** Gets the scaled (float) Y-axis magnetometer value in Gauss.
     * @return The scaled magnetometer value in Gauss.
     */
    inline float getMagY() { return (float)OUT_Y_M / mag_scale_ / 32768; };
    /** Gets the scaled (float) Z-axis magnetometer value in Gauss.
     * @return The scaled magnetometer value in Gauss.
     */
    inline float getMagZ() { return (float)OUT_Z_M / mag_scale_ / 32768; };
    /** Reads the raw (signed 16-bit integer) magnetometer values into the three
     * axis value containers provided.
     * @param x Container for the X value.
     * @param y Container for the Y value.
     * @param z Container for the Z value.
     */
    void getRawMagInto(short &x, short &y, short &z);
    /** Reads the scaled (float) magnetometer values in Gauss into the three
     * axis value containers provided.
     * @param x Container for the X value.
     * @param y Container for the Y value.
     * @param z Container for the Z value.
     */
    void getMagInto(float &x, float &y, float &z);
    
    /** Reads the scaled (float) temperature values in &deg; C.
     * @return The current temperature.
     * Note that testing suggests this to be an inaccurate temperature sensor!
     * To read the raw temperature value, simply use the TEMP_OUT register directly.
     */
    inline float getTemp() { return TEMP_OUT / 8.0 + 25; };
    
};

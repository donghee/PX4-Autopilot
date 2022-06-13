#pragma once

#include <lib/cdev/CDev.hpp>

#include <drivers/device/spi.h>
#include <perf/perf_counter.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/encapsulated_data.h>
/**
 * \class DIM
 * \brief DIM SPI Driver for encryption or decryption using DIM Hardware
 *
 * provies random nubmer generator, encryption, decryption and character device like a (/dev/dim0)
 *
 */
class DIM : public device::SPI, public I2CSPIDriver<DIM>
{
public:

	/**
	 * DIM: KSE DIM SPI Driver constructor
	 *
	 * @param bus_option       SPI bus option
	 * @param bus       SPI bus
	 * @param device       device pin map
	 * @param bus_frequency       SPI bus frequency hz
	 * @param spi_mode       SPI mode
	 * @param drdy_gpio       SPI data ready pin
	 * @return          DIM object
	 */
	DIM(I2CSPIBusOption bus_option, int bus, int32_t device, int bus_frequency,
	    spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);

	/**
	 * DIM: KSE DIM SPI Driver deconstructor
	 *
	 */
	virtual ~DIM();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	/**
	 * SPI bus initialize and make device file. (like a "/dev/dim0")
	 */
	int		init();

	/**
	 * current SPI Bus status
	 * @return
	 */
	void			print_status() override;

	/**
	 * Module interface. add a execute or measure fuction of periodic execution in RunImpl()
	 * @return          success(0) or fail(-1) of file exit
	 */
	void			RunImpl();

	/**
	 * random number generator using DIM
	 * @param pbRandom       output of random number generator
	 * @param usRandomSize       input random number size
	 * @return          success(0) or fail(-1) of random number
	 */
	int16_t _kcmvpDrbg(uint8_t *pbRandom, uint16_t usRandomSize);

	/**
	 * encrypt or decrypt using DIM
	 * @param[out] pbOutput       output data
	 * @param[in] pbInput        input data
	 * @param[in] usInputSize        input data size
	 * @param bKeyType        key type: if bAlg is KCMVP_AES <KCMVP_AES_128_KEY('40'), KCMVP_AES_128_KEY('41'), if bAlg is KCMVP_ARIA <KCMVP_AES_128_KEY('42')> or KCMVP_ARIA128_KEY('50'), KCMVP_ARIA192_KEY('51') KCMVP_ARIA256_KEY('52') >
	 * @param usKeyIndex        encryption key index stored on DIM (0~7)
	 * @param pbIv        Iv Buffer
	 * @param usIvSize    size of Iv buffer
	 * @param pbAuth      auth buffer
	 * @param usAuthSize  size of Auth buffer
	 * @param pbTag       tag buffer
	 * @param usTagSize   size of tag buffer
	 * @param bEnDe       < ENCRYPT(0) or DECRYPT(1) >
	 * @param bAlg        < KCMVP_AES('40') or KCMVP_ARIA('50') >
	 * @return          success(0) or fail(-1) of encryption or decryption
	 */
	int16_t _kcmvpGcm(uint8_t *pbOutput, uint8_t *pbInput, uint16_t usInputSize,
			  uint8_t bKeyType, uint16_t usKeyIndex, uint8_t *pbIv,
			  uint16_t usIvSize, uint8_t *pbAuth, uint16_t usAuthSize,
			  uint8_t *pbTag, uint16_t usTagSize, uint8_t bEnDe,
			  uint8_t bAlg);

	int16_t	_kcmvpGetKey(uint8_t *pbKey, uint16_t *pusKeySize, uint8_t bKeyType,
			     uint16_t usKeyIndex);

	/**
	 * DIM Driver daemon command interface
	 * @param cli       argument of command
	 * @return
	 */
	void custom_method(const BusCLIArguments &cli) override;

	/**
	 * open file operator for DIM Driver file interface ("/dev/dim0")
	 * @param filep       file path to open
	 * @return          success(0) or fail(-1) of file open
	 */
	int open(struct file *filep);

	/**
	 * ioctl file operator ("/dev/dim0")
	 * @param filp       file descriptor
	 * @param cmd       command (DIM_ENCRYPT, DIM_DECRYPT, DIM_MAVLINK_ENCRYPT, DIM_IS_POWER_ON)
	 * @param arg       argument of command
	 * @return          success(0) or fail(-1) of file ioctl
	 */
	int		ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

protected:
	int		probe() override;

	/**
	 * exit and DIM power off
	 * @return          success(0) or fail(-1) of exit
	 */
	void exit_and_cleanup() override;


private:
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;

	const spi_drdy_gpio_t _drdy_gpio;

	bool    is_power_on;

	// BITS SWAP
	typedef union { uint8_t a[2]; uint16_t b; } __bitcast_u16;

	static inline uint16_t le16(uint16_t x)
	{
		__bitcast_u16 y = {
			.a = { (uint8_t)(x), (uint8_t)(x >> 8) }
		};
		return y.b;
	}

	static inline uint16_t be16(uint16_t x)
	{
		__bitcast_u16 y = {
			.a = { (uint8_t)(x >> 8), (uint8_t)(x) }
		};
		return y.b;
	}

	// #if __BYTE_ORDER == __LITTLE_ENDIAN
	uint16_t bswap16(uint16_t x) { return be16(x); }
	// #endif

	// #if __BYTE_ORDER == __BIG_ENDIAN
	// uint16_t bswap16(uint16_t x) { return le16(x); }
	// #endif


	/**
	 * \brief data type of DIM hardware status
	 */
#pragma pack(push, 1) // Ensure proper memory alignment.
	struct DimPowerReport {
		uint8_t stx;
		uint8_t dir;
		uint8_t offset;
		uint8_t len;
		uint16_t rv;
		uint8_t propriety_data1[5];
		uint8_t version[3];
		uint8_t life_cycle;
		uint8_t serial_number[8];
		uint8_t pin_type;
		uint8_t max_pin_retry_count;
		uint16_t kcmvp_key_count;
		uint16_t cert_key_count;
		uint16_t io_data_size;
		uint16_t info_file_size;
		uint8_t padding[31];
	} dim_power_report{};
#pragma pack(pop)

	/**
	 * \brief data type of response of DIM hardware
	 */
#pragma pack(push, 1) // Ensure proper memory alignment.
	struct DimReport {
		uint8_t stx;
		uint8_t dir;
		uint8_t offset;
		uint8_t len;
		uint8_t data[60];
	} dim_report{};
#pragma pack(pop)

	/**
	 * \brief data typeof request command of DIM hardware
	 */
#pragma pack(push, 1) // Ensure proper memory alignment.
	struct Command {
		uint8_t stx;
		uint8_t dir;
		uint8_t offset;
		uint8_t len;
		union {
			uint16_t msgid;
			uint8_t msg[2];
		};
		uint8_t data[58];
	} dim_cmd{};
#pragma pack(pop)

	/**
	 * DIM Driver daemon start
	 * @return          success(0) or fail(-1) of start daemon
	 */
	void			start();

	/**
	 * DIM Driver daemon stop
	 * @return          success(0) or fail(-1) of stop daemon
	 */
	void            stop();

	/**
	 * KSE DIM Power On
	 * @return          success(0) or fail(-1) of power on
	 */
	int			power_on();

	/**
	 * KSE DIM Power off
	 * @return          success(0) or fail(-1) of power off
	 */
	int			power_off();

	/**
	 * Reset driver module, inherite from PX4 Driver Class
	*/
	int			reset();

	/**
	 * execute main loop
	 * @return          success(0) or fail(-1) of file exit
	 */
	int			measure();


	/**
	 * encrypt and decrypt test of DIM
	 * @return          success(0) or fail(-1) of test
	 */
	int        encrypt_self_test();

	/**
	 * encrypt and decrypt test on file using DIM
	 * @param file_name       file name where encrpted data store
	 * @param _plain_text       input data
	 * @param count       byte size of input data
	 * @return          success(0) or fail(-1) of test
	 */
	int        encrypt_test(const char *file_name, uint8_t *_plain_text, size_t count);

	int        decrypt_test(uint8_t *_encrypted_text, size_t count);

	int        encrypt_test2();

	int        getKey();

	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	uORB::Publication<debug_array_s>			_debug_array_pub {ORB_ID(debug_array)};
	uORB::Publication<encapsulated_data_s>			_encapsulated_data_pub {ORB_ID(encapsulated_data)};

	int			_class_instance;

	uint8_t buffer[512];

};

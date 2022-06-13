#include "DIM.hpp"
#include <math.h>

#define DIM_ENCRYPT 0
#define DIM_DECRYPT 1
#define DIM_MAVLINK_ENCRYPT 2
#define DIM_MAVLINK_DECRYPT 3
#define DIM_IS_POWER_ON 4

//// KSE DIM ///////////////////////////////////////////////////////////////////

#define MAX_KCMVP_KEY_COUNT     8
#define MAX_CERT_KEY_COUNT     96
#define MAX_IO_DATA_SIZE     2944
#define MAX_INFO_FILE_SIZE   8192

//// KSE ///////////////////////////////////////////////////////////////////////

#define KSE_TRUE   1
#define KSE_FALSE  0
#define NOT_USED   0
#define NULL_PTR   ((void *)0)

//// KCMVP /////////////////////////////////////////////////////////////////////
#define KCMVP_AES128_KEY     0x40
#define KCMVP_AES192_KEY     0x41
#define KCMVP_AES256_KEY     0x42
#define KCMVP_ARIA128_KEY    0x50
#define KCMVP_ARIA192_KEY    0x51
#define KCMVP_ARIA256_KEY    0x52
#define KCMVP_HMAC_KEY       0x70
#define KCMVP_ECDSA_KEYPAIR  0x80
#define KCMVP_ECDSA_PRI_KEY  0x81
#define KCMVP_ECDSA_PUB_KEY  0x82
#define KCMVP_ECDH_KEYPAIR   0x90
#define KCMVP_ECDH_PRI_KEY   0x91
#define KCMVP_ECDH_PUB_KEY   0x92

#define ENCRYPT  0
#define DECRYPT  1


// Stall time between SPI transfers
static constexpr uint32_t T_STALL = 250000; // TODO: Why this stall time is so big? check spec
// static constexpr uint32_t T_STALL = 400000; // TODO: Why this stall time is so big? check spec
static constexpr uint32_t DIM_DEFAULT_RATE = 1;

using namespace time_literals;

#define DIM0_DEVICE_PATH "/dev/dim"

DIM::DIM(I2CSPIBusOption bus_option, int bus, int32_t device, int bus_frequency,
	 spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SPI(0xA0, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_transfers(perf_alloc(PC_COUNT, MODULE_NAME": bad transfers")),
	_drdy_gpio(drdy_gpio)
{
	is_power_on = false;
	_class_instance = - 1;
}

DIM::~DIM()
{
	if (_class_instance != -1) {
		unregister_class_devname(DIM0_DEVICE_PATH, _class_instance);
	}

	// delete the perf counters
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
}


int
DIM::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		// if probe/setup failed, bail now
		DEVICE_DEBUG("SPI init failed (%i)", ret);
		return ret;
	}

	_class_instance = register_class_devname(DIM0_DEVICE_PATH);
	start();
	return PX4_OK;
}

int DIM::reset()
{
	return OK;
}

int DIM::power_on()
{
	// Power ON
	memset(&dim_cmd, 0, sizeof(dim_cmd));
	dim_cmd.stx = 0xa5;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 0;
	dim_cmd.len = 0x2;
	dim_cmd.msgid = 0;
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	// read
	transfer(nullptr, (uint8_t *)&dim_power_report, sizeof(dim_power_report));
	dim_power_report.rv = bswap16(dim_power_report.rv);

	// read
	// printf("EXPECT 0xa5 0x06 0x00 0x1d 0x6f 0x1c 0x82 0x01..?: %x %x %x %x %d \n",
	//        dim_power_report.stx, dim_power_report.dir, dim_power_report.offset, dim_power_report.len,
	//        (int16_t)dim_power_report.rv);

	if (dim_power_report.len == 0x02 && dim_power_report.rv == 0) {
		printf("Power On\r\n");
	} // else {

	//     printf("Error Power On\r\n");
	//     return -1;
	// }

	if (dim_power_report.len == 0x1d) {
		// byte swap
		dim_power_report.kcmvp_key_count = bswap16(dim_power_report.kcmvp_key_count);
		dim_power_report.cert_key_count = bswap16(dim_power_report.cert_key_count);
		dim_power_report.io_data_size = bswap16(dim_power_report.io_data_size);
		dim_power_report.info_file_size = bswap16(dim_power_report.info_file_size);
		printf("  * Version    : %X.%02X.%02X\r\n",
		       dim_power_report.version[0], dim_power_report.version[1], dim_power_report.version[2]);
		printf("  * Serial Number    : %02X%02X%02X%02X%02X%02X%02X%02X\r\n",
		       dim_power_report.serial_number[0], dim_power_report.serial_number[1],  dim_power_report.serial_number[2],
		       dim_power_report.serial_number[3], dim_power_report.serial_number[4], dim_power_report.serial_number[5],
		       dim_power_report.serial_number[6], dim_power_report.serial_number[7]);
		printf("  * MaxKcmvpKeyCount : %d\r\n", dim_power_report.kcmvp_key_count);
		printf("  * MaxCertKeyCount  : %d\r\n", dim_power_report.cert_key_count);
		printf("  * MaxIoDataSize    : %d\r\n", dim_power_report.io_data_size);
		printf("  * FileSize         : %d\r\n", dim_power_report.info_file_size);
	}

	is_power_on = true;
	return OK;
}

int DIM::power_off()
{
	// Power OFF
	dim_cmd.stx = 0xa5;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 0;
	dim_cmd.len = 0x2;
	dim_cmd.msgid = 0x01 << 8 | 0x00; // 0x0001 lsb
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(1000000);
	transfer(nullptr, (uint8_t *)&dim_power_report, sizeof(dim_power_report));
	// read result value
	DEVICE_DEBUG("EXPECT 0xa5 0x06 0x00 0x02 0x00 0x00: %x %x %x %x %d \n",
		     dim_power_report.stx, dim_power_report.dir, dim_power_report.offset,   dim_power_report.len,
		     dim_power_report.rv);
	dim_power_report.rv = bswap16(dim_power_report.rv);

	// read
	// printf("EXPECT 0xa5 0x06 0x00 0x1d 0x6f 0x1c 0x82 0x01..?: %x %x %x %x %d \n",
	//        dim_power_report.stx, dim_power_report.dir, dim_power_report.offset, dim_power_report.len,
	//        (int16_t)dim_power_report.rv);

	if (dim_power_report.rv == 0) {
		printf("Power Off\r\n");

	} else {
		printf("Error Power off\r\n");
		return -1;
	}

	is_power_on = false;
	return OK;
}

/**
 * Probe DIM Device in SPI bus
 * @return          exist DIM Devie on SPI bus
 */
int
DIM::probe()
{
	int ret = power_on();

	if (ret != OK) {
		DEVICE_DEBUG("DIM power on error (%i)", ret);
		return ret;
	}

	return PX4_OK;
}


int
DIM::encrypt_self_test()
{
	uint8_t encrypted_text[256], plain_text[256], abIv[16], abAuth[128], abTag[16];
	_kcmvpDrbg(plain_text, 256);
	_kcmvpDrbg(abIv, 16);
	_kcmvpDrbg(abAuth, 128);
	// ARIA (0x50) Encrypt
	_kcmvpGcm(encrypted_text, plain_text, 256, KCMVP_ARIA128_KEY, 0, abIv, 16,
		  abAuth, 128, abTag, 16, ENCRYPT, 0x50);
	printf("  * Plain Data :\r\n    ");

	for (int i = 0; i < 256; i++) {
		printf("%02X", plain_text[i]);

		if ((i < 255) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");
	printf("  * IV :\r\n    ");

	for (int i = 0; i < 16; i++) {
		printf("%02X", abIv[i]);
	}

	printf("\r\n");
	printf("  * Auth :\r\n    ");

	for (int i = 0; i < 128; i++) {
		printf("%02X", abAuth[i]);

		if ((i < 127) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");
	printf("  * Encrypted Data :\r\n    ");

	for (int i = 0; i < 256; i++) {
		printf("%02X", encrypted_text[i]);

		if ((i < 255) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");
	printf("  * Tag :\r\n    ");

	for (int i = 0; i < 16; i++) {
		printf("%02X", abTag[i]);
	}

	printf("\r\n");
	// ARIA Decrypt
	memset(&plain_text, 0, sizeof(plain_text));
	_kcmvpGcm(plain_text, encrypted_text, 256, KCMVP_ARIA128_KEY, 0, abIv, 16,
		  abAuth, 128, abTag, 16, DECRYPT, 0x50);
	printf("  * Plain Data :\r\n    ");

	for (int i = 0; i < 256; i++) {
		printf("%02X", plain_text[i]);

		if ((i < 255) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");
	return OK;
}

int
DIM::encrypt_test(const char *file_name, uint8_t *_plain_text, size_t count)
{
	uint8_t encrypted_text[256], plain_text[256], abIv[16], abAuth[128], abTag[16];
	//_kcmvpDrbg(plain_text, 256);
	memcpy(plain_text, _plain_text, count);
	_kcmvpDrbg(abIv, 16);
	_kcmvpDrbg(abAuth, 128);
	printf("Input: %d\r\n", count);
	printf("  * Plain Data \r\n    ");

	for (int i = 0; i < 256; i++) {
		printf("%02X", plain_text[i]);

		if ((i < 255) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");
	// ARIA (0x50) Encrypt
	_kcmvpGcm(encrypted_text, plain_text, 256, KCMVP_ARIA128_KEY, 0, abIv, 16,
		  abAuth, 128, abTag, 16, ENCRYPT, 0x50);
	// Write MTD and Read MTD
	int _fd = ::open(file_name, O_RDWR | O_CREAT); // O_APPEND

	if (_fd == -1) {
		printf("Error open %s\n", file_name);
		// return -errno;
	}

	::write(_fd, encrypted_text, sizeof(encrypted_text));

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	memset(&encrypted_text, 0, sizeof(encrypted_text));
	_fd = ::open(file_name, O_RDWR);

	if (_fd == -1) {
		printf("Error open %s\n", file_name);
		// return -errno;
	}

	::read(_fd, encrypted_text, sizeof(encrypted_text));
	printf("  * Write Encryted Data to %s :\r\n    ", file_name);

	for (int i = 0; i < 256; i++) {
		printf("%02X", encrypted_text[i]);

		if ((i < 255) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");
	printf("  * Auth :\r\n    ");

	for (int i = 0; i < 128; i++) {
		printf("%02X", abAuth[i]);

		if ((i < 127) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	// ARIA (0x50) Decrypt
	memset(&plain_text, 0, sizeof(plain_text));
	_kcmvpGcm(plain_text, encrypted_text, 256, KCMVP_ARIA128_KEY, 0, abIv, 16,
		  abAuth, 128, abTag, 16, DECRYPT, 0x50);
	printf("  * Read Encrypted Data from %s and Decrypt it \r\n    ", file_name);

	for (int i = 0; i < 256; i++) {
		printf("%02X", plain_text[i]);

		if ((i < 255) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");

	// Send Encrypt TEXT to Onboard using MAVLink
	// encapsulated_data
	encapsulated_data_s data_topic{};
	data_topic.timestamp = hrt_absolute_time();
	data_topic.seqnr = 0;
	memcpy(data_topic.data, encrypted_text, 256);
	memcpy(data_topic.data + 256, abIv, 16);
	memcpy(data_topic.data + 256 + 16, abAuth, 128);
	memcpy(data_topic.data + 256 + 16 + 128, abTag, 16);
	_encapsulated_data_pub.publish(data_topic);

	return PX4_OK;
}


int
DIM::decrypt_test(uint8_t *_encrypted_text, size_t count)
{
	uint8_t encrypted_text[256], plain_text[256], abIv[16], abAuth[128], abTag[16];

	memcpy(encrypted_text, _encrypted_text, 256);
	memcpy(abIv, _encrypted_text + 256, 16);
	memcpy(abAuth, _encrypted_text + 256 + 16, 128);
	memcpy(abTag, _encrypted_text + 256 + 16 + 128, 16);

	// ARIA (0x50) Decrypt
	memset(&plain_text, 0, sizeof(plain_text));
	_kcmvpGcm(plain_text, encrypted_text, 256, KCMVP_ARIA128_KEY, 0, abIv, 16,
		  abAuth, 128, abTag, 16, DECRYPT, 0x50);
	printf("  * Read Encrypted Data from GCS and Decrypt it \r\n    ");

	for (int i = 0; i < 256; i++) {
		printf("%02X", plain_text[i]);

		if ((i < 255) && ((i + 1) % 32 == 0)) {
			printf("\r\n    ");
		}
	}

	printf("\r\n");

	return PX4_OK;
}


int
DIM::getKey()
{
	int ret;

	// get key 0
	uint8_t abPubKey0[16];
	uint16_t usSize0 = 0;
	memset(&abPubKey0, 0, sizeof(abPubKey0));
	ret = _kcmvpGetKey(abPubKey0, &usSize0, KCMVP_ARIA128_KEY, 0);

	if (ret < 0) {
		printf("Error kcmvpGetKey: %d\n", ret);
	}

	printf("kcmvpGetKey Size: %d\n", usSize0);
	printf("kcmvpGetKey: \n");

	for (int i = 0; i < usSize0; i++) {
		printf("%02X", abPubKey0[i]);
	}

	printf("\r\n");

	return PX4_OK;
}

void
DIM::start()
{
	int ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("DIM probe error (%i)", ret);
	}

	// start polling at the specified rate
	//    ScheduleOnInterval((1_s / DIM_DEFAULT_RATE), 10000);
}

void
DIM::stop()
{
	ScheduleClear();

	if (is_power_on == true) {
		int ret = power_off();

		if (ret != OK) {
			DEVICE_DEBUG("DIM power off error (%i)", ret);
			return;
		}

		is_power_on = false;
	}
}

int
DIM::open(struct file *filep)
{
	// printf("open_first\n");
	return CDev::open_first(filep);
}
int
DIM::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;
	typedef struct {
		const char *filepath;
		uint8_t buffer[256];
	} file_io_t;

	switch (cmd) {
	case DIM_ENCRYPT:
		ret = encrypt_test(((file_io_t *) arg)->filepath, ((file_io_t *) arg)->buffer, 256);
		break;

	case DIM_DECRYPT:
		// ret = encrypt_test ((const char*)arg);
		ret = 0;
		break;

	case DIM_MAVLINK_ENCRYPT:
		printf("Save MAVLink #33 message on SD\r\n");
		ret = encrypt_test(((file_io_t *) arg)->filepath, ((file_io_t *) arg)->buffer, 256);
		// ret = 0;
		break;

	case DIM_MAVLINK_DECRYPT:
		printf("Received MAVLink #33 message from GCS\r\n");
		ret = decrypt_test((uint8_t*) arg, 512);
		// ret = 0;
		break;


	case DIM_IS_POWER_ON:
		ret = is_power_on;
		break;

	default:
		break;
	}

	return ret;
}

void
DIM::exit_and_cleanup()
{
	if (_drdy_gpio != 0) {
		// Disable data ready callback
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);
	}

	int ret;
	ret = power_off();

	if (ret != OK) {
		DEVICE_DEBUG("DIM power off error (%i)", ret);
	}

	I2CSPIDriverBase::exit_and_cleanup();
}

void
DIM::RunImpl()
{
	measure();
}

int
DIM::measure()
{
	perf_begin(_sample_perf);
	// DEVICE_DEBUG("DIM measure");
	vehicle_global_position_s global_pos;
	vehicle_local_position_s lpos;

	if (_gpos_sub.update(&global_pos)) {
		printf("gps %f, %f, %d\n",
		       global_pos.lat * 1e7,
		       global_pos.lon * 1e7,
		       sizeof(global_pos));
	}

	if (_lpos_sub.update(&lpos)) {
		printf("lpos %lf, %lf, %lf, %d\n",
		       (double)lpos.x,
		       (double)lpos.y,
		       (double)lpos.z,
		       sizeof(lpos)
		      );
		return true;
	}

	perf_end(_sample_perf);
	return OK;
}

void
DIM::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
}

int16_t
DIM::_kcmvpDrbg(uint8_t *pbRandom, uint16_t usRandomSize)
{
	uint16_t pbRandomCursor = 0;

	// Check KSE power state.
	if (!is_power_on) {
		return -1;
	}

	// Check input.
	if ((pbRandom == NULL_PTR) || (usRandomSize == 0) || (usRandomSize > 256)) {
		return -1;
	}

	memset(&dim_cmd, 0, sizeof(dim_cmd));
	// printf("Generate random number: %d\n", usRandomSize);
	// request 0
	dim_cmd.stx = 0xa5;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 0;
	dim_cmd.len = 4;
	dim_cmd.msgid = 0x10 << 8 | 0x02; // 0x0210
	dim_cmd.data[0] = usRandomSize >> 8; // 256 lsb
	dim_cmd.data[1] = usRandomSize & 0xff;
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));

	// int ret = (dim_report.data[0] << 8)| dim_report.data[1];

	for (int i = 2; i < dim_report.len; i++) {
		pbRandom[pbRandomCursor] = dim_report.data[i];
		pbRandomCursor += 1;;
	}

	// request middle block
	while (dim_report.stx == 0xa1 || dim_report.stx == 0x11) {
		dim_cmd.stx = dim_report.stx;
		dim_cmd.dir = dim_report.dir;
		dim_cmd.offset = dim_report.offset;
		dim_cmd.len = dim_report.len;
		dim_cmd.dir = 0xFE;
		transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
		px4_usleep(T_STALL);
		transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));

		for (int i = 0; i < dim_report.len; i++) {
			pbRandom[pbRandomCursor] = dim_report.data[i];
			pbRandomCursor += 1;
		}

		if (dim_report.stx == 0x15) { // if last block, break
			break;
		}
	}

	return OK;
}

int16_t
DIM::_kcmvpGcm(uint8_t *pbOutput, uint8_t *pbInput, uint16_t usInputSize,
	       uint8_t bKeyType, uint16_t usKeyIndex, uint8_t *pbIv,
	       uint16_t usIvSize, uint8_t *pbAuth, uint16_t usAuthSize,
	       uint8_t *pbTag, uint16_t usTagSize, uint8_t bEnDe,
	       uint8_t bAlg)
{
	uint16_t pbOutputCursor = 0;

	// Check KSE power state.
	if (!is_power_on) {
		return -1;
	}

	// Check input.
	if (((pbOutput == NULL_PTR) && (usInputSize != 0)) ||
	    ((pbInput == NULL_PTR) && (usInputSize != 0)) ||
	    (pbIv == NULL_PTR) ||
	    ((pbAuth == NULL_PTR) && (usAuthSize != 0)) ||
	    ((pbTag == NULL_PTR) && (usTagSize != 0)) ||
	    (((bAlg & 0xF0) ^ bKeyType) > 0x02) ||
	    (usKeyIndex >= MAX_KCMVP_KEY_COUNT) ||
	    (usIvSize == 0) || (usTagSize > 16) ||
	    (usInputSize + usIvSize + usAuthSize > MAX_IO_DATA_SIZE) ||
	    ((bEnDe != ENCRYPT) && (bEnDe != DECRYPT))) {
		return -1;
	}

	memset(&dim_cmd, 0, sizeof(dim_cmd));
	memset(&buffer, 0, sizeof(buffer));
	memcpy(buffer, pbIv, usIvSize);
	memcpy(buffer + usIvSize, pbInput, usInputSize);
	memcpy(buffer + usIvSize + usInputSize, pbAuth, usAuthSize);

	if (bEnDe == DECRYPT) {
		printf("Decryption: %d\n", usInputSize);
		memcpy(buffer + usIvSize + usInputSize + usAuthSize, pbTag, usTagSize);
		//    int total_send_data = 12 + 16 + 256 + 128; // 12 + 400
		//    int total_recv_data = total_send_data + usTagSize;

	} else {
		printf("Encryption: %d\n", usInputSize);
	}

	// request 0
	dim_cmd.stx = 0xa1;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 0;
	dim_cmd.len = 60;
	dim_cmd.msgid = ((uint16_t)(bAlg | 0x07) << 8) | 0x02; // 0x0257 lsb
	//    dim_cmd.msg[0] = 0x02;
	//    dim_cmd.msg[1] = 0x57;
	dim_cmd.data[0] = bKeyType;
	dim_cmd.data[1] = (uint8_t)(usKeyIndex >> 8);
	dim_cmd.data[2] = (uint8_t)(usKeyIndex);
	dim_cmd.data[3] = bEnDe;
	dim_cmd.data[4] = (uint8_t)(usIvSize >> 8);
	dim_cmd.data[5] = (uint8_t)(usIvSize);
	dim_cmd.data[6] = (uint8_t)(usInputSize >> 8);
	dim_cmd.data[7] = (uint8_t)(usInputSize);
	dim_cmd.data[8] = (uint8_t)(usAuthSize >> 8);
	dim_cmd.data[9] = (uint8_t)(usAuthSize);
	dim_cmd.data[10] = (uint8_t)(usTagSize >> 8);
	dim_cmd.data[11] = (uint8_t)(usTagSize);
	// left data 58 - 12 = 46

	for (int i = 0; i < 46; i++) {
		dim_cmd.data[12 + i] = buffer[i];
	}

	// -----------
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	// And response
	transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));
	// request 1
	dim_cmd.stx = 0x11;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 1;
	dim_cmd.len = 60;
	dim_cmd.msg[0] = buffer[46];
	dim_cmd.msg[1] = buffer[47];

	for (int i = 48; i < 48 + 58; i++) {
		dim_cmd.data[i - 48] = buffer[i];
	}

	// -----------
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	// And response
	transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));
	// request 2
	dim_cmd.stx = 0x11;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 2;
	dim_cmd.len = 60;
	dim_cmd.msg[0] = buffer[48 + 58];
	dim_cmd.msg[1] = buffer[49 + 58];

	for (int i = 50 + 58; i < 50 + 58 + 58; i++) {
		dim_cmd.data[i - (50 + 58)] = buffer[i];
	}

	// -----------
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	// And response
	transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));
	// request 3
	dim_cmd.stx = 0x11;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 3;
	dim_cmd.len = 60;
	dim_cmd.msg[0] = buffer[50 + 58 + 58];
	dim_cmd.msg[1] = buffer[51 + 58 + 58];

	for (int i = 52 + 58 + 58; i < 52 + 58 + 58 + 58; i++) {
		dim_cmd.data[i - (52 + 58 + 58)] = buffer[i];
	}

	// -----------
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	// And response
	transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));
	// request 4
	dim_cmd.stx = 0x11;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 4;
	dim_cmd.len = 60;
	dim_cmd.msg[0] = buffer[52 + 58 + 58 + 58];
	dim_cmd.msg[1] = buffer[53 + 58 + 58 + 58];

	for (int i = 54 + 58 + 58 + 58; i < 54 + 58 + 58 + 58 + 58; i++) {
		dim_cmd.data[i - (54 + 58 + 58 + 58)] = buffer[i];
	}

	// -----------
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	// And response
	transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));
	// request 5
	dim_cmd.stx = 0x11;
	dim_cmd.dir = 0x05;
	dim_cmd.offset = 5;
	dim_cmd.len = 60;
	dim_cmd.msg[0] = buffer[54 + 58 + 58 + 58 + 58];
	dim_cmd.msg[1] = buffer[55 + 58 + 58 + 58 + 58];

	for (int i = 56 + 58 + 58 + 58 + 58; i < 56 + 58 + 58 + 58 + 58 + 58; i++) {
		dim_cmd.data[i - (56 + 58 + 58 + 58 + 58)] = buffer[i];
	}

	// -----------
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	// And response
	transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));
	memset(&dim_cmd, 0, sizeof(dim_cmd));

	if (bEnDe == ENCRYPT) {
		// request 6
		dim_cmd.stx = 0x15;
		dim_cmd.dir = 0x05;
		dim_cmd.offset = 6;
		dim_cmd.len = (400 - 348 + 2); // 54
		dim_cmd.msg[0] = buffer[56 + 58 + 58 + 58 + 58 + 58];
		dim_cmd.msg[1] = buffer[57 + 58 + 58 + 58 + 58 + 58];

		for (int i = 58 + 58 + 58 + 58 + 58 + 58; i < 400; i++) {
			dim_cmd.data[i - (58 + 58 + 58 + 58 + 58 + 58)] = buffer[i];
		}

		// -----------
		transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd)); // not 54+4
		px4_usleep(T_STALL);
		// And response
		transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));

		for (int i = 2; i < dim_report.len; i++) {
			pbOutput[pbOutputCursor] = dim_report.data[i];
			pbOutputCursor += 1;
		}

	} else { // DECRYPT
		dim_cmd.stx = 0x11;
		dim_cmd.dir = 0x05;
		dim_cmd.offset = 6;
		dim_cmd.len = 60; // 54 + 6
		dim_cmd.msg[0] = buffer[56 + 58 + 58 + 58 + 58 + 58];
		dim_cmd.msg[1] = buffer[57 + 58 + 58 + 58 + 58 + 58];

		for (int i = 58 + 58 + 58 + 58 + 58 + 58; i < 58 + 58 + 58 + 58 + 58 + 58 + 58; i++) {
			dim_cmd.data[i - (58 + 58 + 58 + 58 + 58 + 58)] = buffer[i];
		}

		// -----------
		transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
		px4_usleep(T_STALL);
		// And response
		transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));
		// request 7
		dim_cmd.stx = 0x15;
		dim_cmd.dir = 0x05;
		dim_cmd.offset = 7;
		dim_cmd.len = (416 - 408 + 2) ; // 10
		dim_cmd.msg[0] = buffer[58 + 58 + 58 + 58 + 58 + 58 + 58];
		dim_cmd.msg[1] = buffer[59 + 58 + 58 + 58 + 58 + 58 + 58];

		for (int i = 60 + 58 + 58 + 58 + 58 + 58 + 58; i < 416; i++) {
			dim_cmd.data[i - (60 + 58 + 58 + 58 + 58 + 58 + 58)] = buffer[i];
		}

		// -----------
		transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
		px4_usleep(T_STALL);
		// And response
		transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));

		for (int i = 2; i < dim_report.len; i++) {
			pbOutput[pbOutputCursor] = dim_report.data[i];
			pbOutputCursor += 1;
		}
	}

	// And read encryption data
	while (dim_report.stx == 0xa1 || dim_report.stx == 0x11) {
		dim_cmd.stx = dim_report.stx;
		dim_cmd.dir = dim_report.dir;
		dim_cmd.offset = dim_report.offset;
		dim_cmd.len = dim_report.len;
		dim_cmd.dir = 0xFE;
		transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
		px4_usleep(T_STALL);
		transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));

		for (int i = 0; i < dim_report.len; i++) {
			// check overflow pbOutput
			if (bEnDe == ENCRYPT && dim_report.stx == 0x15 && i == (dim_report.len - usTagSize)) {
				break;
			}

			pbOutput[pbOutputCursor] = dim_report.data[i];
			pbOutputCursor += 1;
		}

		if (bEnDe == ENCRYPT && dim_report.stx == 0x15) { // if last block, break
			for (int i = 0; i < usTagSize ; i++) {
				pbTag[i] = dim_report.data[dim_report.len - (usTagSize - i)];
			}

			break;
		}
	}

	return OK;
}

int16_t
DIM::_kcmvpGetKey(uint8_t *pbKey, uint16_t *pusKeySize, uint8_t bKeyType,
		  uint16_t usKeyIndex)
{
	int16_t sRv;

	// Check KSE power state.
	if (!is_power_on) {
		return -1;
	}

	// Check input.
	if ((pusKeySize == NULL_PTR) ||
	    ((bKeyType != KCMVP_AES128_KEY) && (bKeyType != KCMVP_AES192_KEY) &&
	     (bKeyType != KCMVP_AES256_KEY) && (bKeyType != KCMVP_ARIA128_KEY) &&
	     (bKeyType != KCMVP_ARIA192_KEY) && (bKeyType != KCMVP_ARIA256_KEY) &&
	     (bKeyType != KCMVP_HMAC_KEY) && (bKeyType != KCMVP_ECDSA_PRI_KEY) &&
	     (bKeyType != KCMVP_ECDSA_PUB_KEY) &&
	     (bKeyType != KCMVP_ECDH_PRI_KEY) &&
	     (bKeyType != KCMVP_ECDH_PUB_KEY)) ||
	    (usKeyIndex >= MAX_KCMVP_KEY_COUNT)) {
		return -1;
	}

	memset(&dim_cmd, 0x00, sizeof(dim_cmd));
	memset(&dim_report, 0x00, sizeof(dim_report));

	// request 0
	dim_cmd.stx = 0xa5;
	dim_cmd.dir = 0x05;
	// dim_cmd.dir = 0xFE;
	dim_cmd.offset = 0;
	dim_cmd.len = 2 + 3; // ?
	// dim_cmd.msgid = ((uint16_t)(0x02) << 8) | 0x02; // 0x0202 lsb
	dim_cmd.msg[0] = 0x02;
	dim_cmd.msg[1] = 0x02;
	dim_cmd.data[0] = bKeyType;
	dim_cmd.data[1] = (uint8_t)(usKeyIndex >> 8);
	dim_cmd.data[2] = (uint8_t)(usKeyIndex);

	// -----------
	transfer((uint8_t *)&dim_cmd, nullptr, sizeof(dim_cmd));
	px4_usleep(T_STALL);
	// And response
	transfer(nullptr, (uint8_t *)&dim_report, sizeof(dim_report));
	px4_usleep(T_STALL);

	// printf("%02X %02X %02X ", dim_report.stx, dim_report.dir, dim_report.offset);
	// printf("%02X %02X %02X: ", dim_report.len, dim_report.data[0], dim_report.data[1]);
	// printf("%02X %02X %02X \n", dim_report.data[0], dim_report.data[1], dim_report.data[2]);
	sRv = (int16_t)(((uint16_t)dim_report.data[0] << 8) |
			((uint16_t)dim_report.data[1]));

	for (int i = 0; i < (dim_report.len - 2); i++) {
		pbKey[i] = dim_report.data[i + 2];
	}

	*pusKeySize = dim_report.len - 2;

	return sRv;
}

void
DIM::custom_method(const BusCLIArguments &cli)
{
	uint8_t plain_text[256];
	memset(&plain_text, 0, sizeof(plain_text));
	_kcmvpDrbg(plain_text, 256);

	switch (cli.custom1) {
	case 1:
		encrypt_test("/fs/mtd_dim", plain_text, 256);
		break;

	case 2:
		encrypt_test("/fs/microsd/dim.txt", plain_text, 256);
		break;

	case 3:
		_kcmvpDrbg(plain_text, cli.custom2);

		for (int i = 0; i < cli.custom2; i++) {
			printf("%02X", plain_text[i]);
		}

		printf("\r\n");
		break;

	case 4:
		encrypt_self_test();
		break;

	case 5:
		getKey();
		break;


	}
}

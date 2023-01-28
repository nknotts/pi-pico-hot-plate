#include "MAX31855.hpp"

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <task.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace b1g {
namespace sensor {

namespace {

// https://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html

const std::vector<float> DCOEF_A = {
    0.0000000e00,
    2.5173462e01,
    -1.1662878e00,
    -1.0833638e00,
    -8.9773540e-01,
    -3.7342377e-01,
    -8.6632643e-02,
    -1.0450598e-02,
    -5.1920577e-04};

const std::vector<float> DCOEF_B = {
    0.000000e00,
    2.508355e01,
    7.860106e-02,
    -2.503131e-01,
    8.315270e-02,
    -1.228034e-02,
    9.804036e-04,
    -4.413030e-05,
    1.057734e-06,
    -1.052755e-08,
};

const std::vector<float> DCOEF_C = {
    -1.318058e02,
    4.830222e01,
    -1.646031e00,
    5.464731e-02,
    -9.650715e-04,
    8.802193e-06,
    -3.110810e-08,
};

float calculate_temperature(float VTOTAL, const std::vector<float>& DCOEF) {
	float temperature = 0.0;
	for (size_t i = 0; i < DCOEF.size(); ++i) {
		temperature += DCOEF[i] * pow(VTOTAL, i);
	}
	return temperature;
}

} // namespace

MAX31855::MAX31855(spi_inst_t* spi, uint chip_select_pin)
    : spi_dev{spi}, cs_pin{chip_select_pin} {
}

// https://github.com/adafruit/Adafruit_CircuitPython_MAX31855/blob/main/adafruit_max31855.py

std::optional<MAX31855::Data> MAX31855::Sample() {
	uint8_t read_buf[4] = {};

	// cs_select
	asm volatile("nop \n nop \n nop");
	gpio_put(cs_pin, 0);
	asm volatile("nop \n nop \n nop");

	int ret = spi_read_blocking(spi_dev, 0, read_buf, sizeof(read_buf));

	// cs_deselect
	asm volatile("nop \n nop \n nop");
	gpio_put(cs_pin, 1);
	asm volatile("nop \n nop \n nop");

	int ttag_ms = xTaskGetTickCount();

	if (ret != sizeof(read_buf)) {
		// spi read error
		return std::nullopt;
	}

	if (std::all_of(read_buf, read_buf + sizeof(read_buf), [](auto val) { return val == 0; })) {
		// all zeros, assume max chip not connected to spi bus
		return std::nullopt;
	}

	if (read_buf[3] & 0x01) {
		// thermocouple not connected
		return std::nullopt;
	}

	if (read_buf[3] & 0x02) {
		// short circuit to ground
		return std::nullopt;
	}

	if (read_buf[3] & 0x04) {
		// short circuit to power
		return std::nullopt;
	}

	if (read_buf[1] & 0x01) {
		// faulty reading
		return std::nullopt;
	}

	uint16_t temp = (read_buf[0] << 8) | read_buf[1];
	uint16_t refer = (read_buf[2] << 8) | read_buf[3];

	// Thermocouple temperature in degrees Celsius, computed using
	// raw voltages and NIST approximation for Type K, see:
	//  https://srdata.nist.gov/its90/download/type_k.tab

	// temperature of remote thermocouple junction
	float TR = (temp >> 2) / 4.0;

	// temperature of device (cold junction)
	float TAMB = (refer >> 4) * 0.0625;

	// thermocouple voltage based on MAX31855's uV/degC for type K (table 1)
	float VOUT = 0.041276 * (TR - TAMB);

	// cold junction equivalent thermocouple voltage
	float VREF = (TAMB >= 0.0)
	                 ? (-0.176004136860e-01 +
	                    0.389212049750e-01 * TAMB +
	                    0.185587700320e-04 * pow(TAMB, 2) +
	                    -0.994575928740e-07 * pow(TAMB, 3) +
	                    0.318409457190e-09 * pow(TAMB, 4) +
	                    -0.560728448890e-12 * pow(TAMB, 5) +
	                    0.560750590590e-15 * pow(TAMB, 6) +
	                    -0.320207200030e-18 * pow(TAMB, 7) +
	                    0.971511471520e-22 * pow(TAMB, 8) +
	                    -0.121047212750e-25 * pow(TAMB, 9) +
	                    0.1185976 * exp(-0.1183432e-03 * pow(TAMB - 0.1269686e03, 2)))
	                 : (0.394501280250e-01 * TAMB +
	                    0.236223735980e-04 * pow(TAMB, 2) +
	                    -0.328589067840e-06 * pow(TAMB, 3) +
	                    -0.499048287770e-08 * pow(TAMB, 4) +
	                    -0.675090591730e-10 * pow(TAMB, 5) +
	                    -0.574103274280e-12 * pow(TAMB, 6) +
	                    -0.310888728940e-14 * pow(TAMB, 7) +
	                    -0.104516093650e-16 * pow(TAMB, 8) +
	                    -0.198892668780e-19 * pow(TAMB, 9) +
	                    -0.163226974860e-22 * pow(TAMB, 10));

	// total thermoelectric voltage
	float VTOTAL = VOUT + VREF;

	Data out;
	out.ttag_ms = ttag_ms;
	// srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
	if (VTOTAL >= -5.891 && VTOTAL <= 0.0) {
		out.temperature_C = calculate_temperature(VTOTAL, DCOEF_A);
		return out;
	} else if (VTOTAL > 0 && VTOTAL <= 20.644) {
		out.temperature_C = calculate_temperature(VTOTAL, DCOEF_B);
		return out;
	} else if (VTOTAL > 20.644 && VTOTAL <= 54.886) {
		out.temperature_C = calculate_temperature(VTOTAL, DCOEF_C);
		return out;
	}

	// Total thermoelectric voltage out of range
	return std::nullopt;
}

} // namespace sensor
} // namespace b1g

#pragma once

#include <hardware/spi.h>

#include <cstdint>
#include <optional>

namespace b1g {
namespace sensor {

class MAX31855 {
public:
	MAX31855(spi_inst_t* spi, uint chip_select_pin);

	struct Data {
		uint32_t ttag_ms;
		float temperature_C;
	};

	std::optional<Data> Sample();

private:
	spi_inst_t* spi_dev;
	const uint cs_pin;
};

} // namespace sensor
} // namespace b1g

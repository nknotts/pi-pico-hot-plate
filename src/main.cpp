#include <secrets.hpp>

#include <b1g/log/Logger.hpp>
#include <b1g/pid/pid.hpp>
#include <b1g/sensor/MAX31855.hpp>

#include <libHotPlate_version.h>

#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include <hardware/rtc.h>
#include <hardware/spi.h>
#include <hardware/watchdog.h>
#include <pico/binary_info.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>
#include <pico/util/datetime.h>

#include <b1g_font.h>
#include <ssd1306.h>
#include <textRenderer/TextRenderer.h>

#include <lwip/api.h>
#include <lwip/apps/sntp.h>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include <tusb.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <ctime>
#include <functional>
#include <memory>
#include <variant>

template <>
class std::default_delete<netbuf> {
public:
	void operator()(netbuf* ptr) const {
		netbuf_delete(ptr);
	}
};

namespace {

constexpr const uint8_t MAX_TEMPERATURE_SET_POINT_C = 240;
constexpr const uint8_t MAX_TEMPERATURE_MEASURED_C = 250;

using namespace b1g;

#define I2C_DEVICE i2c1
const uint I2C_SDA_PIN = 26;
const uint I2C_SCL_PIN = 27;

#define SPI_DEVICE spi0
const uint SPI_TX_PIN = 19;
const uint SPI_SCK_PIN = 18;
const uint SPI_CSN_PIN = 17;
const uint SPI_RX_PIN = 16;

const uint RELAY_ENABLED_PIN = 20; // connected to electro mechanical relay
const uint RELAY_PWM_PIN = 21;     // connected to SSR control pin

const uint DISABLE_HEATER_PIN = 0;
const uint START_HEAT_CURVE_PIN = 1;

std::optional<sensor::MAX31855::Data> temperature_data;
SemaphoreHandle_t temperature_mutex;

std::optional<uint32_t> ip_address;
SemaphoreHandle_t ip_addresss_mutex;

MessageBufferHandle_t sample_msg_buffer;
constexpr const size_t SAMPLE_BUF_SIZE = MQTT_OUTPUT_RINGBUF_SIZE;

constexpr const size_t UDP_RECV_BUF_SIZE = 4096;
MessageBufferHandle_t udp_recv_msg_buffer;

constexpr const size_t CDC_CONNECTED_BUF_SIZE = 8;
MessageBufferHandle_t cdc_connected_msg_buffer;

constexpr const size_t SNTP_BUF_SIZE = 2 * sizeof(uint32_t);
MessageBufferHandle_t sntp_msg_buffer;

constexpr const TickType_t MUTEX_WAIT_TIME = 10000;

class DisableHeater {
};

class StartHeatCurve {
};

struct TemperatureSetPoint {
	uint8_t temperature_C;
};

struct PwmSetPoint {
	uint8_t pwm_percent;
};

using HeaterCommand = std::variant<DisableHeater, StartHeatCurve, TemperatureSetPoint, PwmSetPoint>;

constexpr const size_t HEATER_COMMAND_QUEUE_SIZE = 32;
QueueHandle_t heater_command_queue;

enum class HeaterStateType {
	Idle,
	Pwm,
	Temperature,
	Curve
};

struct HeaterState {
	HeaterStateType state_type = HeaterStateType::Idle;
	uint8_t pwm_percent = {};
	uint8_t target_temperature_C = {};
	std::optional<float> measured_temperature_C = {};
	TickType_t current_tick = {};
	TickType_t curve_tbegin = {};
	TickType_t curve_tend = {};
};

HeaterState heater_state;
SemaphoreHandle_t heater_state_mutex;

template <typename Type>
Type function_guard(SemaphoreHandle_t mtx, std::function<Type()> fn) {
	xSemaphoreTake(mtx, MUTEX_WAIT_TIME);
	Type out = fn();
	xSemaphoreGive(mtx);
	return out;
}

template <> // explicit specialization for T = void
void function_guard(SemaphoreHandle_t mtx, std::function<void()> fn) {
	xSemaphoreTake(mtx, MUTEX_WAIT_TIME);
	fn();
	xSemaphoreGive(mtx);
}

// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts... {
	using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

struct HeaterCurvePoint {
	float time_s;
	float temperature_C;
};

const std::vector<HeaterCurvePoint> HEATER_CURVE = {
    {0.0, 30.0},
    {60.0, 150.0},
    {120.0, 150.0},
    {165.0, 240.0},
};

std::optional<uint8_t> TemperatureCurveLookup(uint32_t time_ms) {
	float time_s = time_ms / 1000.0;
	const auto heater_curve_it1 = std::find_if(
	    HEATER_CURVE.begin(),
	    HEATER_CURVE.end(),
	    [time_s](const HeaterCurvePoint& point) {
		    return time_s < point.time_s;
	    });

	if (heater_curve_it1 == HEATER_CURVE.end()) {
		return std::nullopt;
	}

	const auto heater_curve_it0 = std::prev(heater_curve_it1);

	// y = m * x + b
	// m = (y1 - y0) / (x1 - x0)
	// b = y0 - (y1 - y0) / (x1 - x0) * x0
	float x0 = heater_curve_it0->time_s;
	float x1 = heater_curve_it1->time_s;
	float y0 = heater_curve_it0->temperature_C;
	float y1 = heater_curve_it1->temperature_C;

	float temp_C = y0 + ((y1 - y0) / (x1 - x0)) * (time_s - x0);
	return static_cast<uint8_t>(std::round(temp_C));
}

void heater_task(__unused void* params) {
	gpio_init(RELAY_ENABLED_PIN);
	gpio_put(RELAY_ENABLED_PIN, 0);
	gpio_set_dir(RELAY_ENABLED_PIN, GPIO_OUT);

	gpio_set_function(RELAY_PWM_PIN, GPIO_FUNC_PWM);
	auto pwm_slice = pwm_gpio_to_slice_num(RELAY_PWM_PIN);
	auto pwm_channel = pwm_gpio_to_channel(RELAY_PWM_PIN);
	const float CLOCK_DIVIDER = 250.0; // input 125 MHz  > output 0.5 MHz
	pwm_set_clkdiv(pwm_slice, CLOCK_DIVIDER);
	const uint16_t WRAP_US = 50000 - 1; // 10 Hz PWM -> 0.5 MHz / 10 = 50,000 us
	pwm_set_wrap(pwm_slice, WRAP_US);
	pwm_set_enabled(pwm_slice, true);
	const uint PWM_PERCENT_SCALE_FACTOR = 500; // 0 -> 0, 100 -> 50,000
	pwm_set_chan_level(pwm_slice, pwm_channel, 0);

	auto last_wake_time = xTaskGetTickCount();
	HeaterCommand command;
	while (true) {
		auto now = xTaskGetTickCount();
		auto measured_temperature_C = function_guard<std::optional<float>>(
		    temperature_mutex, [] {
			    return temperature_data.has_value() ? std::optional(temperature_data->temperature_C) : std::nullopt;
		    });

		bool is_temperature_safe = measured_temperature_C.has_value() && *measured_temperature_C < MAX_TEMPERATURE_MEASURED_C;
		is_temperature_safe = true;
		if (!is_temperature_safe) {
			if (heater_state.state_type != HeaterStateType::Idle) {
				HeaterCommand cmd = DisableHeater{};
				// xQueueSend(heater_command_queue, &cmd, 100);
				log::warn("Disabling heater due to unsafe temperature");
			}
		}

		// process commands
		while (xQueueReceive(heater_command_queue, &command, 0) == pdTRUE) {
			std::visit(
			    overloaded{
			        [](DisableHeater) {
				        function_guard<void>(temperature_mutex, [] {
					        heater_state.state_type = HeaterStateType::Idle;
					        heater_state.pwm_percent = 0;
					        heater_state.target_temperature_C = 0;
				        });
			        },
			        [is_temperature_safe, now](StartHeatCurve) {
				        if (is_temperature_safe) {
					        if (heater_state.state_type != HeaterStateType::Curve) {
						        function_guard<void>(temperature_mutex, [now] {
							        heater_state.state_type = HeaterStateType::Curve;
							        heater_state.pwm_percent = 0;
							        heater_state.target_temperature_C = 0;
							        heater_state.curve_tbegin = now;
							        heater_state.curve_tend = now + static_cast<TickType_t>(HEATER_CURVE.back().time_s) * 1000;
						        });
					        }
				        } else {
					        log::warn("Ignoring curve command due to unsafe temperature");
				        }
			        },
			        [is_temperature_safe](TemperatureSetPoint set_point) {
				        if (is_temperature_safe) {
					        function_guard<void>(temperature_mutex, [set_point = set_point.temperature_C] {
						        heater_state.state_type = HeaterStateType::Temperature;
						        heater_state.pwm_percent = 0;
						        heater_state.target_temperature_C = set_point;
					        });
				        } else {
					        log::warn("Ignoring temperature command due to unsafe temperature");
				        }
			        },
			        [is_temperature_safe](PwmSetPoint set_point) {
				        function_guard<void>(temperature_mutex, [set_point = set_point.pwm_percent] {
					        heater_state.state_type = HeaterStateType::Pwm;
					        heater_state.pwm_percent = set_point;
					        heater_state.target_temperature_C = 0;
				        });
			        },
			    },
			    command);
		}

		// update state
		uint8_t target_temperature_C = heater_state.target_temperature_C;
		if (heater_state.state_type == HeaterStateType::Curve) {
			auto curve_temperature_C = TemperatureCurveLookup(now - heater_state.curve_tbegin);
			if (curve_temperature_C.has_value()) {
				target_temperature_C = curve_temperature_C.value();
			} else {
				target_temperature_C = 0;
				HeaterCommand cmd = DisableHeater{};
				xQueueSend(heater_command_queue, &cmd, 100);
				log::info("Curve finished, disabling");
			}
		}

		uint8_t pwm_percent = heater_state.pwm_percent;
		if (target_temperature_C > 0) {
			// pid
		}

		gpio_put(RELAY_ENABLED_PIN, pwm_percent > 0 ? 1 : 0);
		pwm_set_chan_level(pwm_slice, pwm_channel, pwm_percent * PWM_PERCENT_SCALE_FACTOR);

		function_guard<void>(heater_state_mutex, [now, measured_temperature_C, target_temperature_C, pwm_percent] {
			heater_state.current_tick = now;
			heater_state.pwm_percent = pwm_percent;
			heater_state.measured_temperature_C = measured_temperature_C;
			heater_state.target_temperature_C = target_temperature_C;
		});

		vTaskDelayUntil(&last_wake_time, 100);
	}
}

void query_button(uint pin, bool& last, const HeaterCommand& command) {
	auto val = gpio_get(pin);
	if (val == 0 && val != last) {
		xQueueSend(heater_command_queue, &command, 100);
		log::info("query_button: {}, sending command", pin);
	}
	last = val;
}

void button_task(__unused void* params) {
	gpio_init(DISABLE_HEATER_PIN);
	gpio_set_dir(DISABLE_HEATER_PIN, GPIO_IN);
	gpio_pull_up(DISABLE_HEATER_PIN);

	gpio_init(START_HEAT_CURVE_PIN);
	gpio_set_dir(START_HEAT_CURVE_PIN, GPIO_IN);
	gpio_pull_up(START_HEAT_CURVE_PIN);

	HeaterCommand disable_heater_command = DisableHeater{};
	HeaterCommand start_heat_curve_command = StartHeatCurve{};
	auto last_disable_heater = gpio_get(DISABLE_HEATER_PIN);
	auto last_start_heat_curve = gpio_get(START_HEAT_CURVE_PIN);
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		query_button(DISABLE_HEATER_PIN, last_disable_heater, disable_heater_command);
		query_button(START_HEAT_CURVE_PIN, last_start_heat_curve, start_heat_curve_command);
		vTaskDelayUntil(&last_wake_time, 20);
	}
}

void blink_task(__unused void* params) {
	bool on = false;
	int counter = 0;
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		watchdog_update();
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
		on = !on;
		vTaskDelayUntil(&last_wake_time, 500);
	}
}

void sample_task(__unused void* params) {
	// init spi
	spi_inst_t* spi = SPI_DEVICE;
	spi_init(spi, 1000 * 1000);
	gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
	gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);
	gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI);

	// init chip select
	gpio_init(SPI_CSN_PIN);
	gpio_set_dir(SPI_CSN_PIN, GPIO_OUT);
	gpio_put(SPI_CSN_PIN, 1);

	sensor::MAX31855 dev{spi, SPI_CSN_PIN};
	int counter = 0;
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		auto data = dev.Sample();
		function_guard<void>(temperature_mutex, [data = std::move(data)] { temperature_data = std::move(data); });

		vTaskDelayUntil(&last_wake_time, 1000);
	}
}

bool is_wifi_connected() {
	return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

void connect_wifi() {
	while (!is_wifi_connected()) {
		log::info("Connecting to WiFi...");
		sntp_stop();
		function_guard<void>(ip_addresss_mutex, [] { ip_address = std::nullopt; });

		auto connect_ret = cyw43_arch_wifi_connect_blocking(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

		auto link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
		if (connect_ret == 0 && link_status == CYW43_LINK_UP) {
			auto local_IP = ip4_addr_get_u32(netif_ip4_addr(cyw43_state.netif));
			log::info("Successfully connected to WiFi: IP Address {}.{}.{}.{}",
			          (local_IP & 0xff),
			          ((local_IP >> 8) & 0xff),
			          ((local_IP >> 16) & 0xff),
			          (local_IP >> 24));

			sntp_init();
			function_guard<void>(ip_addresss_mutex, [local_IP] { ip_address = local_IP; });
		} else {
			log::error("Connect failed, retrying");
			vTaskDelay(1000);
		}
	}
}

std::unique_ptr<netbuf> CreateNetbufRef(const char* buf, size_t len) {
	auto nb = netbuf_new();

	if (netbuf_ref(nb, buf, len) == ERR_OK) {
		return std::unique_ptr<netbuf>(nb);
	}

	netbuf_delete(nb);
	return nullptr;
}

std::optional<uint8_t> extract_number(const std::string& str, const std::string& prefix) {
	if (str.compare(0, prefix.size(), prefix) != 0) {
		return std::nullopt;
	}

	auto num_str = str.substr(prefix.size());
	if (num_str.empty()) {
		return std::nullopt;
	}

	if (num_str.size() > 3) {
		return std::nullopt;
	}

	uint8_t ret_val;
	auto result = std::from_chars(num_str.data(), num_str.data() + num_str.size(), ret_val);
	if (result.ptr != (num_str.data() + num_str.size())) {
		return std::nullopt;
	}

	return ret_val;
}

void udp_recv_task(__unused void* params) {
	char buf[256];

	while (true) {
		auto len = xMessageBufferReceive(udp_recv_msg_buffer, buf, sizeof(buf), 1000);
		if (len > 0) {
			std::string str{buf, len};
			str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());

			log::debug("Received UDP message: {}", str);
			auto temp = extract_number(str, "temp");
			auto pwm_percent = extract_number(str, "pwm");
			if (temp.has_value()) {
				if (temp.value() <= MAX_TEMPERATURE_SET_POINT_C) {
					HeaterCommand cmd = TemperatureSetPoint{temp.value()};
					xQueueSend(heater_command_queue, &cmd, 100);
					log::info("UDP Temperature Set Point: {}", temp.value());
				}
			} else if (pwm_percent.has_value()) {
				if (pwm_percent.value() <= 100) {
					HeaterCommand cmd = PwmSetPoint{pwm_percent.value()};
					xQueueSend(heater_command_queue, &cmd, 100);
					log::info("UDP PWM Command: {}", pwm_percent.value());
				}
			} else if (str == "idle" || str == "stop") {
				HeaterCommand cmd = DisableHeater{};
				xQueueSend(heater_command_queue, &cmd, 100);
				log::info("UDP Stop Command");
			} else if (str == "curve") {
				HeaterCommand cmd = StartHeatCurve{};
				xQueueSend(heater_command_queue, &cmd, 100);
				log::info("UDP Start Heat Curve Command");
			}
		}
	}
}

void udp_recv_cb(void*, udp_pcb*, pbuf* buf, const ip4_addr* ip, uint16_t port) {
	BaseType_t taskWoken = pdFALSE;
	xMessageBufferSendFromISR(udp_recv_msg_buffer, buf->payload, buf->len, &taskWoken);
	pbuf_free(buf);
	portYIELD_FROM_ISR(taskWoken);
}

const char* heater_state_type_name(HeaterStateType type) {
	switch (type) {
	case HeaterStateType::Idle:
		return "Idle";
	case HeaterStateType::Pwm:
		return "Pwm";
	case HeaterStateType::Temperature:
		return "Temperature";
	case HeaterStateType::Curve:
		return "Curve";
	}
	return "Unknown";
}

void network_task(__unused void* params) {
	if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA)) {
		log::error("WiFi init failed");
		return;
	}

	cyw43_arch_enable_sta_mode();

	{
		sntp_setoperatingmode(SNTP_OPMODE_POLL);

		ip_addr_t ntp_ip;
		ipaddr_aton(NTP_HOST, &ntp_ip);
		sntp_setserver(0, &ntp_ip);
	}

	{
		auto task_return = xTaskCreate(blink_task, "blink_task", configMINIMAL_STACK_SIZE, nullptr, 1, nullptr);
		assert(task_return == pdPASS);
	}

	auto udp_connection = udp_new();
	assert(udp_connection != nullptr);
	ip_set_option(udp_connection, SOF_BROADCAST);

	udp_bind(udp_connection, IP_ADDR_ANY, UDP_BIND_PORT);
	udp_recv(udp_connection, udp_recv_cb, nullptr);

	ip_addr_t recv_host;
	ipaddr_aton(UDP_REMOTE_HOST, &recv_host);

	// sensor::AHT21::Data sample_data{};
	char json_buf[256];
	char optional_json_buf[128];
	char temperature_buf[8];
	uint32_t sequence_number{};
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		if (!is_wifi_connected()) {
			connect_wifi();
		} else {
			vTaskDelayUntil(&last_wake_time, 1000);

			auto tick_now = xTaskGetTickCount();
			datetime_t date_now;
			if (!rtc_get_datetime(&date_now)) {
				continue;
			}

			struct tm tm_now {};
			tm_now.tm_year = date_now.year - 1900;
			tm_now.tm_mon = date_now.month - 1;
			tm_now.tm_mday = date_now.day;
			tm_now.tm_wday = date_now.dotw;
			tm_now.tm_hour = date_now.hour;
			tm_now.tm_min = date_now.min;
			tm_now.tm_sec = date_now.sec;
			tm_now.tm_isdst = false;
			auto time_now = mktime(&tm_now);

			auto state = function_guard<HeaterState>(heater_state_mutex, [] { return heater_state; });
			if (state.measured_temperature_C.has_value()) {
				snprintf(temperature_buf, sizeof(temperature_buf), "%.1f", *state.measured_temperature_C);
			} else {
				snprintf(temperature_buf, sizeof(temperature_buf), "null");
			}

			switch (state.state_type) {
			case HeaterStateType::Temperature:
				snprintf(optional_json_buf, sizeof(optional_json_buf), ", \"target_temperature_C\": %u", state.target_temperature_C);
				break;
			case HeaterStateType::Curve: {
				TickType_t elapsed = state.current_tick - state.curve_tbegin;
				TickType_t remain = (state.current_tick < state.curve_tend) ? (state.curve_tend - state.current_tick) : 0;
				snprintf(optional_json_buf,
				         sizeof(optional_json_buf),
				         ", \"target_temperature_C\": %u, \"curve_elapsed_ms\": %u, \"curve_remaining_ms\": %u",
				         state.target_temperature_C,
				         elapsed,
				         remain);
			} break;
			default:
				sprintf(optional_json_buf, "");
			}

			auto len = snprintf(
			    json_buf,
			    sizeof(json_buf),
			    "{\"id\": %u, \"ttag_s\": %lld, \"uptime_ms\": %u, \"heater_state\": \"%s\", \"pwm_percent\": %u, \"measured_temperatre_C\": %s%s }\n",
			    ++sequence_number,
			    time_now,
			    state.current_tick,
			    heater_state_type_name(state.state_type),
			    state.pwm_percent,
			    temperature_buf,
			    optional_json_buf);

			auto udp_buf = CreateNetbufRef(json_buf, len);
			if (udp_buf) {
				auto ret = udp_sendto(udp_connection, udp_buf->p, &recv_host, UDP_REMOTE_PORT);
				log::info("network_task -> success: {}, temperature_C: {} ", (ret == ERR_OK) ? "true" : "false", temperature_buf);
			}
		}
	}
}

void tusb_task(__unused void* params) {
	while (true) {
		tud_task();
		vTaskDelay(1);
	}
}

void cdc_connected_task(__unused void* params) {
	uint8_t itf{};
	while (true) {
		auto len = xMessageBufferReceive(cdc_connected_msg_buffer, &itf, 1, 1000);
		if (len == 1) {
			log::info("Connected to b1g enviro monitor: {}", VERSION);
		}
	}
}

void sntp_task(__unused void* params) {
	uint32_t ntp_sec{};
	while (true) {
		auto len = xMessageBufferReceive(sntp_msg_buffer, &ntp_sec, sizeof(ntp_sec), 7200000);
		if (len == sizeof(ntp_sec)) {
			static uint32_t ntp_to_unix = (70 * 365 + 17) * 86400U;
			time_t unix_time = ntp_sec - ntp_to_unix;

			struct tm datetime;
			gmtime_r(&unix_time, &datetime);

			datetime_t t = {
			    .year = static_cast<int16_t>(datetime.tm_year + 1900),
			    .month = static_cast<int8_t>(datetime.tm_mon + 1),
			    .day = static_cast<int8_t>(datetime.tm_mday),
			    .dotw = static_cast<int8_t>(datetime.tm_wday), // 0 is Sunday, so 5 is Friday
			    .hour = static_cast<int8_t>(datetime.tm_hour),
			    .min = static_cast<int8_t>(datetime.tm_min),
			    .sec = static_cast<int8_t>(datetime.tm_sec)};

			if (rtc_set_datetime(&t)) {
				char buf[64];
				datetime_to_str(buf, sizeof(buf), &t);
				log::info("Got SNTP Time: {}", buf);
			} else {
				log::error("SNTP Time Failed");
			}
		}
	}
}

uint8_t get_ip_byte(uint32_t ip, uint32_t shift) {
	return (ip >> shift) & 0xff;
}

void get_ip_address_str(char* buf, size_t max_len) {
	auto ip = function_guard<std::optional<uint32_t>>(ip_addresss_mutex, [] { return ip_address; });
	if (ip.has_value()) {
		snprintf(buf, max_len, "%u.%u.%u.%u",
		         get_ip_byte(ip.value(), 0),
		         get_ip_byte(ip.value(), 8),
		         get_ip_byte(ip.value(), 16),
		         get_ip_byte(ip.value(), 24));
	} else {
		snprintf(buf, max_len, "disconnected");
	}
}

void get_datetime_str(char* buf, size_t max_len) {
	datetime_t date_now;
	if (rtc_get_datetime(&date_now)) {
		datetime_to_str(buf, max_len, &date_now);
	} else {
		snprintf(buf, max_len, "unknown");
	}
}

void get_temperature_meas_str(char* buf, size_t max_len) {
	auto data = function_guard<std::optional<sensor::MAX31855::Data>>(temperature_mutex, [] { return temperature_data; });
	if (data.has_value()) {
		snprintf(buf, max_len, "%2.1f", data->temperature_C);
	} else {
		snprintf(buf, max_len, "????");
	}
}

void lcd_task(__unused void* params) {
	i2c_inst_t* i2c = I2C_DEVICE;
	i2c_init(i2c, 400 * 1000);
	gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_SDA_PIN);
	gpio_pull_up(I2C_SCL_PIN);

	using namespace pico_ssd1306;

	SSD1306 display = SSD1306(i2c, 0x3C, Size::W128xH32);
	display.setOrientation(0);

	char ip_str_buf[32];
	char datetime_str_buf[32];
	char temperature_meas_str_buf[8];
	char line_buf[32] = {};
	auto counter = 0;
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		auto state = function_guard<HeaterState>(heater_state_mutex, [] { return heater_state; });

		get_ip_address_str(ip_str_buf, sizeof(ip_str_buf));
		get_datetime_str(datetime_str_buf, sizeof(datetime_str_buf));
		get_temperature_meas_str(temperature_meas_str_buf, sizeof(temperature_meas_str_buf));

		snprintf(line_buf, sizeof(line_buf), "Uptime: %d", counter++);

		display.clear();
		drawText(&display, font_b1g_5x8, ip_str_buf, 0, 0);
		drawText(&display, font_b1g_5x8, datetime_str_buf, 0, 8);
		drawText(&display, font_b1g_5x8, line_buf, 0, 16);
		display.sendBuffer();

		vTaskDelayUntil(&last_wake_time, 1000);
	}
}

} // namespace

// Invoked when CDC interface connection status changes
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
	if (itf == CDC_ITF_LOG && dtr) {
		BaseType_t taskWoken = pdFALSE;
		xMessageBufferSendFromISR(cdc_connected_msg_buffer, &itf, 1, &taskWoken);
		portYIELD_FROM_ISR(taskWoken);
	}
}

void sntp_set_system_time_us(uint32_t sec, uint64_t us) {
	BaseType_t taskWoken = pdFALSE;
	xMessageBufferSendFromISR(sntp_msg_buffer, &sec, sizeof(sec), &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

int main() {
	watchdog_enable(1000, 1);

	auto task_return = xTaskCreate(network_task, "network_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(sample_task, "sample_task", 2048, nullptr, 2, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(heater_task, "heater_task", 2048, nullptr, 4, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(button_task, "button_task", 2048, nullptr, 3, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(udp_recv_task, "udp_recv_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(tusb_task, "tusb_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(cdc_connected_task, "cdc_connected_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(sntp_task, "sntp_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(lcd_task, "lcd_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	sample_msg_buffer = xMessageBufferCreate(SAMPLE_BUF_SIZE);
	assert(sample_msg_buffer != nullptr);

	udp_recv_msg_buffer = xMessageBufferCreate(UDP_RECV_BUF_SIZE);
	assert(udp_recv_msg_buffer != nullptr);

	cdc_connected_msg_buffer = xMessageBufferCreate(CDC_CONNECTED_BUF_SIZE);
	assert(cdc_connected_msg_buffer != nullptr);

	heater_command_queue = xQueueCreate(HEATER_COMMAND_QUEUE_SIZE, sizeof(HeaterCommand));

	sntp_msg_buffer = xMessageBufferCreate(SNTP_BUF_SIZE);
	assert(sntp_msg_buffer != nullptr);

	temperature_mutex = xSemaphoreCreateMutex();
	assert(temperature_mutex != nullptr);

	ip_addresss_mutex = xSemaphoreCreateMutex();
	assert(ip_addresss_mutex != nullptr);

	heater_state_mutex = xSemaphoreCreateMutex();
	assert(heater_state_mutex != nullptr);

	tusb_init(); // initialize tinyusb stack
	rtc_init();
	log::error("Starting FreeRTOS on core 0");
	vTaskStartScheduler();

	return 0;
}

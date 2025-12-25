#include <stdio.h>
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// cấu hình I2C cho INA219
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define INA219_ADDR                 0x40
#define INA219_CONFIG_32V_2A_CONT   0x399F // 32V, 2A, bus+shunt continuous (datasheet) đo chính xác hơn 
// cấu hình PWM cho điều khiển DC-DC
#define PWM_GPIO                    25
#define PWM_FREQ_HZ                 20000
#define PWM_RESOLUTION              LEDC_TIMER_10_BIT
#define PWM_CHANNEL                 LEDC_CHANNEL_0
#define Duty_Max                   0.6f
static const char *TAG = "MPPT";

// INA219 register addresses
#define INA219_REG_CONFIG           0x00
#define INA219_REG_SHUNTVOLTAGE     0x01
#define INA219_REG_BUSVOLTAGE       0x02
#define INA219_REG_POWER            0x03
#define INA219_REG_CURRENT          0x04
#define INA219_REG_CALIBRATION      0x05

// Hàm khởi tạo I2C
void i2c_master_init()
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};
	i2c_param_config(I2C_MASTER_NUM, &conf);
	i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Ghi 16 bit vào INA219
esp_err_t ina219_write16(uint8_t reg, uint16_t value)
{
    uint8_t data[3];
    data[0] = reg;// địa chỉ thanh ghi trong INA219
    data[1] = (value >> 8) & 0xFF; // ghi giá trị 16 bit, MSB trước
    data[2] = value & 0xFF;// LSB sau
    return i2c_master_write_to_device(
		I2C_MASTER_NUM, 
		INA219_ADDR, 
		data, 
		3, 
		1000 / portTICK_PERIOD_MS);
}

// Đọc 16 bit từ INA219
esp_err_t ina219_read16(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    esp_err_t ret = i2c_master_write_read_device(
		I2C_MASTER_NUM, 
		INA219_ADDR, 
		&reg, 1, 
		data, 2, 
		1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        *value = (data[0] << 8) | data[1];
    }
    return ret;
}

// Khởi tạo INA219 (ví dụ: calibration cho 0.1 ohm shunt, 3.2A max)
void ina219_init()
{
	// Ghi thanh ghi CONFIG: 32V, continuous, bus+shunt
	ina219_write16(INA219_REG_CONFIG, INA219_CONFIG_32V_2A_CONT);
	// Calibration value cho 0.1Ω shunt, 2A max (có thể chỉnh lại nếu cần)
	ina219_write16(INA219_REG_CALIBRATION, 4096);
}

// Lọc số đơn giản (low-pass filter)
float filter_lpf(float prev, float input, float alpha) {
	return alpha * input + (1.0f - alpha) * prev;
}

// Đọc điện áp bus (V)
float ina219_read_bus_voltage()
{
	uint16_t value = 0;
	ina219_read16(INA219_REG_BUSVOLTAGE, &value);
	// 1 bit = 4mV, 13 bit data
	return ((value >> 3) * 4.0) / 1000.0;// datasheet INA219
}

// Đọc dòng (A)
float ina219_read_current()
{
	uint16_t value = 0;
	ina219_read16(INA219_REG_CURRENT, &value);
	// Giá trị này phụ thuộc vào calibration, ví dụ 1LSB = 0.1mA
	int16_t signed_value = (int16_t)value;
	return signed_value * 0.0001; // 0.1mA/LSB
}

// Khởi tạo PWM GPIO25
void pwm_init()
{
	ledc_timer_config_t timer_conf = {
		.speed_mode = LEDC_HIGH_SPEED_MODE, // chế độ high-speed 
		.timer_num = LEDC_TIMER_0,// sử dụng timer 0
		.duty_resolution = PWM_RESOLUTION,// độ phân giải 10 bit
		.freq_hz = PWM_FREQ_HZ,// tần số 20kHz
		.clk_cfg = LEDC_AUTO_CLK// tự chọn clock nguồn
	};
	ledc_timer_config(&timer_conf);

	ledc_channel_config_t channel_conf = {
		.gpio_num = PWM_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,// chế độ high-speed
		.channel = PWM_CHANNEL,// kênh 0
		.timer_sel = LEDC_TIMER_0,// sử dụng timer 0 đã cấu hình
		.duty = 0,// khởi tạo duty 0
		.hpoint = 0// hpoint 0
	};	
	ledc_channel_config(&channel_conf);
}

// Set duty PWM (0.0 - 1.0)
void pwm_set_duty(float duty)
{
	if (duty < 0) duty = 0;
	if (duty > Duty_Max) duty = Duty_Max;
	uint32_t duty_val = (uint32_t)(duty * ((1 << 10) - 1));
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, duty_val);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);
}

// Thuật toán MPPT P&O
float mppt_perturb_observe(float v, float i)
{
	static float prev_v = 0, prev_p = 0, duty = 0.05;
	float p = v * i;
	float delta_v = v - prev_v;
	float delta_p = p - prev_p;
	float step = 0.001f; // giảm độ hung hăng
	if (delta_p > 0) {
		if (delta_v < 0) duty += step;
		else duty -= step;
	} else {
		if (delta_v < 0) duty -= step;
		else duty += step;
	}
	prev_v = v;
	prev_p = p;
	return duty;
}

void app_main(void)
{
	i2c_master_init();
	ina219_init();
	pwm_init();

	float duty = 0.0f; // khởi tạo duty ban đầu
	pwm_set_duty(duty);// đặt duty = 0.0
	ESP_LOGI(TAG, "SOFT-START: PWM duty = 0.0, chờ 500ms trước khi hệ thống bắt đầu");
	vTaskDelay(pdMS_TO_TICKS(500));

	// Đo Voc (điện áp hở mạch)
	float Voc = 0, vpin = 0, ipin = 0;
	for (int k = 0; k < 10; ++k) {
		float v = ina219_read_bus_voltage();
		Voc += v;
		vTaskDelay(pdMS_TO_TICKS(20));
	}
	Voc /= 10.0f; // tính giá trị trung bình	
	ESP_LOGI(TAG, "Đo Voc = %.2fV", Voc);

	// Lọc số khởi tạo
	vpin = Voc; //
	ipin = 0;//
	float alpha = 0.2f; // hệ số lọc lpf
	float duty_min = 0.05f, duty_max = 0.6f; // giới hạn duty tối đa 0.6
	float step = 0.003f;
	int state = 0; // 0: soft-start, 1: MPPT, 2: bảo vệ
	int 	soft_start_done = 0;

	while (1) {
		float v = ina219_read_bus_voltage();
		float i = ina219_read_current();
		vpin = filter_lpf(vpin, v, alpha);
		ipin = filter_lpf(ipin, i, alpha);
		float p = vpin * ipin;

		// Bảo vệ: tắt PWM nếu Vpin < 2V hoặc lỗi INA219 hoặc quá dòng
		if (vpin < 2.0f || vpin > Voc * 1.2f || ipin > 2.0f) {
			pwm_set_duty(duty_min); // duty về thấp ngay
			state = 2;
			ESP_LOGW(TAG, "BẢO VỆ: Vpin=%.2fV, Ipin=%.2fA, Duty=%.3f", vpin, ipin, duty_min);
			vTaskDelay(pdMS_TO_TICKS(200));
			continue;
		}

		// Soft-start: chỉ tăng duty khi Vpin > 0.6Voc mới chuyển sang MPPT
		if (!soft_start_done) {
			if (vpin < 0.6f * Voc) {
				duty = duty_min;
				pwm_set_duty(duty);
				state = 0; // soft-start
				ESP_LOGI(TAG, "SOFT-START: Vpin=%.2fV < 0.6Voc=%.2fV, duty về thấp=%.3f", vpin, 0.6f*Voc, duty);
				vTaskDelay(pdMS_TO_TICKS(50));
				continue;
			} else {
				soft_start_done = 1;
				ESP_LOGI(TAG, "SOFT-START DONE, chuyển sang MPPT");
			}
		}

		// MPPT P&O, luôn giữ Vpin >= 0.6Voc
		if (vpin < 0.6f * Voc) {
			duty = duty_min;
			pwm_set_duty(duty);
			ESP_LOGW(TAG, "MPPT BẢO VỆ: Vpin=%.2fV < 0.6Voc=%.2fV, duty về thấp=%.3f", vpin, 0.6f*Voc, duty);
			vTaskDelay(pdMS_TO_TICKS(50));
			continue;
		}

		// MPPT
		state = 1;
		duty = mppt_perturb_observe(vpin, ipin);
		if (duty > duty_max) duty = duty_max;
		if (duty < duty_min) duty = duty_min;
		pwm_set_duty(duty);
		ESP_LOGI(TAG, "MPPT: V=%.2fV I=%.3fA P=%.3fW Duty=%.3f", vpin, ipin, p, duty);
		vTaskDelay(pdMS_TO_TICKS(40));
	}
}

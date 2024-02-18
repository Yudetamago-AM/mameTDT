#include <stdint.h>
#include <stdio.h>
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "pio_out.pio.h"

#define abs(x) (((x)<0)?(-(x)):(x))

const uint8_t pin_mux1 = 18; // LOW: REF_START, HIGH: TRAN_START
const uint8_t pin_mux2 = 8; // LOW: TRAN_START, HIGH: TRAN_STOP
const uint8_t pin_ref_start = 19;
const uint8_t pin_trig_pulse = 21; // Rising edge: Trigger pulse
const uint8_t pin_mes_vcc = 6; // LOW: ON, HIGH: OFF
const uint8_t pin_int_tdc = 13; // HIGH: data ready, LOW: else

const uint8_t pin_spi1_cs = 9; // Active LOW
const uint8_t pin_spi1_sck = 10;
const uint8_t pin_spi1_mosi = 11;
const uint8_t pin_spi1_miso = 12;

enum tdc_mode {
	RISE_RISE = 0b10000001,
	RISE_FALL = 0b10010001,
	FALL_RISE = 0b10001001,
	FALL_FALL = 0b10011001,
};

uint32_t readData(uint8_t addr) {
	uint8_t buf[3] = {0};
	gpio_put(pin_spi1_cs, 0);
	asm volatile("nop \n nop \n nop");
	spi_write_blocking(spi1, &addr, 1);
	spi_read_blocking(spi1, 0x00, buf, 3);
	asm volatile("nop");
	gpio_put(pin_spi1_cs, 1);
	return ((buf[0] << 16) | (buf[1] << 8) | buf[2]);
}

double measure(PIO pio, uint8_t sm, enum tdc_mode mode) {
	uint8_t wbuf[2] = {0};
	//    uint32_t time_before = time_us_32(), time_after;
	//    printf("trigg. %d ms, ", time_before / 1000);

	// start generating pulse by PIO
	pio_sm_set_enabled(pio, sm, true);

	// start measuring in TDC
	gpio_put(pin_spi1_cs, 0);
	asm volatile("nop \n nop \n nop");
	wbuf[0] = 0b01000000; // Write Reg 0x00
						  //  wbuf[1] = 0b10000001; // Mode 1, START: Rising, STOP: Rising Edge
						  //	wbuf[1] = 0b10010001; // Mode 1, START: Rising, STOP: Falling Edge
						  //  wbuf[1] = 0b10011001; // Mode 1, START: Falling, STOP: Falling Edge
						  //  wbuf[1] = 0b10001001; // Mode 1, START: Falling, STOP: Rising Edge
						  //	wbuf[2] = 0b01111000; // 128 times averaging(should be auto incrementing enabled)
	wbuf[1] = mode;
	spi_write_blocking(spi1, wbuf, 2);
	asm volatile("nop");
	gpio_put(pin_spi1_cs, 1);

	//while (gpio_get(pin_int_tdc) == 0){tight_loop_contents();}
	sleep_us(2);
	//    printf("gpio_get(pin_int_tdc): %d\n", gpio_get(pin_int_tdc));

	// end generating pulse by PIO
	pio_sm_set_enabled(pio, sm, false);

	// read out data
	uint32_t time1 = readData(0x10);
	//    uint32_t clock_cnt1 = readData(0x11);
	uint32_t calib1 = readData(0x1b);
	uint32_t calib2 = readData(0x1c);
	double cal_count = (calib2 - calib1) / (10.0 - 1.0);
	double norm_LSB = 1 / (cal_count * 12.0 / 1000);
	//double tof1 = time1 * norm_LSB + clock_cnt1 * (1 / (12.0));
	double tof1 = time1 * norm_LSB;
	//   printf("tof: %f ns, elapsed: %d us\n", tof1, (time_us_32()-time_before));
	//printf("time1: %d, clock_cnt1: %d, calib1: %d, calib2: %d, cal_count: %f, norm_LSB: %f\n", time1, clock_cnt1, calib2, cal_count, norm_LSB);
	//   printf("time1: %d, calib1: %d, calib2: %d, cal_count: %f, norm_LSB: %f\n", time1, calib1, calib2, cal_count, norm_LSB);
	// 	time_after = time_us_32();
	//	printf("elapsed: %d us, ", time_after - time_before);

	return tof1;
}

double measure_avg256(PIO pio, uint8_t sm, uint8_t mux2, uint8_t mode) {
	double sum = 0.0, start = 0.0, stop = 0.0, diff = 0.0;
	printf("trigg. %d ms, mux2: 0x%x, mode: 0x%x ", time_us_32() / 1000, mux2, mode);
	printf("START: ");
	gpio_put(pin_mux2, (mux2>>1));
	asm volatile("nop");

	sum = 0;
	//			before = time_us_32();
	for (int8_t i = 0; i < 64; i++) sum += measure(pio, sm, mode);
	for (int8_t i = 0; i < 64; i++) sum += measure(pio, sm, mode);
	for (int8_t i = 0; i < 64; i++) sum += measure(pio, sm, mode);
	for (int8_t i = 0; i < 64; i++) sum += measure(pio, sm, mode);
	//			after = time_us_32();
	//			printf("T: %d us, ", (after - before) / 64);
	start = sum / 256.0;

	printf("tof: %2.3f, ", start);
	printf("STOP: ");
	gpio_put(pin_mux2, (mux2&0x01));
	asm volatile("nop");

	sum = 0;
	for (int8_t i = 0; i < 64; i++) sum += measure(pio, sm, mode);
	for (int8_t i = 0; i < 64; i++) sum += measure(pio, sm, mode);
	for (int8_t i = 0; i < 64; i++) sum += measure(pio, sm, mode);
	for (int8_t i = 0; i < 64; i++) sum += measure(pio, sm, mode);
	stop = sum / 256.0;
	diff = stop - start;
	printf("tof: %2.3f, ", stop);
	printf("diff: %2.3f", diff);
	printf("\n");

	return diff;
}

int main(void) {
	stdio_init_all();
	sleep_ms(2000);
	printf("hello, this is main\n");

	//    gpio_init(pin_trig_pulse);
	//    gpio_set_dir(pin_trig_pulse, GPIO_OUT);
	//    gpio_put(pin_trig_pulse, 0); // no pulse

	gpio_init(pin_mes_vcc);
	gpio_set_dir(pin_mes_vcc, GPIO_OUT);
	gpio_put(pin_mes_vcc, 0); // on

	gpio_init(pin_mux1);
	gpio_set_dir(pin_mux1, GPIO_OUT);
	gpio_put(pin_mux1, 0); // REF_START

	gpio_init(pin_mux2);
	gpio_set_dir(pin_mux2, GPIO_OUT);
	gpio_put(pin_mux2, 0);

	//    gpio_init(pin_ref_start);
	//    gpio_set_dir(pin_ref_start, GPIO_OUT);
	//    gpio_put(pin_ref_start, 0);

	gpio_init(pin_int_tdc);
	gpio_set_dir(pin_int_tdc, GPIO_IN);

	gpio_init(pin_spi1_cs);
	gpio_set_dir(pin_spi1_cs, GPIO_OUT);
	gpio_put(pin_spi1_cs, 1); // Active LOW

	spi_init(spi1, 10 * 1000 * 1000); // 10MHz
	gpio_set_function(pin_spi1_miso, GPIO_FUNC_SPI);
	gpio_set_function(pin_spi1_sck, GPIO_FUNC_SPI);
	gpio_set_function(pin_spi1_mosi, GPIO_FUNC_SPI);

	uint8_t wbuf[2] = {0};
	gpio_put(pin_spi1_cs, 0);
	asm volatile("nop \n nop \n nop");
	wbuf[0] = 0b01000011; // Write Reg 0x03
	wbuf[1] = 0b00000111; // enable interrupt
	spi_write_blocking(spi1, wbuf, 3);
	asm volatile("nop");
	gpio_put(pin_spi1_cs, 1);
	//    gpio_put(pin_spi1_cs, 0);
	//    asm volatile("nop \n nop \n nop");
	//    wbuf[0] = 0b01000001; // Write Reg 0x01
	//    wbuf[1] = 0b01111000; // enable 128 times averaging
	//    spi_write_blocking(spi1, wbuf, 3);
	//    asm volatile("nop");
	//    gpio_put(pin_spi1_cs, 1);

	// PIO settings
	PIO pio = pio0;
	uint8_t offset = pio_add_program(pio, &pio_out_program);
	uint8_t sm = pio_claim_unused_sm(pio, true);
	pio_out_program_init(pio, sm, offset, pin_ref_start);

	char c = 0;
	while (1) {
		if ((c = getchar_timeout_us(10*1000)) == 't') {
			// A: stArt, O: stOp, R: Rise, F: Fall
			double mes_AA_RR = 0.0, mes_AA_FF = 0.0,
				   mes_OO_RR = 0.0, mes_OO_FF = 0.0;
			const double V_RR = 2.8;
			double V_FF = 0.5;

			// START to START: mux2==0b00
			// RISE to RISE
			mes_AA_RR = measure_avg256(pio, sm, 0b00, RISE_RISE);
			// FALL to FALL
			mes_AA_FF = measure_avg256(pio, sm, 0b00, FALL_FALL);

			// START to STOP: mux2==0b01
			// RISE to RISE
			mes_OO_RR = measure_avg256(pio, sm, 0b01, RISE_RISE);
			// FALL to FALL
			mes_OO_FF = measure_avg256(pio, sm, 0b01, FALL_FALL);

			double t_RR = mes_OO_RR - abs(mes_AA_RR);
			double t_FF = mes_OO_FF - abs(mes_AA_FF);

			// guess true propagation delay
			double x = (V_FF*t_RR - V_RR*t_FF) / (t_FF - t_RR);
			x = abs(x);
			double t_p = (t_FF * x) / (V_FF + x);
			printf("V_RR: %1.1f V_FF: %1.1f ", V_RR, V_FF);
			printf("x: %2.3f t_p: %2.3f\n", x, t_p);

		}
		//sleep_ms(1);
	}
}

/* An STM32 HAL library written for the the MAX30100 pulse oximeter and heart rate sensor. */
/* Libraries by @eepj www.github.com/eepj */
#include "max30100_for_stm32_hal.h"
#include "main.h"
#include "stdio.h"
#include <math.h>
#ifdef __cplusplus
extern "C"{
#endif

I2C_HandleTypeDef *_max30100_ui2c;
UART_HandleTypeDef *_max30100_uuart;
uint8_t _max30100_it_byte = 0x00;
uint8_t _max30100_mode;
uint8_t _max30100_mode_prev;
uint16_t _max30100_ir_sample[16];
uint16_t _max30100_red_sample[16];
uint8_t _max30100_ir_current;
uint8_t _max30100_red_current;
uint8_t _max30100_ir_current_prev;
uint8_t _max30100_red_current_prev;
float _max30100_temp;

/* --- New: history buffers for improved algos --- */
#define HISTORY_SIZE 64
static uint32_t ir_history[HISTORY_SIZE];
static uint32_t red_history[HISTORY_SIZE];
static uint8_t history_idx = 0;
static uint8_t history_filled = 0;

/* --- Internal state for peak detection --- */
static uint32_t last_peak_time_ms = 0;
static uint8_t peak_flag = 0;
static float last_ir_rms = 0.0f;
static float last_filtered = 0.0f;
static float last_bpm = 0.0f;

void MAX30100_Init(I2C_HandleTypeDef *ui2c, UART_HandleTypeDef *uuart){
    _max30100_ui2c = ui2c;
    _max30100_uuart = uuart;

    // Reset thi?t b?
    MAX30100_WriteReg(MAX30100_MODE_CONFIG, (1 << MAX30100_RESET));
    HAL_Delay(10);

    // Thoát ch? d? shutdown (bit7 = 0)
    MAX30100_WriteReg(MAX30100_MODE_CONFIG, 0x00);
    HAL_Delay(10);

    // C?u hình SPO2: d? phân gi?i, t?c d? l?y m?u
    MAX30100_SetSpO2SampleRate(MAX30100_SPO2SR_100);  // 100Hz
    MAX30100_SetLEDPulseWidth(MAX30100_LEDPW_1600);   // Ð? phân gi?i cao nh?t

    // C?u hình dòng LED (IR và RED)
    MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_50_0, MAX30100_LEDCURRENT_50_0);

    // B?t ch? d? SPO2
    MAX30100_SetMode(MAX30100_SPO2_MODE);

    // Xóa FIFO d? b?t d?u m?i
    MAX30100_ClearFIFO();

    // Clear history
    for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
        ir_history[i] = 0;
        red_history[i] = 0;
    }
    history_idx = 0;
    history_filled = 0;

    HAL_Delay(100);
}



uint8_t MAX30100_ReadReg(uint8_t regAddr){
	uint8_t reg = regAddr, result;
	HAL_I2C_Master_Transmit(_max30100_ui2c, MAX30100_I2C_ADDR, &reg, 1, MAX30100_TIMEOUT);
	HAL_I2C_Master_Receive(_max30100_ui2c, MAX30100_I2C_ADDR, &result, 1, MAX30100_TIMEOUT);
	return result;
}

void MAX30100_WriteReg(uint8_t regAddr, uint8_t byte){
	uint8_t reg[2] = { regAddr, byte };
	HAL_I2C_Master_Transmit(_max30100_ui2c, MAX30100_I2C_ADDR, reg, 2, MAX30100_TIMEOUT);
}

void MAX30100_EnableInterrupt(uint8_t a_full, uint8_t tmp_rdy, uint8_t hr_rdy, uint8_t spo2){
	uint8_t itReg = ((a_full & 0x01) << MAX30100_ENB_A_FULL) | ((tmp_rdy & 0x01) << MAX30100_ENB_TMP_RDY) | ((hr_rdy & 0x01) << MAX30100_ENB_HR_RDY) | ((spo2 & 0x01) << MAX30100_ENB_SPO2_RDY);
	MAX30100_WriteReg(MAX30100_INTERRUPT_ENB, itReg);
}

void MAX30100_InterruptHandler(void){
	uint8_t itReg = MAX30100_ReadReg(MAX30100_INTERRUPT);
	if((itReg >> MAX30100_A_FULL) & 0x01){
		MAX30100_ReadFIFO();
		if(_max30100_mode == MAX30100_HRONLY_MODE)
			MAX30100_PlotIrToUART(_max30100_uuart, _max30100_ir_sample, 16);
		else if(_max30100_mode == MAX30100_SPO2_MODE)
			MAX30100_PlotBothToUART(_max30100_uuart, _max30100_red_sample, _max30100_ir_sample, 16);
		MAX30100_SetMode(_max30100_mode);
	}else if((itReg >> MAX30100_TMP_RDY) & 0x01){
		_max30100_temp = MAX30100_ReadTemperature();
		MAX30100_EnableInterrupt(1, 0, 0, 0);
	}else if((itReg >> MAX30100_HR_RDY) & 0x01){

	}else if((itReg >> MAX30100_SPO2_RDY) & 0x01){

	}
}

void MAX30100_SetMode(enum MAX30100_Mode mode){
	_max30100_mode = mode;
	uint8_t modeReg = (MAX30100_ReadReg(MAX30100_MODE_CONFIG) & ~(0x07)) | ((mode << MAX30100_MODE) & 0x07);
	if(mode == MAX30100_SPO2_MODE)
		modeReg |= 0x08;
	else
		modeReg &= ~0x08;
	MAX30100_WriteReg(MAX30100_MODE_CONFIG, modeReg);
	if(_max30100_mode == MAX30100_SPO2_MODE)
		MAX30100_EnableInterrupt(0, 1, 0, 0);
	else if(_max30100_mode == MAX30100_HRONLY_MODE)
		MAX30100_EnableInterrupt(1, 0, 0, 0);
	else
		MAX30100_EnableInterrupt(0, 0, 0, 0);
}
//ok
void MAX30100_SetSpO2SampleRate(enum MAX30100_SpO2SR sr){
	uint8_t spo2Reg = MAX30100_ReadReg(MAX30100_SPO2_CONFIG);
	spo2Reg = ((sr << MAX30100_SPO2_SR) & 0x1c) | (spo2Reg & ~0x1c);
	MAX30100_WriteReg(MAX30100_SPO2_CONFIG, spo2Reg);
}
//ok
void MAX30100_SetLEDPulseWidth(enum MAX30100_LEDPulseWidth pw){
	uint8_t spo2Reg = MAX30100_ReadReg(MAX30100_SPO2_CONFIG);
	spo2Reg = ((pw << MAX30100_LED_PW) & 0x03) | (spo2Reg & ~0x03);
	MAX30100_WriteReg(MAX30100_SPO2_CONFIG, spo2Reg);
}
void MAX30100_SetLEDCurrent(enum MAX30100_LEDCurrent redpa, enum MAX30100_LEDCurrent irpa){
	_max30100_red_current = redpa;
	_max30100_ir_current = irpa;
	MAX30100_WriteReg(MAX30100_LED_CONFIG, (redpa << MAX30100_LED_RED_PA) | irpa);
}

void MAX30100_ClearFIFO(void){
	MAX30100_WriteReg(MAX30100_FIFO_WR_PTR, 0x00);
	MAX30100_WriteReg(MAX30100_FIFO_RD_PTR, 0x00);
	MAX30100_WriteReg(MAX30100_OVF_COUNTER, 0x00);
}

void MAX30100_ReadFIFO(void){
	//uint8_t fifo_wr_ptr = MAX30100_ReadReg(MAX30100_FIFO_WR_PTR);
	//uint8_t fifo_rd_ptr = MAX30100_ReadReg(MAX30100_FIFO_RD_PTR);
	uint8_t num_sample = 64;//(fifo_wr_ptr - fifo_rd_ptr) * 4;
	uint8_t fifo_data[64] = { 0 };
	uint8_t reg = MAX30100_FIFO_DATA;
	HAL_I2C_Master_Transmit(_max30100_ui2c, MAX30100_I2C_ADDR, &reg, 1, MAX30100_TIMEOUT);
	HAL_I2C_Master_Receive(_max30100_ui2c, MAX30100_I2C_ADDR, fifo_data, num_sample, MAX30100_TIMEOUT);
	for(uint8_t i = 0; i < num_sample; i += 4){
		_max30100_ir_sample[i / 4] = (fifo_data[i] << 8) | fifo_data[i + 1];
		_max30100_red_sample[i / 4] = (fifo_data[i + 2] << 8) | fifo_data[i + 3];
	}

}

float MAX30100_ReadTemperature(){
	int8_t tempInt = (int8_t) MAX30100_ReadReg(MAX30100_TMP_INTEGER);
	uint8_t tempFrac = MAX30100_ReadReg(MAX30100_TMP_FRACTION);
	return (tempInt + tempFrac / 16.0);
}

void MAX30100_PlotTemperatureToUART(UART_HandleTypeDef *uuart){
	uint8_t tempInt = _max30100_temp / 1;
	uint8_t tempFrac = (_max30100_temp - tempInt) * 10;
	char data[15];
	sprintf(data, "temp:%d.%d\n", tempInt, tempFrac);
	HAL_UART_Transmit(uuart, (uint8_t*)data, strlen(data), MAX30100_TIMEOUT);
}

void MAX30100_PlotIrToUART(UART_HandleTypeDef *uuart, uint16_t *samples, uint8_t sampleSize){
	char data[10];
	for(uint8_t i = 0; i< sampleSize; i++){
		sprintf(data, "s:%d\n", samples[i]);
		HAL_UART_Transmit(uuart, (uint8_t*)data, strlen(data), MAX30100_TIMEOUT);
	}
}

void MAX30100_PlotBothToUART(UART_HandleTypeDef *uuart, uint16_t *samplesRed, uint16_t *samplesIr, uint8_t sampleSize){
	char data[20];
	for(uint8_t i = 0; i< sampleSize; i++){
		sprintf(data, "red:%d\tir:%d\n", samplesRed[i], samplesIr[i]);
		HAL_UART_Transmit(uuart, (uint8_t*)data, strlen(data), MAX30100_TIMEOUT);
	}
}

void MAX30100_Stop(void){
	_max30100_mode = MAX30100_IDLE_MODE;
	MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_0_0, MAX30100_LEDCURRENT_0_0);
	MAX30100_WriteReg(MAX30100_INTERRUPT_ENB, 0x00);
}

void MAX30100_Pause(void){
	_max30100_mode_prev = _max30100_mode;
	_max30100_red_current_prev = _max30100_red_current;
	_max30100_ir_current_prev = _max30100_ir_current;
	MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_0_0, MAX30100_LEDCURRENT_0_0);
	MAX30100_SetMode(MAX30100_IDLE_MODE);
}

void MAX30100_Resume(void){
	MAX30100_SetLEDCurrent(_max30100_red_current_prev, _max30100_ir_current_prev);
	MAX30100_SetMode(_max30100_mode_prev);
}

// Ir and red algorithms (improved)

/* Helper: push latest raw samples into history buffer.
   Call this after MAX30100_ReadFIFO() — we take index 0 sample as newest. */
static void _push_history(void)
{
    uint32_t ir_val = _max30100_ir_sample[0];
    uint32_t red_val = _max30100_red_sample[0];

    ir_history[history_idx] = ir_val;
    red_history[history_idx] = red_val;

    history_idx++;
    if (history_idx >= HISTORY_SIZE) {
        history_idx = 0;
        history_filled = 1;
    }
}

/* Helper: compute mean of history (use only filled length if not yet full) */
static float _history_mean(uint32_t *buf)
{
    uint32_t len = history_filled ? HISTORY_SIZE : history_idx;
    if (len == 0) return 0.0f;
    uint64_t sum = 0;
    for (uint32_t i = 0; i < len; i++) sum += buf[i];
    return (float)sum / (float)len;
}

/* Helper: compute RMS (root mean square) of deviations (AC component) */
static float _history_rms(uint32_t *buf, float dc)
{
    uint32_t len = history_filled ? HISTORY_SIZE : history_idx;
    if (len == 0) return 0.0f;
    double acc = 0.0;
    for (uint32_t i = 0; i < len; i++) {
        double d = (double)buf[i] - (double)dc;
        acc += d * d;
    }
    acc = acc / (double)len;
    return (float)sqrt(acc);
}

/* Improved heart rate calculation (returns BPM as float).
   Uses RMS of IR history to detect peaks with dynamic threshold. */
float MAX30100_HeartRate_Calculate(void)
{
    // Push latest sample into history
    _push_history();

    // Need some history
    uint32_t len = history_filled ? HISTORY_SIZE : history_idx;
    if (len < 8) return last_bpm; // not enough data yet

    // Compute DC and AC (RMS)
    float ir_dc = _history_mean(ir_history);
    float ir_ac_rms = _history_rms(ir_history, ir_dc);

    // Simple bandpass-like smoothing: current filtered value = newest - dc
    float newest = (float)ir_history[(history_idx + HISTORY_SIZE - 1) % HISTORY_SIZE];
    float filtered = newest - ir_dc;

    // Dynamic threshold: proportional to RMS, plus a small floor
    float threshold = ir_ac_rms * 0.6f;
    if (threshold < 200.0f) threshold = 200.0f;

    uint32_t now = HAL_GetTick();

    // Peak detection: rising edge crosses threshold
    if ((filtered > threshold) && (last_filtered <= threshold) && !peak_flag) {
        peak_flag = 1;
        if (last_peak_time_ms != 0) {
            uint32_t diff = now - last_peak_time_ms;
            if (diff > 300 && diff < 2000) {
                last_bpm = 60000.0f / (float)diff;
            }
        }
        last_peak_time_ms = now;
    }

    // Reset peak flag on falling edge
    if (filtered < (threshold * 0.6f)) {
        peak_flag = 0;
    }

    last_filtered = filtered;
    last_ir_rms = ir_ac_rms;

    return last_bpm;
}

/* Improved SpO2 calculation (returns % as float).
   Uses ratio of AC/DC where AC is RMS of deviations. */
float MAX30100_SpO2_Calculate(void)
{
    // Ensure history includes latest sample
    _push_history();

    uint32_t len = history_filled ? HISTORY_SIZE : history_idx;
    if (len < 8) return 0.0f; // not enough data

    // DC components (mean)
    float ir_dc = _history_mean(ir_history);
    float red_dc = _history_mean(red_history);

    if (ir_dc <= 0.0f || red_dc <= 0.0f) return 0.0f;

    // AC components (RMS of deviations)
    float ir_ac = _history_rms(ir_history, ir_dc);
    float red_ac = _history_rms(red_history, red_dc);

    // Avoid division by zero or extremely small AC (no finger)
    if (ir_ac < 1.0f || red_ac < 1.0f) return 0.0f;

    // Compute ratio R = (REDac/REDdc) / (IRac/IRdc)
    float ratio = (red_ac / red_dc) / (ir_ac / ir_dc);

    // Empirical formula from Maxim / literature
    float spo2 = 104.0f - 17.0f * ratio;

    // Clamp to reasonable values
    if (spo2 > 100.0f) spo2 = 99.0f;
    if (spo2 < 50.0f) spo2 = 50.0f;

    return spo2;
}

#ifdef __cplusplus
}
#endif

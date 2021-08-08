/*
 * functions.h
 *
 *  Created on: 8 dic 2020
 *      Author: Brian
 */

void io_write(const uint8_t pin, const uint8_t value)
{
	if (value == 0) io_status[0] &= ~(1 << pin);
	if (value == 1) io_status[0] |= (1 << pin);
	if (value > 1) io_status[0] = value;
	if ((HAL_I2C_Master_Transmit(&hi2c1, 0x20 << 1, io_status, 1, 1000)) != HAL_OK) Error_Handler();
}

void io_read()
{
	temp = io_status[0];
	io_status[0] = 0b00011111;
	if ((HAL_I2C_Master_Transmit(&hi2c1, 0x20 << 1, io_status, 1, 1000)) != HAL_OK) Error_Handler();

	if ((HAL_I2C_Master_Receive(&hi2c1, 0x20 << 1, io_status, 1, 1000)) != HAL_OK) Error_Handler();
	input_state[0] = io_status[0] & (1 << UP_INPUT);
	input_state[1] = io_status[0] & (1 << DOWN_INPUT);
	input_state[2] = io_status[0] & (1 << SELECT_INPUT);
	input_state[3] = io_status[0] & (1 << UNDO_INPUT);
	input_state[4] = io_status[0] & (1 << TRIGGER_INPUT);

	io_status[0] = temp;
	if ((HAL_I2C_Master_Transmit(&hi2c1, 0x20 << 1, io_status, 1, 1000)) != HAL_OK) Error_Handler();
}

 void beep(int on, int off) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	HAL_Delay(on);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_Delay(off);
}

void lcdstatus() {
	ssd1306_Fill(Black);
	char buffer[32];
	if (menu_state == 0) {

		ssd1306_SetCursor(0, 0);
		sprintf(buffer, "VBAT = %.1fV", bat_read);
		ssd1306_WriteString(buffer, Font_7x10, White);

		ssd1306_SetCursor(0, 17);
		sprintf(buffer, "VBUS = %.1fV", LLC_read);
		ssd1306_WriteString(buffer, Font_7x10, White);

		ssd1306_SetCursor(0, 34);
		sprintf(buffer, "FREQ = %.1fHz", freq);
		ssd1306_WriteString(buffer, Font_7x10, White);

		ssd1306_SetCursor(0, 51);
		sprintf(buffer, "RAMP = %.1fmS", ramp_duration);
		ssd1306_WriteString(buffer, Font_7x10, White);

		float w1 = (bat_read - VBAT_DISCHARGED) / (VBAT_CHARGED - VBAT_DISCHARGED);
		if (w1 < 0) w1 = 0;
		float w2 = (LLC_read - VLLC_min) / (VLLC_max - VLLC_min);
		if (w2 < 0) w2 = 0;
		ssd1306_DrawRectangle(97, 0, 127, 10, White);
		ssd1306_DrawRectangle(97, 17, 127, 27, White);
		ssd1306_FillRect(97, 0, 97 + 30*w1, 10);
		ssd1306_FillRect(97, 17, 97 + 30*w2, 27);

	}

	if (menu_state > 0)
	{
		uint8_t ypos = 0;
		if (menu_state == 1) goto menu1;
		if (menu_state == 2) goto menu2;
		if (menu_state == 3) goto menu3;
		if (menu_state == 4) goto menu4;
		if (menu_state == 5) goto menu5;
		if (menu_state == 6) goto menu6;
		if (menu_state == 7) goto menu7;

		menu1:
		ssd1306_SetCursor(0, ypos);
		ypos += 17;
		sprintf(buffer, "FREQ=%.1fHz", freq);
		ssd1306_WriteString(buffer, Font_7x10, White);
		if(ypos == 17) ssd1306_WriteString(" <-", Font_7x10, White);

		menu2:
		ssd1306_SetCursor(0, ypos);
		ypos += 17;
		sprintf(buffer, "RAMP=%.1fmS", ramp_duration);
		ssd1306_WriteString(buffer, Font_7x10, White);
		if(ypos == 17) ssd1306_WriteString(" <-", Font_7x10, White);

		menu3:
		ssd1306_SetCursor(0, ypos);
		ypos += 17;
		sprintf(buffer, "RAMPMIN=%.1fV", ramp_min);
		ssd1306_WriteString(buffer, Font_7x10, White);
		if(ypos == 17) ssd1306_WriteString(" <-", Font_7x10, White);

		menu4:
		ssd1306_SetCursor(0, ypos);
		ypos += 17;
		sprintf(buffer, "RAMPMAX=%.1fV", ramp_max);
		ssd1306_WriteString(buffer, Font_7x10, White);
		if(ypos == 17) ssd1306_WriteString(" <-", Font_7x10, White);

		menu5:
		ssd1306_SetCursor(0, ypos);
		ypos += 17;
		sprintf(buffer, "VBUS=%.1fV", LLC_set);
		ssd1306_WriteString(buffer, Font_7x10, White);
		if(ypos == 17) ssd1306_WriteString(" <-", Font_7x10, White);

		menu6:
		ssd1306_SetCursor(0, ypos);
		ypos += 17;
		sprintf(buffer, "BUCK Kp=%.3f", BUCK_kp);
		ssd1306_WriteString(buffer, Font_7x10, White);
		if(ypos == 17) ssd1306_WriteString(" <-", Font_7x10, White);

		menu7:
		ssd1306_SetCursor(0, ypos);
		ypos += 17;
		sprintf(buffer, "BUCK Ki=%.3f", BUCK_ki);
		ssd1306_WriteString(buffer, Font_7x10, White);
		if(ypos == 17) ssd1306_WriteString(" <-", Font_7x10, White);

	}
	ssd1306_UpdateScreen();
}

void menu_nav() {

	if (input_state[0] && input_state[1] && input_state[2] && input_state[3])i = 1;
	if (input_state[0] && input_state[1] && (!input_state[2]) && input_state[3]) {
		if (menu_state < 7) menu_state++; else menu_state = 0;
	}
	if (input_state[0] && input_state[1] && input_state[2] && (!input_state[3])) {
		if (menu_state > 0) menu_state--; else menu_state = 7;
	}


	if (menu_state == 1)menu_variable = freq;
	if (menu_state == 2)menu_variable = ramp_duration;
	if (menu_state == 3)menu_variable = ramp_min;
	if (menu_state == 4)menu_variable = ramp_max;
	if (menu_state == 5)menu_variable = LLC_set;
	if (menu_state == 6)menu_variable = BUCK_kp;
	if (menu_state == 7)menu_variable = BUCK_ki;

	if ((!input_state[0]) && input_state[1] && input_state[2] && input_state[3]) {
		if (menu_variable >= 1000)menu_variable += (i * 100.0);
		if (menu_variable >= 100 && menu_variable < 1000)menu_variable += (i * 10.0);
		if (menu_variable >= 10 && menu_variable < 100)menu_variable += i;
		if (menu_variable >= 1 && menu_variable < 10)menu_variable += (i / 10.0);
		if (menu_variable >= 0.1 && menu_variable < 1)menu_variable += (i / 100.0);
		if (menu_variable < 0.1)menu_variable += (i / 1000.0);
		i++;
	}

	if (input_state[0] && (!input_state[1]) && input_state[2] && input_state[3]) {
		if (menu_variable >= 1000)menu_variable -= (i * 100.0);
		if (menu_variable >= 100 && menu_variable < 1000)menu_variable -= (i * 10.0);
		if (menu_variable >= 10 && menu_variable < 100)menu_variable -= i;
		if (menu_variable >= 1 && menu_variable < 10)menu_variable -= (i / 10.0);
		if (menu_variable >= 0.1 && menu_variable < 1)menu_variable -= (i / 100.0);
		if (menu_variable <= 0.1)menu_variable -= (i / 1000.0);
		i++;
	}

	if (menu_state == 1)freq = menu_variable;
	if (menu_state == 2)ramp_duration = menu_variable;
	if (menu_state == 3)ramp_min = menu_variable;
	if (menu_state == 4)ramp_max = menu_variable;
	if (menu_state == 5)LLC_set = menu_variable;
	if (menu_state == 6)BUCK_kp = menu_variable;
	if (menu_state == 7)BUCK_ki = menu_variable;

	if (i > 10)i = 10;
	if (freq > FREQMAX)freq = FREQMAX;
	if (freq < FREQMIN)freq = FREQMIN;
	if (ramp_duration > RAMPMAX)ramp_duration = RAMPMAX;
	if (ramp_duration < RAMPMIN)ramp_duration = RAMPMIN;
	if (ramp_min < VBUCK_min)ramp_min = VBUCK_min;
	if (ramp_min > ramp_max)ramp_max = ramp_min;
	if (ramp_max < ramp_min)ramp_min = ramp_max;
	if (ramp_min > VBUCK_max)ramp_min = VBUCK_max;
	if (ramp_max > VBUCK_max)ramp_max = VBUCK_max;
	if (LLC_set < VLLC_min)LLC_set = VLLC_min;
	if (LLC_set > VLLC_max)LLC_set = VLLC_max;
	if (BUCK_kp < 0)BUCK_kp = 0;
	if (BUCK_kp > 100000)BUCK_kp = 100000;
	if (BUCK_ki < 0)BUCK_ki = 0;
	if (BUCK_ki > 100000)BUCK_ki = 100000;

}



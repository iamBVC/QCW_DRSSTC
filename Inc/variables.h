//pcf8574
#define TRIGGER_INPUT 0
#define UNDO_INPUT    1
#define UP_INPUT      2
#define SELECT_INPUT  3
#define DOWN_INPUT    4
#define RED_LED       7
#define YELLOW_LED       6
#define GREEN_LED       5

//parameters limits
#define RAMPMAX 20  //mS
#define RAMPMIN 1   //mS
#define FREQMAX 10  //Hz
#define FREQMIN 1   //Hz
#define VBUCK_min 0
#define VBUCK_max 280
#define VLLC_min 0
#define VLLC_max 300
#define VBAT_CHARGED 22.2
#define VBAT_DISCHARGED 18.0

//timers
#define TIM_CLK 100000000.0
#define TIM1_PSC 1.0
#define TIM2_PSC 1.0
#define TIM16_PSC 10000.0
#define TIM17_PSC 1.0
#define OLED_FRQ 10.0
#define SAMPLE_FRQ 30000.0
#define TIM2_DTM 850.0
#define BUCK_frq 70000.0
#define TIM1_DTM 400.0
#define LLC_frq 40000.0

const uint32_t TIM1_CLK = TIM_CLK / (TIM1_PSC * 2.0);
const uint32_t TIM2_CLK = TIM_CLK / (TIM2_PSC * 2.0);
const uint32_t TIM16_CLK = TIM_CLK / TIM16_PSC;
const uint16_t TIM16_ARR = TIM_CLK / (TIM16_PSC * OLED_FRQ);
const uint32_t TIM17_CLK = TIM_CLK / TIM17_PSC;
const uint16_t TIM17_ARR = TIM_CLK / (TIM17_PSC * SAMPLE_FRQ);

uint16_t TIM1_ARR = 0;
uint16_t TIM1_DTMV = 0;
uint32_t TIM2_ARR = 0;
uint32_t TIM2_DTMV = 0;

//PID
float PID_Pout, PID_Iout, PID_Imem, PID_err, PID_err_mem, BUCK_pwm, BUCK_pwm2 = 0.0;


float bat_read, LLC_read, BUCK_read, ramp_inc, menu_variable = 0.0;
uint8_t menu_state, temp, fire_status, i = 0;
uint8_t input_state[5] = {0,0,0,0,0};
uint8_t io_status[1] = {0};
uint32_t ramp_ctr, oled_tick = 0;

//initial values
float freq = 5.0;
float ramp_duration = 10.0;
float ramp_min = 30.0;
float ramp_max = 150.0;
float LLC_set = 200.0;
float BUCK_set = 30.0;
float BUCK_kp = 0.0;
float BUCK_ki = 0.3;

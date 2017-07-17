
#include "ap_peripheral_handling.h"
#include "ap_state_config.h"
#include "ap_state_handling.h"

//#include "drv_l1_system.h"
#include "driver_l1.h"
#include "drv_l1_cdsp.h"


#define LED_STATUS_FLASH		1
#define LED_STATUS_BLINK		2

#define CRAZY_KEY_TEST			0		// Send key events faster than human finger can do
#define LED_ON					1
#define LED_OFF 				0

static INT8U	led_status; //0: nothing  1: flash	2: blink
static INT8U	led_cnt;
static INT8U	power_keyup = 1;

static INT32U	led_mode;
static INT16U	g_led_count;
static INT8U	g_led_r_state; //0 = OFF;	1=ON;	2=Flicker
static INT8U	g_led_g_state;
static INT8U	g_led_flicker_state; //0=同时闪烁	1=交替闪烁
static INT8U	led_red_flag;
static INT8U	led_green_flag;
static INT8U	led_night_flag;
static INT16U	power_off_cnt = 0;
INT8U			IR_flag = 0; //红外
static INT8U	mode_count = 0; //短按计数，控制不同模式
INT8U			video_720_flag = 1; //进入720p录像标志位
INT8U			video_1080_flag = 0; //进入1080P录像标志位
static INT8U	capture_flag = 0; //拍照标志位
static INT8U	capture_first = 0; //进入录像标志
static INT8U	motion_wait_to_record = 0; //进入录像标志

//INT8U motion_first_flag=0;//进入第一次移动侦测标志，
//static INT8U	   charge_full_flag=0;
#if TV_DET_ENABLE
INT8U			tv_plug_in_flag;
INT8U			tv_debounce_cnt = 0;
INT8U			usbd_plug_in_flag = 0; //用于解决第一次读盘不操作关机的问题
static INT8U	tv = !TV_DET_ACTIVE;
static INT8U	backlight_tmr = 0;

#endif

#if 1 //C_SCREEN_SAVER == CUSTOM_ON

INT8U			auto_off_force_disable = 0;
void ap_peripheral_auto_off_force_disable_set(INT8U);

#endif

static INT8U	led_flash_timerid;
static INT16U	config_cnt;

//----------------------------
typedef struct 
{
INT8U			byRealVal;
INT8U			byCalVal;
} AD_MAP_t;


//----------------------------
extern void avi_adc_gsensor_data_register(void * *msgq_id, INT32U * msg_id);
INT8U			gsensor_data[2][32] =
{
	0
};




#define C_BATTERY_STABLE_THRESHOLD 4  // Defines threshold number that AD value is deemed stable

#if C_BATTERY_DETECT			== CUSTOM_ON
static INT16U	low_voltage_cnt;
static INT32U	battery_value_sum = 0;
static INT8U	bat_ck_cnt = 0;
extern INT8S	video_record_sts;

#endif



#if USE_ADKEY_NO
static INT8U	ad_detect_timerid;
static INT16U	ad_value;
static KEYSTATUS ad_key_map[USE_ADKEY_NO + 1];

//static INT16U ad_key_cnt = 0;
#define C_RESISTOR_ACCURACY 	5//josephhsieh@140418 3			// 2% accuracy
#define C_KEY_PRESS_WATERSHED	600//josephhsieh@140418 175
#define C_KEY_STABLE_THRESHOLD	4//josephhsieh@140418 3			// Defines threshold number that AD value of key is deemed stable
#define C_KEY_FAST_JUDGE_THRESHOLD 40			// Defines threshold number that key is should be judge before it is release. 0=Disable
#define C_KEY_RELEASE_STABLE_THRESHOLD 4  // Defines threshold number that AD value is deemed stable

INT16U			adc_key_value;

//static INT8U	ad_value_cnt ;
INT32U			key_pressed_cnt;
INT8U			fast_key_exec_flag;
INT8U			normal_key_exec_flag;
INT8U			long_key_exec_flag;

#endif

static INT32U	key_active_cnt;
static INT8U	power_off_timerid;
static INT8U	usbd_detect_io_timerid;
static KEYSTATUS key_map[USE_IOKEY_NO];
static INT8U	key_detect_timerid;
static INT16U	adp_out_cnt;
static INT16U	usbd_cnt;
static INT8U	up_firmware_flag = 0;
static INT8U	flash_flag = 0;

static INT8U	r_up_firmware_flag = 0;
static INT8U	r_flash_flag = 0;

#if USB_PHY_SUSPEND 			== 1
static INT16U	phy_cnt = 0;

#endif

static INT16U	adp_cnt;
INT8U			adp_status;
static INT8U	battery_low_flag = 0;

INT8U			usbd_exit;
INT8U			s_usbd_pin;

//extern INT8U MODE_KEY_flag;
//	prototypes
void ap_peripheral_key_init(void);
void ap_peripheral_rec_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_function_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_next_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_prev_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_ok_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_sos_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_usbd_plug_out_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_pw_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_menu_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_video_prev_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_ir_change_exe(INT16U * tick_cnt_ptr);

#if KEY_FUNTION_TYPE			== SAMPLE2
void ap_peripheral_capture_key_exe(INT16U * tick_cnt_ptr);

#endif

void ap_peripheral_null_key_exe(INT16U * tick_cnt_ptr);

#if USE_ADKEY_NO
void ap_peripheral_ad_detect_init(INT8U adc_channel, void(*bat_detect_isr) (INT16U data));
void ap_peripheral_ad_check_isr(INT16U value);

#endif

extern volatile INT8U video_down_flag;
extern volatile INT8U pic_down_flag;

extern INT8U usb_state_get(void);
extern void usb_state_set(INT8U flag);


static void init_usbstate(void)
{

	static INT8U	usb_dete_cnt = 0;
	static INT8U	err_cnt = 0;

	while (++err_cnt < 100)
	{
		if (sys_pwr_key1_read())
			usb_dete_cnt++;
		else 
		{
			usb_dete_cnt		= 0;
			break;
		}

		if (usb_dete_cnt > 3)
			break;

		OSTimeDly(2);
	}

	if (usb_dete_cnt > 3)
		usb_state_set(3);

	err_cnt 			= 0;


}


void ap_peripheral_init(void)
{
#if TV_DET_ENABLE
	INT32U			i;

#endif

	power_off_timerid	= usbd_detect_io_timerid = led_flash_timerid = 0xFF;
	key_detect_timerid	= 0xFF;

	//LED IO init
	//gpio_init_io(LED, GPIO_OUTPUT);
	//gpio_set_port_attribute(LED, ATTRIBUTE_HIGH);
	//gpio_write_io(LED, DATA_LOW);
	//led_status = 0;
	//led_cnt = 0;
	init_usbstate();


	//DBG_PRINT("usb=%d\r\n",usb_state_get());
	gpio_init_io(CHARGE_PIN, GPIO_INPUT);
	gpio_set_port_attribute(CHARGE_PIN, ATTRIBUTE_LOW);
	gpio_write_io(CHARGE_PIN, 1);					//pull low

#if 0
	gpio_init_io(IR_CTRL, GPIO_OUTPUT);
	gpio_set_port_attribute(IR_CTRL, ATTRIBUTE_HIGH);
	gpio_write_io(IR_CTRL, DATA_LOW);
#endif

	gpio_init_io(AV_IN_DET, GPIO_INPUT);
	gpio_set_port_attribute(AV_IN_DET, ATTRIBUTE_LOW);
	gpio_write_io(AV_IN_DET, !TV_DET_ACTIVE);		//pull high or low

#if TV_DET_ENABLE
	tv_plug_in_flag 	= 0;

#if 1

	for (i = 0; i < 5; i++)
	{
		if (gpio_read_io(AV_IN_DET) == !TV_DET_ACTIVE)
		{
			DBG_PRINT("jdjfnghg+++=\r\n");
			break;
		}

		OSTimeDly(1);
	}

	if (i == 5)
	{
		DBG_PRINT("xx++++\r\n");
		tv					= TV_DET_ACTIVE;
		tv_plug_in_flag 	= 1;
	}

#endif

#endif

	power_off_cnt		= PERI_POWER_OFF;
	ap_peripheral_key_init();
	LED_pin_init();

#if USE_ADKEY_NO
	ad_detect_timerid	= 0xFF;
	ap_peripheral_ad_detect_init(AD_KEY_DETECT_PIN, ap_peripheral_ad_check_isr);

#else

	adc_init();
#endif

	config_cnt			= 0;

	//MODE_KEY_flag = 2;
}



void ap_peripheral_led_set(INT8U type)
{
#ifdef PWM_CTR_LED
	INT8U			byPole;
	INT16U			wPeriod = 0;
	INT16U			wPreload = 0;
	INT8U			byEnable;

	if (type)
	{ //high
		ap_peripheral_PWM_LED_high();
	}
	else 
	{ //low
		ap_peripheral_PWM_LED_low();
	}

#else

	gpio_write_io(LED, type);
	led_status			= 0;
	led_cnt 			= 0;
#endif
}


void ap_peripheral_led_flash_set(void)
{
#ifdef PWM_CTR_LED
	ap_peripheral_PWM_LED_high();
	led_status			= LED_STATUS_FLASH;
	led_cnt 			= 0;

#else

	gpio_write_io(LED, DATA_HIGH);
	led_status			= LED_STATUS_FLASH;
	led_cnt 			= 0;
#endif
}


void ap_peripheral_auto_off_force_disable_set(INT8U auto_off_disable)
{
	auto_off_force_disable = auto_off_disable;
}


//只在待机状态
extern void ap_video_capture_mode_switch(INT8U DoSensorInit, INT16U EnterAPMode);
extern INT32S vid_enc_disable_sensor_clock(void);
extern void ap_video_record_md_disable(void);


static void Video_res_set(void)
{
	if (ap_state_config_video_resolution_get() == 0) //设置切换录像格式
	{
		ap_state_config_video_resolution_set(2);
	}
	else 
	{
		ap_state_config_video_resolution_set(0);
	}

	vid_enc_disable_sensor_clock(); 				//关摄像头
	ap_video_capture_mode_switch(0, STATE_VIDEO_RECORD); //切换录像格式，第一个参数为0是为了不更新摄像头驱动，这函数里有开摄像头的程序

}


void ap_peripheral_led_blink_set(void)
{

#ifdef PWM_CTR_LED
	INT8U			byPole = 1;
	INT16U			wPeriod = 0x6000;
	INT16U			wPreload = 0x2fff;
	INT8U			byEnable = TRUE;

	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	  ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable) ; 
	DBG_PRINT("PWM0/1 blink on	750ms,380ms \r\n");

#else

	gpio_write_io(LED, DATA_HIGH);
	led_status			= LED_STATUS_BLINK;
	led_cnt 			= 0;
#endif
}


void LED_pin_init(void)
{
	INT32U			type;

	//led init as ouput pull-low
	gpio_init_io(LED1, GPIO_OUTPUT);
	gpio_set_port_attribute(LED1, ATTRIBUTE_HIGH);
	gpio_write_io(LED1, DATA_LOW);

	gpio_init_io(LED2, GPIO_OUTPUT);
	gpio_set_port_attribute(LED2, ATTRIBUTE_HIGH);
	gpio_write_io(LED2, DATA_LOW);


	led_night_flag		= 0;
	led_red_flag		= LED_OFF;
	led_green_flag		= LED_OFF;
	type				= LED_INIT;
	msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);

	//msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_ZD, &type, sizeof(INT8U), MSG_PRI_NORMAL);	
	sys_registe_timer_isr(LED_blanking_isr);		//timer base c to start adc convert
}


extern INT8U	card_space_less_flag;
INT8U			PREV_LED_TYPE = 0;


static void charge_flash_pro(void)
{
	if (gpio_read_io(CHARGE_PIN) == 0)
	{

		if (usb_state_get() == 1)
			led_green_on();
		else 
			led_green_off();

		led_red_off();
		g_led_r_state		= 2;

		// g_led_r_state=2;
	}
	else 
	{
		if (usb_state_get() == 1)
			led_green_on();
		else 
			led_green_off();

		led_red_on();


	}
}


extern INT8U	record_led_flag;


void set_led_mode(LED_MODE_ENUM mode)
{
	INT8U			i, type = 0;
	static INT8U	prev_mode = 0xaa;

	led_mode			= mode;

	g_led_g_state		= 0;						//3oE??÷oiAAA
	g_led_r_state		= 0;
	g_led_flicker_state = 0;

	PREV_LED_TYPE		= prev_mode;

	switch ((INT32U)
	mode)
	{
		case LED_INIT: //开机蓝灯长亮为待机模式
			led_red_on();
			led_green_off();
			DBG_PRINT("led_type = LED_INIT\r\n");
			break;

		case LED_UPDATE_PROGRAM:
			g_led_g_state = 1;
			DBG_PRINT("led_type = LED_UPDATE_PROGRAM\r\n");
			break;

		case LED_UPDATE_FINISH:
			led_red_off();
			led_green_on();
			DBG_PRINT("led_type = LED_UPDATE_FINISH\r\n");
			break;

		case LED_UPDATE_FAIL:
			sys_release_timer_isr(LED_blanking_isr);

			for (i = 0; i < 2; i++)
			{
				led_all_off();
				OSTimeDly(15);
				led_green_on();
				OSTimeDly(15);
				led_all_off();
			}

			DBG_PRINT("led_type = LED_UPDATE_FAIL\r\n");
			sys_registe_timer_isr(LED_blanking_isr);
			break;

		case LED_USB_CONNECT:
			//led_red_on();
			break;

		case LED_CHARGEING:
			if (prev_mode != mode)
			{
				g_led_count 		= 0;
			}

			if (usb_state_get() == 1)
				led_green_on();
			else 
				led_green_off();

			led_red_off();
			g_led_r_state = 2;
			DBG_PRINT("led_type = LED_CHARGEING\r\n");
			break;

		case LED_RECORD: //720p蓝灯闪3下灯灭后
			if (prev_mode != mode)
			{
				g_led_count 		= 0;
			}

			sys_release_timer_isr(LED_blanking_isr);
			led_green_off();
			led_red_off();
			OSTimeDly(20);

			for (i = 0; i < 3; i++)
			{
				led_green_off();
				led_red_on();
				OSTimeDly(20);
				led_green_off();
				led_red_off();
				OSTimeDly(20);
			}

			sys_registe_timer_isr(LED_blanking_isr);

			//if(IR_flag)
			//	g_led_r_state=7;
			//	else
			g_led_r_state = 8;
			DBG_PRINT("led_type = LED_RECORD\r\n");
			break;

		case LED_WAITING_RECORD: //720p 蓝灯蓝灯长亮待机模式
			if (usb_state_get()) //如果边充边录时，在待机时不显示录像的等待状态，而显示充电状态
			{
				charge_flash_pro();
			}
			else 
			{
				if (led_night_flag)
				{
					led_red_on();
					led_green_on();
				}
				else 
				{
					led_red_on();
					led_green_off();
				}
			}

			DBG_PRINT("led_type = LED_WAITING_RECORD\r\n");
			break;

		case LED_RECORD1080: //1080p 蓝灯长亮红灯闪三下同时熄灭
			if (prev_mode != mode)
			{
				g_led_count 		= 0;
			}

			sys_release_timer_isr(LED_blanking_isr);
			led_green_off();
			led_red_off();
			OSTimeDly(20);

			for (i = 0; i < 3; i++)
			{
				led_green_on();
				OSTimeDly(20);
				led_green_off();
				OSTimeDly(20);
			}

			led_green_off();
			led_red_off();
			sys_registe_timer_isr(LED_blanking_isr);
			g_led_g_state = 7;
			DBG_PRINT("led_type = LED_RECORD1080\r\n");
			break;

		case LED_WAITING_RECORD1080: //1080p 红蓝灯长亮为1080P录像模式
			if (usb_state_get()) //如果边充边录时，在待机时不显示录像的等待状态，而显示充电状态
			{
				charge_flash_pro();
			}
			else 
			{
				if (led_night_flag)
				{
					led_red_on();
					led_green_on();
				}
				else 
				{
					led_red_off();
					led_green_on();
				}
			}

			DBG_PRINT("led_type = LED_WAITING_RECORD1080\r\n");
			break;

		case LED_AUDIO_RECORD:
			DBG_PRINT("led_type = LED_AUDIO_RECORD\r\n");
			break;

		case LED_WAITING_AUDIO_RECORD:
			DBG_PRINT("led_type = LED_WAITING_AUDIO_RECORD\r\n");
			break;

		case LED_CAPTURE: //红灯闪1下
			// sys_release_timer_isr(LED_blanking_isr);
			{

				led_green_on();
				led_red_off();

				// OSTimeDly(20);
				// led_green_on();
			}
			//sys_registe_timer_isr(LED_blanking_isr);	
			DBG_PRINT("led_type = LED_CAPTURE\r\n");
			break;

		case LED_CAPTURE_FAIL:
			for (i = 0; i < 2; i++)
			{
				led_all_off();
				OSTimeDly(50);
				led_red_on();
				OSTimeDly(50);
			}

		case LED_WAITING_CAPTURE: //红灯亮
			if (usb_state_get()) //如果边充边录时，在待机时不显示录像的等待状态，而显示充电状态
			{
				charge_flash_pro();
			}
			else 
			{
				if (led_night_flag)
				{
					led_red_on();
					led_green_on();
				}
				else 
				{
					led_red_on();
					led_green_on();
				}
			}

			DBG_PRINT("led_type = LED_WAITING_CAPTURE\r\n");
			break;

		case LED_IR_ON: //红外 红灯闪2下
#if 0
			sys_release_timer_isr(LED_blanking_isr);
			led_green_off();
			led_red_off();
			OSTimeDly(20);

			for (i = 0; i < 2; i++)
			{
				led_red_on();
				OSTimeDly(20);
				led_red_off();
				OSTimeDly(20);
			}

			sys_registe_timer_isr(LED_blanking_isr);

			if (usb_state_get()) //如果边充边录时，在待机时不显示录像的等待状态，而显示充电状态
			{
				charge_flash_pro();
			}
			else if ((video_record_sts & 0x02) == 0) //恢复原来的待机状态,不在录像，就在待机
			{
				if (video_720_flag == 1)
				{
					led_red_off();
					led_green_on();
				}
				else if (video_1080_flag == 1)
				{
					led_red_on();
					led_green_on();
				}
				else if (capture_flag == 1)
				{
					led_red_on();
					led_green_off();
				}
			}

			DBG_PRINT("led_type = LED_IR_ON\r\n");
#endif

			break;

		case LED_IR_OFF: //红外 红灯闪3下
#if 0
			sys_release_timer_isr(LED_blanking_isr);
			led_green_off();
			led_red_off();
			OSTimeDly(20);

			for (i = 0; i < 3; i++)
			{
				led_red_on();
				OSTimeDly(20);
				led_red_off();
				OSTimeDly(20);
			}

			sys_registe_timer_isr(LED_blanking_isr);

			if (usb_state_get()) //如果边充边录时，在待机时不显示录像的等待状态，而显示充电状态
			{
				charge_flash_pro();
			}
			else if ((video_record_sts & 0x02) == 0) //恢复原来的待机状态,不在录像，就在待机
			{
				if (video_720_flag == 1)
				{
					led_red_off();
					led_green_on();
				}
				else if (video_1080_flag == 1)
				{
					led_red_on();
					led_green_on();
				}
				else if (capture_flag == 1)
				{
					led_red_on();
					led_green_off();
				}
			}

			DBG_PRINT("led_type = LED_IR_OFF\r\n");
#endif

			break;

		case LED_MOTION_DETECTION: //720p	红蓝灯同时闪
			if (led_night_flag)
				led_red_off();
			else 
			{
				led_red_off();
				led_green_off();
				g_led_g_state		= 2;
				g_led_r_state		= 1;
				g_led_count 		= 0;
			}

			DBG_PRINT("led_type = LED_MOTION_DETECTION\r\n");
			break;

		case LED_MONTION_WAIT: //720p  蓝灯长亮红灯闪3下后灭灯
			if (usb_state_get()) //如果边充边录时，在待机时不显示录像的等待状态，而显示充电状态
			{
				charge_flash_pro();
			}
			else 
			{
				led_red_on();
				g_led_g_state		= 2;
			}

			break;

		case LED_MOTION_DETECTION1080: //1080p	
			if (led_night_flag)
				led_red_off();
			else 
			{
				g_led_g_state		= 3;
				g_led_r_state		= 3;

			}

			DBG_PRINT("led_type = LED_MOTION_DETECTION\r\n");
			break;

		case LED_MONTION_WAIT1080: //1080p 红灯闪3下，蓝灯长亮红灯闪3下
			if (usb_state_get()) //如果边充边录时，在待机时不显示录像的等待状态，而显示充电状态
			{
				charge_flash_pro();
			}
			else 
			{
				led_green_on();
				g_led_r_state		= 1;
			}

			break;

		case LED_CARD_DETE_SUC:
			if (storage_sd_upgrade_file_flag_get() == 2)
				break;

			if ((prev_mode == LED_CHARGE_FULL) || (prev_mode == LED_CHARGEING))
			{
				if (usb_state_get())
				{
					charge_flash_pro();
				}

				break;
			}

			sys_release_timer_isr(LED_blanking_isr);
			led_green_off();
			led_red_off();
			OSTimeDly(15);
			led_green_off();
			led_red_on();

			// OSTimeDly(15);
			sys_registe_timer_isr(LED_blanking_isr);
			break;

		case LED_NO_SDC:
#if 1

			if (usb_state_get())
			{
				charge_flash_pro();
			}
			else 
			{
				type				= 1;
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SHUTOFF_SET, &type, sizeof(INT8U), MSG_PRI_NORMAL);
			}

			DBG_PRINT("led_type = LED_NO_SDC\r\n");
#endif

			break;

#if 1
		case LED_SDC_FULL:
		case LED_CARD_NO_SPACE:
			if (storage_sd_upgrade_file_flag_get() == 2)
				break;

			if (usb_state_get())
			{
				charge_flash_pro();
			}
			else 
			{
				led_red_on();
				led_green_on();
				OSTimeDly(50);
				msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);

				//type=1;
				//msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SHUTOFF_SET, &type, sizeof(INT8U), MSG_PRI_NORMAL);
			}

			break;

#endif

		case LED_TELL_CARD:
			break;

		case LED_LOW_BATT:
			if (storage_sd_upgrade_file_flag_get() == 2)
				break;

#if 0
			sys_release_timer_isr(LED_blanking_isr);

			for (i = 0; i < 25; i++)
			{
				led_red_on();
				led_green_on();
				OSTimeDly(10);
				led_green_off();
				led_red_off();
				OSTimeDly(10);
			}

			sys_registe_timer_isr(LED_blanking_isr);
			msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
#endif

			type = 2;
			msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SHUTOFF_SET, &type, sizeof(INT8U), MSG_PRI_NORMAL);
			break;

		case LED_LOW_BATT1:
			//g_led_r_state=1;
			break;

		case LED_CHARGE_FULL:
			if (usb_state_get() == 1)
				led_green_on();
			else 
				led_green_off();

			led_red_on();
			DBG_PRINT("led_type = LED_CHARGE_FULL\r\n");
			break;
	}

	prev_mode			= mode;
}


void night_led_ctrl(INT8U on)
{
	if (on)
	{
		gpio_write_io(M_LED_0, DATA_LOW);
		gpio_write_io(M_LED_1, DATA_LOW);
		gpio_write_io(M_LED_2, DATA_LOW);
		gpio_write_io(M_LED_3, DATA_LOW);
		gpio_write_io(M_LED_4, DATA_LOW);
		gpio_write_io(M_LED_5, DATA_LOW);
		gpio_write_io(M_LED_6, DATA_LOW);

	}
	else 
	{
		gpio_write_io(M_LED_0, DATA_HIGH);
		gpio_write_io(M_LED_1, DATA_HIGH);
		gpio_write_io(M_LED_2, DATA_HIGH);
		gpio_write_io(M_LED_3, DATA_HIGH);
		gpio_write_io(M_LED_4, DATA_HIGH);
		gpio_write_io(M_LED_5, DATA_HIGH);
		gpio_write_io(M_LED_6, DATA_HIGH);
	}
}


void led_red_on(void)
{
	if (led_red_flag != LED_ON)
	{
		gpio_write_io(LED2, DATA_HIGH);
		led_red_flag		= LED_ON;
	}
}


void led_green_on(void)
{
	if (led_green_flag != LED_ON)
	{
		gpio_write_io(LED1, DATA_HIGH);
		led_green_flag		= LED_ON;
	}
}


void led_all_off(void)
{
	if (led_green_flag != LED_OFF)
	{
		gpio_write_io(LED1, DATA_LOW);
		led_green_flag		= LED_OFF;
	}

	if (led_red_flag != LED_OFF)
	{
		gpio_write_io(LED2, DATA_LOW);
		led_red_flag		= LED_OFF;
	}
}


void led_green_off(void)
{
	if (led_green_flag != LED_OFF)
	{
		gpio_write_io(LED1, DATA_LOW);
		led_green_flag		= LED_OFF;
	}
}


void led_red_off(void)
{
	//if(usb_state_get())
	//	return;
	if (led_red_flag != LED_OFF)
	{
		gpio_write_io(LED2, DATA_LOW);
		led_red_flag		= LED_OFF;
	}
}


extern INT8U	video_stop_flag;


void LED_blanking_isr(void)
{

	//if(card_space_less_flag)
	//	return;
	if (g_led_count++ == 127)
	{
		g_led_count 		= 0;
	}

	if (video_stop_flag)
		return;

	if (g_led_g_state == 1)
	{
		if (g_led_count % 10 == 0)
		{
			if (up_firmware_flag == 1)
			{
				led_green_off();
				up_firmware_flag	= 0;
			}
			else 
			{
				led_green_on();
				up_firmware_flag	= 1;
			}
		}
	}
	else if (g_led_g_state == 2)
	{
		if (g_led_count / 64 == g_led_flicker_state)
			led_green_on();
		else 
			led_green_off();
	}
	else if (g_led_g_state == 3)
	{

		if (g_led_count % 32 == 0)
		{
			if (flash_flag == 0)
			{
				led_green_on();
				flash_flag			= 1;
			}
			else 
			{
				led_green_off();
				flash_flag			= 0;
			}

		}
	}
	else if (g_led_g_state == 8)
	{
		if (g_led_count % 128 == 0)
		{
			if (r_flash_flag <= 7)
			{
				r_flash_flag++;
			}
			else if (r_flash_flag == 8)
			{
				led_green_on();
				r_flash_flag		= 9;
			}
			else if (r_flash_flag == 9)
			{
				led_green_off();
				r_flash_flag		= 0;
			}

		}
	}
	else if (g_led_g_state == 9)
	{
		if (g_led_count % 128 == 0)
		{
			if (r_flash_flag < 1)
				r_flash_flag++;
			else if (r_flash_flag == 1)
			{
				led_green_on();
				r_flash_flag		= 2;
			}
			else 
			{
				led_green_off();
				r_flash_flag		= 0;
			}
		}
	}

	else if (g_led_g_state == 7)
	{
		if (g_led_count % 128 == 0)
		{
			if (r_flash_flag < 2)
				r_flash_flag++;
			else if (r_flash_flag == 2)
			{
				led_green_on();
				r_flash_flag		= 3;
			}
			else 
			{
				led_green_off();
				r_flash_flag		= 0;
			}
		}
	}



	if (g_led_r_state == 1)
	{
		if (g_led_count / 64 == g_led_flicker_state)
			led_red_on();
		else 
			led_red_off();
	}

	else if (g_led_r_state == 2)
	{
#if 0

		if (g_led_count < 384)
			led_red_on();
		else 
			led_red_off();

#endif

		if (g_led_count % 128 == 0)
		{
			if (r_flash_flag == 0)
			{
				led_red_off();
				r_flash_flag		= 1;
			}
			else 
			{
				led_red_on();
				r_flash_flag		= 0;
			}
		}
	}
	else if (g_led_r_state == 3)
	{
		if (g_led_count % 32 == 0)
		{
			if (flash_flag == 0)
			{
				led_red_on();

				//r_flash_flag=1;
			}
			else 
			{
				led_red_off();

				//r_flash_flag=0;
			}


		}
	}

	else if (g_led_r_state == 7)
	{
		if (g_led_count % 128 == 0)
		{
			if (r_flash_flag < 1)
				r_flash_flag++;
			else if (r_flash_flag == 1)
			{
				led_red_on();
				r_flash_flag		= 2;
			}
			else 
			{
				led_red_off();
				r_flash_flag		= 0;
			}
		}
	}
	else if (g_led_r_state == 8)
	{
		if (g_led_count % 128 == 0)
		{
			if (r_flash_flag < 2)
			{
				r_flash_flag++;
			}
			else if (r_flash_flag == 2)
			{
				led_red_on();
				r_flash_flag		= 3;
			}
			else if (r_flash_flag == 3)
			{
				led_red_off();
				r_flash_flag		= 0;
			}

			//			if(r_flash_flag<=7)
			//			{
			//				r_flash_flag++;
			//			}
			//			else if(r_flash_flag == 8)
			//			{
			//				led_red_on();
			//				r_flash_flag=9;
			//			}
			//			else if(r_flash_flag ==9)
			//			{
			//				led_red_off();
			//				r_flash_flag=0;
			//			}
		}
	}

	//	ap_peripheral_key_judge();
	// msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SINGLE_JUGE, &type, sizeof(INT8U), MSG_PRI_NORMAL);
}





#if C_MOTION_DETECTION			== CUSTOM_ON


void ap_peripheral_motion_detect_judge(void)
{
	INT32U			result;

	//DBG_PRINT("-\r\n");//fan
	if (video_down_flag)
		return;

	result				= hwCdsp_MD_get_result();

	//DBG_PRINT("MD_result = 0x%x\r\n",result);
	if (result > 0x36)
	{
		msgQSend(ApQ, MSG_APQ_MOTION_DETECT_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
	}
}


void ap_peripheral_motion_detect_start(void)
{
	motion_detect_status_set(MOTION_DETECT_STATUS_START);
}


void ap_peripheral_motion_detect_stop(void)
{
	motion_detect_status_set(MOTION_DETECT_STATUS_STOP);
}


#endif

#if USE_ADKEY_NO


void ap_peripheral_ad_detect_init(INT8U adc_channel, void(*ad_detect_isr) (INT16U data))
{
#if C_BATTERY_DETECT				== CUSTOM_ON
	battery_value_sum	= 0;
	bat_ck_cnt			= 0;
#endif

	//	ad_value_cnt = 0;
	adc_init();
	adc_vref_enable_set(TRUE);
	adc_conv_time_sel(4);
	adc_manual_ch_set(adc_channel);
	adc_manual_callback_set(ad_detect_isr);

	if (ad_detect_timerid == 0xFF)
	{
		ad_detect_timerid	= AD_DETECT_TIMER_ID;
		sys_set_timer((void *) msgQSend, (void *) PeripheralTaskQ, MSG_PERIPHERAL_TASK_AD_DETECT_CHECK,
			 ad_detect_timerid, PERI_TIME_INTERVAL_AD_DETECT);
	}
}


void ap_peripheral_ad_check_isr(INT16U value)
{
	ad_value			= value;
}


INT16U adc_key_release_calibration(INT16U value)
{
	return value;
}


void ap_peripheral_clr_screen_saver_timer(void)
{
	key_active_cnt		= 0;
}


#if 0 //(KEY_TYPE == KEY_TYPE1)||(KEY_TYPE==KEY_TYPE2)||(KEY_TYPE==KEY_TYPE3)||(KEY_TYPE==KEY_TYPE4)||(KEY_TYPE==KEY_TYPE5)


#else

/*
	0.41v => 495
	0.39v =>
	0.38v => 460
	0.37v => 
	0.36v => 440
	0.35v =>
	0.34v =>
*/
enum 
{
BATTERY_CNT = 64, 
BATTERY_Lv3 = 2200 * BATTERY_CNT,					//2200	2500
BATTERY_Lv2 = 2175 * BATTERY_CNT,					//2175	2475
BATTERY_Lv1 = 2155 * BATTERY_CNT,					//2155	2455
BATTERY_Lv0 = 2125 * BATTERY_CNT					//2135	2300
};


static INT32U	ad_time_stamp;


//#define SA_TIME	50	//seconds, for screen saver time. Temporary use "define" before set in "STATE_SETTING".
void ap_peripheral_ad_key_judge(void)
{

	INT32U			t;

	t					= OSTimeGet();

	if ((t - ad_time_stamp) < 2)
	{
		return;
	}

	ad_time_stamp		= t;

	adc_manual_ch_set(AD_KEY_DETECT_PIN);

	//DBG_PRINT("%d, ",(ad_value>>4));
#if C_BATTERY_DETECT				== CUSTOM_ON
	ap_peripheral_battery_check_calculate();
#endif

	adc_manual_sample_start();

}


#endif // AD-Key

#endif

#if C_BATTERY_DETECT			== CUSTOM_ON

INT32U			previous_direction = 0;
extern void ap_state_handling_led_off(void);
extern INT8U	display_str_battery_low;


#define BATTERY_GAP 			10*BATTERY_CNT


static INT8U ap_peripheral_smith_trigger_battery_level(INT32U direction)
{
	static INT8U	bat_lvl_cal_bak = (INT8U)

	BATTERY_Lv3;
	INT8U			bat_lvl_cal;

	// DBG_PRINT("(%d)\r\n", battery_value_sum);
	if (battery_value_sum >= BATTERY_Lv3)
	{
		bat_lvl_cal 		= 4;
	}
	else if ((battery_value_sum < BATTERY_Lv3) && (battery_value_sum >= BATTERY_Lv2))
	{
		bat_lvl_cal 		= 3;
	}
	else if ((battery_value_sum < BATTERY_Lv2) && (battery_value_sum >= BATTERY_Lv1))
	{
		bat_lvl_cal 		= 2;
	}
	else if ((battery_value_sum < BATTERY_Lv1) && (battery_value_sum >= BATTERY_Lv0))
	{
		bat_lvl_cal 		= 1;
	}
	else if (battery_value_sum < BATTERY_Lv0)
		bat_lvl_cal = 0;

	if ((direction == 0) && (bat_lvl_cal > bat_lvl_cal_bak))
	{
		if (battery_value_sum >= BATTERY_Lv3 + BATTERY_GAP)
		{
			bat_lvl_cal 		= 4;
		}
		else if ((battery_value_sum < BATTERY_Lv3 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv2 + BATTERY_GAP))
		{
			bat_lvl_cal 		= 3;
		}
		else if ((battery_value_sum < BATTERY_Lv2 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv1 + BATTERY_GAP))
		{
			bat_lvl_cal 		= 2;
		}
		else if ((battery_value_sum < BATTERY_Lv1 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv0 + BATTERY_GAP))
		{
			bat_lvl_cal 		= 1;
		}
		else if (battery_value_sum < BATTERY_Lv0 + BATTERY_GAP)
			bat_lvl_cal = 0;
	}


	if ((direction == 1) && (bat_lvl_cal < bat_lvl_cal_bak))
	{
		if (battery_value_sum >= BATTERY_Lv3 - BATTERY_GAP)
		{
			bat_lvl_cal 		= 4;
		}
		else if ((battery_value_sum < BATTERY_Lv3 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv2 - BATTERY_GAP))
		{
			bat_lvl_cal 		= 3;
		}
		else if ((battery_value_sum < BATTERY_Lv2 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv1 - BATTERY_GAP))
		{
			bat_lvl_cal 		= 2;
		}
		else if ((battery_value_sum < BATTERY_Lv1 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv0 - BATTERY_GAP))
		{
			bat_lvl_cal 		= 1;
		}
		else if (battery_value_sum < BATTERY_Lv0 - BATTERY_GAP)
			bat_lvl_cal = 0;
	}


	bat_lvl_cal_bak 	= bat_lvl_cal;
	return bat_lvl_cal;

}


void ap_peripheral_battery_check_calculate(void)
{
	INT8U			bat_lvl_cal;
	INT32U			direction = 0, led_type;

	if (adp_status == 0)
	{
		//unkown state
		return;
	}
	else if (adp_status == 1)
	{
		//adaptor in state
		direction			= 1;					//low voltage to high voltage

		if (previous_direction != direction)
		{
			//msgQSend(ApQ, MSG_APQ_BATTERY_CHARGED_SHOW, NULL, NULL, MSG_PRI_NORMAL);
		}

		previous_direction	= direction;
	}
	else 
	{
		//adaptor out state
		direction			= 0;					//high voltage to low voltage

		if (previous_direction != direction)
		{
			//msgQSend(ApQ, MSG_APQ_BATTERY_CHARGED_CLEAR, NULL, NULL, MSG_PRI_NORMAL);
		}

		previous_direction	= direction;
	}

	battery_value_sum	+= (ad_value >> 4);

	// DBG_PRINT("%d, ",(ad_value>>4));
	bat_ck_cnt++;

	if (bat_ck_cnt >= BATTERY_CNT)
	{
		DBG_PRINT("[%d]\r\n", (ad_value >> 4));
		bat_lvl_cal 		= ap_peripheral_smith_trigger_battery_level(direction);

		// DBG_PRINT("b:[%d],", bat_lvl_cal);
		if (!battery_low_flag)
		{
			msgQSend(ApQ, MSG_APQ_BATTERY_LVL_SHOW, &bat_lvl_cal, sizeof(INT8U), MSG_PRI_NORMAL);
		}

		if ((bat_lvl_cal == 0) && (direction == 0))
		{
			low_voltage_cnt++;

			if (low_voltage_cnt > 3)
			{
				low_voltage_cnt 	= 0;
				ap_state_handling_led_off();

#if C_BATTERY_LOW_POWER_OFF 					== CUSTOM_ON

				if (!battery_low_flag)
				{
					battery_low_flag	= 1;
					{
						INT8U			type;

						msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
						type				= FALSE;
						msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_FREESIZE_CHECK_SWITCH, &type, sizeof(INT8U),
							 MSG_PRI_NORMAL);
						type				= BETTERY_LOW_STATUS_KEY;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U),
							 MSG_PRI_NORMAL);

						//msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
						//DBG_PRINT("fankun3\r\n");
						led_type			= LED_LOW_BATT;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U),
							 MSG_PRI_NORMAL);

						//msgQSend(ApQ, MSG_APQ_BATTERY_LOW_SHOW, NULL, sizeof(INT8U), MSG_PRI_NORMAL);
					}
				}

#endif
				//OSTimeDly(100);
				//display_str_battery_low = 1;
				//msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
			}
		}
		else 
		{
			if (battery_low_flag)
			{
				INT8U			type;

				battery_low_flag	= 0;
				msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_START, NULL, NULL, MSG_PRI_NORMAL);
				type				= GENERAL_KEY;
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
				led_type			= PREV_LED_TYPE;
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
			}

			low_voltage_cnt 	= 0;
		}

		bat_ck_cnt			= 0;
		battery_value_sum	= 0;
	}
}


#endif






void ap_peripheral_night_mode_set(INT8U type)
{
#if 0

	if (type)
	{
		gpio_write_io(IR_CTRL, 1);
	}
	else 
	{
		gpio_write_io(IR_CTRL, 0);
	}

#endif
}


void ap_peripheral_key_init(void)
{
	INT32U			i;

	gp_memset((INT8S *) &key_map, NULL, sizeof(KEYSTATUS));
	ap_peripheral_key_register(GENERAL_KEY);

#if 1

	for (i = 0; i < USE_IOKEY_NO; i++)
	{
		//	DBG_PRINT("key_map:%d\r\n",key_map[i].key_io);
		if (key_map[i].key_io != PW_KEY)
		{
			key_map[i].key_cnt	= 0;
			gpio_init_io(key_map[i].key_io, GPIO_INPUT);
			gpio_set_port_attribute(key_map[i].key_io, ATTRIBUTE_LOW);
			gpio_write_io(key_map[i].key_io, (key_map[i].key_active) ^ 1);

			// DBG_PRINT("INIT\r\n");
		}
	}

#endif

	//gpio_init_io(SWITCH_KEY, GPIO_INPUT);		
	//gpio_set_port_attribute(SWITCH_KEY, ATTRIBUTE_LOW);	
	//gpio_write_io(SWITCH_KEY, 1);
	//switch_key_state=gpio_read_io(SWITCH_KEY);
}


extern void Time_card_storage(TIME_T * intime);


void ap_peripheral_time_set(void)
{
#if 0
	TIME_T			READ_TIME;

	DBG_PRINT("TIME_SET1\r\n");
	ap_state_config_timefile_get(&READ_TIME);
	Time_card_storage(&READ_TIME);

#if USING_EXT_RTC					== CUSTOM_ON
	ap_state_handling_calendar_init();

#else

	ap_state_handing_intime_init(READ_TIME);

#endif

#endif

}


void ap_peripheral_key_register(INT8U type)
{
	INT32U			i;

	if (type == GENERAL_KEY)
	{
		key_map[0].key_io	= PW_KEY;
		key_map[0].key_function = (KEYFUNC)
		ap_peripheral_pw_key_exe;
		key_map[0].key_active = 1;
		key_map[0].key_long = 0;

		key_map[1].key_io	= FUN_KEY;
		key_map[1].key_function = (KEYFUNC)
		ap_peripheral_video_prev_exe;
		key_map[1].key_active = 1;
		key_map[1].key_long = 0;

		// key_map[2].key_io = IRCTR_KEY;
		// key_map[2].key_function = (KEYFUNC) ap_peripheral_ir_change_exe;
		// key_map[2].key_active=1;
		// key_map[2].key_long=0;
	}
	else if (type == USBD_DETECT)
	{
#if USE_IOKEY_NO

		for (i = 0; i < USE_IOKEY_NO; i++)
		{
			if ((key_map[i].key_io != PW_KEY) && (key_map[i].key_io != FUN_KEY))
				key_map[i].key_io = NULL;
		}

#endif

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}
	else if (type == DISABLE_KEY)
	{
#if USE_IOKEY_NO

		for (i = 0; i < USE_IOKEY_NO; i++)
		{
			key_map[i].key_io	= NULL;
		}

#endif

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}
	else if (type == BETTERY_LOW_STATUS_KEY)
	{
		key_map[0].key_io	= PW_KEY;
		key_map[0].key_function = (KEYFUNC)
		ap_peripheral_pw_key_exe;

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}

}


extern INT8U ap_state_config_auto_off_get(void);

INT8U			long_pw_key_pressed = 0;

#if CRAZY_KEY_TEST				== 1
INT8U			crazy_key_enable = 0;
INT32U			crazy_key_cnt = 0;

#endif

//INT8U led_red_prev_on_flag=0;

/**********************************************************************************
*
*
*
************************************************************************************/
void ap_peripheral_key_judge(void)
{
	INT32U			i, key_press = 0;
	INT16U			key_down = 0;


	for (i = 0; i < USE_IOKEY_NO; i++)
	{
		if (key_map[i].key_io) //
		{

			if (key_map[i].key_io == PW_KEY) //
			{
				key_down			= sys_pwr_key0_read();

			}
			else 
			{
				if (key_map[i].key_active)
					key_down = gpio_read_io(key_map[i].key_io);
				else 
					key_down = !gpio_read_io(key_map[i].key_io);
			}

			if (key_map[i].key_io == FUN_KEY)
			{
				if (key_down)
				{
					power_off_cnt		= PERI_POWER_OFF;

					if (!key_map[i].key_long)
					{

						key_map[i].key_cnt	+= 1;

#if SUPPORT_LONGKEY 									== CUSTOM_ON

						if (key_map[i].key_cnt >= Long_Single_width)
						{
							key_map[i].key_long = 1;

							//if(power_keyup == 0)
							key_map[i].key_function(& (key_map[i].key_cnt));
						}

#endif
					}
					else 
					{
						key_map[i].key_cnt	= 0;
					}

					if (key_map[i].key_cnt == 65535)
					{
						key_map[i].key_cnt	= 16;
					}
				}
				else 
				{
					if (key_map[i].key_long)
					{
						key_map[i].key_long = 0;
						key_map[i].key_cnt	= 0;

					}

					if (key_map[i].key_cnt >= Short_Single_width) //Short_Single_width
					{

						if (power_keyup == 0)
						{
							key_map[i].key_function(& (key_map[i].key_cnt));
						}

						key_press			= 1;
					}

					key_map[i].key_cnt	= 0;

					// if(key_map[i].key_io == FUN_KEY)
					//		power_keyup=0;
				}
			}
			else if (key_map[i].key_io == PW_KEY)
			{
				if (key_down) //如果按下时间大于六s，则执行下面语句，即关机
				{
					power_off_cnt		= PERI_POWER_OFF;

					if (!key_map[i].key_long)
					{

						key_map[i].key_cnt	+= 1;

						if (key_map[i].key_cnt >= Long_Single_width)
						{
							if ((video_record_sts & 0x06) == 0x02)
							{
								if (video_720_flag == 1)
								{
									led_green_on();
								}
								else if (video_1080_flag == 1)
								{
									led_red_on();
								}
							}
						}

#if SUPPORT_LONGKEY 									== CUSTOM_ON

						if (key_map[i].key_cnt >= Long_Single_width_power_6s)
						{
							key_map[i].key_long = 1;

							if (power_keyup == 0)
								key_map[i].key_function(& (key_map[i].key_cnt));
						}

#endif
					}
					else 
					{
						key_map[i].key_cnt	= 0;
					}

					if (key_map[i].key_cnt == 65535)
					{
						key_map[i].key_cnt	= 16;
					}
				}
				else //如果放开
				{
					if (key_map[i].key_long) //大于6s，不执行下面的语句
					{
						key_map[i].key_long = 0;
						key_map[i].key_cnt	= 0;

					}

					if (key_map[i].key_cnt >= Long_Single_width) //2s 按下时间在大于2s到6s之间
					{

						if (power_keyup == 0)
						{
							if ((video_record_sts & 0x06) == 0x02)
							{
								if (video_720_flag == 1)
								{
									led_green_off();
								}
								else if (video_1080_flag == 1)
								{
									led_red_off();
								}
							}

							key_map[i].key_function(& (key_map[i].key_cnt));
						}

						key_press			= 1;
					}
					else if (key_map[i].key_cnt >= Short_Single_width) //Short_Single_width
					{

						if (power_keyup == 0)
						{
							key_map[i].key_function(& (key_map[i].key_cnt));
						}

						key_press			= 1;
					}

					key_map[i].key_cnt	= 0;

					if (key_map[i].key_io == PW_KEY)
						power_keyup = 0;
				}
			}
		}
	}


#if 1

	if (!auto_off_force_disable && !s_usbd_pin && ! (usb_state_get()))
	{
		if (power_off_cnt)
		{
			power_off_cnt--;

			if (power_off_cnt <= 0)
			{
				DBG_PRINT("fankun2\r\n");
				msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
			}
		}
	}
	else 
		power_off_cnt = PERI_POWER_OFF;

#endif

#if 0

	if (s_usbd_pin)
	{
		if (gpio_read_io(CHARGE_PIN))
		{
			if (charge_full_flag == 0)
			{
				charge_cnt++;

				if (charge_cnt > 100)
				{
					charge_full_flag	= 1;
					charge_cnt			= 0;
					led_type			= LED_CHARGE_FULL;
					msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
				}
			}
		}
		else 
		{
			charge_full_flag	= 0;
			charge_cnt			= 0;
		}
	}

#endif

	//if(++switch_key_read_cnt > 16)
	//	{
	// switch_key_state=gpio_read_io(SWITCH_KEY);
	// switch_key_read_cnt=0;
	//	}
}


void ap_peripheral_charge_det(void)
{
	INT16U			pin_state = 0;
	static INT16U	prev_pin_state = 0;
	INT16U			led_type;
	static INT8U	loop_cnt = 0;
	static INT16U	prev_ledtype = 0;

	pin_state			= gpio_read_io(CHARGE_PIN);
	DBG_PRINT("pin_state=%d", pin_state);

	if (pin_state == prev_pin_state)
	{
		loop_cnt++;
	}
	else 
		loop_cnt = 0;

	if (loop_cnt >= 3)
	{


		loop_cnt			= 3;

		if (pin_state)
			led_type = LED_CHARGE_FULL;
		else 
			led_type = LED_CHARGEING;

		if (prev_ledtype != led_type)
		{
			prev_ledtype		= led_type;

			if (((video_record_sts & 0x06) == 0))
			{
				if (storage_sd_upgrade_file_flag_get() != 2)
					msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
			}

			//msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
		}
	}

	prev_pin_state		= pin_state;
}


static int ap_peripheral_power_key_read(int pin)
{
	int 			status;


#if (								KEY_TYPE == KEY_TYPE1)||(KEY_TYPE == KEY_TYPE2)||(KEY_TYPE == KEY_TYPE3)||(KEY_TYPE == KEY_TYPE4)||(KEY_TYPE == KEY_TYPE5)
	status				= gpio_read_io(pin);

#else

	switch (pin)
	{
		case PWR_KEY0:
			status = sys_pwr_key0_read();
			break;

		case PWR_KEY1:
			status = sys_pwr_key1_read();
			break;
	}

#endif

	if (status != 0)
		return 1;

	else 
		return 0;
}


void ap_peripheral_adaptor_out_judge(void)
{
	//DBG_PRINT("NA");
	adp_out_cnt++;

	switch (adp_status)
	{
		case 0: //unkown state
			if (ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				//DBG_PRINT("xx:%d",adp_cnt);
				adp_cnt++;

				if (adp_cnt > 16)
				{
					adp_out_cnt 		= 0;
					adp_cnt 			= 0;
					adp_status			= 1;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);

#if C_BATTERY_DETECT								== CUSTOM_ON && USE_ADKEY_NO

					//battery_lvl = 1;
#endif
				}
			}
			else 
			{
				adp_cnt 			= 0;
			}

			//	DBG_PRINT("yy:%d\r\n",adp_out_cnt);
			if (adp_out_cnt > 24)
			{
				adp_out_cnt 		= 0;
				adp_status			= 3;

#if C_BATTERY_DETECT							== CUSTOM_ON && USE_ADKEY_NO

				//battery_lvl = 2;
				low_voltage_cnt 	= 0;
#endif
			}

			break;

		case 1: //adaptor in state
			if (!ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if (adp_out_cnt > 8)
				{
					adp_status			= 2;

#if C_BATTERY_DETECT								== CUSTOM_ON
					low_voltage_cnt 	= 0;
#endif

					// 璝棵辊玂臔秨璶翴獹璉
					if (screen_saver_enable)
					{
						screen_saver_enable = 0;

#if C_SCREEN_SAVER										== CUSTOM_ON
						ap_state_handling_lcd_backlight_switch(1);
#endif
					}
				}
			}
			else 
			{
				adp_out_cnt 		= 0;
			}

			break;

		case 2: //adaptor out state
			if (!ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if ((adp_out_cnt > PERI_ADP_OUT_PWR_OFF_TIME))
				{
					//DBG_PRINT("1111111111111111111\r\n");
					//adp_out_cnt=320;
#if 1

					//ap_peripheral_pw_key_exe(&adp_out_cnt);
#endif

					adp_out_cnt 		= 0;
					usb_state_set(0);
					msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
				}

				adp_cnt 			= 0;
			}
			else 
			{
				adp_cnt++;

				if (adp_cnt > 3)
				{
					adp_out_cnt 		= 0;
					adp_status			= 1;
					usbd_exit			= 0;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				}
			}

			break;

		case 3: //adaptor initial out state
			if (ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if (adp_out_cnt > 3)
				{
					adp_out_cnt 		= 0;
					adp_status			= 1;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				}
			}
			else 
			{
				if (usb_state_get())
				{
					usb_state_set(0);

					if (video_720_flag)
					{
						led_red_off();
						led_green_on();
					}
					else if (video_1080_flag)
					{
						led_red_on();
						led_green_on();
					}
					else 
					{
						led_red_on();
						led_green_off();
					}
				}

				adp_out_cnt 		= 0;
			}

			break;

		default:
			break;
	}

	///DBG_PRINT("USB_PIN=%d\r\n",s_usbd_pin);
	if (s_usbd_pin == 1)
	{
		usbd_cnt++;

		if (!ap_peripheral_power_key_read(C_USBDEVICE_PIN))
		{
			if (usbd_cnt > 3)
			{
				ap_peripheral_usbd_plug_out_exe(&usbd_cnt);
			}
		}
		else 
		{
			usbd_cnt			= 0;
		}
	}

#if USB_PHY_SUSPEND 				== 1

	if (s_usbd_pin == 0)
	{
		if (ap_peripheral_power_key_read(C_USBDEVICE_PIN))
		{
			if (phy_cnt == PERI_USB_PHY_SUSPEND_TIME)
			{
				// disable USB PHY CLK for saving power
				DBG_PRINT("Turn Off USB PHY clk (TODO)\r\n");
				phy_cnt++;							// ヘ琌 Turn Off 暗Ω
			}
			else if (phy_cnt < PERI_USB_PHY_SUSPEND_TIME)
			{
				phy_cnt++;
			}
		}
		else 
		{
			phy_cnt 			= 0;
		}
	}

#endif

}


void ap_peripheral_function_key_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_MODE, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		DBG_PRINT("function_Key\r\n");

		if (*tick_cnt_ptr > 24)
		{
		}
		else 
		{
			DBG_PRINT("MODE_ACTIVE\r\n");
			DBG_PRINT("*tick_cnt_ptr=%d\r\n", *tick_cnt_ptr);
			msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_next_key_exe(INT16U * tick_cnt_ptr)
{
	INT8U data			= 0;

	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_DOWN, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			msgQSend(ApQ, MSG_APQ_FORWARD_FAST_PLAY, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_NEXT_KEY_ACTIVE, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_prev_key_exe(INT16U * tick_cnt_ptr)
{
	INT8U data			= 0;

	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_UP, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			msgQSend(ApQ, MSG_APQ_BACKWORD_FAST_PLAY, &data, sizeof(INT8U), MSG_PRI_NORMAL);

		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_PREV_KEY_ACTIVE, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_ok_key_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_OK, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			//for test
			msgQSend(ApQ, MSG_APQ_INIT_THUMBNAIL, NULL, NULL, MSG_PRI_NORMAL);

			//			msgQSend(ApQ, MSG_APQ_FILE_LOCK_DURING_RECORDING, NULL, NULL, MSG_PRI_NORMAL);
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_FUNCTION_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


#if KEY_FUNTION_TYPE			== SAMPLE2

void ap_peripheral_capture_key_exe(INT16U * tick_cnt_ptr)
{
	INT32U led_type;

#if GPDV_BOARD_VERSION				!= GPCV1237A_Aerial_Photo	
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_OK, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		msgQSend(ApQ, MSG_APQ_CAPTURE_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
	}

#else

	DBG_PRINT("PIC_FLAG=%d\r\n", pic_down_flag);

	if (!s_usbd_pin)
	{
		if (ap_state_handling_storage_id_get() == NO_STORAGE)
		{

			led_type			= LED_TELL_CARD;
			msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
		}

		if ((ap_state_handling_storage_id_get() != NO_STORAGE) && (!pic_down_flag) && (!card_space_less_flag) &&
			 (!video_down_flag))
		{
#if SUPPORT_LONGKEY 						== CUSTOM_ON

			if (*tick_cnt_ptr > 24)
			{
			}
			else 
#endif

			{
				DBG_PRINT("[CAPTUER_ACTIVE...]\r\n");
				msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
			}
		}
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
		else 
		{
			if (!pic_down_flag)
				OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
	}

#endif

	* tick_cnt_ptr		= 0;
}


#endif

void ap_peripheral_sos_key_exe(INT16U * tick_cnt_ptr)
{
#if CRAZY_KEY_TEST					== 1	

	if (!crazy_key_enable)
	{
		crazy_key_enable	= 1;
	}
	else 
	{
		crazy_key_enable	= 0;
	}

	*tick_cnt_ptr		= 0;
	return;

#endif

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_SOS_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_usbd_plug_out_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_DISCONNECT_TO_PC, NULL, NULL, MSG_PRI_NORMAL);
	*tick_cnt_ptr		= 0;
}


void ap_peripheral_pw_key_exe(INT16U * tick_cnt_ptr)
{
//	INT8U i, led_type;

	if ((!s_usbd_pin) && (!pic_down_flag) && (!video_down_flag))
	{

#if SUPPORT_LONGKEY 					== CUSTOM_ON

		if (*tick_cnt_ptr > 383)
		{
			if ((video_record_sts & 0x06) == 0)
			{
				if (usb_state_get() == 0)
					msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
			}

		}

		else 
#endif

		{
			if ((ap_state_handling_storage_id_get() != NO_STORAGE) && (!card_space_less_flag))
			{

				if (*tick_cnt_ptr > 191)
				{
					if (usb_state_get() == 0) //没有接usb或火牛时执行下面的程序，否则不执行
					{
						if (tv_plug_status_get() == 0) //不在TVout时
						{
							if ((video_record_sts & 0x02) && ((video_record_sts & 0x04) == 0))
							{
								if (IR_flag == 0)
								{
									IR_flag 			= 1;
									night_led_ctrl(1);

									if (video_720_flag == 1)
									{
//										led_green_on();
//										OSTimeDly(20);
//										led_green_off();
										g_led_r_state		= 7;
									}
									else if (video_1080_flag == 1)
									{
//										led_red_on();
//										OSTimeDly(20);
//										led_red_off();
										g_led_g_state		= 9;
									}

									//led_type = LED_IR_ON;
									// msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
								}
								else if (IR_flag == 1)
								{
									IR_flag 			= 0;
									night_led_ctrl(0);

									if (video_720_flag == 1)
									{
										//										led_green_on();
										//										OSTimeDly(20);
										//										led_green_off();
										g_led_r_state		= 8;
									}
									else if (video_1080_flag == 1)
									{
										//g_led_g_state = 8;
										//										led_red_on();
										//										OSTimeDly(20);
										//										led_red_off();
										g_led_g_state		= 7;
									}

									//led_type = LED_IR_OFF;
									// msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
								}
							}
						}
						else 
						{
							IR_flag 			= 0;
							night_led_ctrl(0);
						}
					}
					else //有接usb或火牛时关红外
					{
						IR_flag 			= 0;
						night_led_ctrl(0);
					}
				}
				else 
				{
#if 0
					sys_release_timer_isr(LED_blanking_isr);

					for (i = 0; i < 3; i++)
					{
						led_red_off();
						OSTimeDly(10);
						led_red_on();
						OSTimeDly(10);
					}

					sys_registe_timer_isr(LED_blanking_isr);
#endif

					if ((video_record_sts & 0x04) == 0x04) //如果在移动侦测状态
					{
						if ((video_record_sts & 0x02) == 0) //如果bu在录像
						{
							motion_wait_to_record = 1; //在移动侦测待机时置一

							if (video_720_flag == 1) //退出移动侦测待机，显示录像的led灯状态
							{
								led_red_on();
								led_green_off();

								//OSTimeDly(30);	
							}
							else if (video_1080_flag == 1)
							{
								led_red_off();
								led_green_on();

								//OSTimeDly(30);	
							}

							//				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_MOTION_DETECT_DELAY, NULL, NULL, MSG_PRI_NORMAL);
							ap_video_record_md_disable();
							ap_state_config_md_set(0);

						}
					}

					if (motion_wait_to_record == 0) //
					{
						if (video_720_flag == 1)
						{
							if (video_record_sts & 0x02)
							{
								IR_flag 			= 0;
								night_led_ctrl(0);
							}

							msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
						}
						else if (video_1080_flag == 1)
						{
							if (video_record_sts & 0x02)
							{
								IR_flag 			= 0;
								night_led_ctrl(0);
							}

							msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
						}
						else if (capture_flag == 1)
						{
							msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
						}
					}
					else 
						motion_wait_to_record = 0; //如果为一，则只做清零操作，再按功能键时，执行相应的功能

				}


			}
		}

	}
	else if (s_usbd_pin)
	{
		//DBG_PRINT("usb_switch\r\n");
		if (ap_state_handling_storage_id_get() != NO_STORAGE)
			OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_video_prev_exe(INT16U * tick_cnt_ptr)
{
	INT8U i, type;

	if ((!s_usbd_pin) && (!pic_down_flag) && (!card_space_less_flag) && (!video_down_flag))
	{
		if (ap_state_handling_storage_id_get() != NO_STORAGE)
		{
			//if(*tick_cnt_ptr > 319)
			//{
			//}
			//else
			//{
			if (*tick_cnt_ptr > 191) //移动侦测
			{
				if ((video_record_sts & 0x06) == 0)
				{
#if 0

					if (video_720_flag == 1)
					{
						type				= LED_MONTION_WAIT;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
					else if (video_1080_flag == 1)
					{
						type				= LED_MONTION_WAIT1080;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
					}

#endif

					//motion_first_flag=1;//进入移动侦测待机时，待机led动作
					msgQSend(ApQ, MSG_APQ_MD_SET, NULL, NULL, MSG_PRI_NORMAL);
				}

				else if ((video_record_sts & 0x04) == 0x04) //如果在移动侦测状态，只是关移动侦测
				{
					if ((video_record_sts & 0x02) == 0x02) //如果在录像
						msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					else 
					{
						//ap_video_record_md_icon_clear_all();
						//				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_MOTION_DETECT_DELAY, NULL, NULL, MSG_PRI_NORMAL);
						ap_video_record_md_disable();
						ap_state_config_md_set(0);

						if (video_720_flag == 1) //退出移动侦测待机，显示录像的led灯状态
						{
							led_red_on();
							led_green_off();
							g_led_r_state		= 0;

							//OSTimeDly(5);	
						}
						else if (video_1080_flag == 1)
						{
							led_red_off();
							led_green_on();
							g_led_g_state		= 0;

							//OSTimeDly(5);	
						}
					}
				}


			}
			else //mode_count=0， 720p录像；mode_count=1， 1080p录像；mode_count=2， 拍照；
			{
#if 0

				if ((video_record_sts & 0x04) == 0x04) //如果在移动侦测状态，只是关移动侦测
				{
					if ((video_record_sts & 0x02) == 0x02) //如果在录像
						msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					else 
					{
						//ap_video_record_md_icon_clear_all();
						//				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_MOTION_DETECT_DELAY, NULL, NULL, MSG_PRI_NORMAL);
						ap_video_record_md_disable();
						ap_state_config_md_set(0);

						if (video_720_flag == 1) //退出移动侦测待机，显示录像的led灯状态
						{
							led_red_on();
							led_green_off();
							OSTimeDly(30);
						}
						else if (video_1080_flag == 1)
						{
							led_red_on();
							led_green_on();
							OSTimeDly(30);
						}
					}
				}
				else 
#endif

				if ((video_record_sts & 0x06) == 0)
				{
					mode_count++;

					if (mode_count == 3)
					{
						mode_count			= 0;
					}

					if (mode_count == 0)
					{
						video_720_flag		= 1;
						video_1080_flag 	= 0;
						capture_flag		= 0;
						Video_res_set();
						type				= LED_WAITING_RECORD;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
					else if (mode_count == 1)
					{
						video_720_flag		= 0;
						video_1080_flag 	= 1;
						capture_flag		= 0;
						Video_res_set();
						type				= LED_WAITING_RECORD1080;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
					else if (mode_count == 2)
					{
						video_720_flag		= 0;
						video_1080_flag 	= 0;
						capture_flag		= 1;
						type				= LED_WAITING_CAPTURE;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
					}

					//msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
				}


			}
			//}
		}
	}
	else if (s_usbd_pin)
	{

		//if(ap_state_handling_storage_id_get()!= NO_STORAGE)
		//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
	}

	*tick_cnt_ptr		= 0;

	//static INT8U	 mode_count=0;//短按计数，控制不同模式
	//static INT8U video_720_flag=1;//进入720p录像标志位
	//static INT8U video_1080_flag=0;//进入1080P录像标志位
	//static INT8U capture_flag=0;//拍照标志位
	//static INT8U capture_first=0;//进入录像标志
}


void ap_peripheral_ir_change_exe(INT16U * tick_cnt_ptr)
{


}


void ap_peripheral_menu_key_exe(INT16U * tick_cnt_ptr)
{

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_null_key_exe(INT16U * tick_cnt_ptr)
{

}


#if GPDV_BOARD_VERSION			!= GPCV1237A_Aerial_Photo
void ap_TFT_backlight_tmr_check(void)
{
	if (backlight_tmr)
	{
		backlight_tmr--;

		if ((backlight_tmr == 0) && (tv == !TV_DET_ACTIVE))
		{
			//gpio_write_io(TFT_BL, DATA_HIGH);	//turn on LCD backlight
#if _DRV_L1_TFT 							== 1
			tft_backlight_en_set(1);
#endif
		}
	}
}


#endif

//+++ TV_OUT_D1
#if TV_DET_ENABLE
INT8U tv_plug_status_get(void)
{
	return tv_plug_in_flag;
}


#endif

//---
void Close_tvoutput(void)
{
#if 1

	if (tv_plug_in_flag)
	{
		//if (ap_state_handling_storage_id_get() != NO_STORAGE)//处理无卡进pc_camera无图问题
		usbd_plug_in_flag	= 1;
		tv_plug_in_flag 	= 0;
		msgQSend(ApQ, MSG_APQ_TV_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);
	}

#endif
}


void ap_peripheral_tv_detect(void)
{
#if TV_DET_ENABLE
	INT8U temp;

	if (usb_state_get() == 1)
	{
		tv_debounce_cnt 	= 0;
		return;
	}

#if 1

	if (usb_state_get() == 2)
	{
		tv_debounce_cnt 	= 0;

		if (tv_plug_in_flag)
		{
			tv_plug_in_flag 	= 0;
			msgQSend(ApQ, MSG_APQ_TV_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);
		}

		msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_STOP, NULL, NULL, MSG_PRI_NORMAL);

		//OSTimeDly(10);
		return;
	}

#endif

	if (tv_plug_in_flag)
	{
		IR_flag 			= 0;
		night_led_ctrl(0);							//关红外
	}

	temp				= gpio_read_io(AV_IN_DET);

	if (temp != tv) //if(0)
	{
		tv_debounce_cnt++;

		if (tv_debounce_cnt > 4)
		{
			tv_debounce_cnt 	= 0;
			tv					= temp;

			if (tv == !TV_DET_ACTIVE)
			{ //display use TFT

				//backlight_tmr = PERI_TIME_BACKLIGHT_DELAY;	//delay some time to enable LCD backlight so that no noise shown on LCD
				//gpio_write_io(SPEAKER_EN, DATA_HIGH);	//open local speaker
				//+++ TV_OUT_D1
				tv_plug_in_flag 	= 0;
				msgQSend(ApQ, MSG_APQ_TV_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);

				//---
			}
			else 
			{ //display use TV

				//gpio_write_io(SPEAKER_EN, DATA_LOW);	//mute local speaker
				//gpio_write_io(TFT_BL, DATA_LOW);		//turn off LCD backlight
				//+++ TV_OUT_D1
				tv_plug_in_flag 	= 1;
				DBG_PRINT("dddddddd++++=\r\n");
				msgQSend(ApQ, MSG_APQ_TV_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);

				//---
			}
		}
	}
	else 
	{
		tv_debounce_cnt 	= 0;
	}

#endif
}


#ifdef SDC_DETECT_PIN
void ap_peripheral_SDC_detect_init(void)
{

	gpio_init_io(SDC_DETECT_PIN, GPIO_INPUT);
	gpio_set_port_attribute(SDC_DETECT_PIN, ATTRIBUTE_LOW);
	gpio_write_io(SDC_DETECT_PIN, 1);				//pull high
}


INT32S ap_peripheral_SDC_at_plug_OUT_detect()
{
	INT32S ret;
	BOOLEAN cur_status;
	ap_peripheral_SDC_detect_init();
	cur_status			= gpio_read_io(SDC_DETECT_PIN);
	DBG_PRINT("SDC_DETECT_PIN_=%d\r\n", cur_status);

	if (cur_status)
	{ //plug_out
		ret 				= -1;
	}
	else 
	{ //plug_in	
		ret 				= 0;
	}

	return ret;
}



INT32S ap_peripheral_SDC_at_plug_IN_detect()
{
	INT32S ret;
	BOOLEAN cur_status;
	ap_peripheral_SDC_detect_init();
	cur_status			= gpio_read_io(SDC_DETECT_PIN);
	DBG_PRINT("SDC_DETECT_PIN=%d\r\n", cur_status);

	if (cur_status)
	{ //plug_out
		ret 				= -1;
	}
	else 
	{ //plug_in
		ret 				= 0;
	}

	return ret;
}


#endif


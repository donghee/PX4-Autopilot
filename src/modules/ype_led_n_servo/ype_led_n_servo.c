/****************************************************************************************************************************
 * @file 		ype_led_n_servo.c
 *
 * @author 		mgyoo <mgyoo@ypelec.co.kr>
 *                      ,YP Electronics ,South Korea
 *
 * @version		v1.0(2026-04-21)
 *
 * @date		2026-04-17 (First Update Date)
 * 			2026-04-21 (Last Update Date)
 ****************************************************************************************************************************/

// Library
// C Standard
	#include <stdio.h>
	#include <stdint.h>
	#include <stdlib.h>
	#include <string.h>
	#include <unistd.h>

// px4
	#include <px4_platform_common/px4_config.h>
	#include <px4_platform_common/log.h>
	#include <px4_platform_common/tasks.h>
	#include <px4_platform_common/posix.h>
	#include <poll.h>
	#include <mathlib/mathlib.h>
	#include <float.h>
// System
	#include <lib/parameters/param.h>
	#include <systemlib/mavlink_log.h>
// NuttX OS
	#include <systemlib/err.h>
	#include <drivers/drv_hrt.h>
// Mathematics
	#include <math.h>
// PX4 Basic Topic
	#include <uORB/uORB.h>
	#include <uORB/topics/system_power.h>     	// system power status MSG (Polling MSG) : Approx. 100Hz
	#include <uORB/topics/actuator_test.h>
	#include <uORB/topics/actuator_servos.h>
	#include <uORB/topics/input_rc.h>
	#include <uORB/topics/vehicle_attitude.h>
	#include <uORB/topics/parameter_update.h>
	#include <uORB/topics/vehicle_status.h>
// User Defined Topic


//==============================================================================================================================
// Macros/Variables for this application (Task/Thread)

// Const. & Vars  : Task
	#define YPE_UART_prior		237			// 255 is first priority    [ 0 ~ 255 ]
	#define T_OUT_ms		15			// Task Time Out time [ms]
	#define RX_Debug 		0

// Const. & Vars  : Scaling
	#define usec2sec		0.000001f		// Converting Value usec to sec

// Const. & Vars  : ORB Topic Subscriber & Structure for this application
	int _SysPower_sub;
	struct system_power_s	SysPower_s;

// Thread Variable ( Exit, Status, Handle )
	static bool thread_should_exit = false;		// Thread Exit	 Flag
	static bool thread_running = false;		// Thread Status Flag
	static int user_daemon_task;			// Handle of Thread

// Function  : Exported Main Function & Main loop function pre-call
	__EXPORT int ype_led_n_servo_main(int argc, char *argv[]);
	int YPE_LED_N_SERVO_TASK_main(int argc, char *argv[]);

// Function  : Print the correct usage.
	static void usage(const char *reason);
	static void
	usage(const char *reason){
		if (reason)	warnx("%s\n", reason);
		warnx("usage: ex {start|stop|status} [-p <additional params>]\n\n");}

//==============================================================================================================================
// User Defined Variables

//  Mavlink log uORB handle
 	static orb_advert_t mavlink_log_pub_user = NULL;

// Const. & Vars  : ORB Structure & Subscriber ID SUB & PUB
	// Subscription
		int _rc_rx_sub;
		int _v_att_sub;
		int _v_status_sub;
		int _param_update_sub;

		struct input_rc_s _rc_rx_s;
		struct vehicle_attitude_s _v_att_s;
		struct vehicle_status_s _v_status_s;

	// Publication
		orb_advert_t _servo_test_pub;
		orb_advert_t _servo_pub;

		struct actuator_test_s _test_servo_s;
		struct actuator_servos_s _servo_s;

// Time
	static hrt_abstime last_time = 0;

// About Servo
	#define ACTUATOR_TEST_ACTION_DO_CONTROL  1 // 명령
	#define ARMING_STATE_ARMED 2	           // Arming  상태
	// RC Input Values
		#define max_position 1999 // 조종기 값으로부터 측정
		#define min_position 999  // 조종기 값으로부터 측정
		#define delta_position (0.5f*(float)(max_position-min_position))
		#define mid_position 1499 // 조종기 값으로부터 측정
		#define mid_min (mid_position-YPE_DIAL_DEAD)
		#define mid_max (mid_position+YPE_DIAL_DEAD)
	// Servo Outputs
		#define min_pitch (-90.0f+YPE_SERVO_DELTA)
		#define max_pitch (20.0f+YPE_SERVO_DELTA)
		#define del_pitch (max_pitch-min_pitch)
	// Parameters
		static param_t _param_pitch_20deg;
		static param_t _param_pitch_rate;
		static param_t _param_dial_dead;
		static param_t _param_servo_delta;
		static int32_t PWM_MAIN_MAX5 = 2000;    // [us]
		static int32_t PWM_MAIN_MIN5 = 1000;    // [us]
		static int32_t PWM_MAIN_PITCH20DEG = 1750; // [us]
		static float YPE_DIAL_PITCHRATE    = 50.0f; // [deg/s]
		static int32_t YPE_DIAL_DEAD       = 50; // [us]
		static float YPE_SERVO_DELTA       = 0.0f; // [deg]
	// Main Varaiables
		float Servo_value    =   0.0f;   // -1.0 ~ 1.0
		float Servo_position = -90.0f;   // [deg]
		float Dial_Velocity  =   0.0f;
		bool deadzone_dial = true;

//==============================================================================================================================
// User Defined Function

	// topic 업데이트 및 저장
	void GET_led_n_servo(void);
	void GET_led_n_servo(void){
		bool updated = false;
		orb_check(_rc_rx_sub, &updated);
		if(updated){	orb_copy(ORB_ID(input_rc), _rc_rx_sub, &_rc_rx_s);		}

		updated = false;
		orb_check(_v_att_sub, &updated);
		if(updated){	orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att_s);	}

		updated = false;
		orb_check(_v_status_sub, &updated);
		if(updated){	orb_copy(ORB_ID(vehicle_status), _v_status_sub, &_v_status_s);	}
	}

	void CAL_servo_value(void);
	void CAL_servo_value(void){
		// Parameter 업데이트
		bool param_updated = false;
		orb_check(_param_update_sub, &param_updated);
		if (param_updated) {
			struct parameter_update_s p_update;
			orb_copy(ORB_ID(parameter_update), _param_update_sub, &p_update);
			param_get(_param_pitch_20deg, &PWM_MAIN_PITCH20DEG);
			param_get(_param_pitch_rate,  &YPE_DIAL_PITCHRATE);
			param_get(_param_dial_dead,   &YPE_DIAL_DEAD);
			param_get(_param_servo_delta, &YPE_SERVO_DELTA);
		}

		// Dial Velocity 계산
		float del_pos = delta_position;
		if(fabsf(del_pos) < FLT_EPSILON){ del_pos = 1.0f; }
		uint16_t Dial_Position = (_rc_rx_s.values[8]);
		if(mid_min < Dial_Position && Dial_Position < mid_max){
			Dial_Velocity = 0.0f;
			deadzone_dial = true;
		}else if(mid_max <= Dial_Position){
			Dial_Velocity = (((float)(Dial_Position-mid_position))/delta_position)*YPE_DIAL_PITCHRATE;
			deadzone_dial = false;
		}else if(Dial_Position <= mid_min){
			Dial_Velocity = -1.0f*(((float)(mid_position-Dial_Position))/delta_position)*YPE_DIAL_PITCHRATE;
			deadzone_dial = false;
		}else{
			Dial_Velocity = Dial_Velocity;
			deadzone_dial = false;
		}

		// delta time 계산
		hrt_abstime now = hrt_absolute_time();
		float dt = (float)(now-last_time)*usec2sec;
		if(dt > 0.02f){dt = 0.01f;}
		last_time = now;

		// 적분 수행 --> Servo 각 계산
		if(deadzone_dial == true){ Servo_position = Servo_position;
		}else{ 			   Servo_position = Servo_position + (Dial_Velocity*dt);}

		// 적분 결과 Saturation
		if(Servo_position >  max_pitch){
			Servo_position =  max_pitch;
		}else if(Servo_position < min_pitch){
			Servo_position = min_pitch;
		}else{
			Servo_position = Servo_position;
		}

		// Servo 출력 변환
		float del = del_pitch;
		if(fabsf(del) < FLT_EPSILON){ del = 110.0f; }
		float denom = (float)(PWM_MAIN_MAX5 - PWM_MAIN_MIN5);
		if(fabsf(denom) < FLT_EPSILON){ denom = 1000.0f; }
		float temp  =  2.0f*(((float)(PWM_MAIN_PITCH20DEG - PWM_MAIN_MIN5))/
		                     (denom));
		Servo_value = -1.0f + (temp)*((Servo_position-min_pitch)/del);
		if(Servo_value >  1.0f){ Servo_value =  1.0f; }
		if(Servo_value < -1.0f){ Servo_value = -1.0f; }

		// Not Armed
			_test_servo_s.value = Servo_value;
		// Armed
			_servo_s.control[0] = Servo_value;
			// actuator_servos timestamp_sample 업데이트
			_servo_s.timestamp_sample = hrt_absolute_time();
	}

	unsigned char out;
	void SET_led_n_servo(void);
	void SET_led_n_servo(void){

		if(_v_status_s.arming_state == ARMING_STATE_ARMED){
			if(_servo_pub != NULL){
				_servo_s.timestamp = hrt_absolute_time();
				orb_publish(  ORB_ID(actuator_servos), _servo_pub, &_servo_s );
				out = 2;
			}else{	out = 3;	}
		}else{
			if(_servo_test_pub != NULL){
				_test_servo_s.function = 201; // Servo #1
				_test_servo_s.action = ACTUATOR_TEST_ACTION_DO_CONTROL;
				_test_servo_s.timeout_ms = 0;
				_test_servo_s.timestamp = hrt_absolute_time();
				orb_publish(  ORB_ID(actuator_test), _servo_test_pub, &_test_servo_s  );
				out = 0;
			}else{	out = 1;	}
		}
	}

	void INIT_ype_led_n_servo(void);
	void INIT_ype_led_n_servo(void){

		// subscription 초기화 및 업데이트
		_rc_rx_sub = orb_subscribe(ORB_ID(input_rc));
		_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

		memset(&_servo_s, 0, sizeof(_servo_s));
		_v_status_sub = orb_subscribe(ORB_ID(vehicle_status));

		memset(&_servo_s, 0, sizeof(_servo_s));
		memset(&_test_servo_s, 0, sizeof(_test_servo_s));
		_servo_test_pub = orb_advertise(ORB_ID(actuator_test), &_test_servo_s);
		_servo_pub = orb_advertise(ORB_ID(actuator_servos), &_servo_s);

		// PX4 parameter 초기화 및 업데이트
		_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
		param_get(param_find("PWM_MAIN_MAX5"), &PWM_MAIN_MAX5);
		param_get(param_find("PWM_MAIN_MIN5"), &PWM_MAIN_MIN5);

		// YPE parameter 초기화 및 업데이트
		_param_pitch_20deg = param_find("YPE_SERVO_20DEG");
		_param_pitch_rate = param_find("YPE_DIAL_RATE");
    		param_get(_param_pitch_20deg, &PWM_MAIN_PITCH20DEG);
		param_get(_param_pitch_rate, &YPE_DIAL_PITCHRATE);
		_param_dial_dead   = param_find("YPE_DIAL_DEAD");
		_param_servo_delta = param_find("YPE_SERVO_DN_OFF");
		param_get(_param_dial_dead,   &YPE_DIAL_DEAD);
		param_get(_param_servo_delta, &YPE_SERVO_DELTA);

		// 초기 위치를 min_pitch + offset 으로 맞춤
    		Servo_position = -90.0f + YPE_SERVO_DELTA;

    		last_time = hrt_absolute_time();
	}

// Task Main Function
int ype_led_n_servo_main(int argc, char *argv[])
{
	// Input : "null"
	if (argc < 2) {
		usage("missing command");
		return 1;
	}
	// Input : "start"
	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running\n");
			return 0;
		}
		// Task spawn
		thread_should_exit = false;
		user_daemon_task = px4_task_spawn_cmd("ype_led_n_servo",	// Task Name
				 	 	 	SCHED_DEFAULT,	   	// Scheduling Method
							YPE_UART_prior,		// SCHED_PRIORITY_DEFAULT // ( Original Code )
							2500,		   	// Memory Stack Size [Bytes]
							YPE_LED_N_SERVO_TASK_main,	// The name of TASK Function
							(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}
	// Input : "stop"
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}
	// Input : "status"
	if (!strcmp(argv[1], "status")) {
		if (thread_running)	warnx("\trunning\n");
		else	warnx("\tnot started\n");
		return 0;
	}
	// Input : undefined input
	usage("unrecognized command");
	return 1;
}

//==============================================================================================================================
// Thread Main Function
int YPE_LED_N_SERVO_TASK_main(int argc, char *argv[]){

	// POLLING Setting : ORB Subscribe
	// - Run module apps at the same frequency as adc (100Hz)
	_SysPower_sub		  = orb_subscribe(ORB_ID(system_power));
	px4_pollfd_struct_t fds[] = {
		{ .fd = _SysPower_sub,   .events = POLLIN },	// fds[0]  : Local Position Message(Position Controller)
	};

	#if RX_Debug
		int count_d =0;
	#endif

	// Thread is Start!!
	thread_running = true;
	// Initialization for this application
	INIT_ype_led_n_servo();
	mavlink_log_info(&mavlink_log_pub_user, "[YPE] LED & SERVO APP is started");

	// Infinite Loop
	while (!thread_should_exit){
		// Get the Polling the Event
		// px4_poll function Input :
		// File Descriptor, Number of Msg., Timeout [ms]
		int poll_ret = px4_poll( &fds[0], (sizeof(fds)/sizeof(fds[0])), T_OUT_ms);
		if(poll_ret == 0) {					// CASE  :  Time out
			warn("Time out %d, %d \r", poll_ret, errno);
			continue;	// Go to while statement
		}
		if(poll_ret <  0) {					// CASE  :  Poll Error
			warn("Poll error \r");
			continue;	// Go to while statement
		}
		if(poll_ret >  0) {					// CASE  :  No Time out/Error
			if (fds[0].revents & POLLIN){			// fds[0] : Local Position Event is Enabled
			// Copy the Sub#1 Data to local buffer
			orb_copy(ORB_ID(system_power),_SysPower_sub,&SysPower_s);
				//-------------------------------------------------------Start Point : User Task #1
				GET_led_n_servo();
				CAL_servo_value();
				SET_led_n_servo();
				#if RX_Debug
					count_d++;
					if (count_d>200){
						mavlink_log_info(&mavlink_log_pub_user, "%1.2f SRV:%1.2f SRV_T:%1.2f,%d",
								 (double)Servo_value, (double)_servo_s.control[0], (double)_test_servo_s.value, out);
						count_d = 0;
					}
				#endif
				//-------------------------------------------------------End Point   : User Task #1

			}// Event  End

		}// Poll Loop End

	}// Infinite Loop End
	// Thread is Stopped!!
	warnx("exiting.\n");
	thread_running = false;
	return 0;
}
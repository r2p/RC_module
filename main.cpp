#include <cstdio>

#include "ch.h"
#include "hal.h"

#include <r2p/Middleware.hpp>
#include <r2p/msg/motor.hpp>

#include <led/nodes/led.hpp>

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(MODULE_NAME, "BOOT_"MODULE_NAME);

uint16_t in_values[12] = {0};
uint16_t out_values[12] = {0};
static const uint16_t failsafe_values[12] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

systime_t last_update = 0;
bool failsafe = true;
int idx = 0;

static void icuwidthcb(ICUDriver *icup) {
	const icucnt_t width = icuGetWidth(icup);

	chSysLockFromIsr();

	if ((width > 2500) || (width < 500)) {
		// sync pulse or glitch
		idx = 0;
		chSysUnlockFromIsr();
		return;
	}

	in_values[idx++] = width;

	if (idx >= 8) {
		idx = 0;
	}

	last_update = chTimeNow();

	chSysUnlockFromIsr();
}

static void icuperiodcb(ICUDriver *icup) {

}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  1000000,                                  /* 1 MHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  NULL,
  ICU_CHANNEL_2,
  0
};

/*
 * RC input node.
 */
struct rcin_node_conf {
	const char * name;
	const char * topic;
};

msg_t rcin_node(void * arg) {
	rcin_node_conf * conf = (rcin_node_conf *) arg;
	r2p::Node node(conf->name);
	r2p::Publisher<r2p::ServoMsg> servo_pub;
	r2p::ServoMsg * msgp;

	(void) arg;
	chRegSetThreadName(conf->name);
	node.advertise(servo_pub, conf->topic);

	icuStart(&ICUD4, &icucfg);
	icuEnable(&ICUD4);

	while (!chThdShouldTerminate()) {
		if (servo_pub.alloc(msgp)) {
			for (int i = 0; i < 8; i++) {
				msgp->pulse[i] = in_values[i];
			}

			servo_pub.publish(msgp);
		}

		chThdSleepMilliseconds(20);
	}

	return CH_SUCCESS;
}


/*
 * PWM cyclic callback.
 */
static void pwmcb(PWMDriver *pwmp) {
	const uint16_t * values;

	(void) pwmp;
	chSysLockFromIsr()

	if (failsafe) {
		values = failsafe_values;
	} else {
		values = out_values;
	}

	if (pwmp == &PWMD1) {
		pwm_lld_enable_channel(pwmp, 0, values[6]);
		pwm_lld_enable_channel(pwmp, 1, values[7]);
		pwm_lld_enable_channel(pwmp, 2, values[5]);
		pwm_lld_enable_channel(pwmp, 3, values[4]);
	}

	if (pwmp == &PWMD2) {
		pwm_lld_enable_channel(pwmp, 0, values[8]);
		pwm_lld_enable_channel(pwmp, 1, values[9]);
		pwm_lld_enable_channel(pwmp, 2, values[10]);
		pwm_lld_enable_channel(pwmp, 3, values[11]);
	}

	if (pwmp == &PWMD3) {
		pwm_lld_enable_channel(pwmp, 0, values[0]);
		pwm_lld_enable_channel(pwmp, 1, values[1]);
		pwm_lld_enable_channel(pwmp, 2, values[2]);
		pwm_lld_enable_channel(pwmp, 3, values[3]);
	}

	chSysUnlockFromIsr();
}

/*
 * PWM configuration.
 */
static PWMConfig pwmcfg = { 1000000, /* 1 MHz PWM clock frequency.   */
20000, /* 50 Hz frequency. */
pwmcb, { { PWM_OUTPUT_ACTIVE_HIGH, NULL }, { PWM_OUTPUT_ACTIVE_HIGH, NULL }, { PWM_OUTPUT_ACTIVE_HIGH, NULL }, {
		PWM_OUTPUT_ACTIVE_HIGH, NULL } }, 0, 0};

/*
 * RC output node.
 */
struct rcout_node_conf {
	const char * name;
	const char * topic;
};

bool servo_cb(const r2p::ServoMsg &msg) {

	for (int i = 0; i < 8; i++) {
		out_values[i] = msg.pulse[i];
	}

	last_update = chTimeNow();

	return true;
}

msg_t rcout_node(void * arg) {
	rcout_node_conf * conf = (rcout_node_conf *) arg;
	r2p::Node node(conf->name);
	r2p::Subscriber<r2p::ServoMsg, 5> servo_sub(servo_cb);

	(void) arg;

	chRegSetThreadName(conf->name);
	node.subscribe(servo_sub, conf->topic);

	pwmStart(&PWMD1, &pwmcfg);
	pwmStart(&PWMD2, &pwmcfg);
	pwmStart(&PWMD3, &pwmcfg);

	while (!chThdShouldTerminate()) {
		node.spin(r2p::Time::ms(1000));
	}

	return CH_SUCCESS;
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	rcin_node_conf rcin_conf = {"rcin_node", "rcin"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, rcin_node, &rcin_conf);

	rcout_node_conf rcout_conf = {"rcout_node", "rcout"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, rcout_node, &rcout_conf);

	for (;;) {
		if (chTimeNow() - last_update > 500) {
			failsafe = true;
		} else {
			failsafe = false;
		}

		r2p::Thread::sleep(r2p::Time::ms(100));
	}

	return CH_SUCCESS;
}
}

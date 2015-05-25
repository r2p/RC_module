#include <cstdio>

#include "ch.h"
#include "hal.h"

#include <r2p/Middleware.hpp>
#include <r2p/msg/std_msgs.hpp>

#include <led/nodes/led.hpp>

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(MODULE_NAME, "BOOT_"MODULE_NAME);

uint16_t ppm[12] = {1500};
static const uint16_t failsafe_values[12] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

systime_t last_update = 0;
bool failsafe = true;
int idx = 0;

static void icuwidthcb(ICUDriver *icup) {
	const icucnt_t width = icuGetWidth(icup);

	chSysLockFromIsr();
	last_update = chTimeNow();

	if ((width > 2500) || (width < 500)) {
		// sync pulse or glitch
		idx = 0;
		chSysUnlockFromIsr();
		return;
	}

	ppm[idx++] = width;

	if (idx >= 8) {
		idx = 0;
	}

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

	(void) arg;

	chRegSetThreadName(conf->name);

	icuStart(&ICUD4, &icucfg);
	icuEnable(&ICUD4);

	while (!chThdShouldTerminate()) {
		if (chTimeNow() - last_update > 100) {
			failsafe = true;
		} else {
			failsafe = false;
		}

		chThdSleepMilliseconds(100);
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
		values = ppm;
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

msg_t rcout_node(void * arg) {
	rcout_node_conf * conf = (rcout_node_conf *) arg;
	r2p::Node node(conf->name);

	(void) arg;

	chRegSetThreadName(conf->name);

	pwmStart(&PWMD1, &pwmcfg);
	pwmStart(&PWMD2, &pwmcfg);
	pwmStart(&PWMD3, &pwmcfg);

	while (!chThdShouldTerminate()) {
//		value += 10;
//
//		if (value > 2000) {
//			value = 1000;
//			chThdSleepMilliseconds(990);
//		}
		chThdSleepMilliseconds(10);
	}

	return CH_SUCCESS;
}

/*
 * Test node
 */
msg_t test_pub_node(void *arg) {
	r2p::Node node("test_pub");
	r2p::Publisher<r2p::String64Msg> pub;
	r2p::String64Msg * msgp;
	uint16_t * uuid = (uint16_t *)0x1FFFF7AC;

	(void) arg;
	chRegSetThreadName("test_pub");

	node.advertise(pub, "test");

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(1000);

	sprintf(msgp->data, "\n\n"MODULE_NAME" module [0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]", uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);
	pub.publish(msgp);
	chThdSleepMilliseconds(100);

	while (!chThdShouldTerminate()) {
		while (!pub.alloc(msgp)) chThdSleepMilliseconds(1000);
		sprintf(msgp->data, "\n %4d %4d %4d %4d %4d %4d %4d %4d]", ppm[0], ppm[1], ppm[2], ppm[3], ppm[4], ppm[5], ppm[6], ppm[7]);
		pub.publish(msgp);
		chThdSleepMilliseconds(500);
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

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, test_pub_node, NULL);

	rcin_node_conf rcin_conf = {"rcin_node", "rc"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, rcin_node, &rcin_conf);

	rcout_node_conf rcout_conf = {"rcout_node", "rc"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, rcout_node, &rcout_conf);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}

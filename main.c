#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <future>
#include <chrono>
#include <regex>
#include <fstream>
#include <mutex>
#include <fcntl.h> 
#include <errno.h> 
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <time.h>


#include <sys/types.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <gpiod.h>
#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>

#define LED_SIGNAL_LEVEL_PERIOD		2000
#define LED_NETWORK_SEARCH_PERIOD	100
#define MAX_CONNECTION_RETRY		30

#define RSSI_LOG_FILE "/tmp/rssi.txt"
#define MODEM_EVENT_PIPE "/tmp/modem_event"

std::string dev_model = "";

#define PORTA	0
#define PORTB	1
#define PORTC	2
#define PORTD	3
#define PORTE	4
#define PORTF	5
#define PORTG	6
#define PORTH	7
#define PORTI	8

#define GPIO_CREATE_PIN(PORT, PIN) ((PORT * 32) + PIN)

#define NR100_GSM_POWER_PIN		GPIO_CREATE_PIN(PORTE, 7)
#define NR100_SIM_SELECT_PIN		GPIO_CREATE_PIN(PORTG, 9)
#define NR100_SIM1_PRESENCE_PIN		GPIO_CREATE_PIN(PORTI, 4)
#define NR100_SIM2_PRESENCE_PIN		GPIO_CREATE_PIN(PORTG, 8)

#define NR200_GSM_POWER_PIN			40
#define NR200_SIM_SELECT_PIN		41
#define NR200_SIM_PRESENCE_PIN		42

#define ACTIVE_HIGH	1
#define ACTIVE_LOW	0
uint8_t gsm_pwr_logic = ACTIVE_LOW;

#define LED_SIM1_DIR "/sys/class/leds/green:psim"
#define LED_SIM2_DIR "/sys/class/leds/green:ssim"
#define LED_GPS_DIR "/sys/class/leds/green:gps"

#define CTRL_Z "\x1A"
#define CRLF "\r\n"
#define OK_CRLF "OK\r\n"
#define CRLF_OK_CRLF "\r\nOK\r\n"

typedef struct {
	std::string utc;
	std::string latitude;
	std::string longitude;
	std::string hdop;
	std::string altitude;
	std::string fix;
	std::string cog;
	std::string spkm;
	std::string spkn;
	std::string date;
	std::string nsat;
} gps_data_t;

typedef struct {
	bool init;
	bool init_gps;
	bool send_status;
} modem_flag_t;

typedef enum {
	LED_SIM1 = 0,
	LED_SIM2 = 1,
	LED_GPS = 2
} led_t;

const char *led_name[] = {
	[LED_SIM1] = "LED_SIM1",
	[LED_SIM2] = "LED_SIM2",
	[LED_GPS] = "LED_GPS"
};

typedef enum {
	SIM2 = 0,
	SIM1 = 1
} sim_t;

const char *sim_name[] = {
	[SIM2] = "SIM2",
	[SIM1] = "SIM1"
};

typedef enum {
	SIM_NOT_PRESENT = 0,
	SIM_PRESENT = 1
} sim_presence_t;

const char *sim_presence_name[] = {
	[SIM_NOT_PRESENT]  = "SIM_NOT_PRESENT",
	[SIM_PRESENT] = "SIM_PRESENT"
};

typedef enum {
	DEFAULT_SIM = 0,
	ACTIVE_SIM,
	PROTOCOL,
	DEVICE,
	PINCODE,
	APN,
	AUTH,
	NET_TYPE,
	PDP_TYPE,
	USERNAME,
	PASSWORD,
	PHONE_NUMBER,
	FAILOVER_CONDITION,
	FAILOVER_SWITCH_DELAY,
	SWITCHBACK_EN,
	SWITCHBACK_TIMEOUT,
	ENABLE,
	IMEI,
	NETWORK_STATUS,
	PLMN_MCC,
	PLMN_MNC,
	PLMN_DESCRIPTION,
	RSSI
} config_t;

int exec(const std::string &command);
int gsm_power_on(void);
int gsm_power_off(void);
int gps_power_on(void);
int gps_power_off(void);

#define PIPE_READ_END	0
#define PIPE_WRITE_END	1

struct gpiod_line *gsm_power_pin;
struct gpiod_line *sim_sel_pin;
struct gpiod_line *sim1_presence_pin;
struct gpiod_line *sim2_presence_pin;

static int
log(const char *func, const char *msg) {
	printf("[GSM]-%-34s ---> %s\n", func, msg);
	return 0;
}

static int
log(const std::string &func, const std::string &message) {
	return log(func.c_str(), message.c_str());
}

static int
rssi_log_init(void) {
	exec("/bin/touch /tmp/rssi.txt");
	return 0;
}

static int
rssi_log(const std::string &rssi) {
	FILE *fp = fopen("/tmp/rssi.txt", "a");
	if (fp == NULL)
		return errno;
    
	time_t raw_time;
	time(&raw_time);
	struct tm t;
	gmtime_r(&raw_time, &t);
	fprintf(fp, "[%4d/%02d/%02d]-[%02d:%02d:%02d] %s\n", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, rssi.c_str());
	fclose(fp);
	exec("if [ $(du -k /tmp/rssi.txt | cut -f1) -gt 64 ]; then sed -i 1,32d /tmp/rssi.txt; fi");
	return 0;
}

class cmd_t {
    private:
    std::string command;
    std::string std_out;
    std::string std_err;
    
    public:
    cmd_t(const std::string &cmd) {
		this->command = cmd;
	}
    
    std::string get_stdout(void) {
		return this->std_out;
	}
    
    std::string get_stderr(void) {
		return this->std_err;
	}
    
    int execute(void) {
		int fd_stdout[2] = {0, 0};
		int fd_stderr[2] = {0, 0};
		log("[EXEC]", "[" + this->command + "]");
        
		if (pipe(fd_stdout) != 0) {
			log("[EXEC]-[PIPE]", "[STDOUT]-[ERROR: " + std::to_string(errno) + "]");
			return -EPIPE;
		}
        
		if (pipe(fd_stderr) != 0) {
			log("[EXEC]-[PIPE]", "[STDERR]-[ERROR: " + std::to_string(errno) + "]");
			close(fd_stdout[PIPE_READ_END]);
			close(fd_stdout[PIPE_WRITE_END]);
			return -EPIPE;
		}
        
		pid_t pid = fork();
		if (pid == -1) {
			log("[EXEC]", "[FORK ERROR]");
			close(fd_stdout[PIPE_READ_END]);
			close(fd_stdout[PIPE_WRITE_END]);
			close(fd_stderr[PIPE_READ_END]);
			close(fd_stderr[PIPE_WRITE_END]);
			return errno;
		}
        
		if (pid == 0) { // child
			close(fd_stdout[PIPE_READ_END]);
			close(fd_stderr[PIPE_READ_END]);
			dup2(fd_stdout[PIPE_WRITE_END], STDOUT_FILENO);
			dup2(fd_stderr[PIPE_WRITE_END], STDERR_FILENO);
			execl("/bin/sh", "/bin/sh", "-c", this->command.c_str(), NULL);
			int err = errno;
			close(fd_stdout[PIPE_WRITE_END]);
			close(fd_stderr[PIPE_WRITE_END]);
			_exit(err);
		} else { // parent
			char buffer[128];
			close(fd_stdout[PIPE_WRITE_END]);
			close(fd_stderr[PIPE_WRITE_END]);
			int n;
			this->std_out.clear();
			while ((n = read(fd_stdout[PIPE_READ_END], buffer, (sizeof(buffer) / sizeof(char)) - 1)) > 0) {
				buffer[n] = '\0';
				this->std_out.append(buffer);
			}
            
			this->std_err.clear();
			while ((n = read(fd_stderr[PIPE_READ_END], buffer, (sizeof(buffer) / sizeof(char)) - 1)) > 0) {
				buffer[n] = '\0';
				this->std_err.append(buffer);
			}
            
			log("[EXEC]-[STDOUT]", "[" + this->std_out + "]");
			log("[EXEC]-[STDERR]", "[" + this->std_err + "]");
			close(fd_stdout[PIPE_READ_END]);
			close(fd_stderr[PIPE_READ_END]);
		}
        
		int wstatus;
		pid_t w = waitpid(pid, &wstatus, 0);
		if (w == -1) {
			log("[EXEC]-[CHILD]", "[ERROR: " + std::to_string(WEXITSTATUS(wstatus)) + "]");
			log("[EXEC]", "[EXIT MODEM APP DUE TO ERROR]");
			exit(EXIT_FAILURE);
		}
        
		return WEXITSTATUS(wstatus);
	}
    
    ~cmd_t() {
	}
};

int
exec(const std::string &command) {
	cmd_t cmd(command);
	return cmd.execute();
}

int
exec(const std::string &command, std::string *std_out, std::string *std_err) {
	cmd_t cmd(command);
	int ret = cmd.execute();
	*std_out = cmd.get_stdout();
	*std_err = cmd.get_stderr();
	return ret;
}

int
exec(const std::string &command, std::string *std_out) {
	cmd_t cmd(command);
	int ret = cmd.execute();
	*std_out = cmd.get_stdout();
	return ret;
}

static int
get_dev_model(std::string *dev_model) {
	return exec("cat /proc/device-tree/model", dev_model);
}

static int
nr100_gpio_init(void) {
	gsm_pwr_logic = ACTIVE_LOW;
	struct gpiod_chip *gpio_chip;
	gpio_chip = gpiod_chip_open_by_name("gpiochip0");
	if (!gpio_chip)
		goto io_err;
    
	// GSM_POWER_PIN init
	log("[GPIO]-[INIT]",  "[GSM POWER]-[PIN: " + std::to_string(NR100_GSM_POWER_PIN) + "]");
	gsm_power_pin = gpiod_chip_get_line(gpio_chip, NR100_GSM_POWER_PIN);
	if (!gsm_power_pin)
		goto io_err;
    
	if (gpiod_line_request_output(gsm_power_pin, "gsm_power", !gsm_pwr_logic) != 0)
		goto io_err;
    
	// SIM_SELECT_PIN init
	log("[GPIO]-[INIT]",  "[SIM SELECT]-[PIN: " + std::to_string(NR100_SIM_SELECT_PIN) + "]");
	sim_sel_pin = gpiod_chip_get_line(gpio_chip, NR100_SIM_SELECT_PIN);
	if (!sim_sel_pin)
		goto io_err;
    
	if (gpiod_line_request_output(sim_sel_pin, "gsm_sim_sel", 0) != 0)
		goto io_err;
    
	// SIM1_PRESENCE_PIN init
	log("[GPIO]-[INIT]",  "[SIM1 PRESENCE]-[PIN: " + std::to_string(NR100_SIM1_PRESENCE_PIN) + "]");
	sim1_presence_pin = gpiod_chip_get_line(gpio_chip, NR100_SIM1_PRESENCE_PIN);
	if (!sim1_presence_pin)
		goto io_err;
    
	if (gpiod_line_request_input(sim1_presence_pin, "gsm_sim1_presence") != 0)
		goto io_err;
    
	// SIM2_PRESENCE_PIN init
	log("[GPIO]-[INIT]",  "[SIM2 PRESENCE]-[PIN: " + std::to_string(NR100_SIM2_PRESENCE_PIN) + "]");
	sim2_presence_pin = gpiod_chip_get_line(gpio_chip, NR100_SIM2_PRESENCE_PIN);
	if (!sim2_presence_pin)
		goto io_err;
    
	if (gpiod_line_request_input(sim2_presence_pin, "gsm_sim2_presence") != 0)
		goto io_err;
    
	return 0;
    
    io_err:
	log("[GPIO]-[INIT]", "[ERROR]");
	return -EIO;
}

static int
nr200_gpio_init(void) {
	gsm_pwr_logic = ACTIVE_HIGH;
	struct gpiod_chip *gpio_chip;
	struct gpiod_chip *gpio_chip0;
	struct gpiod_chip *gpio_chip1;
	gpio_chip0 = gpiod_chip_open_by_name("gpiochip0");
	if (!gpio_chip0)
		goto io_err;
    
	gpio_chip1 = gpiod_chip_open_by_name("gpiochip1");
	if (!gpio_chip1)
		goto io_err;
    
	// GSM_POWER_PIN init
	log("[GPIO]-[INIT]",  "[GSM POWER]-[PIN: " + std::to_string(NR200_GSM_POWER_PIN) + "]");
	gpio_chip = (NR200_GSM_POWER_PIN / 32 == 0) ? gpio_chip0 : gpio_chip1;
	gsm_power_pin = gpiod_chip_get_line(gpio_chip, NR200_GSM_POWER_PIN % 32);
	if (!gsm_power_pin)
		goto io_err;
    
	if (gpiod_line_request_output(gsm_power_pin, "gsm_power", !gsm_pwr_logic) != 0)
		goto io_err;
    
	// SIM_SELECT_PIN init
	log("[GPIO]-[INIT]",  "[SIM SELECT]-[PIN: " + std::to_string(NR200_SIM_SELECT_PIN) + "]");
	gpio_chip = (NR200_SIM_SELECT_PIN / 32 == 0) ? gpio_chip0 : gpio_chip1;
	sim_sel_pin = gpiod_chip_get_line(gpio_chip, NR200_SIM_SELECT_PIN % 32);
	if (!sim_sel_pin)
		goto io_err;
    
	if (gpiod_line_request_output(sim_sel_pin, "gsm_sim_sel", 0) != 0)
		goto io_err;
    
	// SIM1_PRESENCE_PIN init
	log("[GPIO]-[INIT]",  "[SIM1 PRESENCE]-[PIN: " + std::to_string(NR200_SIM1_PRESENCE_PIN) + "]");
	gpio_chip = (NR200_SIM1_PRESENCE_PIN / 32 == 0) ? gpio_chip0 : gpio_chip1;
	sim1_presence_pin = gpiod_chip_get_line(gpio_chip, NR200_SIM1_PRESENCE_PIN % 32);
	if (!sim1_presence_pin)
		goto io_err;
    
	if (gpiod_line_request_input(sim1_presence_pin, "gsm_sim1_presence") != 0)
		goto io_err;
    
	// SIM2_PRESENCE_PIN init
	log("[GPIO]-[INIT]",  "[SIM2 PRESENCE]-[PIN: " + std::to_string(NR200_SIM2_PRESENCE_PIN) + "]");
	gpio_chip = (NR200_SIM2_PRESENCE_PIN / 32 == 0) ? gpio_chip0 : gpio_chip1;
	sim2_presence_pin = gpiod_chip_get_line(gpio_chip, NR200_SIM2_PRESENCE_PIN % 32);
	if (!sim2_presence_pin)
		goto io_err;
    
	if (gpiod_line_request_input(sim2_presence_pin, "gsm_sim2_presence") != 0)
		goto io_err;
    
	return 0;
    
    io_err:
	log("[GPIO]-[INIT]", "[ERROR]");
	return -EIO;
}

static int
gpio_init(void) {
	log("[GPIO]-[INIT]", "[START]");
	if (get_dev_model(&dev_model) != 0)
		return -ENODEV;
    
	log("[GPIO]-[HW]", "[" + dev_model + "]");
	if (dev_model == "SAA NR-200") {
		return nr200_gpio_init();
	} else if (dev_model == "") {
		return nr100_gpio_init();
	}
    
	return -ENODEV;
}

int
set_led_duty_cycle(led_t led, uint16_t period, uint8_t duty_cycle) {
	log("[LED]-[PERIOD: " + std::to_string(period) + "]-[DUTY: " + std::to_string(duty_cycle) + "]", std::string("[") + led_name[led] + "]");
    
	if (duty_cycle > 100) {
		log("[LED]", "[ERROR]-[DUTY CYCLE > 100]");
		return -EINVAL;
	}
	
	uint16_t delay_on = ((uint32_t)period * duty_cycle) / 100;
	uint16_t delay_off = period - delay_on;
    
	std::string led_dir;
	switch (led) {
        case LED_SIM1:
		led_dir = LED_SIM1_DIR;
		break;
        case LED_SIM2:
		led_dir = LED_SIM2_DIR;
		break;
        case LED_GPS:
		led_dir = LED_GPS_DIR;
		break;
        default:
		log("[LED]", "[ERROR]-[LED NAME]");
		return -EINVAL;
	}
    
	exec(std::string("echo timer > ") + led_dir + "/trigger");
	exec(std::string("echo \'") + std::to_string(delay_off) + "\' > " + led_dir + "/delay_off");
	exec(std::string("cat ") + led_dir + "/delay_off");
	exec(std::string("echo \'") + std::to_string(delay_on) + "\' > " + led_dir + "/delay_on");
	exec(std::string("cat ") + led_dir + "/delay_on");
	return 0;
}

static int
led_off(led_t led) {
	log("[LED]-[OFF]", std::string("[") + led_name[led] + "]");
	std::string led_dir;
	switch (led) {
        case LED_SIM1:
		led_dir = LED_SIM1_DIR;
		break;
        case LED_SIM2:
		led_dir = LED_SIM2_DIR;
		break;
        case LED_GPS:
		led_dir = LED_GPS_DIR;
		break;
        default:
		log("[LED]", "[ERROR]-[LED NAME]");
		return -EINVAL;
	}
     
	exec(std::string("echo none > ") + led_dir + "/trigger");
	exec(std::string("echo \'0\' > ") + led_dir + "/brightness");
	return 0;
}

static int
led_on(led_t led) {
	log("[LED]-[ON]", std::string("[") + led_name[led] + "]");
	std::string led_dir;
	switch (led) {
        case LED_SIM1:
		led_dir = LED_SIM1_DIR;
		break;
        case LED_SIM2:
		led_dir = LED_SIM2_DIR;
		break;
        case LED_GPS:
		led_dir = LED_GPS_DIR;
		break;
        default:
		log("[LED]", "[ERROR]-[LED NAME]");
		return -EINVAL;
	}
    
	exec(std::string("echo none > ") + led_dir + "/trigger");
	exec(std::string("echo \'255\' > ") + led_dir + "/brightness");
	return 0;
}

static int
led_init(void) {
	log("[LED]-[INIT]", "[START]");
	led_off(LED_SIM1);
	led_off(LED_SIM2);
	led_off(LED_GPS);
	return 0;
}

int
get_imei(std::string *imei) {
	int ret = exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-imei | sed \'s/[^0-9]//g\'", imei);
	if (ret != 0)
		return ret;
    
	if (imei->size())
		imei->pop_back(); // remove "\n"
    
	if (imei->empty()) {
		log("[GET IMEI]", "[ERROR]");
		return -EINVAL;
	}
    
	log("[IMEI]", "[" + *imei + "]");
	return 0;
}

int
save_imei(sim_t sim, const std::string *imei) {
	std::string tmp;
	std::string idx;
	if (sim == SIM1) {
		idx = "1";
	} else if (sim == SIM2) {
		idx = "2";
	} else {
		return -EINVAL;
	}
    
	exec("uci get simcard.@sim" + idx + "_status[0].imei", &tmp);
	if (tmp.size())
		tmp.pop_back(); // remove "\n"
    
	if (imei->compare(tmp) != 0) {
		log("[IMEI]-[SAVE]", "[" + *imei + "]");
		exec("uci set simcard.@sim" + idx + "_status[0].imei=\'" + *imei + "\'");
		exec("uci commit simcard");
	}
	return 0;
}

int
get_config(sim_t sim, config_t config, std::string *val) {
	std::string idx;
	if (sim == SIM1) {
		idx = "1";
	} else if (sim == SIM2) {
		idx = "2";
	} else {
		return -EINVAL;
	}
    
	if (val == nullptr)
		return -EINVAL;
    
	val->clear();
	int ret;
    
	switch (config) {
        case ENABLE:
		ret = exec("uci get simcard.@general[0].sim" + idx + "_en", val);
		break;
        case DEVICE:
		ret = exec("uci get simcard.@sim" + idx + "[0].device", val);
		break;
        case PINCODE:
		ret = exec("uci get simcard.@sim" + idx + "[0].pincode", val);
		break;
        case APN:
		ret = exec("uci get simcard.@sim" + idx + "[0].apn", val);
		break;
        case AUTH:
		ret = exec("uci get simcard.@sim" + idx + "[0].auth", val);
		break;
        case NET_TYPE:
		ret = exec("uci get simcard.@sim" + idx + "[0].nettype", val);
		break;
        case PDP_TYPE:
		ret = exec("uci get simcard.@sim" + idx + "[0].pdptype", val);
		break;
        case USERNAME:
		ret = exec("uci get simcard.@sim" + idx + "[0].username", val);
		break;
        case PASSWORD:
		ret = exec("uci get simcard.@sim" + idx + "[0].password", val);
		break;
        default:
		return -EINVAL;
	}
    
	if (ret != 0)
		return ret;
    
	if (val->size())
		val->pop_back(); // remove "\n"
    
	return 0;
}

int
get_config(config_t config, std::string *val) {
	if (val == nullptr)
		return -EINVAL;
    
	val->clear();
	int ret;
	switch (config) {
        case DEFAULT_SIM:
		ret = exec("uci get simcard.@general[0].defaultsim", val);
		break;
        case ACTIVE_SIM:
		ret = exec("uci get simcard.@general[0].active_sim", val);
		break;
        case PHONE_NUMBER:
		ret = exec("uci get simcard.@general[0].sms_number", val);
		break;
        case FAILOVER_CONDITION:
		ret = exec("uci get simcard.@failover[0].condition", val);
		break;
        case FAILOVER_SWITCH_DELAY:
		ret = exec("uci get simcard.@failover[0].delay", val);
		break;
        case SWITCHBACK_EN:
		ret = exec("uci get simcard.@switchback[0].enable", val);
		break;
        case SWITCHBACK_TIMEOUT:
		ret = exec("uci get simcard.@switchback[0].timeout", val);
		break;
        default:
		return -EINVAL;
	}
    
	if (ret != 0)
		return ret;
    
	if (val->size())
		val->pop_back(); // remove "\n"
    
	return 0;
}

int
set_sim_status(sim_t sim, config_t config, const std::string *val) {
	std::string idx;
	if (sim == SIM1) {
		idx = "1";
	} else if (sim == SIM2) {
		idx = "2";
	} else {
		return -EINVAL;
	}
    
	if (val == nullptr)
		return -EINVAL;
    
	int ret;
	switch (config) {
        case ENABLE:
		ret = exec("uci set simcard.@sim" + idx + "_status[0].enable=\'" + *val + "\'");
		break;
        case IMEI:
		ret = exec("uci set simcard.@sim" + idx + "_status[0].imei=\'" + *val + "\'");
		break;
        case NETWORK_STATUS:
		ret = exec("uci set simcard.@sim" + idx + "_status[0].net_status=\'" + *val + "\'");
		break;
        case PLMN_MCC:
		ret = exec("uci set simcard.@sim" + idx + "_status[0].plmn_mcc=\'" + *val + "\'");
		break;
        case PLMN_MNC:
		ret = exec("uci set simcard.@sim" + idx + "_status[0].plmn_mnc=\'" + *val + "\'");
		break;
        case PLMN_DESCRIPTION:
		ret = exec("uci set simcard.@sim" + idx + "_status[0].plmn_description=\'" + *val + "\'");
		break;
        case NET_TYPE:
		ret = exec("uci set simcard.@sim" + idx + "_status[0].nettype=\'" + *val + "\'");
		break;
        case RSSI:
		ret = exec("uci set simcard.@sim" + idx + "_status[0].rssi=\'" + *val + "\'");
		break;
        default:
		return -EINVAL;
	}
    
	if (ret != 0)
		return ret;
    
	exec("uci commit simcard");
	exec("sync");
	return 0;
}

int
set_sim_status(sim_t sim, config_t config, const char *str) {
	if (str == nullptr)
		return -EINVAL;
    
	std::string val = str;
	return set_sim_status(sim, config, &val);
}

int
clear_all_sim_status(sim_t sim) {
	set_sim_status(sim, NETWORK_STATUS, "");
	set_sim_status(sim, PLMN_MCC, "");
	set_sim_status(sim, PLMN_MNC, "");
	set_sim_status(sim, PLMN_DESCRIPTION, "");
	set_sim_status(sim, NET_TYPE, "");
	set_sim_status(sim, RSSI, "");
	return 0;
}

int
set_config(config_t config, const std::string *val) {
	if (val == nullptr)
		return -EINVAL;
    
	int ret;
	switch (config) {
        case ACTIVE_SIM:
		ret = exec("uci set simcard.@general[0].active_sim=\'" + *val + "\'");
		break;
        default:
		return -EINVAL;
	}
    
	if (ret != 0)
		return ret;
    
	exec("uci commit simcard");
	exec("sync");
	return 0;
}

int
set_config(config_t config, const char *val) {
	if (val == nullptr)
		return -EINVAL;
    
	std::string str = val;
	return set_config(config, &str);
}

int
set_modem_config(config_t config, const std::string *val) {
	if (val == nullptr)
		return -EINVAL;
    
	int ret;
	switch (config) {
        case NET_TYPE:
		ret = exec("uci set network.modem.modes=\'" + *val + "\'");
		break;
        case APN:
		ret = exec("uci set network.modem.apn=\'" + *val + "\'");
		break;
        case PINCODE:
		ret = exec("uci set network.modem.pincode=\'" + *val + "\'");
		break;
        case AUTH:
		ret = exec("uci set network.modem.auth=\'" + *val + "\'");
		break;
        case USERNAME:
		ret = exec("uci set network.modem.username=\'" + *val + "\'");
		break;
        case PASSWORD:
		ret = exec("uci set network.modem.password=\'" + *val + "\'");
		break;
        case PDP_TYPE:
		ret = exec("uci set network.modem.pdptype=\'" + *val + "\'");
		break;
        default:
		return -EINVAL;
	}
    
	exec("uci commit network");
	exec("sync");
	return ret;
}

int
set_modem_config(config_t config, const char *val) {
	if (val == nullptr)
		return -EINVAL;
    
	std::string str = val;
	return set_modem_config(config, &str);
}

int
get_sim_presence_status(sim_t sim, sim_presence_t *status) {
	int pin_level;
	switch (sim) {
        case SIM1:
		// read presense pin for sim 1
		pin_level = gpiod_line_get_value(sim1_presence_pin);
		break;
        case SIM2:
		// read presense pin for sim 2
		pin_level = gpiod_line_get_value(sim2_presence_pin);
		break;
        default:
		return -EINVAL;
	}
    
	if (pin_level)
		*status = SIM_NOT_PRESENT;
	else
		*status = SIM_PRESENT;
    
	return 0;
}

int
select_sim(sim_t sim) {
	log("[SIM SELECT]", std::string("[") + sim_name[sim] + "]");
	bool sim1_sel_level;
	if (dev_model == "SAA NR-200") {
		sim1_sel_level = 0;
	} else {
		sim1_sel_level = 1;
	}
    
	switch (sim) {
        case SIM1:
		gpiod_line_set_value(sim_sel_pin, sim1_sel_level);
		break;
        case SIM2:
		gpiod_line_set_value(sim_sel_pin, !sim1_sel_level);
		break;
        default:
		return -EINVAL;
	}
    
	std::string str = sim_name[sim];
	set_config(ACTIVE_SIM, &str);
    
	std::string val;
	if (get_config(sim, NET_TYPE, &val) != 0)
		val = "";
    
	set_modem_config(NET_TYPE, &val);
    
	if (get_config(sim, APN, &val) != 0)
		val = "";
    
	set_modem_config(APN, &val);
    
	if (get_config(sim, PINCODE, &val) != 0)
		val = "";
    
	set_modem_config(PINCODE, &val);
    
	if (get_config(sim, AUTH, &val) != 0)
		val = "none";
    
	if ((val != "none") || (val != "pap") || (val != "chap") || (val != "both"))
		val = "none";
    
	set_modem_config(AUTH, &val);
    
	if (val == "none") {
		set_modem_config(USERNAME, "");
		set_modem_config(PASSWORD, "");
	} else {
		if (get_config(sim, USERNAME, &val) != 0)
			val = "";
        
		set_modem_config(USERNAME, &val);
		if (get_config(sim, PASSWORD, &val) != 0)
			val = "";
        
		set_modem_config(PASSWORD, &val);
	}
    
	if (get_config(sim, PDP_TYPE, &val) != 0)
		val = "";
    
	set_modem_config(PDP_TYPE, &val);
	exec("/etc/init.d/network reload");
	return 0;
}

bool
file_exist(const char *file) {
	std::ifstream f(file);
	return f.good();
}

int
get_conection_status(bool *connection_state) {
	std::string std_out = "";
	exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-data-status", &std_out);
	if (std_out.find("\"connected\"") != std::string::npos) {
		*connection_state = true;
		return 0;
	} else if (std_out.find("\"disconnected\"") != std::string::npos) {
		*connection_state = false;
		return 0;
	}
    
	return -EIO;
}

int
get_rssi(int *rssi) {
	std::string std_out;
	if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-signal-info | sed 's/[\\t\" ,{}]*//g' | grep 'rssi\\|signal' | cut -f2 -d:", &std_out) != 0)
		return -EIO;
    
	if (sscanf(std_out.c_str(), "%d", rssi) != 1)
		return -ENODATA;
    
	return 0;
}

int
get_interface_ip(const std::string interface, std::string *ip) {
	if (exec("ifconfig " + interface + " | grep -Eo \'inet (addr:)\?([0-9]*\\.){3}[0-9]*\' | grep -Eo \'([0-9]{1,3}\\.){3}[0-9]{1,3}\'", ip) != 0)
		return -EIO;
    
	int a, b, c, d;
	if (sscanf(ip->c_str(),"%d.%d.%d.%d", &a, &b, &c, &d) == 4) {
		ip->pop_back(); // remove "\n"
		return 0;
	}
    
	return -EIO;
}

int
get_wwan_ip(std::string *ip) {
	return get_interface_ip("wwan0", ip);
}

int
get_lan_ip(std::string *ip) {
	return get_interface_ip("br-lan", ip);
}

int
wait_for_wwan_ip(uint8_t timeout) {
	uint8_t t = 0;
	std::string ip;
	do {
		if (get_wwan_ip(&ip) == 0) {
			log("[MODEM]", "[IP: " + ip + "]-[DELAY: " + std::to_string(t) + "SEC]");
			return 0;
		}
        
		t += 5;
		sleep(5);
	} while (t <= timeout);
    
	log("[MODEM]", "[IP: NOT FOUND]-[DELAY: " + std::to_string(timeout) + "SEC]");
	return -ETIME;
}

int
wait_for_network_connection(uint8_t timeout) {
	uint8_t t = 0;
	bool connection_state;
	do {
		if (get_conection_status(&connection_state) == 0) {
			if (connection_state == true) {
				log("[MODEM]", "[CONNECTED]-[DELAY: " + std::to_string(t) + "SEC]");
				return 0;
			}
		}
        
		t++;
		sleep(1);
	} while (t <= timeout);
    
	log("[MODEM]", "[DISCONNECTED]-[DELAY: " + std::to_string(t) + "SEC]");
	return -ETIME;
}

void
device_reboot(void) {
	log("[NR100]", "[REBOOT AFTER 120 SEC]");
	gps_power_off();
	gsm_power_off();
	exec("sync; sleep 5; reboot -d 15 &");
	sleep(180);
	exit(EXIT_FAILURE);
}

int
wait_for_cdc_wdm0(uint8_t timeout) {
	uint8_t t = 0;
	do {
		if (file_exist("/dev/cdc-wdm0")) {
			log("[MODEM]", "[FOUND: /dev/cdc-wdm0]-[DELAY: " + std::to_string(t) + "SEC]");
			return 0;
		}
        
		t++;
		sleep(1);
	} while (t <= timeout);
    
	log("[MODEM]", "[NOT FOUND: /dev/cdc-wdm0]-[DELAY: " + std::to_string(t) + "SEC]");
	return -ETIME;
}

int
set_interface_attribs(int fd, int speed) {
	struct termios tty;
	if (tcgetattr(fd, &tty) != 0) {
		log("[MODEM]-[AT PORT]", "[tcgetattr]-[ERROR: " + std::to_string(errno) + " (" + strerror(errno) + ")]");
		return -EIO;
	}
    
	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);
    
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
	tty.c_cflag |= CS8; // 8 bits per byte
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
	tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
    
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    
	tty.c_cc[VMIN]  = 0; // read doesn't block
	tty.c_cc[VTIME] = 10; // 1 second read timeout
    
	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		log("[MODEM]-[AT PORT]", "[tcsetattr]-[ERROR: " + std::to_string(errno) + " (" + strerror(errno) + ")]");
		return -EIO;
	}
    
	return 0;
}

class modem_tty_t {
    private:
	int fd = 0;
	static modem_tty_t *instance;
	modem_tty_t() {
	}
    public:
	static modem_tty_t* get_instance(void) {
		if (modem_tty_t::instance != nullptr)
			return modem_tty_t::instance;
        
		modem_tty_t::instance = new modem_tty_t();
		return modem_tty_t::instance;
	}
    
	int open_port(void) {
		std::string port;
		if (this->fd > 0) {
			log("[AT PORT]-[OPEN]", "[ALREADY OPENED]");
			return 0;
		} else {
			if (exec("ls $(readlink -f $(find /sys/devices | grep cdc-wdm0/device) | sed 's/:1.4$/:1.2/g') | grep ttyUSB", &port) != 0) {
				log("[AT PORT]", "[ERROR: ttyUSB NOT FOUND]");
				return -ENODEV;
			}
			port.insert(0, "/dev/");
			port.pop_back(); // remove "\n"
		}
        
		this->fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
		if (this->fd <= 0) {
			log("[AT PORT]-[OPEN]-[" + port + "]", "[ERROR: " + std::to_string(errno) + " (" + strerror(errno) + ")]");
			return -EIO;
		}
        
		if (set_interface_attribs(this->fd, B115200) != 0) { // set speed to 115,200 bps, 8n1 (no parity)
			this->close_port();
			return -EIO;
		}
        
		log("[AT PORT]-[OPEN]-[" + port + "]", "[OK]");
		return 0;
	}
    
	int close_port(void) {
		close(this->fd);
		this->fd = 0;
		log("[AT PORT]-[CLOSE]", "[OK]");
		return 0;
	}
    
	int exec_at_cmd(const std::string &cmd, std::string *res) {
		log("[AT PORT]-[CMD]", "[" + cmd + "]");
		if (this->fd <= 0) {
			log("[AT PORT]-[ERROR]", "[PORT NOT OPENED]");
			return -EIO;
		}
        
		if (res == nullptr)
			return -EINVAL;
        
		tcflush(this->fd, TCIOFLUSH);
		write(this->fd, cmd.c_str(), cmd.size());
		write(this->fd, CRLF, sizeof(CRLF));
		tcflush(this->fd, TCIOFLUSH);
		usleep((cmd.size() + 500) * 100);
		char buffer[128];
		res->clear();
		int n;
		while ((n = read(fd, buffer, (sizeof(buffer) / sizeof(char)) - 1)) > 0) {
			buffer[n] = '\0';
			res->append(buffer);
		}
        
		log("[AT PORT]-[RES]", std::string("[LEN: ") + std::to_string(res->size()) + "]-[" + *res + "]");
        
		if(res->size() < sizeof(OK_CRLF))
			return -EIO;
        
		if (res->compare(res->size() - sizeof(OK_CRLF) + 1, sizeof(OK_CRLF), OK_CRLF) != 0)
			return -EIO;
        
		return 0;
	}
    
	int exec_at_cmd(const std::string &cmd) {
		std::string res;
		return this->exec_at_cmd(cmd, &res);
	}
    
	~modem_tty_t() {
		this->close_port();
	}
};

modem_tty_t* modem_tty_t::instance = nullptr;

int
send_sms(const std::string &target, const std::string &message) {
	std::string res;
	if (modem_tty_t::get_instance()->open_port() != 0)
		return -EIO;
    
	modem_tty_t::get_instance()->exec_at_cmd("AT+CMGS=\"" + target + "\"", &res);
	if (strstr(res.c_str(), ">") == nullptr)
		return -EIO;
    
	log("[AT PORT]-[MESSAGE]", "[" + message + "]");
	modem_tty_t::get_instance()->exec_at_cmd(message + CTRL_Z, &res);
	log("[AT PORT]-[RES]", std::string("[LEN: ") + std::to_string(res.size()) + "]-[" + res + "]");
	return 0;
}

int
receive_sms(int *id, std::string *number, std::string *message) {
	if (modem_tty_t::get_instance()->open_port() != 0)
		return -EIO;
    
	std::string res;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CMGL=\"ALL\"", &res) != 0)
		return -EIO;
    
	char dummy_char[2];
	char phone_number[100];
	if (sscanf(res.c_str(), "\r\n+CMGL: %d,%*[\"A-Z ],\"%[+0-9]\"%1[,]", id, phone_number, dummy_char) == 3)
		*number = phone_number;
	else
		return -EIO;
    
	log("[SMS]-[NEW]", "[ID: " + std::to_string(*id) + "]-[No: " + *number + "]");
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CMGR=" + std::to_string(*id), &res) != 0)
		return -EIO;
    
	if (sscanf(res.c_str(), "\r\n+CMGR: %*[^\r\n]%*1[\r]%1[\n]", dummy_char) == 1) {
		res.erase(0, res.find(CRLF) + sizeof(CRLF) - 1); // erase heading CRLF
		res.erase(0, res.find(CRLF) + sizeof(CRLF) - 1); // erare first line
	} else {
		return -EIO;
	}
    
	if (res.compare(res.size() - sizeof(CRLF_OK_CRLF) + 1, sizeof(CRLF_OK_CRLF), CRLF_OK_CRLF) == 0) {
		res.erase(res.size() - sizeof(CRLF_OK_CRLF) + 1);
		res.erase(res.size() - sizeof(CRLF) + 1);
	} else {
		return -EIO;
	}
    
	*message = res;
	return 0;
}

int
delete_sms(int id) {
	return modem_tty_t::get_instance()->exec_at_cmd("AT+CMGD=" + std::to_string(id));
}

int
modem_init(void) {
	if (modem_tty_t::get_instance()->open_port() != 0)
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("ATE0") != 0) // echo off
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("ATV1") != 0) // TA response format
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CPMS=\"SM\",\"SM\",\"SM\"") != 0) // prefered sms storage
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+IFC=0,0") != 0) // flow control: none
		return -EIO;
	//if (modem_tty_t::get_instance()->exec_at_cmd("AT+CNMI=1,2,,1") != 0) // sms event reporting configuration
	//	return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CMGF=1") != 0) // message format: text mode
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CSMP=17,167,0,0") != 0) // set sms text mode parameters
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CLIP=1") != 0) // set caller id on
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CRC=0") != 0) // set incoming call indication off
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CMEE=2") != 0) // error message format
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+QURCCFG=\"urcport\",\"usbat\"") != 0) // configure urc indication option
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+QINDCFG=\"all\",0,1") != 0) // urc indication configuration
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+CSCS=\"IRA\"") != 0) // set character set
		return -EIO;
    
	return 0;
}

int
gps_power_off(void) {
	led_off(LED_GPS);
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+QGPSEND") != 0) // Turn OFF GNSS
		return -EIO;
    
	return 0;
}

int
gps_power_on(void) {
	led_off(LED_GPS);
	if (modem_tty_t::get_instance()->open_port() != 0)
		return -EIO;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+QGPS=1") != 0) { // Turn ON GNSS
		gps_power_off();
		return -EIO;
	}
    
	set_led_duty_cycle(LED_GPS, LED_NETWORK_SEARCH_PERIOD, 50);
	return 0;
}

int
gps_get_location_info(gps_data_t *gps_data) {
	if (modem_tty_t::get_instance()->open_port() != 0)
		return -EIO;
	std::string res;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+QGPS?", &res) != 0) {
		gps_power_on();
		return -EIO;
	} else {
		char gps_status[2];
		if ((sscanf(res.c_str(), "\r\n+QGPS: %1[0-9]", gps_status) != 1) || (gps_status[0] != '1')) {
			gps_power_on();
			return -EIO;
		}
	}
    
	std::string loc;
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+QGPSLOC=2", &loc) != 0) {
		set_led_duty_cycle(LED_GPS, LED_NETWORK_SEARCH_PERIOD, 50);
		return -EIO;
	}
    
	char utc[11], latitude[11], longitude[11], hdop[11], altitude[11], fix[11], cog[11], spkm[11], spkn[11], date[11], nsat[11];
	if (sscanf(loc.c_str(), "\r\n+QGPSLOC: %10[0-9.],%10[0-9.],%10[0-9.],%10[0-9.],%10[0-9.],%10[0-9],%10[0-9.],%10[0-9.],%10[0-9.],%10[0-9],%10[0-9]",
               utc, latitude, longitude, hdop, altitude, fix, cog, spkm, spkn, date, nsat) != 11) {
		set_led_duty_cycle(LED_GPS, LED_NETWORK_SEARCH_PERIOD, 50);
		return -EIO;
	}
    
	*gps_data = {
		.utc = utc,
		.latitude = latitude,
		.longitude = longitude,
		.hdop = hdop,
		.altitude = altitude,
		.fix = fix,
		.cog = cog,
		.spkm = spkm,
		.spkn = spkn,
		.date = date,
		.nsat = nsat
	};
    
	led_on(LED_GPS);
	return 0;
}

int
get_network_type(std::string *val) {
	if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-signal-info | sed 's/[\\t\" ,{}]*//g' | grep 'type' | cut -f2 -d:", val) != 0)
		return -EIO;
    
	if (val->size())
		val->pop_back(); // remove "\n"
    
	return 0;
}

int
get_plmn(std::string *mcc, std::string *mnc, std::string *description) {
	if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-serving-system | grep plmn_mcc | cut -f2 -d: | sed 's/ //g' | sed 's/,$//g'", mcc) != 0)
		return -EIO;
    
	if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-serving-system | grep plmn_mnc | cut -f2 -d: | sed 's/ //g' | sed 's/,$//g'", mnc) != 0)
		return -EIO;
    
	if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-serving-system | grep plmn_description | cut -f2 -d: | sed 's/ //g' | sed 's/,$//g'", description) != 0)
		return -EIO;
    
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+QSPN", description) != 0)
		return -EIO;
    
	if (description->compare(description->size() - sizeof(CRLF_OK_CRLF) + 1, sizeof(CRLF_OK_CRLF), CRLF_OK_CRLF) == 0) {
		description->erase(description->size() - sizeof(CRLF_OK_CRLF) + 1);
		description->erase(description->size() - sizeof(CRLF) + 1);
	} else {
		return -EIO;
	}
    
	exec("echo '" + *description + "' | cut -f2 -d: | cut -f1 -d, | sed 's/^ //g'", description);
    
	if (mcc->size())
		mcc->pop_back(); // remove "\n"
    
	if (mnc->size())
		mnc->pop_back(); // remove "\n"
    
	if (description->size())
		description->pop_back(); // remove "\n"
    
	return 0;
}

int
get_plmn_mcc(std::string *mcc) {
	if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-serving-system | grep plmn_mcc | cut -f2 -d: | sed 's/ //g' | sed 's/,$//g'", mcc) != 0)
		return -EIO;
    
	if (mcc->size())
		mcc->pop_back(); // remove "\n"
    
	return 0;
}

int
get_plmn_mnc(std::string *mnc) {
	if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-serving-system | grep plmn_mnc | cut -f2 -d: | sed 's/ //g' | sed 's/,$//g'", mnc) != 0)
		return -EIO;
    
	if (mnc->size())
		mnc->pop_back(); // remove "\n"
    
	return 0;
}

int
get_plmn_description(std::string *description) {
	if (modem_tty_t::get_instance()->exec_at_cmd("AT+QSPN", description) != 0)
		return -EIO;
    
	if (description->compare(description->size() - sizeof(CRLF_OK_CRLF) + 1, sizeof(CRLF_OK_CRLF), CRLF_OK_CRLF) == 0) {
		description->erase(description->size() - sizeof(CRLF_OK_CRLF) + 1);
		description->erase(description->size() - sizeof(CRLF) + 1);
	} else {
		return -EIO;
	}
    
	exec("echo '" + *description + "' | cut -f2 -d: | cut -f1 -d, | sed 's/^ //g'", description);
    
	if (description->size())
		description->pop_back(); // remove "\n"
    
	return 0;
}

int
modem_send_status(std::string &phone_number) {
	std::string str;
	std::string msg;
    //	if (get_config(SOFTWARE_VERSION, &str) != 0)
    //		return -EIO;
    
	exec("cat /etc/os-release | grep VERSION_ID | cut -f2 -d= | sed 's/\"//g'", &str);
	msg += "nr100: " + str + "\n";
	struct sysinfo sinfo;
	if ((sysinfo(&sinfo) != 0) || (exec("date -d@" + std::to_string(sinfo.uptime) + " -u +%H:%M:%S", &str) != 0))
		return -EIO;
    
	msg += "uptime: " + std::to_string(sinfo.uptime / ((long)60 * 60 * 24)) + " day, " + str;
	if (get_lan_ip(&str) != 0)
		return -EIO;
    
	msg += "lan: " + str + "\n";
	if (get_wwan_ip(&str) != 0)
		return -EIO;
    
	msg += "wwan: " + str + "\n";
	if (get_config(ACTIVE_SIM, &str) != 0)
		return -EIO;
    
	if (str == sim_name[SIM1])
		msg += "sim1: ";
	else if (str == sim_name[SIM2])
		msg += "sim2: ";
	else
		return -EIO;
    
	if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-signal-info | sed 's/[\\t\" ,{}]*//g' | grep 'type' | cut -f2 -d:", &str) != 0)
		return -EIO;
    
	if (str.size())
		str.pop_back(); // remove "\n"
    
	if (str.size() < 3)
		return -EIO;
    
	msg += str + ", ";
	int rssi;
	if (get_rssi(&rssi) != 0)
		return -EIO;
    
	msg += std::to_string(rssi) + " dBm\n";
    
	if (exec("/usr/sbin/get_digital_in", &str) != 0)
		return -EIO;
    
	if (str.size())
		str.pop_back(); // remove "\n"
    
	msg += "input: " + str;
    
	return send_sms(phone_number, msg);
}

int
create_modem_event_pipe(void) {
	return mknod(MODEM_EVENT_PIPE, S_IFIFO | 0660, 0); // create pipe
}

int
get_event(std::string *event) {
	int fd;
	char buffer[128];
	fd = open(MODEM_EVENT_PIPE, O_RDONLY | O_NONBLOCK, 0); // open pipe
	if (fd <= 0)
		return -EIO;
    
	event->clear();
	int n;
	while ((n = read(fd, buffer, (sizeof(buffer) / sizeof(char)) - 1)) > 0) {
		buffer[n] = '\0';
		event->append(buffer);
	}
    
	close(fd);
	if (event->empty()) {
		log("[EVENT]", "[NO EVENT]");
		return -ENODATA;
	}
    
	log("[EVENT]", std::string("[LEN: ") + std::to_string(event->size()) + "]-[" + *event + "]");
	return 0;
}

int
show_rssi(sim_t sim, int rssi) {
	uint8_t signal_level;
	if (rssi > -65) {
		signal_level = 90;
	} else if (rssi <= -65 && rssi > -75) {
		signal_level = 75;
	} else if (rssi <= -75 && rssi> -85) {
		signal_level = 50;
	} else {
		signal_level = 10;
	}
    
	if (sim == SIM1) {
		led_off(LED_SIM2);
		set_led_duty_cycle(LED_SIM1, LED_SIGNAL_LEVEL_PERIOD, signal_level);
	} else if (sim == SIM2) {
		led_off(LED_SIM1);
		set_led_duty_cycle(LED_SIM2, LED_SIGNAL_LEVEL_PERIOD, signal_level);
	} else
		return -EINVAL;
    
	return 0;
}

// I suspect the PWM generating issue comes from this as a legacy from NR100
/* 
int
gsm_power_on(void) {
	log("[GSM POWER]", "[ON]");
	for (int i = 0; i < 35; i++) {
		usleep(50);
		for (int j = 0; j < 10; j++) {
			gpiod_line_set_value(gsm_power_pin, gsm_pwr_logic);
		}
		gpiod_line_set_value(gsm_power_pin, !gsm_pwr_logic);
	}
    
	gpiod_line_set_value(gsm_power_pin, gsm_pwr_logic);
	return 0;
}
*/

int 
gsm_power_on(void)
{
	if(dev_model != "SAA NR-200")
	{
		log("[GSM] POWER", "[ON]");
		for (int i = 0; i < 35; i++)
		{
			usleep(50);
			for (int j = 0; j < 10; j++)
			{
				gpiod_line_set_value(gsm_power_pin, gsm_pwr_logic);
			}
			gpiod_line_set_value(gsm_power_pin, !gsm_pwr_logic);
		}
		
		gpiod_line_set_value(gsm_power_pin, gsm_pwr_logic);
		return 0;
	}
	else
	{
	log("[GSM POWER]", "[ON]");

	gpiod_line_set_value(gsm_power_pin, gsm_pwr_logic);
	return 0;
	}
}

int
gsm_power_off(void) {
	log("[GSM POWER]", "[OFF]");
	if (gpiod_line_get_value(gsm_power_pin) != gsm_pwr_logic)
		return 0;
    
	modem_tty_t::get_instance()->exec_at_cmd("AT+QPOWD=1");
	modem_tty_t::get_instance()->close_port();
	sleep(65);
	gpiod_line_set_value(gsm_power_pin, !gsm_pwr_logic);
	return 0;
}

int
main(int argc, char *argv[]) {
	log("[SERVICE]", "[START]");
	std::string event;
	modem_flag_t modem_flag;
	rssi_log_init();
	led_init();
	sleep(1);
	create_modem_event_pipe();
	if (gpio_init())
		device_reboot();
    
	std::string str;
	sim_t current_sim = SIM1;
	if (get_config(DEFAULT_SIM, &str) == 0) {
		if (str == "sim1") {
			current_sim = SIM1;
		} else if (str == "sim2") {
			current_sim = SIM2;
		} else {
			current_sim = SIM1;
			str = "sim1";
			set_config(DEFAULT_SIM, &str);
		}
	}
    
	bool sim1_en = true;
	bool sim2_en = true;
	bool connection = false;
	uint8_t connection_error = 0;
    
	if (get_config(SIM1, ENABLE, &str) == 0) {
		if (str == "0")
			sim1_en = false;
	}
	set_sim_status(SIM1, ENABLE, (sim1_en == true) ? "YES" : "NO");
    
	if (get_config(SIM2, ENABLE, &str) == 0) {
		if (str == "0")
			sim2_en = false;
	}
	set_sim_status(SIM2, ENABLE, (sim2_en == true) ? "YES" : "NO");
    
	while (1) {
		uint8_t connection_retry_counter = 0;
		while (connection == false) {
			connection_retry_counter++;
			if (connection_retry_counter <= MAX_CONNECTION_RETRY) {
				log("[CONNECTION]", "[ATTEMPT: " + std::to_string(connection_retry_counter) + "]");
			} else {
				log("[CONNECTION]", "[ERROR]-[RETRY COUNT: " + std::to_string(MAX_CONNECTION_RETRY) + "]");
				device_reboot();
			}
            
			clear_all_sim_status(SIM1);
			clear_all_sim_status(SIM2);
			led_off(LED_SIM1);
			led_off(LED_SIM2);
			gps_power_off();
			gsm_power_off();
			sleep(5);
			exec("kill -9 `pgrep qmi`");
			if ((current_sim == SIM1) && (sim1_en == false )) {
				current_sim = SIM2;
				continue;
			}
            
			if ((current_sim == SIM2) && (sim2_en == false)) {
				current_sim = SIM1;
				continue;
			}
            
			if (current_sim == SIM1) {
				set_led_duty_cycle(LED_SIM1, LED_NETWORK_SEARCH_PERIOD, 50);
			} else {
				set_led_duty_cycle(LED_SIM2, LED_NETWORK_SEARCH_PERIOD, 50);
			}
			
			sim_presence_t sim_presence;
			get_sim_presence_status(current_sim, &sim_presence);
			if (sim_presence == SIM_NOT_PRESENT) {
				log(std::string("[") + sim_name[current_sim] + "]", "[NOT PRESENT]");
				current_sim = (current_sim == SIM1) ? SIM2 : SIM1;
				sleep(5);
				continue;
			}
            
			select_sim(current_sim);
			gsm_power_on();
			if (wait_for_cdc_wdm0(30) != 0) {
				current_sim = (current_sim == SIM1) ? SIM2 : SIM1;
				continue;
			}
            
            
			if (wait_for_wwan_ip(120) == 0) {
				log("[EXEC]", "[/etc/init.d/ipsec restart]");
				system("/etc/init.d/ipsec restart");
				connection_error = 0;
				connection = true;
				modem_flag.init = true;
				modem_flag.init_gps = true;
				modem_flag.send_status = true;
			} else {
				current_sim = (current_sim == SIM1) ? SIM2 : SIM1;
			}
		}
        
		std::string nettype;
		if (get_network_type(&nettype) == 0) {
			set_sim_status(current_sim, NET_TYPE, &nettype);
		}
        
		std::string network_status;
		if (exec("timeout -s KILL 1s uqmi -d /dev/cdc-wdm0 --get-data-status | sed 's/\"//g'", &network_status) == 0) {
			set_sim_status(current_sim, NETWORK_STATUS, &network_status);
		}
        
		std::string plmn_mcc, plmn_mnc, plmn_desc;
		if (get_plmn_mcc(&plmn_mcc) == 0)
			set_sim_status(current_sim, PLMN_MCC, &plmn_mcc);
        
		if (get_plmn_mnc(&plmn_mnc) == 0)
			set_sim_status(current_sim, PLMN_MNC, &plmn_mnc);
        
		if (get_plmn_description(&plmn_desc) == 0)
			set_sim_status(current_sim, PLMN_DESCRIPTION, &plmn_desc);
        
		std::string imei;
		if (get_imei(&imei) == 0)
			save_imei(current_sim, &imei);
        
		int rssi;
		if (get_rssi(&rssi) == 0) {
			show_rssi(current_sim, rssi);
			std::string rssi_str = std::to_string(rssi) + " dBm";
			log(std::string("[SIM: ") + sim_name[current_sim] + "]", "[RSSI: " + rssi_str + "]");
			rssi_log(std::string("[SIM: ") + sim_name[current_sim] + "] [RSSI: " + rssi_str + "]");
			set_sim_status(current_sim, RSSI, &rssi_str);
		}
        
		for (int i = 0; i < 12; i++) {
			sleep(5);
            
			bool connection_state;
			std::string wwan_ip;
			if ((get_wwan_ip(&wwan_ip) == 0) && (get_conection_status(&connection_state) == 0)) {
				if (connection_state == true) {
					if (connection_error > 0)
						connection_error--;
				} else {
					log(std::string("[") + sim_name[current_sim] + "]", "[CONNECTION FAILED]");
					if (++connection_error > 30) {
						connection = false;
						break;
					}
				}
			} else if (++connection_error > 30) {
				connection = false;
				break;
			}
            
			sim_presence_t sim_presence;
			get_sim_presence_status(current_sim, &sim_presence);
			if (sim_presence == SIM_NOT_PRESENT) {
				log(std::string("[") + sim_name[current_sim] + "]", "[NOT PRESENT]");
				current_sim = (current_sim == SIM1) ? SIM2 : SIM1;
				connection = false;
				break;
			}
            
			if (modem_flag.init) {
				if (modem_init() == 0)
					modem_flag.init = false;
			}
            
			if (modem_flag.init == false) {
				if (modem_flag.init_gps == true) {
					if (gps_power_on() == 0)
						modem_flag.init_gps = false;
				} else {
					gps_data_t loc;
					if (gps_get_location_info(&loc) == 0) {
						std::string gps_log;
						gps_log += "date: " + loc.date + "\n";
						gps_log = "utc: " + loc.utc + "\n";
						gps_log += "latitude: " + loc.latitude + "\n";
						gps_log += "longitude: " + loc.longitude + "\n";
						gps_log += "altitude: " + loc.altitude + "\n";
						gps_log += "hdop: " + loc.hdop + "\n";
						gps_log += "fix: " + loc.fix + "\n";
						gps_log += "cog: " + loc.cog + "\n";
						gps_log += "spkm: " + loc.spkm + "\n";
						gps_log += "spkn: " + loc.spkn + "\n";
						gps_log += "nsat: " + loc.nsat + "\n";
                        
						exec("echo '" + gps_log + "' > /tmp/gps_info");
					} else {
						exec("echo 'not fix now' > /tmp/gps_info");
					}
				}
                
				if (modem_flag.send_status) {
					std::string phone_number;
					if (get_config(PHONE_NUMBER, &phone_number) == 0) {
						if (modem_send_status(phone_number) == 0)
							modem_flag.send_status = false;
					}
				}
			}
            
			int id;
			std::string number, text;
			if (receive_sms(&id, &number, &text) == 0) {
				log("[SMS]-[ID: " + std::to_string(id) + "]-[No: " + number + "]", "[" + text + "]");
				if (delete_sms(id) == 0)
					log("[SMS]-[ID: " + std::to_string(id) + "]", "[OK]");
				else
					log("[SMS]-[ID: " + std::to_string(id) + "]", "[ERROR]");
                
				std::string valid_phone_number;
				if (get_config(PHONE_NUMBER, &valid_phone_number) == 0 && valid_phone_number.compare(number) == 0) {
					if (text.compare("status") == 0) {
						modem_flag.send_status = true;
					} else if (text.compare("reboot") == 0) {
						send_sms(number, "nr100 will reboot in 2 minutes");
						sleep(5);
						device_reboot();
					}
				}
			}
            
			if (get_event(&event) == 0) {
				std::string valid_phone_number;
				if (get_config(PHONE_NUMBER, &valid_phone_number) == 0)
					send_sms(valid_phone_number, event);
			}
		}
	};
    
	exit(EXIT_FAILURE);
}


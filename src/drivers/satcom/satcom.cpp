#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>

#include <nuttx/config.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/param/param.h>

#include "drivers/drv_satcom.h"
#include "satcom.h"

#ifdef ERROR
#undef ERROR
#endif
static const int ERROR = -1;

satcom *satcom::instance;
int satcom::task_handle;

int satcom::param_timeout_s;
int satcom::param_read_interval_s;

satcom::satcom():
	CDev("satcom", SATCOM_DEVICE_PATH)
{}

int satcom::start(int argc, char *argv[])
{
	warnx("starting");

	if (satcom::instance != nullptr) {
		warnx("already started");
		return ERROR;
	}

	satcom::instance = new satcom();

	satcom::task_handle = px4_task_spawn_cmd("satcom", SCHED_DEFAULT,
			      SCHED_PRIORITY_SLOW_DRIVER, 2048, (main_t)&satcom::main_loop_helper, argv);

	return OK;
}

int satcom::stop()
{
	if (satcom::instance == nullptr) {
		warnx("not started");
		return ERROR;
	}

	warnx("stopping...");

	satcom::instance->task_should_exit = true;

	// give it enough time to stop
	param_timeout_s = 10;

	for (int i = 0; (i < param_timeout_s + 1) && (satcom::task_handle != -1); i++) {
		sleep(1);
	}

	// well, kill it anyway, though this may crash
	if (satcom::task_handle != -1) {
		warnx("killing task forcefully");

		::close(satcom::instance->uart_fd);
		task_delete(satcom::task_handle);
		satcom::task_handle = -1;
		delete satcom::instance;
		satcom::instance = nullptr;
	}

	return OK;
}

void satcom::status()
{
	if (satcom::instance == nullptr) {
		warnx("not started");
		return;

	}

	warnx("started");
	warnx("state %d", instance->state);

	warnx("TX: S: %d E: %d F: %d", instance->tx_buf_start_idx, instance->tx_buf_end_idx, instance->tx_buf_free);
}

void satcom::test(int argc, char *argv[])
{
	if (instance == nullptr) {
		warnx("not started");
		return;
	}

	if (instance->state != SATCOM_STATE_STANDBY || instance->test_pending) {
		warnx("test already running");
		return;
	}

	if (argc > 2) {
		strcpy(instance->test_command, argv[2]);

	} else {
		instance->test_command[0] = 0;
	}

	instance->schedule_test();
}

void satcom::main_loop_helper(int argc, char *argv[])
{
	// start the main loop and stay in it
	satcom::instance->main_loop(argc, argv);

	// tear down everything after the main loop exits
	::close(satcom::instance->uart_fd);
	satcom::task_handle = -1;
	delete satcom::instance;
	satcom::instance = nullptr;

	warnx("stopped");
}

void satcom::main_loop(int argc, char *argv[])
{
	pthread_mutex_init(&tx_buf_mutex, NULL);

	int arg_i = 3;
	int arg_uart_name = 0;

	while (arg_i < argc) {
		if (!strcmp(argv[arg_i], "-d")) {
			arg_i++;
			arg_uart_name = arg_i;

		} else if (!strcmp(argv[arg_i], "-v")) {
			warnx("verbose mode ON");
			verbose = true;
		}

		arg_i++;
	}

	if (arg_uart_name == 0) {
		warnx("no satcom modem UART port provided!");
		task_should_exit = true;
		return;
	}

	if (open_uart(argv[arg_uart_name]) != SATCOM_UART_OK) {
		warnx("failed to open UART port!");
		task_should_exit = true;
		return;
	}

	param_t param_pointer;

	param_pointer = param_find("SATCOM_TIMEOUT");
	param_get(param_pointer, &param_timeout_s);

	if (param_timeout_s == -1) {
		param_timeout_s = 30;
	}

	param_pointer = param_find("SATCOM_READINT");
	param_get(param_pointer, &param_read_interval_s);

	if (param_read_interval_s == -1) {
		param_read_interval_s = 10;
	}

	if (verbose) { warnx("timeout %d read interval %d", param_timeout_s, param_read_interval_s); }

	while (!task_should_exit) {
		switch (state) {
		case SATCOM_STATE_STANDBY:
			standby_loop();
			break;

		case SATCOM_STATE_CSQ:
			csq_loop();
			break;

		case SATCOM_STATE_SBDSESSION:
			sbdsession_loop();
			break;

		case SATCOM_STATE_TEST:
			test_loop();
			break;
		}

		if (new_state != state) {
			if(verbose) { warnx("SWITCHING STATE FROM %d TO %d", state, new_state); }
			state = new_state;
		} else {
			usleep(100000);	// 100ms
		}


//		bool write_pending = tx_msg_count > 0;
//		// only try to get a new msg if the previous was read by the upper layer
//		bool read_pending = (rx_msg_read_idx == rx_msg_len) &&
//				    (hrt_absolute_time() - last_read_time > (uint64_t)param_read_interval_s * 1000000);
//
//		if (!(write_pending || read_pending || test_procedure_pending)) {
//			continue;
//		}

//		if(update_signal_quality() != SATCOM_OK)
//		{
//			warnx("signal strength fail");
//			continue;
//		}

//		test_procedure_pending = false;

		//warnx("sig %d net %d", signal_strength, network_status);

//		if(signal_strength >= 1 && network_status == 1)
//		{
//			if(write_pending)
//			{
//				//warnx("writing...");
//				size_t len = tx_msg_len[0];
//
//				satcom_status ret = send_tx_buf(tx_buf, len, param_timeout_s * 1000);
//
//				if(ret == SATCOM_OK)
//				{
//					pthread_mutex_lock(&tx_buf_mutex);
//
//					memmove(tx_buf, tx_buf + len, SATCOM_TX_BUF_LEN - len - tx_buf_free);
//					memmove(tx_msg_len, tx_msg_len + 1, 4 * (SATCOM_MAX_TX_MSG - 1));
//					tx_buf_free += len;
//
//					tx_msg_count--;
//
//					pthread_mutex_unlock(&tx_buf_mutex);
//					//warnx("... writing done");
//				}
//				else
//				{
//					//warnx("... write failed");
//				}
//			}

//			if(read_pending && !task_should_exit)
//			{
//				//warnx("reading...");
//
//				satcom_status ret = get_msg(param_timeout_s * 1000);
//
//				if(ret == SATCOM_OK)
//				{
//					last_read_time = hrt_absolute_time();
//
//					rx_msg_len = 0;					// in case poll comes just now
//					rx_msg_read_idx = 0;
//					rx_msg_len = rx_msg.data_len;
//
//					poll_notify(POLLIN);
//					//warnx("... reading done");
//				}
//				else if(ret == SATCOM_NO_MSG)
//				{
//					last_read_time = hrt_absolute_time();
//					//warnx("... reading done, no msg");
//				}
//				else
//				{
//					//warnx("... read failed");
//				}
//			}
//		}
	}
}

void satcom::standby_loop(void)
{
	if (test_pending) {
		test_pending = false;

		if (!strcmp(test_command, "csq")) {
			csq_pending = true;
		} else if (!strcmp(test_command, "send")) {
			write(0, "kreczmer", 8);
			return;
		} else {
			time_counter = hrt_absolute_time();
			start_test();
		}
	}

	if(tx_buf_free != SATCOM_TX_BUF_LEN) {
		send_tx_buf();
	}

	if(tx_pending || rx_pending) {
		if(1) { //if(signal_quality > 0) {
			start_sbd_session();
			return;
		} else {
			csq_pending = true;
		}
	}

	if(csq_pending) {
		start_csq();
		return;
	}

	int res = read_at();

	if(res == SATCOM_RESULT_SBDRING) {
		// got a ring alert, MT message waiting for download
		ring_pending = true;
		rx_pending = true;
	}
}

void satcom::csq_loop(void)
{
	int res = read_at();

	if (res == SATCOM_RESULT_NA) {
		return;
	}

	if(res != SATCOM_RESULT_OK){
		if (verbose){ warnx("UPDATE SIGNAL QUALITY: ERROR"); }
		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	if (strncmp((const char *)rx_buf, "+CSQ:", 5)) {
		if (verbose) { warnx("UPDATE SIGNAL QUALITY: WRONG ANSWER:"); }
		if (verbose) { warnx("%s", rx_buf); }
		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	signal_quality = rx_buf[5] - 48;
	csq_pending = false;

	if (verbose) { warnx("SIGNAL QUALITY: %d", signal_quality); }

	new_state = SATCOM_STATE_STANDBY;
}

void satcom::sbdsession_loop(void)
{
	int res = read_at();

	if (res == SATCOM_RESULT_NA) {
		return;
	}

	if(res != SATCOM_RESULT_OK){
		if (verbose){ warnx("SBD SESSION: ERROR"); }
		if (verbose){ warnx("SBD SESSION: RESULT %d", res); }
		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	if (strncmp((const char*)rx_buf, "+SBDIX:", 7)){
		if (verbose) { warnx("SBD SESSION: WRONG ANSWER:"); }
		if (verbose) { warnx("%s", rx_buf); }
		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	int mo_status, mt_status, mt_len, mt_queued;
	const char *p = (const char*)rx_buf + 7;
	char **rx_buf_parse = (char**)&p;

	mo_status = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	strtol(*rx_buf_parse, rx_buf_parse, 10); // MOMSN, ignore it
	(*rx_buf_parse)++;
	mt_status = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	strtol(*rx_buf_parse, rx_buf_parse, 10); // MTMSN, ignore it
	(*rx_buf_parse)++;
	mt_len = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	mt_queued = strtol(*rx_buf_parse, rx_buf_parse, 10);

	if (verbose) { warnx("MO ST: %d, MT ST: %d, MT LEN: %d, MT QUEUED: %d", mo_status, mt_status, mt_len, mt_queued); }

	switch(mo_status){
	case 0:
	case 2:
	case 3:
	case 4:
		if (verbose) { warnx("SBD SESSION: SUCCESS"); }
		ring_pending = false;
		tx_pending = false;
		rx_pending = false;
		if (mt_len > 0) {
			// read the message here
			void message_received(void);
		}
		break;

	case 1:
		if (verbose) { warnx("SBD SESSION: MO SUCCESS, MT FAIL"); }
		tx_pending = false;
		break;

	case 32:
		if (verbose) { warnx("SBD SESSION: NO NETWORK SIGNAL"); }
		warnx("ASSUMING MSG SENT FOR SIMULATION");	// TODO REMOVE
		tx_pending = false;							// TODO REMOVE
		break;

	default:
		if (verbose) { warnx("SBD SESSION: FAILED (%d)", mo_status); }
	}

	new_state = SATCOM_STATE_STANDBY;
}

void satcom::test_loop(void)
{
	int res = read_at();
	if(res != SATCOM_RESULT_NA){
		warnx("TEST RESULT: %d, LENGTH %d\nDATA:\n%s", res, rx_msg_len, rx_buf);
		warnx("TEST DONE, TOOK %lld MS", (hrt_absolute_time() - time_counter)/1000);
		new_state = SATCOM_STATE_STANDBY;
	}
}

ssize_t satcom::write(struct file *filp, const char *buffer, size_t buflen)
{
	warnx("write");

	if (buflen > tx_buf_free) {
		return 0;
	}

	pthread_mutex_lock(&tx_buf_mutex);

	for(int i = 0; i < buflen; i++){
		tx_buf[tx_buf_end_idx] = buffer[i];
		tx_buf_end_idx = (tx_buf_end_idx + 1) % SATCOM_TX_BUF_LEN;
	}

	tx_buf_free -= buflen;

	pthread_mutex_unlock(&tx_buf_mutex);

	return buflen;
}

ssize_t satcom::read(struct file *filp, char *buffer, size_t buflen)
{
	if (rx_msg_read_idx < rx_msg_len) {
		size_t bytes_to_copy = rx_msg_len - rx_msg_read_idx;

		if (bytes_to_copy > buflen) {
			bytes_to_copy = buflen;
		}

//		memcpy(buffer, &rx_msg.data[rx_msg_read_idx], bytes_to_copy);

		rx_msg_read_idx += bytes_to_copy;

		return bytes_to_copy;

	} else {
		return -EAGAIN;
	}
}

int satcom::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return CDev::ioctl(filp, cmd, arg);
}

pollevent_t satcom::poll_state(struct file *filp)
{
	if (rx_msg_read_idx < rx_msg_len) {
		return POLLIN;

	} else {
		return 0;
	}
}

void satcom::send_tx_buf()
{
	if (!is_modem_ready()) {
		if (verbose) { warnx("SEND SBD: MODEM NOT READY!"); }
		return;
	}

	pthread_mutex_lock(&tx_buf_mutex);

	int msg_len = SATCOM_TX_BUF_LEN - tx_buf_free;
	int s_idx = tx_buf_start_idx;

	char command[13];
	sprintf(command, "AT+SBDWB=%d", msg_len);
	write_at(command);

	if (read_at() != SATCOM_RESULT_READY) {
		if (verbose) { warnx("SEND SBD: MODEM NOT RESPONDING!"); }
		return;
	}

	int sum = 0;

	warnx("WRITING TO MODEM:");

	while(s_idx != tx_buf_end_idx){
		::write(uart_fd, tx_buf + s_idx, 1);
		sum += *(tx_buf + s_idx);
		printf("%c ", *(tx_buf + s_idx));
		s_idx = (s_idx + 1) % SATCOM_TX_BUF_LEN;
	}

	printf("\n");

	uint8_t checksum[2] = {(uint8_t)(sum / 256), (uint8_t)(sum & 255)};
	::write(uart_fd, checksum, 2);

	if (read_at() != SATCOM_RESULT_OK) {
		if (verbose) { warnx("SEND SBD: ERROR WHILE WRITING DATA TO MODEM!"); }
		pthread_mutex_unlock(&tx_buf_mutex);
		return;
	}

	if (rx_buf[0] != '0') {
		if (verbose) { warnx("SEND SBD: ERROR WHILE WRITING DATA TO MODEM! (%d)", rx_buf[0] - '0'); }
		pthread_mutex_unlock(&tx_buf_mutex);
		return;
	}

	warnx("WROTE DATA");

	tx_buf_start_idx = s_idx;
	tx_buf_free += msg_len;

	pthread_mutex_unlock(&tx_buf_mutex);

	tx_pending = true;
}

void satcom::message_received(void)
{
	if (verbose) { warnx("MESSAGE RECEIVED"); }
}

satcom_status satcom::get_msg(uint32_t timeout_ms)
{
	return SATCOM_OK;
}

void satcom::start_csq(void)
{
	if(verbose) { warnx("UPDATING SIGNAL QUALITY"); }

	if (!is_modem_ready()) {
		if (verbose) { warnx("UPDATE SIGNAL QUALITY: MODEM NOT READY!"); }

		return;
	}

	write_at("AT+CSQ");
	new_state = SATCOM_STATE_CSQ;
}

void satcom::start_sbd_session(void)
{
	if(verbose) { warnx("STARTING SBD SESSION"); }

	if (!is_modem_ready()) {
		if (verbose) { warnx("SBD SESSION: MODEM NOT READY!"); }

		return;
	}

	if(ring_pending){
		write_at("AT+SBDIXA");
	} else {
		write_at("AT+SBDIX");
	}

	new_state = SATCOM_STATE_SBDSESSION;
}

void satcom::start_test(void)
{
	int res = read_at();

	if (res != SATCOM_RESULT_NA) {
		warnx("SOMETHING WAS IN BUFFER");
		printf("TEST RESULT: %d, LENGTH %d\nDATA:\n%s\nRAW DATA:\n", res, rx_msg_len, rx_buf);

		for (int i = 0; i < rx_msg_len; i++) {
			printf("%d ", rx_buf[i]);
		}

		printf("\n");
	}

	if(!is_modem_ready()){
		warnx("MODEM NOT READY!");
		return;
	}

	if (strlen(test_command) != 0) {
		warnx("TEST %s", test_command);
		write_at(test_command);
		new_state = SATCOM_STATE_TEST;
	} else {
		warnx("TEST DONE");
	}
}

satcom_uart_status satcom::open_uart(char *uart_name)
{
	if (verbose) { warnx("opening satcom modem UART: %s", uart_name); }

	uart_fd = ::open(uart_name, O_RDWR | O_BINARY);

	if (uart_fd < 0) {
		if (verbose) { warnx("UART open failed!"); }

		return SATCOM_UART_OPEN_FAIL;
	}

	// set the UART speed to 19200
	struct termios uart_config;
	tcgetattr(uart_fd, &uart_config);
	cfsetspeed(&uart_config, 19200);
	tcsetattr(uart_fd, TCSANOW, &uart_config);

	if (verbose) { warnx("UART opened"); }

	return SATCOM_UART_OK;
}

bool satcom::is_modem_ready(void)
{
	write_at("AT");

	if (read_at() == SATCOM_RESULT_OK) {
		return true;

	} else {
		return false;
	}
}

void satcom::write_at(const char *command)
{
	if (verbose) { warnx("WRITING AT COMMAND: %s", command); }

	::write(uart_fd, command, strlen(command));
	::write(uart_fd, "\r", 1);
}

satcom_result_code satcom::read_at(void)
{
	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	uint8_t buf = 0;
	int last_rn_idx = 0;
	int rx_buf_pos = 0;
	rx_msg_len = 0;

	while (1) {
		if (::poll(&fds[0], 1, 10) > 0) {
			if (::read(uart_fd, &buf, 1) > 0) {
				if (rx_buf_pos == 0 && (buf == '\r' || buf == '\n')) {
					// ignore the leading \r\n
					continue;
				}

				rx_buf[rx_buf_pos++] = buf;

				if (rx_buf[rx_buf_pos - 1] == '\n' && rx_buf[rx_buf_pos - 2] == '\r') {
					// found the \r\n delimiter
					if (rx_buf_pos == last_rn_idx + 2)
						; // second in a row, ignore it
					else if (!strncmp((const char *)&rx_buf[last_rn_idx], "OK\r\n", 4)) {
						rx_buf[rx_msg_len] = 0; 	// null terminator after the information response for printing purposes
						return SATCOM_RESULT_OK;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "ERROR\r\n", 7)) {
						return SATCOM_RESULT_ERROR;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "SBDRING\r\n", 9)) {
						return SATCOM_RESULT_SBDRING;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "READY\r\n", 7)) {
						return SATCOM_RESULT_READY;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "HARDWARE FAILURE", 16)) {
						return SATCOM_RESULT_HWFAIL;

					} else {
						rx_msg_len = rx_buf_pos;	// that was the information response, result code incoming
					}

					last_rn_idx = rx_buf_pos;
				}
			}

		} else {
			break;
		}
	}

	return SATCOM_RESULT_NA;
}

void satcom::schedule_test(void)
{
	test_pending = true;
}

int satcom_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {
		return satcom::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		return satcom::stop();

	} else if (!strcmp(argv[1], "status")) {
		satcom::status();
		return OK;

	} else if (!strcmp(argv[1], "test")) {
		satcom::test(argc, argv);
		return OK;
	}

	warnx("usage: satcom {start|stop|status|test} [-d uart_device | -v]");

	return ERROR;
}

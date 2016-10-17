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

satcom* satcom::instance;
int satcom::task_handle;

int satcom::param_timeout_s;
int satcom::param_read_interval_s;

satcom::satcom():
	CDev("satcom", SATCOM_DEVICE_PATH)
{}

int satcom::start(int argc, char *argv[])
{
	warnx("starting");

	if(satcom::instance != nullptr)
	{
		warnx("already started");
		return ERROR;
	}

	satcom::instance = new satcom();

	satcom::task_handle = px4_task_spawn_cmd("satcom", SCHED_DEFAULT,
			SCHED_PRIORITY_SLOW_DRIVER, 2000, (main_t)&satcom::main_loop_helper, argv);

	return OK;
}

int satcom::stop()
{
	if(satcom::instance == nullptr)
	{
		warnx("not started");
		return ERROR;
	}

	warnx("stopping...");

	satcom::instance->task_should_exit = true;

	// give it enough time to stop
	param_timeout_s = 10;
	for(int i = 0; (i < param_timeout_s + 1) && (satcom::task_handle != -1); i++)
	{
		sleep(1);
	}

	// well, kill it anyway, though this may crash
	if (satcom::task_handle != -1)
	{
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
	if(satcom::instance == nullptr)
	{
		warnx("not started");
	}
	else
	{
		warnx("started");

//		satcom::instance->test_procedure_pending = true;
//		while(satcom::instance->test_procedure_pending)
//			usleep(100000);

//		warnx("signal strength %d/5", satcom::instance->signal_quality);
//		warnx("network status %d/1", satcom::instance->network_status);
//		warnx("tx buff: %d/%d, %d free", satcom::instance->tx_msg_count, SATCOM_MAX_TX_MSG, satcom::instance->tx_free);
	}
}

void satcom::test(int argc, char *argv[])
{
	if(instance == nullptr)
	{
		warnx("not started");
		return;
	}

	if (argc > 2)
		strcpy(instance->test_command, argv[2]);
	else
		instance->test_command[0] = 0;

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
	pthread_mutex_init(&_tx_buffer_mutex, NULL);

	int arg_i = 3;
	int arg_uart_name = 0;
	while(arg_i < argc) {
		if(!strcmp(argv[arg_i], "-d")){
			arg_i++;
			arg_uart_name = arg_i;
		}
		else if(!strcmp(argv[arg_i], "-v")){
			warnx("verbose mode ON");
			verbose = true;
		}
		arg_i++;
	}

	if(arg_uart_name == 0){
		warnx("no satcom modem UART port provided!");
		task_should_exit = true;
		return;
	}

	if(open_uart(argv[arg_uart_name]) != SATCOM_UART_OK){
		warnx("failed to open UART port!");
		task_should_exit = true;
		return;
	}

	param_t param_pointer;

	param_pointer = param_find("SATCOM_TIMEOUT");
	param_get(param_pointer, &param_timeout_s);
	if(param_timeout_s == -1)
		param_timeout_s = 30;

	param_pointer = param_find("SATCOM_READINT");
	param_get(param_pointer, &param_read_interval_s);
	if(param_read_interval_s == -1)
		param_read_interval_s = 10;

	if(verbose) warnx("timeout %d read interval %d", param_timeout_s, param_read_interval_s);

	while(!task_should_exit)
	{
		usleep(100000);	// 100ms

		if(test_procedure_pending)
			test_procedure();
		continue;

		bool write_pending = tx_msg_count > 0;
		// only try to get a new msg if the previous was read by the upper layer
		bool read_pending = (rx_msg_read_idx == rx_msg_len) &&
				(hrt_absolute_time() - last_read_time > (uint64_t)param_read_interval_s * 1000000);

		if(!(write_pending || read_pending || test_procedure_pending))
			continue;

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
//				satcom_status ret = send_msg(tx_buf, len, param_timeout_s * 1000);
//
//				if(ret == SATCOM_OK)
//				{
//					pthread_mutex_lock(&_tx_buffer_mutex);
//
//					memmove(tx_buf, tx_buf + len, SATCOM_TX_BUF_LEN - len - tx_free);
//					memmove(tx_msg_len, tx_msg_len + 1, 4 * (SATCOM_MAX_TX_MSG - 1));
//					tx_free += len;
//
//					tx_msg_count--;
//
//					pthread_mutex_unlock(&_tx_buffer_mutex);
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

ssize_t satcom::write(struct file *filp, const char *buffer, size_t buflen)
{
	if(buflen > tx_free || tx_msg_count == SATCOM_MAX_TX_MSG)
		return 0;

	pthread_mutex_lock(&_tx_buffer_mutex);

	uint8_t* buf_start = tx_buf + SATCOM_TX_BUF_LEN - tx_free;
	memcpy(buf_start, buffer, buflen);
	tx_msg_len[tx_msg_count] = buflen;
	tx_free -= buflen;
	tx_msg_count++;

	pthread_mutex_unlock(&_tx_buffer_mutex);

	return buflen;
}

ssize_t satcom::read(struct file *filp, char *buffer, size_t buflen)
{
	if(rx_msg_read_idx < rx_msg_len)
	{
		size_t bytes_to_copy = rx_msg_len - rx_msg_read_idx;

		if(bytes_to_copy > buflen)
			bytes_to_copy = buflen;

//		memcpy(buffer, &rx_msg.data[rx_msg_read_idx], bytes_to_copy);

		rx_msg_read_idx += bytes_to_copy;

		return bytes_to_copy;
	}
	else
	{
		return -EAGAIN;
	}
}

int satcom::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return CDev::ioctl(filp, cmd, arg);
}

pollevent_t satcom::poll_state(struct file *filp)
{
	if(rx_msg_read_idx < rx_msg_len)
		return POLLIN;
	else
		return 0;
}

int satcom::send_msg(uint8_t* msg, int msg_len, uint32_t timeout_ms)
{
	if(!is_modem_ready()){
		if(verbose) warnx("SEND SBD: MODEM NOT READY!");
		return ERROR;
	}

	char command[13];
	sprintf(command, "AT+SBDWB=%d", msg_len);

	if(write_at(command, 100) != SATCOM_RESULT_READY){
		if(verbose) warnx("SEND SBD: MODEM NOT RESPONDING!");
		return ERROR;
	}

	int sum = {0};

	for(int i = 0; i < msg_len; i++){
		::write(uart_fd, msg+i, 1);
		sum += msg[i];
	}

	uint8_t checksum[2] = {(uint8_t)(sum >> 8), (uint8_t)(sum & 255)};

	::write(uart_fd, checksum, 2);

	return OK;
}

satcom_status satcom::get_msg(uint32_t timeout_ms)
{
	return SATCOM_OK;
}

int satcom::update_signal_quality(void)
{
	if(!is_modem_ready()){
		if(verbose) warnx("UPDATE SIGNAL QUALITY: MODEM NOT READY!");
		return ERROR;
	}

	if(write_at("AT+CSQ", 2000) != SATCOM_RESULT_OK){
		if(verbose) warnx("UPDATE SIGNAL QUALITY: TIMED OUT!");
		return ERROR;
	}

	if(strncmp((const char*)rx_buf, "+CSQ:", 5)){
		if(verbose) warnx("UPDATE SIGNAL QUALITY: WRONG ANSWER:");
		if(verbose) warnx("%s", rx_buf);
		return ERROR;
	}

	signal_quality = rx_buf[5]-48;
	if(verbose) warnx("SIGNAL QUALITY: %d", signal_quality);

	return OK;
}

satcom_uart_status satcom::open_uart(char* uart_name)
{
	if(verbose) warnx("opening satcom modem UART: %s", uart_name);

	uart_fd = ::open(uart_name, O_RDWR | O_BINARY);

	if(uart_fd < 0){
		if(verbose) warnx("UART open failed!");
		return SATCOM_UART_OPEN_FAIL;
	}

	// set the UART speed to 19200
	struct termios uart_config;
	tcgetattr(uart_fd, &uart_config);
	cfsetspeed(&uart_config, 19200);
	tcsetattr(uart_fd, TCSANOW, &uart_config);

	if(verbose) warnx("UART opened");
	return SATCOM_UART_OK;
}

void satcom::test_procedure(void)
{
	test_procedure_pending = false;

	if(!strcmp(test_command, "csq")){
		update_signal_quality();
		return;
	}

	if(!strcmp(test_command, "send")){
		send_msg((uint8_t*)"kreczmer", 8, 1000);
		return;
	}

	int res = read_at(100);

	if(res != SATCOM_RESULT_NA){
		warnx("SOMETHING WAS IN BUFFER");
		printf("TEST RESULT: %d, LENGTH %d\nDATA:\n%s\nRAW DATA:\n", res, rx_msg_len, rx_buf);
		for(int i=0; i<rx_msg_len; i++){
			printf("%d ", rx_buf[i]);
		}
		printf("\n");
	}

	warnx("IS MODEM READY %d", is_modem_ready());

	if(strlen(test_command) != 0){
		warnx("TEST %s", test_command);
		res = write_at(test_command, 5000);
	} else {
		warnx("EMPTY TEST");
		res = read_at(100);
	}

	printf("TEST RESULT: %d, LENGTH %d\nDATA:\n%s\nRAW DATA:\n", res, rx_msg_len, rx_buf);
	for(int i=0; i<rx_msg_len; i++){
		printf("%d ", rx_buf[i]);
	}
	printf("\n");
}

bool satcom::is_modem_ready(void)
{
	if (write_at("AT", 100) == SATCOM_RESULT_OK)
		return true;
	else
		return false;
}

satcom_result_code satcom::write_at(const char* command, int timeout_ms)
{
	if(verbose) warnx("WRITING AT COMMAND: %s, TIMEOUT: %dMS", command, timeout_ms);

	::write(uart_fd, command, strlen(command));
	::write(uart_fd, "\r", 1);

	time_counter = hrt_absolute_time();

	satcom_result_code ret = read_at(timeout_ms);

	time_counter = (hrt_absolute_time() - time_counter)/1000;
	if(verbose) warnx("GOT REPLY AFTER %lldMS", time_counter);

	return ret;
}

satcom_result_code satcom::read_at(int timeout_ms)
{
	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	uint8_t buf = 0;
	int nread = 0;
	int last_rn_idx = 0;
	int rx_buf_pos = 0;
	rx_msg_len = 0;

	while(1)
	{
		if(::poll(&fds[0], 1, timeout_ms) > 0)
		{
			nread = ::read(uart_fd, &buf, 1);

			if(nread > 0)
			{
				if(rx_buf_pos == 0 && (buf == '\r' || buf == '\n')){
					// ignore the leading \r\n
					continue;
				}

				rx_buf[rx_buf_pos++] = buf;

				if(rx_buf[rx_buf_pos-1] == '\n' && rx_buf[rx_buf_pos-2] == '\r'){
					// found the \r\n delimiter
					rx_buf[rx_msg_len] = 0; 	// null termination for printing purposes

					if(rx_buf_pos == last_rn_idx+2)
						; // second in a row, ignore it
					else if(!strncmp((const char*)&rx_buf[last_rn_idx], "OK\r\n", 4)){
						return SATCOM_RESULT_OK;
					} else if (!strncmp((const char*)&rx_buf[last_rn_idx], "ERROR\r\n", 7)){
						return SATCOM_RESULT_ERROR;
					} else if (!strncmp((const char*)&rx_buf[last_rn_idx], "SBDRING\r\n", 9)){
						return SATCOM_RESULT_SBDRING;
					} else if (!strncmp((const char*)&rx_buf[last_rn_idx], "READY\r\n", 7)){
						return SATCOM_RESULT_READY;
					} else if (!strncmp((const char*)&rx_buf[last_rn_idx], "HARDWARE FAILURE", 16)){
						return SATCOM_RESULT_HWFAIL;
					} else {
						rx_msg_len = rx_buf_pos;	// that was the information response, result code incoming
					}

					last_rn_idx = rx_buf_pos;
				}
			}
		}
		else
			break;
	}

	return SATCOM_RESULT_NA;
}

void satcom::schedule_test(void)
{
	test_procedure_pending = true;
}

int satcom_main(int argc, char* argv[])
{
	if(!strcmp(argv[1], "start"))
	{
		return satcom::start(argc, argv);
	}
	else if(!strcmp(argv[1], "stop"))
	{
		return satcom::stop();
	}
	else if(!strcmp(argv[1], "status"))
	{
		satcom::status();
		return OK;
	}
	else if(!strcmp(argv[1], "test"))
	{
		satcom::test(argc, argv);
		return OK;
	}

	warnx("usage: satcom {start|stop|status|test} [-d uart_device | -v]");

	return ERROR;
}

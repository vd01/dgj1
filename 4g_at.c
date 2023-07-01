#include <string.h>
#include "4g_at.h"
#include "gprs.h"
#include "igprs.h"
#include "vm_api.h"
#include "uart_receiv_process.h"
#include "rtc_low_power.h"
#include "hwi.h"
#include "common.h"

static T_My4gAtEnv env = {0};

//#define MYDEBUG(...) printf(__VA_ARGS__)
#define MYDEBUG(...)

#define MYLOG(...) printf(__VA_ARGS__)

static void do_http_device_register();
static void do_http_device_boot();
static void do_http_dm_report();
static void do_mqtt_connect();
static void do_mqtt_report();
void http_get_cgw();

//复制字符串到全局缓冲区并返回地址
static void cfg_strcpy(char **dst, char *src) {
	int len = strlen(src);
	if( env.cfg_string_idx + len + 1 > CFG_STRING_BUFF_SIZE ) {
		MYLOG("cfg_string_buff overflow!! cfg_string_idx, src=\"%s\",%d\n", env.cfg_string_idx, src, len);
		return;
	}
	*dst = env.cfg_string_buff + env.cfg_string_idx;
	strcpy(*dst, src);

	env.cfg_string_buff[env.cfg_string_idx+len+1] = 0;
	env.cfg_string_idx += len + 1;
	//MYDEBUG("[4G]cfg_strcpy cfg_string_idx %d\n", env.cfg_string_idx);
}

//添加回调函数及其参数到回调链
static void add_func(my_at_callback func, const char **params) {
	//MYDEBUG("add_func %d %x %x\n", env.callback_cnt, func, params);
	env.callback_funcs[env.callback_cnt] = func;
	env.callback_params[env.callback_cnt] = params;
	env.callback_cnt ++;
}

//调用回调链第一个回调函数
static void call_first() {
	env.callback_cur = 0;
	env.status = MY_AT_BUSY;
	env.last_time = timer_get_ms();
	MYDEBUG("[4G]# call first, cnt:%d, last_time:%d\n", env.callback_cnt, env.last_time);
	env.callback_funcs[env.callback_cur](MY_AT_EVT_BEGIN, env.callback_params[env.callback_cur], NULL);
}

//调用回调链下一个回调函数
static void call_next() {
	if( env.callback_cur == env.callback_cnt - 1 ) {
		MYLOG("[4G]# to idle\n");
		env.status = MY_AT_IDLE;
		return;
	}
	env.callback_cur += 1;
	env.last_time = timer_get_ms();
	MYDEBUG("[4G]callback_cur:%d/%d, last_time:%d\n", env.callback_cur, env.callback_cnt, env.last_time);
	env.callback_funcs[env.callback_cur](MY_AT_EVT_BEGIN, env.callback_params[env.callback_cur], NULL);
}

//回调链失败，结束
static void call_failed() {
	env.callback_cur = 0;
	env.status = MY_AT_IDLE;
	MYLOG("[4G]# to idle\n");
}

//获取网络时间戳
static uint32_t get_network_timestamp() {
	pT_Network nw = NW_GetNetwork();
	uint32_t timestamp;
	timestamp = mktime(nw->time, 24 - nw->time.timezone);
	return timestamp;
}

//获取网络时间字符串
static char *get_network_timestamp_s() {
	uint32_t timestamp = get_network_timestamp();
	static char str_timestamp[14];
	sprintf(str_timestamp, "%d000", timestamp);
	return str_timestamp;
}

//发送数据到模组
static uint16_t send_loop(const char **params, uint8_t calc) {
	//MYDEBUG("[4G]>> ");
	uint16_t len = 0;
	for(uint16_t i=0; params[i]; i++) {
		char *tmp = params[i];
		if( ! strcmp(params[i], "{STR_LEN1}") ) {
			tmp = env.str_len1;
		} else if( ! strcmp(params[i], "{STR_LEN2}") ) {
			tmp = env.str_len2;
		} else if( ! strcmp(params[i], "{STR_LEN3}") ) {
			tmp = env.str_len3;
		} else if( ! strcmp(params[i], "{HOST}") ) {
			tmp = env.gwAddress2;
		} else if( ! strcmp(params[i], "{SN}") ) {
			tmp = env.deviceSN;
		} else if( ! strcmp(params[i], "{DEVICEID}") ) {
			tmp = env.deviceId;
		} else if( ! strcmp(params[i], "{DMTOKEN}") ) {
			tmp = env.dmToken;
		} else if( ! strcmp(params[i], "{IMEI}") ) {
			tmp = env.deviceIMEI;
		} else if( ! strcmp(params[i], "{MQUSER}") ) {
			tmp = env.mqttUser;
		} else if( ! strcmp(params[i], "{MQPASS}") ) {
			tmp = env.mqttPassword;
		} else if( ! strcmp(params[i], "{MQSERVER}") ) {
			tmp = env.mqttServer;
		} else if( ! strcmp(params[i], "{MQPORT}") ) {
			tmp = env.mqttPort;
		} else if( ! strcmp(params[i], "{TIME}") ) {
			tmp = get_network_timestamp_s();
		} else if( ! strcmp(params[i], "{SBP}") ) {
			sprintf(env.str_len2, "%d", env.sbp);
			tmp = env.str_len2;
		} else if( ! strcmp(params[i], "{DBP}") ) {
			sprintf(env.str_len2, "%d", env.dbp);
			tmp = env.str_len2;
		} else if( ! strcmp(params[i], "{PR}") ) {
			sprintf(env.str_len2, "%d", env.pr);
			tmp = env.str_len2;
		}
		//MYDEBUG(tmp);
		if( ! calc ) {
			uart1_send_buff(tmp, strlen(tmp));
		}
		len += strlen(tmp);
	}
	//MYDEBUG("\n");
	if( ! calc ) {
		uart1_send_buff("\r\n", 2);
		//myTimerStart(20);
	}
	return len;
}

//重启4G模组
static void do_4g_restart() {
	MYDEBUG("in do_4g_restart..%d\n", env.begin_4g_restart);
	NW_Reset();
	env.cfg_string_idx = 0;
	env.deviceSN = NULL;
	//static const char *params0[] = {"AT+CFUN=1,1",NULL};
	//send_loop(params0, 0);
	if( 1 == env.begin_4g_restart ) {
		bt_off();
		MYLOG("[4G]shutdown 4g\n");
		env.begin_4g_restart ++;
	} else if( 3 == env.begin_4g_restart ) {
		MYLOG("[4G]powerup 4g\n");
		bt_on();
		env.begin_4g_restart = 0;
	} else {
		env.begin_4g_restart ++;
	}
}

#define MY4G_CONFG_CNT 3

//从Flash读取配置数据
static int vm_read_my4g_config(T_My4G_Config *cfg) {
	MYDEBUG("in vm_read_my4g_config..\n");
	T_My4G_Config tmp_cfgs[MY4G_CONFG_CNT];
	int errs[MY4G_CONFG_CNT];
	uint16_t crcs[MY4G_CONFG_CNT];
	for(int i=0; i<MY4G_CONFG_CNT; i++) {
		errs[i] = vm_api_read(VM_MY4G_CONFIG1+i, (void*)&tmp_cfgs[i]);
		if( ! errs[i] ) {
			crcs[i] = CRC16(&tmp_cfgs[i], sizeof(T_My4G_Config)-sizeof(uint32_t));
			if( !errs[i] && crcs[i] == tmp_cfgs[i].crc ) {
				memcpy(cfg, &tmp_cfgs[i], sizeof(T_My4G_Config));
				return 0;
			}
		}
	}
//	int found_ok = 0;
//	for(int i=0; i<MY4G_CONFG_CNT; i++) {
//		//printf("%d %d %d %d\n", i, errs[i], crcs[i], tmp_cfgs[i].crc);
//		if( !errs[i] && crcs[i] == tmp_cfgs[i].crc ) {
//			memcpy(cfg, &tmp_cfgs[i], sizeof(T_My4G_Config));
//			found_ok = 1;
//			break;
//		}
//	}
//	if( ! found_ok ) {
//		return 1;
//	}
//	for(int i=0; i<3; i++) {
//		if( errs[i] || crcs[i] != tmp_cfgs[i].crc ) {
//			MYLOG("[4G] rewrite VM_MY4G_CONFIG %d\n", i);
//			vm_api_write(VM_MY4G_CONFIG1+i, (void*)cfg);
//		}
//	}
	return 0;
}

//写入配置数据到flash，写入三份
static int vm_write_my4g_config(T_My4G_Config *cfg) {
	int ret = 0;
	cfg->crc = CRC16(cfg, sizeof(T_My4G_Config)-sizeof(uint32_t));
	MYDEBUG("[4G] vm_write_my4g_config crc:%d\n", cfg->crc);
	for(int i=0; i<MY4G_CONFG_CNT; i++) {
		int err = vm_api_write(VM_MY4G_CONFIG1+i, (void*)cfg);
		if( err ) {
			ret = err;
		}
	}
	MYLOG("[4G] vm_write_my4g_config: %d\n", ret);
	return ret;
}

//保存deviceId和dmToken
static int save_my4g_config() {
	T_My4G_Config my4g_cfg = {0};
	int err = vm_read_my4g_config(&my4g_cfg);
	if( ! err ) {
		strcpy(my4g_cfg.andlink_device_id, env.deviceId);
		strcpy(my4g_cfg.andlink_dm_tokne, env.dmToken);
		return vm_write_my4g_config(&my4g_cfg);
	}
	return err;
}

//保存SN，同时清空deviceId和last_dm_report
static int save_my4g_config_sn(char *sn) {
	T_My4G_Config my4g_cfg = {0};
	vm_read_my4g_config(&my4g_cfg);
	strcpy(my4g_cfg.andlink_device_sn, sn);
	memset(my4g_cfg.andlink_device_id, 0, sizeof(my4g_cfg.andlink_device_id));
	my4g_cfg.last_dm_report = 0;
	return vm_write_my4g_config(&my4g_cfg);
}

//保存最后一次DM上报的时间
static void save_last_dm_report() {
	T_My4G_Config my4g_cfg = {0};
	int err = vm_read_my4g_config(&my4g_cfg);
	if( ! err ) {
		my4g_cfg.last_dm_report = get_network_timestamp();
		int ret = vm_write_my4g_config(&my4g_cfg);
		MYLOG("[4G] set last_dm_report to %d, ret:%d\n", my4g_cfg.last_dm_report, ret);
	}
}

//发送简单指令并等待模组响应"0"
static void func_sample_cmd(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		if( ! strlen(reply) || ! strcmp(reply, "0") || ! strcmp(reply, "4") || ! strncmp(reply, "+", 1) ) {
			call_next();
		} else {
            MYLOG("[4G]# func_sample_cmd: AT failed!\n");
            call_failed();
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 10*1000 ) {
			MYLOG("[4G]# func_sample_cmd: timeout! %d\n", cur_time);
			call_failed();
		}
	}
}

//发送简单指令并等待模组响应"CONNECT"
static void func_wait_connect(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "CONNECT";
		if( ! strlen(reply) || ! strcmp(reply, "0") || ! strncmp(reply, "+", 1) ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
            MYLOG("[4G]# func_wait_connect: AT failed!\n");
            call_failed();
            if( env.retry_cnt < MAX_4G_RETRY ) {
				env.begin_4g_restart = 1;
				env.retry_cnt ++;
			} else {
				MYLOG("\n\n4G_ERR:329\n\n");
			}
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 15*1000 ) {
			MYLOG("[4G]# func_wait_connect: timeout! %d\n", cur_time);
			call_failed();
			if( env.retry_cnt < MAX_4G_RETRY ) {
				env.begin_4g_restart = 1;
				env.retry_cnt ++;
			} else {
				MYLOG("\n\n4G_ERR:329\n\n");
			}
		}
	}
}

//发送AT+QMTCFG willex 指令，等待模组响应
static void func_mqtt_willex(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		if( ! strlen(reply) ) {
			call_next();
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 30*1000 ) {
			MYLOG("[4G]# func_mqtt_willex: AT timeout! %d\n", cur_time);
			call_failed();
		}
	}
}

//发送AT+QMTCFG willex 数据，等待模组响应
static void func_mqtt_will_msg(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		if( ! strlen(reply) || ! strcmp(reply, "0") || ! strcmp(reply, "> 0") ) {
			call_next();
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 30*1000 ) {
			MYLOG("[4G]# func_mqtt_will_msg: timeout! %d\n", cur_time);
			call_failed();
		}
	}
}

//发送AT+QMTOPEN指令，等待模组响应
static void func_mqtt_open(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "+QMTOPEN: 0,0";
		if( ! strlen(reply) || ! strcmp(reply, "0") || ! strcmp(reply, "> 0") ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
            MYLOG("[4G]# func_mqtt_open: AT failed!\n");
            call_failed();
            if( env.mqtt_retry_cnt < MAX_4G_RETRY ) {
				env.mqtt_retry_cnt ++;
				//do_mqtt_connect();
				env.begin_4g_restart = 1;
			} else {
				MYLOG("\n\n4G_ERR:331\n\n");
			}
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 5000 ) {
			MYLOG("[4G]# func_mqtt_open: timeout! %d\n", cur_time);
			call_failed();
			if( env.mqtt_retry_cnt < MAX_4G_RETRY ) {
				env.mqtt_retry_cnt ++;
				//do_mqtt_connect();
				env.begin_4g_restart = 1;
			} else {
				MYLOG("\n\n4G_ERR:338\n\n");
			}
		}
	}
}

//发送AT+QMTCONN指令，等待模组响应
static void func_mqtt_conn(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "+QMTCONN: 0,0,0";
		if( ! strlen(reply) || ! strcmp(reply, "0") || ! strcmp(reply, "> 0") ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
            MYLOG("[4G]# func_mqtt_conn: AT failed!\n");
            call_failed();
            if( env.mqtt_retry_cnt < MAX_4G_RETRY ) {
				env.mqtt_retry_cnt ++;
				//do_mqtt_connect();
				env.begin_4g_restart = 1;
			} else {
				MYLOG("\n\n4G_ERR:331\n\n");
			}
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 15*1000 ) {
			MYLOG("[4G]# func_mqtt_conn: timeout! %d\n", cur_time);
			call_failed();
			if( env.mqtt_retry_cnt < MAX_4G_RETRY ) {
				env.mqtt_retry_cnt ++;
				//do_mqtt_connect();
				env.begin_4g_restart = 1;
			} else {
				MYLOG("\n\n4G_ERR:338\n\n");
			}
		}
	}
}

static void dump_reply(uint8_t *reply) {
	uint8_t *p = reply;
	while( *p ) {
		printf("%02X ", *p);
		p++;
	}
	printf("\n");
}


//发送MQTT PUB Topic，等待模组响应
static void func_mqtt_pub_topic(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		if( 0 == strlen(reply) ) {
			call_next();
		} else {
            MYLOG("[4G]# func_mqtt_pub_topic: AT failed!\n");
            dump_reply(reply);
            call_failed();
            if( env.mqtt_retry_cnt < MAX_4G_RETRY ) {
	            env.mqtt_retry_cnt ++;
				env.begin_4g_restart = 1;
            } else {
	            MYLOG("\n\n4G_ERR:332\n\n");
	        }
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 10*1000 ) {
			MYLOG("[4G]# func_mqtt_pub_topic: timeout! %d\n", cur_time);
			call_failed();
			if( env.mqtt_retry_cnt < MAX_4G_RETRY ) {
				env.mqtt_retry_cnt ++;
				//do_mqtt_connect();
				env.begin_4g_restart = 1;
			} else {
				MYLOG("\n\n4G_ERR:339\n\n");
			}
		}
	}
}


//发送MQTT PUB数据，等待模组响应
static void func_mqtt_pub_payload(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "+QMTPUBEX: 0,0,0";
		//printf("%d %d %d %d\n", reply[0], reply[1], reply[2], reply[3]);
		if( ! strlen(reply) || ! strcmp(reply, "0") || ! strncmp(reply, "> ", 2) ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
            MYLOG("[4G]# func_mqtt_pub_payload: AT failed!\n");
            call_failed();
            MYLOG("\n\n4G_ERR:332\n\n");
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 15*1000 ) {
			MYLOG("[4G]# func_mqtt_pub_payload: timeout! %d\n", cur_time);
			call_failed();
			if( env.mqtt_retry_cnt < MAX_4G_RETRY ) {
				env.mqtt_retry_cnt ++;
				//do_mqtt_connect();
				env.begin_4g_restart = 1;
			} else {
				MYLOG("\n\n4G_ERR:339\n\n");
			}
		}
	}
}

//发送AT+QHTTPPOST数据，等待模块响应
static void func_http_post(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "+QHTTPPOST: 0,200";
		if( ! strlen(reply) || ! strcmp(reply, "0") || ! strncmp(reply, "+QHTTPREAD:", 11) ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
            MYLOG("[4G]# func_http_post: AT failed!\n");
            call_failed();
            MYLOG("\n\n4G_ERR:322\n\n");
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 15*1000 ) {
			MYLOG("[4G]# func_http_post: timeout! %d\n", cur_time);
			call_failed();
			if( env.retry_cnt < MAX_4G_RETRY ) {
				env.begin_4g_restart = 1;
				env.retry_cnt ++;
			} else {
				MYLOG("\n\n4G_ERR:329\n\n");
			}
		}
	}
}

//解析服务器响应的resultCode值
static int parse_result_code(char *reply) {
	char *key = "\"resultCode\":";
	char *start = strstr(reply, key);
	if( start ) {
		start += strlen(key);
		char *end = strchr(start, ',');
		if( end ) {
			*end = '\0';
			int code = atoi(start);
			*end = ',';
			return code;
		}
	}
	return -1;
}

//解析json数据中指定的key对应的值
static void parse_json_value(char *reply, char **dst, char *key) {
	char *start = strstr(reply, key);
	if ( start ) {
		start += strlen(key);
		char *end = strchr(start, '\"');
		if ( end ) {
			*end = '\0';
			cfg_strcpy(dst, start);
			*end = '\"';
		}
	}
}

//等待获取云网关接口的响应，并解析数据
static void func_parse_cgw(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "+QHTTPREAD:";
		if( ! strlen(reply) || ! strcmp(reply, "CONNECT") ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
			int resultCode = parse_result_code(reply);
			if( resultCode > 0 ) {
				MYLOG("[4G]# server reponse resultCode: %d\n", resultCode);
				MYLOG("\n\n4G_ERR:321\n\n");
			} else {
				parse_json_value(reply, &(env.gwAddress2), "\"gwAddress2\":\"https:\/\/");
				if( strlen(env.gwAddress2) ) {
					MYDEBUG("[4G]cgw_server_address: %s\n", env.gwAddress2);
				}
			}
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 15*1000 ) {
			MYLOG("[4G]# func_parse_cgw: timeout! %d\n", cur_time);
			call_failed();
			if( env.retry_cnt < MAX_4G_RETRY ) {
				env.begin_4g_restart = 1;
				env.retry_cnt ++;
			} else {
				MYLOG("\n\n4G_ERR:329\n\n");
			}
		}
	}
}

//等待注册接口的响应并解析数据
static void func_parse_device_id(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "+QHTTPREAD:";
		if( ! strlen(reply) || ! strcmp(reply, "CONNECT") ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
			int resultCode = parse_result_code(reply);
			if( resultCode > 0 ) {
				MYLOG("[4G]# server reponse resultCode: %d\n", resultCode);
				MYLOG("\n\n4G_ERR:321\n\n");
			} else {
				parse_json_value(reply, &(env.deviceId), "\"deviceId\":\"");
				parse_json_value(reply, &(env.dmToken), "\"dmToken\":\"");
				if( strlen(env.deviceId) >= 27 && strlen(env.deviceId) <= 38) {
					MYDEBUG("[4G] andlink deviceId: %s\n", env.deviceId);
					int ret = save_my4g_config();
					MYDEBUG("[4G] save andlink deviceId: %d\n", ret);

					//设备注册成功，调用设备上线接口
					do_http_device_boot();
				} else {
					MYLOG("[4G]# parse deviceId failed!!\n");
				}
			}
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 15*1000 ) {
			MYLOG("[4G]# func_parse_device_id: timeout! %d\n", cur_time);
			call_failed();
			if( env.retry_cnt < MAX_4G_RETRY ) {
				env.begin_4g_restart = 1;
				env.retry_cnt ++;
			} else {
				MYLOG("\n\n4G_ERR:329\n\n");
			}
		}
	}
}


//等待设备上线接口响应，解析数据
static void func_parse_mqtt_info(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "+QHTTPREAD:";
		if( ! strlen(reply) || ! strcmp(reply, "CONNECT") ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
		int resultCode = parse_result_code(reply);
			if( resultCode > 0 ) {
				MYLOG("[4G]# server reponse resultCode: %d\n", resultCode);
				MYLOG("\n\n4G_ERR:321\n\n");
			} else {
				char *mqttUrl = NULL;
				parse_json_value(reply, &mqttUrl, "\"mqttUrl\":\"ssl:\/\/");
				if( strlen(mqttUrl) ) {
					printf("mqttUrl: %s\n", mqttUrl);
					char *pColon = strchr(mqttUrl, ':');
					*pColon = '\0';
					cfg_strcpy(&(env.mqttServer), mqttUrl);
					cfg_strcpy(&(env.mqttPort), pColon+1);
				}
				//parse_json_value(reply, &(env.mqttClientId), "\"mqttClientId\":\"");
				parse_json_value(reply, &(env.mqttUser), "\"mqttUser\":\"");
				parse_json_value(reply, &(env.mqttPassword), "\"mqttPassword\":\"");
				if( strlen(env.mqttUser) && strlen(env.mqttPassword) ) {
					MYDEBUG("[4G]mqtt: %s %s %s %s\n", env.mqttServer, env.mqttPort, env.mqttUser, env.mqttPassword);
					uint32_t cur_nw_time = get_network_timestamp();
					if( cur_nw_time > env.last_dm_report + 86400 ) {
						do_http_dm_report();
					} else {
						do_mqtt_connect();
					}
				}
			}
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 15*1000 ) {
			MYLOG("[4G]# func_parse_mqtt_info: timeout! %d\n", cur_time);
			call_failed();
			if( env.retry_cnt < MAX_4G_RETRY ) {
				env.begin_4g_restart = 1;
				env.retry_cnt ++;
			} else {
				MYLOG("\n\n4G_ERR:329\n\n");
			}
		}
	}
}

//等待DM接口的响应
static void func_read_dm_report(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		send_loop(params, 0);
	} else if( MY_AT_EVT_UART == evt_typ ) {
		char *ok_reply = "+QHTTPREAD:";
		if( ! strlen(reply) || ! strcmp(reply, "CONNECT") ) {
			//wait
		} else if( ! strncmp(reply, ok_reply, strlen(ok_reply)) ) {
			call_next();
		} else {
		int resultCode = parse_result_code(reply);
			if( resultCode > 0 ) {
				MYLOG("[4G]# server reponse resultCode: %d\n", resultCode);
			}
		}
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		uint32_t cur_time = timer_get_ms();
		if( cur_time - env.last_time > 15*1000 ) {
			MYLOG("[4G]# func_http_post: timeout! %d\n", cur_time);
			call_failed();
			if( env.retry_cnt < MAX_4G_RETRY ) {
				env.begin_4g_restart = 1;
				env.retry_cnt ++;
			} else {
				MYLOG("\n\n4G_ERR:329\n\n");
			}
		}
	}
}

//在获取云网关地址成功之后被调用，先检查设备是否已经注册，如未注册进入设备注册流程，否则跳转设备上线流程
static void func_check_deviceid(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		if( ! strlen(env.gwAddress2) ) {
			MYLOG("[4G]# get cgw failed,  use default!!\n");
			env.gwAddress2 = "cgw.komect.com:443";
		}

		if ( strlen(env.deviceId)>=27 ) {
			do_http_device_boot();
		} else {
			do_http_device_register();
		}
	}
}

//设备终端管理数据上报之后需要继续执行的操作
static void func_after_dm_report(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		save_last_dm_report();
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		env.after_dm_report_cnt ++;
		if( 3 == env.after_dm_report_cnt ) {
			call_next();
			do_mqtt_connect();
		}
	}
}

//串口指令回应
void static reply_uart_command(uint8_t command, uint8_t *data, uint8_t len) {
	if( len > 32) {
		MYDEBUG("[4G]# reply_uart_command, len %d too large!!\n", len);
		return;
	}
	uint8_t buff[36];
	buff[0] = UPCTRL_SOP;
	buff[1] = UPCTRL_SOP2;
	buff[2] = 0x81;
	buff[3] = len + 1;
	buff[4] = command;
	if( len > 0 ) {
		memcpy(buff+5, data, len);
	}
	buff[5+len] = u8CheckXor(buff+1, 4+len);
	uart0_send_buff(buff, 6+len);
}

//处理串口指令
void my_4g_on_uart_command(uint8_t *data, uint8_t len) {
	uint8_t command = data[0];
	switch( command ) {
	case 0x81:
		if( 2 != len ) {
			MYLOG("[4G]# my_4g_on_uart_command, commmand:%d, invalid data len:%d\n", command, len);
			return;
		}
		if( env.deviceSN && strlen(env.deviceSN) ) {
			reply_uart_command(command, env.deviceSN, strlen(env.deviceSN));
			MYLOG("\n[4G]# my_4g_on_uart_command, reply SN:%s\n", env.deviceSN);
		} else {
			reply_uart_command(command, NULL, 0);
			MYLOG("\n[4G]# my_4g_on_uart_command, no SN!!\n");		
		}
		break;
	case 0x82:
		if( 2 != len ) {
			MYLOG("[4G]# my_4g_on_uart_command, commmand:%d, invalid data len:%d\n", command, len);
			return;
		}
		if( env.deviceIMEI && strlen(env.deviceIMEI) ) {
			reply_uart_command(command, env.deviceIMEI, strlen(env.deviceIMEI));
			MYLOG("\n[4G]# my_4g_on_uart_command, reply IMEI:%s\n", env.deviceIMEI);
		} else {
			reply_uart_command(command, NULL, 0);
			MYLOG("\n[4G]# my_4g_on_uart_command, no IMEI!!\n");
		}
		break;
	case 0x83:
		if( ANDLINK_SN_LEN + 1 != len ) {
			MYLOG("[4G]# my_4g_on_uart_command, commmand:%d, invalid data len:%d\n", command, len);
			return;
		}
		data[ANDLINK_SN_LEN+1] = '\0';
		uint8_t ret = save_my4g_config_sn(data+1);
		reply_uart_command(command, &ret, 1);
		MYLOG("\n[4G]# write sn result:%d\n\n", ret);
		if( ! ret ) {
			env.begin_4g_restart = 1;
		}
		break;
	}
}

//处理4G模组的数据
uint8_t my_4g_at_callback(uint8_t *data, uint16_t len, uint8_t in_srv_cb) {
	if( MY_AT_BUSY == env.status ) {
		if( ! in_srv_cb ) {
			MYLOG("[4G]<< `%s` %d\n", data, len);
		}
		env.callback_funcs[env.callback_cur](MY_AT_EVT_UART, NULL, data);
		return 1;
	}
	return 0;
}

//0.5秒定时调用
void my_4g_at_timer() {
	if( env.begin_4g_restart ) {
		do_4g_restart();
	}
	if( MY_AT_BUSY == env.status ) {
		env.callback_funcs[env.callback_cur](MY_AT_EVNT_TIMER, NULL, NULL);
	}
}

//mqtt连接建立成功后执行的操作
static void func_after_mqtt_connect(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		call_next();
		env.begin_4g_restart = 1;
		return;
		if( env.mqtt_wait_report ) {
			do_mqtt_report();
		}
	}
}

//mqtt数据上报完成之后执行的操作
static void func_after_mqtt_report(my_at_evt_typ evt_typ,  char **params, uint8_t *reply) {
	if( MY_AT_EVT_BEGIN == evt_typ ) {
		MYLOG("\n[4G]# mqtt report SUCCESS!!\n\n");
		env.mqtt_wait_report = FALSE;
		env.retry_cnt = 0;
		BtFalshFlag = BTSTATE_FLASH;
		env.bt_flash_cnt = 1;
	} else if( MY_AT_EVNT_TIMER == evt_typ ) {
		env.bt_flash_cnt ++;
		if( 7 == env.bt_flash_cnt ) {
			env.bt_flash_cnt = 0;
			BtFalshFlag = BTSTATE_SHOW;
			call_next();
		}
	}
}

//建立MQTT连接
static void do_mqtt_connect() {
	env.callback_cnt = 0;

	static const char *params1[] = {"AT+QMTCFG=\"recv/mode\",0,0,1",NULL};
	add_func(func_sample_cmd, params1);
	static const char *params2[] = {"AT+QMTCFG=\"ssl\",0,1,2",NULL};
	add_func(func_sample_cmd, params2);
	static const char *params3[] = {"AT+QMTCFG=\"keepalive\",0,60",NULL};
	add_func(func_sample_cmd, params3);

	static const char *params7[] = {
		"{\"eventType\":\"Offline\",\"deviceId\":\"",
		"{DEVICEID}",
		"\",\"timestamp\":",
		"{TIME}",
		"}",
		NULL
	};
	sprintf(env.str_len1, "%d", send_loop(params7, 1));
	static const char *params6[] = {
		"AT+QMTCFG=\"willex\",0,1,0,0,\"/device/",
		"{DEVICEID}",
		"/upward\",",
		"{STR_LEN1}",
		NULL
	};
	add_func(func_mqtt_willex, params6);
	add_func(func_mqtt_will_msg, params7);

	static const char *params8[] = {"AT+QMTOPEN=0,\"","{MQSERVER}","\",","{MQPORT}",NULL};
	add_func(func_mqtt_open, params8);
	static const char *params9[] = {"AT+QMTCONN=0,\"CMCC-590694-","{IMEI}","\",\"","{MQUSER}","\",\"","{MQPASS}","\"",NULL};
	add_func(func_mqtt_conn, params9);

	static const char *s_payload_mboot[] = {
		"{\"eventType\":\"MBoot\",\"deviceId\":\"",
		"{DEVICEID}",
		"\",\"timestamp\":",
		"{TIME}",
		",\"deviceType\":\"590694\",\"childDeviceId\":\"\"}",
		NULL
	};

	sprintf(env.str_len2, "%d", send_loop(s_payload_mboot, 1));
	static const char *params10[] = {"AT+QMTPUBEX=0,0,0,0,\"/device/", "{DEVICEID}", "/upward\",", "{STR_LEN2}", NULL};
	add_func(func_mqtt_pub_topic, params10);

	add_func(func_mqtt_pub_payload, s_payload_mboot);
	add_func(func_after_mqtt_connect, NULL);

	call_first();
}

//MQTT上报数据(对外接口)
void mqtt_report_data(int16_t sbp, uint8_t dbp, uint8_t pr) {
	env.sbp = sbp;
	env.dbp = dbp;
	env.pr = pr;
	env.mqtt_retry_cnt = 0;
	env.mqtt_wait_report = TRUE;
	do_mqtt_report();
}

//MQTT上报数据
static void do_mqtt_report() {
	const pT_Network nw = NW_GetNetwork();
	if( !strlen(nw->IMEI) ) {
		MYLOG("network initialization is incomplete!!");
		MYLOG("\n\n4G_ERR:319\n\n");
		return;
	}
	if( strlen(env.deviceId) < 27 ) {
		MYLOG("[4G]# do_mqtt_report: no deviceId found!!\n");
		return;
	}
	if( MY_AT_BUSY == env.status ) {
		MYLOG("[4G]# do_mqtt_report: env.status is busy!!\n");
		return;
	}
	if( !strlen(env.mqttServer) || !strlen(env.mqttPort) || !strlen(env.mqttUser) || !strlen(env.mqttPassword) ) {
		MYLOG("[4G]# do_mqtt_report:  missing mqtt auth info, %s:%s %s:%s\n", env.mqttServer, env.mqttPort, env.mqttUser, env.mqttPassword);
		return;
	}
	env.callback_cnt = 0;

	static const char *s_payload_inform[] = {
		"{\"deviceId\":\"",
		"{DEVICEID}",
		"\",\"childDeviceId\":\"\",\"eventType\":\"Inform\",\"timestamp\":",
		"{TIME}",
		",\"seqId\":\"001\",\"data\":{\"params\":[{\"paramCode\":\"bloodPressureHigh\",\"paramValue\":",
		"{SBP}",
		"},{\"paramCode\":\"bloodPressureLow\",\"paramValue\":",
		"{DBP}",
		"},{\"paramCode\":\"pulseRate\",\"paramValue\":",
		"{PR}",
		"}]},\"deviceType\":\"590694\"}",
		NULL,
	};

	sprintf(env.str_len1, "%d", send_loop(s_payload_inform, 1));
	static const char *params1[] = {"AT+QMTPUBEX=0,0,0,0,\"/device/", "{DEVICEID}", "/upward\",", "{STR_LEN1}", NULL};
	add_func(func_mqtt_pub_topic, params1);

	add_func(func_mqtt_pub_payload, s_payload_inform);
	add_func(func_after_mqtt_report, NULL);

	call_first();
}

static u8 read_sn() {
	if( env.deviceSN && strlen(env.deviceSN) ) {
		return 0;
	}

	T_My4G_Config my4g_cfg = {0};
	int err = vm_read_my4g_config(&my4g_cfg);
	MYDEBUG("[4G] vm_read_my4g_config, err:%d, %s %d\n", err, my4g_cfg.andlink_device_sn, strlen(my4g_cfg.andlink_device_sn));

	if ( ! err ) {

		MYLOG("[4G]# read SN ok.\n");
		my4g_cfg.andlink_device_sn[ANDLINK_SN_LEN] = '\0';
		cfg_strcpy(&(env.deviceSN), my4g_cfg.andlink_device_sn);
		cfg_strcpy(&(env.deviceId), my4g_cfg.andlink_device_id);
		env.last_dm_report = my4g_cfg.last_dm_report;
		return 0;
	} else {
		return 1;
	}
}

//获取云网关地址
void http_get_cgw() {
	const pT_Network nw = NW_GetNetwork();
	if( read_sn() ) {
		MYLOG("[4G]# http_get_cgw: read sn failed!!\n");
		return;
	}
	if( !strlen(nw->IMEI) ) {
		MYLOG("[4G]# invalid IMEI!!\n");
		return;
	}
	env.deviceIMEI = nw->IMEI;
	//env.deviceIMEI = "862715060000106";

	reply_uart_command(0x81, env.deviceSN, strlen(env.deviceSN));
	reply_uart_command(0x82, env.deviceIMEI, strlen(env.deviceIMEI));

	MYDEBUG("[4G]http_get_cgw...\n");
	env.callback_cnt = 0;
	
	static const char *params0[] = {"AT+CSQ",NULL};
	add_func(func_sample_cmd, params0);
	static const char *params1[] = {"AT+QHTTPCFG=\"contextid\",1",NULL};
	add_func(func_sample_cmd, params1);
	static const char *params2[] = {"AT+QIACT=1",NULL};
	add_func(func_sample_cmd, params2);
	static const char *params3[] = {"AT+QHTTPCFG=\"requestheader\",1",NULL};
	add_func(func_sample_cmd, params3);

	static const char *params4[] = {"AT+QHTTPURL=54,30",NULL};
	add_func(func_wait_connect, params4);
	static const char *params5[] = {"https://andlink.komect.com/espapi/m2m/rest/devices/cgw",NULL};
	add_func(func_sample_cmd, params5);

	static const char *post_content[] = {
		"POST /espapi/m2m/rest/devices/cgw HTTP/1.1\r\n",
		"Host: andlink.komect.com\r\n",
		"Accept: */*\r\n",
		"Accept-Encoding: deflate\r\n",
		"Content-Type: application/json\r\n",
		"Content-Length: 57\r\n",
		"Accept-Charset: utf-8\r\n",
		"\r\n",
		"{\"deviceType\":\"590694\",\"productToken\":\"ApNTSI6OnRM3MI04\"}",
		NULL
	};
	sprintf(env.str_len2, "%d", send_loop(post_content, 1));
	static const char *params6[] = {"AT+QHTTPPOST=", "{STR_LEN2}", ",15,15", NULL};
	add_func(func_wait_connect, params6);
	add_func(func_http_post, post_content);

	static const char *params7[] = {"AT+QHTTPREAD=15", NULL};
	add_func(func_parse_cgw, params7);
	add_func(func_check_deviceid, NULL);

	call_first();
}

//设备注册
static void do_http_device_register() {
	MYDEBUG("[4G]do_http_device_register...\n");
	env.callback_cnt = 0;

	sprintf(env.str_len1, "%d", strlen(env.gwAddress2) + 32);
	static const char *params1[] = {"AT+QHTTPURL=", "{STR_LEN1}", ",30", NULL};
	add_func(func_wait_connect, params1);
	static const char *params2[] = {"https://", "{HOST}", "/device/inform/bootstrap", NULL};
	add_func(func_sample_cmd, params2);

	sprintf(env.str_len2, "%d", 267+strlen(env.deviceSN)*2+strlen(env.deviceIMEI));
	static const char *post_content[] = {
		"POST /device/inform/bootstrap HTTP/1.1\r\n",
		"Host: ",
		"{HOST}",
		"\r\n",
		"Accept: */*\r\n",
		"Accept-Encoding: deflate\r\n",
		"Content-Type: application/json\r\n",
		"Content-Length: ",
		"{STR_LEN2}",
		"\r\n",
		"Accept-Charset: utf-8\r\n",
		"User-Key: CMCC-Western-590694\r\n",
		"\r\n",
		"{\"deviceMac\":\"",
		"{SN}",
		"\",\"productToken\":\"ApNTSI6OnRM3MI04\",",
		"\"deviceType\":\"590694\",\"timestamp\":",
		"{TIME}",
		",\"protocolVersion\":\"V3.2\",\"deviceExtInfo\":{\"cmei\":\"",
		"{IMEI}",
		"\",\"authMode\":0,\"sn\":\"",
		"{SN}",
		"\",\"manuDate\":\"2023-06\",\"OS\":\"NONE\",",
		"\"chips\":[{\"type\":\"BLE\",\"factory\":",
		"\"quectel\",\"model\":\"EC600E\"}]}}",
		NULL
	};
	sprintf(env.str_len3, "%d", send_loop(post_content, 1));
	static const char *params3[] = {"AT+QHTTPPOST=", "{STR_LEN3}", ",15,15", NULL};
	add_func(func_wait_connect, params3);
	add_func(func_http_post, post_content);

	static const char *params4[] = {"AT+QHTTPREAD=15", NULL};
	add_func(func_parse_device_id, params4);

	call_first();
}

//设备上线
static void do_http_device_boot() {
	MYDEBUG("[4G]do_http_device_boot...\n");
	env.callback_cnt = 0;

	sprintf(env.str_len1, "%d", strlen(env.gwAddress2) + 27);
	static const char *params1[] = {"AT+QHTTPURL=", "{STR_LEN1}", ",30", NULL};
	add_func(func_wait_connect, params1);
	static const char *params2[] = {"https://", "{HOST}", "/device/inform/boot", NULL};
	add_func(func_sample_cmd, params2);
	sprintf(env.str_len2, "%d", 247+strlen(env.deviceId));
	static const char *post_content[] = {
		"POST /device/inform/boot HTTP/1.1\r\n",
		"Host: ",
		"{HOST}",
		"\r\n",
		"Accept: */*\r\n",
		"Accept-Encoding: deflate\r\n",
		"Content-Type: application/json\r\n",
		"Content-Length: ",
		"{STR_LEN2}",
		"\r\n",
		"Accept-Charset: utf-8\r\n",
		"User-Key: CMCC-Western-590694\r\n",
		"\r\n",
		"{\"deviceId\":\"",
		"{DEVICEID}",
		"\",\"childDeviceId\":\"\",\"deviceType\":\"590694\",",
		"\"productToken\":\"ApNTSI6OnRM3MI04\",",
		"\"firmwareVersion\":\"V1.0\",\"softwareVersion\":",
		"\"V1.0\",\"protocolVersion\":\"V3.2\",",
		"\"userBind\":\"1\",\"bootType\":\"1\",",
		"\"ipAddress\":\"\",\"timestamp\":",
		"{TIME}",
		",\"XData\":\"\"}",
		NULL
	};
	sprintf(env.str_len3, "%d", send_loop(post_content, 1));
	static const char *params3[] = {"AT+QHTTPPOST=", "{STR_LEN3}", ",15,15", NULL};
	add_func(func_wait_connect, params3);
	add_func(func_http_post, post_content);

	static const char *params4[] = {"AT+QHTTPREAD=15", NULL};
	add_func(func_parse_mqtt_info, params4);

	call_first();
}

//设备终端管理数据上报
static void do_http_dm_report() {
	MYDEBUG("[4G]do_http_dm_report...\n");
	env.callback_cnt = 0;

	sprintf(env.str_len1, "%d", strlen(env.gwAddress2) + 45);
	static const char *params1[] = {"AT+QHTTPURL=", "{STR_LEN1}", ",30", NULL};
	add_func(func_wait_connect, params1);
	static const char *params2[] = {"https://", "{HOST}", "/device-manage/device/inform/dmReport", NULL};
	add_func(func_sample_cmd, params2);
	sprintf(env.str_len2, "%d", 415+strlen(env.deviceId)+strlen(env.deviceIMEI)+strlen(env.deviceSN));
	static const char *post_content[] = {
		"POST /device-manage/device/inform/dmReport HTTP/1.1\r\n",
		"Host: ",
		"{HOST}",
		"\r\n",
		"Accept: */*\r\n",
		"Accept-Encoding: deflate\r\n",
		"Content-Type: application/json\r\n",
		"Content-Length: ",
		"{STR_LEN2}",
		"\r\n",
		"Accept-Charset: utf-8\r\n",
		"User-Key: CMCC-Western-590694\r\n",
		"dmToken: ",
		"{DMTOKEN}",
		"\r\n",
		"\r\n",
		"{\"deviceId\":\"",
		"{DEVICEID}",
		"\",\"childDeviceId\":\"\",\"deviceType\":\"590694\",\"timestamp\":",
		"{TIME}",
		",\"firmwareVersion\":\"V1.0\",\"softwareVersion\":\"V1.0\",\"cmei\":\"",
		"{IMEI}",
		"\",\"mac\":\"NONE\",\"sn\":\"",
		"{SN}",
		"\",\"OS\":\"NONE\",\"cpuModel\":\"NONE\",",
		"\"romStorageSize\":\"NONE\",",
		"\"ramStorageSize\":\"NONE\",\"networkType\":",
		"\"4G\",\"locationInfo\":\"NONE\",",
		"\"deviceBrand\":\"JAMR\",\"deviceModel\":",
		"\"GP-3002\",\"wlanMac\":\"NONE\",",
		"\"powerSupplyMode\":\"battery\",\"deviceIP\":",
		"\"NONE\",\"deviceManageExtInfo\":{}}",
		NULL
	};
	sprintf(env.str_len3, "%d", send_loop(post_content, 1));
	static const char *params3[] = {"AT+QHTTPPOST=", "{STR_LEN3}", ",15,15", NULL};
	add_func(func_wait_connect, params3);
	add_func(func_http_post, post_content);

	static const char *params4[] = {"AT+QHTTPREAD=15", NULL};
	add_func(func_read_dm_report, params4);

	add_func(func_after_dm_report, NULL);

	call_first();
}


#ifndef __4G_AT_H__
#define __4G_AT_H__

#include <stdint.h>
#include <string.h>

#define MAX_4G_RETRY 2

typedef enum
{
	MY_AT_IDLE,
	MY_AT_BUSY
}my_at_status;

typedef enum {
	MY_AT_EVT_BEGIN,
	MY_AT_EVT_UART,
	MY_AT_EVNT_TIMER,
}my_at_evt_typ;

typedef void (*my_at_callback)(my_at_evt_typ,  char **, uint8_t *);

#define MY_AT_CALLBACK_MAX 16

#define CFG_STRING_BUFF_SIZE 220

typedef struct {
	my_at_status status;

	my_at_callback callback_funcs[MY_AT_CALLBACK_MAX];
	char **callback_params[MY_AT_CALLBACK_MAX];
	uint8_t callback_cnt;
	uint8_t callback_cur;

	uint32_t last_time;

	char cfg_string_buff[CFG_STRING_BUFF_SIZE];
	uint16_t cfg_string_idx;
	char str_len1[4];
	char str_len2[4];
	char str_len3[4];

	int16_t sbp;
	uint8_t dbp;
	uint8_t pr;

	char *gwAddress2;
	char *deviceIMEI;
	char *deviceSN;
	char *deviceId;
	char *dmToken;
	char *mqttServer;
	char *mqttPort;
	//char *mqttClientId;
	char *mqttUser;
	char *mqttPassword;

	uint8_t retry_cnt;
	uint8_t begin_4g_restart;
	uint8_t after_dm_report_cnt;
	uint8_t mqtt_retry_cnt;
	uint8_t mqtt_wait_report;
	uint8_t bt_flash_cnt;
	uint32_t last_dm_report;
}T_My4gAtEnv;

typedef struct {
	char andlink_device_sn[24];
	char andlink_device_id[32];
	char andlink_dm_tokne[48];
	uint32_t last_dm_report;
	uint32_t crc;
}T_My4G_Config;

enum {
	HTTP_REQ_CGW = 1,
	HTTP_REQ_REG,
	HTTP_REQ_BOOT,
	HTTP_REQ_DM,
	MQTT_CONNECT,
	MQTT_REPORT,
};

//提供被本模块外程序调用的函数：
// 1、收到4G模组串口数据后调用的入口函数
extern uint8_t my_4g_at_callback(uint8_t *data, uint16_t len, uint8_t in_srv_cb);
// 2、500ms秒计时器调用的入口函数
extern void my_4g_at_timer();
// 3、网络连接成功后调用开始http连接
extern void http_get_cgw();
// 4、上报测量数据接口函数
extern void mqtt_report_data(int16_t sbp, uint8_t dbp, uint8_t pr);
// 5、处理串口指令并反馈
extern void my_4g_on_uart_command(uint8_t *data, uint8_t len);

//调用到的本模块外的函数：
// 1、uart1_send_buff
// 2、NW_GetNetwork
// 3、mktime
// 4、vm_api_read
// 5、bt_off bt_on
// 6、BtFalshFlag

#endif

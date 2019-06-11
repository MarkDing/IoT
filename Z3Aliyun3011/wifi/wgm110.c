#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "app/framework/include/af.h"
#include <kernel/include/os.h>
#include <common/include/rtos_prio.h>

#include "wrappers_defs.h"

#include "api/wifi_bglib.h"
#include "wgm110.h"


//#define WIFI_DEBUG
#ifdef WIFI_DEBUG
//#define WiFi_DbgPrintln emberAfCorePrintln
//#define WiFi_DbgPrint emberAfCorePrint
#define WiFi_DbgPrintln(...) printf("[%s][%d]", __func__, __LINE__);printf(__VA_ARGS__);printf("\r\n");
#define WiFi_DbgPrint(...) /*printf("[%s][%d]", __func__, __LINE__);*/printf(__VA_ARGS__)
#else
#define WiFi_DbgPrintln(...)
#define WiFi_DbgPrint(...)
#endif
#define WiFi_TracePrintln(...) printf("[%s][%d]", __func__, __LINE__);printf(__VA_ARGS__);printf("\r\n");
#define WiFi_ErrPrintln(...) printf("\033[31m[%s][%d]", __func__, __LINE__);printf(__VA_ARGS__);printf("\r\n");printf("\33[37m");

#define DESC(x) #x

#define WIFI_DEFAULT_TIMEOUT 10000

typedef enum
{
    TLS_AUTH_NONE,
    TLS_AUTH_OPTIONAL,
    TLS_AUTH_MANDATORY,
}TLS_AUTH_MODE;

typedef enum
{
    WLAN_OPER_MODE_STA = 1,
    WLAN_OPER_MODE_AP  = 2,
}WLAN_OPER_MODE;

typedef enum
{
    WLAN_AP_SECURITY_NONE = 0,
    WLAN_AP_SECURITY_WPA  = 1,
    WLAN_AP_SECURITY_WPA2 = 2,
    WLAN_AP_SECURITY_WEP  = 3,
}WLAN_AP_SECURITY;

#define WLAN_AP_DEFAULT_CHANNEL 6


typedef enum
{
    WLAN_STATE_IDLE,
    WLAN_STATE_INIT,
    WLAN_STATE_ON,
    WLAN_STATE_CONNECTED,
}WLAN_STATE;

typedef enum
{
    WLAN_OPER_INVALID,
    WLAN_OPER_DNS_RESOLVE,
    WLAN_OPER_TCP_CONNECT,
    WLAN_OPER_TCP_DISCONNECT,
    WLAN_OPER_TCP_WRITE,
    WLAN_OPER_UDP_LISTEN,
    WLAN_OPER_UDP_CONNECT,
    WLAN_OPER_UDP_BIND,
    WLAN_OPER_UDP_TRANSFER_SIZE,
    WLAN_OPER_MULTICAST_JOIN,
    WLAN_OPER_TLS_SET_CERT,
    WLAN_OPER_TLS_SET_AUTHMODE,
    WLAN_OPER_TLS_CONNECT,
    WLAN_OPER_SET_MODE,
    WLAN_OPER_SET_ON,
    WLAN_OPER_SET_AP_PASSWD,
    WLAN_OPER_SET_AP_HIDDEN,
    WLAN_OPER_SET_AP_SRV,
    WLAN_OPER_START_AP,
    WLAN_OPER_STOP_AP,
    WLAN_OPER_SET_STA_PASSWD,
    WLAN_OPER_CONNECT_SSID,
    WLAN_OPER_ERASE_ALL,
}WLAN_REQ_TYPE;


typedef struct
{
    uint8_t   type;    
    uint8_t   done;    
    uint8_t   input[64];    
    uint8_t   output[64];    
    uint16_t  error;    
}wifi_operation;

#define MAX_EP_SIMULTANEOUS_NUM 2
typedef struct
{
    uint8_t   used;    
    uint8_t   endpoint;
    uint16_t  rx_len;    
    uint8_t   rxdata[4096];
}wifi_ep_state;

#define UDP_DATA_UNIT_MAGIC  0xDEFA
#define MAX_UDP_DATA_BUF_LEN 4096
typedef struct
{
    uint16_t  magic;    
    uint16_t  unit_len;    
    uint32_t  ipaddr;
    uint16_t  udpport;
    uint8_t   endpoint;
}wifi_udpunit_hdr;

/**
 * Define BGLIB library
 */
BGLIB_DEFINE();

static void *            g_wifi_taskid = NULL;
static WLAN_STATE        g_wifi_state = WLAN_STATE_IDLE;
static uint8_t           g_uartrx_buffer[BGLIB_MSG_MAXLEN];
static uint8_t           g_local_mac[6] = {0xFF};
static uint32_t          g_local_ipaddr = 0;
static uint16_t          g_local_udpport = 0;
static void *            g_wifi_mutex = NULL;
static void *            g_uart_mutex = NULL;
static void *            g_recv_mutex = NULL;
static wifi_operation    wifi_oper_ctrl;
static wifi_ep_state     g_wifi_ep_state[MAX_EP_SIMULTANEOUS_NUM];
static uint8_t           g_udp_data_buf[MAX_UDP_DATA_BUF_LEN] = {0};
static char              g_wifi_ssid[128] = {0};
static char              g_wifi_passwd[128] = {0};

static int uart_rx(int data_length, unsigned char* data)
{
    uint16_t    bytes_available = 0;
    uint16_t    bytes_read = 0;
    int         cnt = 0;

    if (data_length > sizeof(g_uartrx_buffer)) {
        WiFi_ErrPrintln("Invalid rx len(%d) > max(%d)", data_length, sizeof(g_uartrx_buffer));
        return -1;
    }

	while (cnt < data_length) {
        bytes_available = emberSerialReadAvailable(comPortUsart3);
        while (bytes_available <= 0) {
            HAL_SleepMs(1);
            bytes_available = emberSerialReadAvailable(comPortUsart3);
        }

        if (bytes_available > data_length - cnt) {
            bytes_available = data_length - cnt;
        }
        
		emberSerialReadData(comPortUsart3, &data[cnt], bytes_available, &bytes_read);
        cnt += bytes_read;
	}

    WiFi_DbgPrintln("rx %d bytes", cnt);
    return cnt;
}

static int uart_tx(int data_length, const unsigned char* data)
{
    if (data_length > sizeof(bglib_temp_msg)) {
        WiFi_ErrPrintln("Invalid tx len(%d) > max(%d)", data_length, sizeof(bglib_temp_msg));
        return -1;
    }

    while (EMBER_SUCCESS != emberSerialWriteData(comPortUsart3, (uint8_t *)data, (uint16_t)data_length)) {
        HAL_SleepMs(1);
        WiFi_ErrPrintln("trace data_length=%d", data_length);
    }
    
    WiFi_DbgPrintln("tx %d bytes", data_length);
    return data_length;
}

static void wifi_on_message_send(uint8 msg_len, uint8* msg_data, uint16 data_len, uint8* data)
{
    HAL_MutexLock(g_uart_mutex);
    uart_tx(msg_len, msg_data);
    if(data_len && data)
    {
        uart_tx(data_len, data);
    }
    HAL_MutexUnlock(g_uart_mutex);
}

static int wifi_uart_init()
{
    EmberStatus status;

    status = emberSerialInit(comPortUsart3, 115200, PARITY_NONE, 1);
    if (EMBER_SUCCESS != status) {
        WiFi_ErrPrintln("init uart failed");
        return WLAN_ERR_OS;
    }

    g_uart_mutex = HAL_MutexCreate();
    if (NULL == g_uart_mutex) { 
        WiFi_ErrPrintln("create uart mutex failed");
        return WLAN_ERR_OS;
    }

    WiFi_DbgPrintln("open console OK");
    return 0;
}

static void wifi_uart_deinit()
{
    if (NULL != g_uart_mutex) {
        HAL_MutexDestroy(g_uart_mutex);
    }
}

static void wifi_endpoint_status_change(uint8_t endpoint, bool up)
{
    uint8_t i;
    
    if (up) {
        for (i = 0; i < MAX_EP_SIMULTANEOUS_NUM; i++) {
            if (true == g_wifi_ep_state[i].used && endpoint == g_wifi_ep_state[i].endpoint) {
                break;
            }
        }

        if (i >= MAX_EP_SIMULTANEOUS_NUM) {
            for (i = 0; i < MAX_EP_SIMULTANEOUS_NUM; i++) {
                if (true != g_wifi_ep_state[i].used) {
                    g_wifi_ep_state[i].endpoint = endpoint;
                    g_wifi_ep_state[i].rx_len = 0;
                    g_wifi_ep_state[i].used = true;
                    break;
                }
            }
        }
    } else {
        for (i = 0; i < MAX_EP_SIMULTANEOUS_NUM; i++) {
            if (true == g_wifi_ep_state[i].used && endpoint == g_wifi_ep_state[i].endpoint) {
                g_wifi_ep_state[i].endpoint = 0xFF;
                g_wifi_ep_state[i].rx_len = 0;
                g_wifi_ep_state[i].used = false;
                break;
            }
        }
    }

    
    WiFi_DbgPrintln("ep(%d) %s", endpoint, up ? "up" : "down");
    for (i = 0; i < MAX_EP_SIMULTANEOUS_NUM; i++) {
        if (true == g_wifi_ep_state[i].used) {
            WiFi_DbgPrintln("index(%d) ep(%d)", i, g_wifi_ep_state[i].endpoint);
        }
    }
    return;
}

static int wifi_endpoint_data_enqueue(uint8_t endpoint, uint8_t *pdata, int len)
{
    uint8_t i;
    int copylen = 0;
    
    HAL_MutexLock(g_recv_mutex);
    for (i = 0; i < MAX_EP_SIMULTANEOUS_NUM; i++) {
        if (true == g_wifi_ep_state[i].used && endpoint == g_wifi_ep_state[i].endpoint) {
            //WiFi_DbgPrintln("ep(%d) %d bytes", endpoint, g_wifi_ep_state[i].rx_len);
            if (g_wifi_ep_state[i].rx_len >= sizeof(g_wifi_ep_state[i].rxdata)) {
                WiFi_ErrPrintln("endpoint %d buf full", endpoint);
            } else {
                copylen = len;
                if (copylen + g_wifi_ep_state[i].rx_len >= sizeof(g_wifi_ep_state[i].rxdata)) {
                    copylen = sizeof(g_wifi_ep_state[i].rxdata) - g_wifi_ep_state[i].rx_len;
                }
                memcpy(&g_wifi_ep_state[i].rxdata[g_wifi_ep_state[i].rx_len],
                       pdata,
                       copylen);
                g_wifi_ep_state[i].rx_len += copylen;
                WiFi_DbgPrintln("ep(%d) enqueue %d bytes", endpoint, copylen);
            }

            break;
        }
    }
    HAL_MutexUnlock(g_recv_mutex);
    return copylen;
}

static int wifi_endpoint_data_dequeue(uint8_t endpoint, uint8_t *pdata, int len)
{
    uint8_t i;
    int copylen = 0;

    HAL_MutexLock(g_recv_mutex);
    for (i = 0; i < MAX_EP_SIMULTANEOUS_NUM; i++) {
        if (true == g_wifi_ep_state[i].used && endpoint == g_wifi_ep_state[i].endpoint) {
            //WiFi_DbgPrintln("ep(%d) %d bytes", endpoint, g_wifi_ep_state[i].rx_len);
            if (g_wifi_ep_state[i].rx_len > 0) {
                if (g_wifi_ep_state[i].rx_len <= len) {
                    copylen = g_wifi_ep_state[i].rx_len;
                    memcpy(pdata, &g_wifi_ep_state[i].rxdata[0], copylen);
                    g_wifi_ep_state[i].rx_len = 0;
                } else {
                    memcpy(pdata, &g_wifi_ep_state[i].rxdata[0], len);
                    memcpy(&g_wifi_ep_state[i].rxdata[0], &g_wifi_ep_state[i].rxdata[len], g_wifi_ep_state[i].rx_len - len);
                    g_wifi_ep_state[i].rx_len -= len;
                    copylen = len;
                }
                
                WiFi_DbgPrintln("ep(%d) dequeue %d bytes", endpoint, copylen);
            }

            break;
        }
    }
    HAL_MutexUnlock(g_recv_mutex);
    return copylen;
}

static void wifi_empty_udpdata_buf(uint8_t endpoint)
{
    uint16_t offset = 0;
    uint16_t next = 0;
    uint16_t tail = 0;
    uint16_t bak = 0;
    wifi_udpunit_hdr *punit = NULL;
    
    HAL_MutexLock(g_recv_mutex);
    /*find tail  */
    while (tail + sizeof(wifi_udpunit_hdr) < sizeof(g_udp_data_buf)) {
        punit = (wifi_udpunit_hdr *)&g_udp_data_buf[tail];
        if (punit->magic != UDP_DATA_UNIT_MAGIC) {
            break;
        }

        tail += sizeof(wifi_udpunit_hdr) + punit->unit_len;
    }

    while (offset < tail) {
        punit = (wifi_udpunit_hdr *)&g_udp_data_buf[offset];
        if (punit->magic != UDP_DATA_UNIT_MAGIC) {
            break;
        }

        next = offset + sizeof(wifi_udpunit_hdr) + punit->unit_len;
        if (endpoint == punit->endpoint) {
            if (next < tail) {
                bak = ((wifi_udpunit_hdr *)&g_udp_data_buf[next])->magic;
                ((wifi_udpunit_hdr *)&g_udp_data_buf[next])->magic = 0;
                memcpy(&g_udp_data_buf[offset], &g_udp_data_buf[next], tail - next);
                ((wifi_udpunit_hdr *)&g_udp_data_buf[offset])->magic = bak;
                tail -= sizeof(wifi_udpunit_hdr) + punit->unit_len;
            }
        } else {
            offset = next;
        }
    }
    HAL_MutexUnlock(g_recv_mutex);
    return;
}

static void wifi_udpdata_dumpqueue()
{
    uint16_t i;
    uint16_t offset = 0;
    wifi_udpunit_hdr *punit = NULL;
    
    HAL_MutexLock(g_recv_mutex);
    while (offset + sizeof(wifi_udpunit_hdr) < sizeof(g_udp_data_buf)) {
        punit = (wifi_udpunit_hdr *)&g_udp_data_buf[offset];
        if (punit->magic != UDP_DATA_UNIT_MAGIC) {
            break;
        }

        WiFi_DbgPrintln("udpdata unit endpoint=%d ip=%d.%d.%d.%d port=%d %d bytes", punit->endpoint, 
                                                                               punit->ipaddr & 0xFF, 
                                                                               (punit->ipaddr >> 8) & 0xFF, 
                                                                               (punit->ipaddr >> 16) & 0xFF, 
                                                                               (punit->ipaddr >> 24) & 0xFF, 
                                                                                punit->udpport,
                                                                                punit->unit_len);
        for (i = 0; i < punit->unit_len; i++) {
            if (0 == (i % 16)) {
                WiFi_DbgPrint("\r\n%04X:", i);
            }
            WiFi_DbgPrint("%02X ", g_udp_data_buf[offset + sizeof(wifi_udpunit_hdr) + i]);
        }
        WiFi_DbgPrint("\r\n");
        
        offset += sizeof(wifi_udpunit_hdr) + punit->unit_len;
    }

    HAL_MutexUnlock(g_recv_mutex);
}

static void wifi_udpdata_enqueue(uint8_t endpoint, uint8_t *pdata, int len, uint32_t ipaddr, uint16_t udpport)
{
    uint16_t tail = 0;
    wifi_udpunit_hdr *punit = NULL;
    
    HAL_MutexLock(g_recv_mutex);
    
    /*find tail  */
    while (tail + sizeof(wifi_udpunit_hdr) < sizeof(g_udp_data_buf)) {
        punit = (wifi_udpunit_hdr *)&g_udp_data_buf[tail];
        if (punit->magic != UDP_DATA_UNIT_MAGIC) {
            break;
        }

        tail += sizeof(wifi_udpunit_hdr) + punit->unit_len;
    }

    if (tail + sizeof(wifi_udpunit_hdr) + len > sizeof(g_udp_data_buf)) {
        WiFi_ErrPrintln("buf full endpoint=%d ip=%d.%d.%d.%d port=%d", endpoint, ipaddr & 0xFF, 
                                                                                 (ipaddr >> 8) & 0xFF, 
                                                                                 (ipaddr >> 16) & 0xFF, 
                                                                                 (ipaddr >> 24) & 0xFF, 
                                                                                 udpport);
        HAL_MutexUnlock(g_recv_mutex);
        return;
    }

    punit = (wifi_udpunit_hdr *)&g_udp_data_buf[tail];
    punit->magic = UDP_DATA_UNIT_MAGIC;
    punit->unit_len = len;
    punit->ipaddr = ipaddr;
    punit->udpport = udpport;
    punit->endpoint = endpoint;
    memcpy(&g_udp_data_buf[tail + sizeof(wifi_udpunit_hdr)], pdata, len);

    WiFi_DbgPrintln("tail=%d ep=%d punit->unit_len=%d", tail, punit->endpoint, punit->unit_len);

    tail += sizeof(wifi_udpunit_hdr) + len;
    if (tail + sizeof(wifi_udpunit_hdr) <= sizeof(g_udp_data_buf)) {
        ((wifi_udpunit_hdr *)&g_udp_data_buf[tail])->magic = 0;
    }

    HAL_MutexUnlock(g_recv_mutex);

    WiFi_DbgPrintln("endpoint=%d ip=%d.%d.%d.%d port=%d enqueue %d bytes", endpoint, 
                                                                           ipaddr & 0xFF, 
                                                                           (ipaddr >> 8) & 0xFF, 
                                                                           (ipaddr >> 16) & 0xFF, 
                                                                           (ipaddr >> 24) & 0xFF, 
                                                                            udpport,
                                                                            len);
    wifi_udpdata_dumpqueue();
    return;
}

static int wifi_udpdata_dequeue(uint8_t endpoint, uint8_t *pdata, int len, uint32_t *pipaddr, uint16_t *pudpport)
{
    int      ret = 0;
    int      readlen = 0;
    uint16_t tail = 0;
    uint16_t offset = 0;
    uint16_t next = 0;
    uint16_t bak = 0;
    wifi_udpunit_hdr *punit = NULL;

    wifi_udpdata_dumpqueue();
    
    HAL_MutexLock(g_recv_mutex);
    
    /*find tail  */
    while (tail + sizeof(wifi_udpunit_hdr) < sizeof(g_udp_data_buf)) {
        punit = (wifi_udpunit_hdr *)&g_udp_data_buf[tail];
        if (punit->magic != UDP_DATA_UNIT_MAGIC) {
            break;
        }

        tail += sizeof(wifi_udpunit_hdr) + punit->unit_len;
    }

    while (offset < tail) {
        punit = (wifi_udpunit_hdr *)&g_udp_data_buf[offset];
        if (punit->magic != UDP_DATA_UNIT_MAGIC) {
            break;
        }

        WiFi_DbgPrintln("offset=%d tail=%d ep=%d punit->unit_len=%d", offset, tail, punit->endpoint, punit->unit_len);

        next = offset + sizeof(wifi_udpunit_hdr) + punit->unit_len;
        if (1/*endpoint == punit->endpoint*/) {
            memcpy(pdata, 
                   &g_udp_data_buf[offset + sizeof(wifi_udpunit_hdr)], 
                   (punit->unit_len > len) ? len : punit->unit_len);
            *pipaddr = punit->ipaddr;
            *pudpport = punit->udpport;
            ret = (punit->unit_len > len) ? len : punit->unit_len;

            WiFi_DbgPrintln("endpoint=%d ip=%d.%d.%d.%d port=%d dequeue %d bytes", endpoint, 
                                                                                   *pipaddr & 0xFF, 
                                                                                   (*pipaddr >> 8) & 0xFF, 
                                                                                   (*pipaddr >> 16) & 0xFF, 
                                                                                   (*pipaddr >> 24) & 0xFF, 
                                                                                    *pudpport,
                                                                                    ret);
            /*move next unit forward  */
            if (next < tail) {
                bak = ((wifi_udpunit_hdr *)&g_udp_data_buf[next])->magic;
                ((wifi_udpunit_hdr *)&g_udp_data_buf[next])->magic = 0;
                memcpy(&g_udp_data_buf[offset], &g_udp_data_buf[next], tail - next);
                ((wifi_udpunit_hdr *)&g_udp_data_buf[offset])->magic = bak;
                tail -= sizeof(wifi_udpunit_hdr) + punit->unit_len;
            } else {
                ((wifi_udpunit_hdr *)&g_udp_data_buf[offset])->magic = 0;
            }
            
            readlen += ret;
            if (readlen >= len) {
                break;
            }
        } else {
            offset = next;
        }
    }

    HAL_MutexUnlock(g_recv_mutex);

    wifi_udpdata_dumpqueue();
    return ret;    
}

bool wifi_is_running()
{
    return (WLAN_STATE_INIT <= g_wifi_state) ? true : false;
}

bool wifi_is_ssid_valid()
{
    return (0 == strlen(g_wifi_ssid)) ? false : true;
}

bool wifi_is_connected()
{
    return (WLAN_STATE_CONNECTED <= g_wifi_state) ? true : false;
}

void *wifi_task(void *para)
{
    uint16_t                msg_length;
    uint16_t                pskey;
    struct wifi_cmd_packet *pck;
    
    wifi_cmd_system_reset(0);
    while (1) {
        uart_rx(BGLIB_MSG_HEADER_LEN, g_uartrx_buffer);
        msg_length = BGLIB_MSG_LEN(g_uartrx_buffer);
        if(msg_length)
        {
            uart_rx(msg_length, &g_uartrx_buffer[BGLIB_MSG_HEADER_LEN]);
        }

        pck = BGLIB_MSG(g_uartrx_buffer);
        switch(BGLIB_MSG_ID(g_uartrx_buffer)) {
            case wifi_rsp_system_hello_id:
                WiFi_DbgPrintln("recv %s", DESC(wifi_rsp_system_hello_id));
                break;
            case wifi_evt_system_boot_id:
                WiFi_TracePrintln("WiFi reset");
                WiFi_DbgPrintln("recv %s", DESC(wifi_evt_system_boot_id));
                g_wifi_state = WLAN_STATE_IDLE;
                pskey = FLASH_PS_KEY_CLIENT_SSID;
                wifi_cmd_flash_ps_load(pskey);
                break;
            case wifi_rsp_flash_ps_load_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_flash_ps_load_id), pck->rsp_flash_ps_load.result);
                if (wifi_err_success == pck->rsp_flash_ps_load.result) {
                    if (FLASH_PS_KEY_CLIENT_SSID == pskey) {
                        WiFi_DbgPrintln("ssid len=%d [%s]", pck->rsp_flash_ps_load.value.len, (char *)pck->rsp_flash_ps_load.value.data);
                        if (pck->rsp_flash_ps_load.value.len < sizeof(g_wifi_ssid)) {
                            memcpy(g_wifi_ssid, pck->rsp_flash_ps_load.value.data, pck->rsp_flash_ps_load.value.len);
                        }
                    } else if (FLASH_PS_KEY_CLIENT_PW == pskey) {
                        WiFi_DbgPrintln("passwd len=%d [%s]", pck->rsp_flash_ps_load.value.len, (char *)pck->rsp_flash_ps_load.value.data);
                        if (pck->rsp_flash_ps_load.value.len < sizeof(g_wifi_passwd)) {
                            memcpy(g_wifi_passwd, pck->rsp_flash_ps_load.value.data, pck->rsp_flash_ps_load.value.len);
                        }
                    }
                }
                if (FLASH_PS_KEY_CLIENT_SSID == pskey) {
                    pskey = FLASH_PS_KEY_CLIENT_PW;
                    wifi_cmd_flash_ps_load(pskey);
                } else if (FLASH_PS_KEY_CLIENT_PW == pskey) {
                    wifi_cmd_config_get_mac(0);
                }
                break;
            case wifi_rsp_config_get_mac_id:
                WiFi_DbgPrintln("recv %s result %d", DESC(wifi_rsp_config_get_mac_id), pck->rsp_config_get_mac.result);
                if (wifi_err_success != pck->rsp_config_get_mac.result){
                    wifi_cmd_system_reset(0);
                }
                if (WLAN_STATE_IDLE == g_wifi_state){
                    g_wifi_state = WLAN_STATE_INIT;
                }
                break;
            case wifi_evt_config_mac_address_id:
                WiFi_DbgPrintln("recv %s", DESC(wifi_evt_config_mac_address_id));
                memcpy(g_local_mac, (uint8_t *)pck->evt_config_mac_address.mac.addr, 6);
                break;  
            case wifi_evt_system_power_saving_state_id:
                WiFi_DbgPrintln("recv %s state=%d", DESC(wifi_evt_system_power_saving_state_id), pck->evt_system_power_saving_state.state);
                if (system_power_saving_state_0 != pck->evt_system_power_saving_state.state) {
                    wifi_cmd_system_set_max_power_saving_state(system_power_saving_state_0);
                }
                break;
            case wifi_rsp_system_set_max_power_saving_state_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_system_set_max_power_saving_state_id), pck->rsp_system_set_max_power_saving_state.result);
                if (0 != pck->rsp_system_set_max_power_saving_state.result) {
                    wifi_cmd_system_reset(0);
                }
                break;
            case wifi_rsp_sme_set_operating_mode_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_sme_set_operating_mode_id), pck->rsp_sme_set_operating_mode.result);
                if (WLAN_OPER_SET_MODE == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->rsp_sme_set_operating_mode.result;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_sme_wifi_on_id:
                WiFi_TracePrintln("WiFi on");
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_sme_wifi_on_id), pck->rsp_sme_wifi_on.result);
                if (wifi_err_success != pck->rsp_sme_wifi_on.result) {
                    if (WLAN_OPER_SET_ON == wifi_oper_ctrl.type) {
                        wifi_oper_ctrl.error = pck->rsp_sme_wifi_on.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_evt_sme_wifi_is_on_id:
                WiFi_TracePrintln("WiFi on...ok");
                WiFi_DbgPrintln("recv %s %d", DESC(wifi_evt_sme_wifi_is_on_id), pck->evt_sme_wifi_is_on.result);
                if (WLAN_OPER_SET_ON == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->evt_sme_wifi_is_on.result;
                    wifi_oper_ctrl.done = true;
                }
                
                if (wifi_err_success == pck->evt_sme_wifi_is_on.result) {
                    if (WLAN_STATE_INIT == g_wifi_state){
                        g_wifi_state = WLAN_STATE_ON;
                    }
                }
                break;
            case wifi_rsp_sme_set_ap_password_id:
                WiFi_DbgPrintln("recv %s status=%d", DESC(wifi_rsp_sme_set_ap_password_id), pck->rsp_sme_set_ap_password.status);
                if (WLAN_OPER_SET_AP_PASSWD == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->rsp_sme_set_ap_password.status;
                    wifi_oper_ctrl.done = true;
                }                
                break;
            case wifi_rsp_sme_start_ap_mode_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_sme_start_ap_mode_id), pck->rsp_sme_start_ap_mode.result);
                if (wifi_err_success != pck->rsp_sme_start_ap_mode.result) {
                    if (WLAN_OPER_START_AP == wifi_oper_ctrl.type) {
                        wifi_oper_ctrl.error = pck->rsp_sme_start_ap_mode.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_evt_sme_ap_mode_failed_id:
                WiFi_DbgPrintln("recv %s reason=%d", DESC(wifi_evt_sme_ap_mode_failed_id), pck->evt_sme_ap_mode_failed.reason);
                if (WLAN_OPER_START_AP == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->evt_sme_ap_mode_failed.reason;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_sme_set_ap_hidden_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_sme_set_ap_hidden_id), pck->rsp_sme_set_ap_hidden.result);
                if (WLAN_OPER_SET_AP_HIDDEN == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->rsp_sme_set_ap_hidden.result;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_https_enable_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_https_enable_id), pck->rsp_https_enable.result);
                if (WLAN_OPER_SET_AP_SRV == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->rsp_https_enable.result;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_sme_stop_ap_mode_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_sme_stop_ap_mode_id), pck->rsp_sme_stop_ap_mode.result);
                if (wifi_err_success != pck->rsp_sme_stop_ap_mode.result) {
                    if (WLAN_OPER_STOP_AP == wifi_oper_ctrl.type) {
                        wifi_oper_ctrl.error = pck->rsp_sme_stop_ap_mode.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_evt_sme_ap_mode_started_id:
                WiFi_DbgPrintln("recv %s", DESC(wifi_evt_sme_ap_mode_started_id));
                if (WLAN_OPER_START_AP == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = 0;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_evt_sme_ap_mode_stopped_id:
                WiFi_DbgPrintln("recv %s", DESC(wifi_evt_sme_ap_mode_stopped_id));
                if (WLAN_OPER_STOP_AP == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = 0;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_sme_set_password_id:
                WiFi_TracePrintln("WiFi set password...ok");
                WiFi_DbgPrintln("recv %s status=%d", DESC(wifi_rsp_sme_set_password_id), pck->rsp_sme_set_password.status);
                if (WLAN_OPER_SET_STA_PASSWD == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->rsp_sme_set_password.status;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_sme_connect_ssid_id:
                WiFi_TracePrintln("WiFi connect...");
                WiFi_DbgPrintln("recv %s %d", DESC(wifi_rsp_sme_connect_ssid_id), pck->rsp_sme_connect_ssid.result);
                if (wifi_err_success != pck->rsp_sme_connect_ssid.result) {
                    if (WLAN_OPER_CONNECT_SSID == wifi_oper_ctrl.type) {
                        wifi_oper_ctrl.error = pck->rsp_sme_connect_ssid.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_evt_sme_connected_id:
                WiFi_TracePrintln("WiFi connected");
                WiFi_DbgPrintln("recv %s %d", DESC(wifi_evt_sme_connected_id), pck->evt_sme_connected.status);
                if (WLAN_OPER_CONNECT_SSID == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = 0;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_evt_sme_connect_failed_id:
                WiFi_TracePrintln("WiFi connect fail");
                WiFi_DbgPrintln("recv %s %d", DESC(wifi_evt_sme_connect_failed_id), pck->evt_sme_connect_failed.reason);
                if (WLAN_OPER_CONNECT_SSID == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->evt_sme_connect_failed.reason;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_flash_ps_erase_all_id:
                WiFi_DbgPrintln("recv %s %d", DESC(wifi_rsp_flash_ps_erase_all_id), pck->rsp_flash_ps_erase_all.result);
                if (WLAN_OPER_ERASE_ALL == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.error = pck->rsp_flash_ps_erase_all.result;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_evt_tcpip_configuration_id:
                WiFi_DbgPrintln("recv %s ip=%d.%d.%d.%d", DESC(wifi_evt_tcpip_configuration_id), 
                    pck->evt_tcpip_configuration.address.u & 0xFF,
                    (pck->evt_tcpip_configuration.address.u >> 8) & 0xFF,
                    (pck->evt_tcpip_configuration.address.u >> 16) & 0xFF,
                    (pck->evt_tcpip_configuration.address.u >> 24) & 0xFF);
                g_local_ipaddr = pck->evt_tcpip_configuration.address.u;
                break;
            case wifi_evt_sme_interface_status_id:
                WiFi_TracePrintln("WiFi set interface up");
                WiFi_DbgPrintln("recv %s status %d", DESC(wifi_evt_sme_interface_status_id), pck->evt_sme_interface_status.status);
                if (1 == pck->evt_sme_interface_status.status) {
                    g_wifi_state = WLAN_STATE_CONNECTED;
                } else {
                    //wifi_cmd_system_reset(0);
                }
                break;
            case wifi_rsp_tcpip_dns_gethostbyname_id:
                WiFi_DbgPrintln("recv %s", DESC(wifi_rsp_tcpip_dns_gethostbyname_id));
                if (WLAN_OPER_DNS_RESOLVE == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_tcpip_dns_gethostbyname.result){
                        wifi_oper_ctrl.error = pck->rsp_tcpip_dns_gethostbyname.result;
                        wifi_oper_ctrl.done = true;
                    }
                }                
                break;
            case wifi_evt_tcpip_dns_gethostbyname_result_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_evt_tcpip_dns_gethostbyname_result_id), pck->evt_tcpip_dns_gethostbyname_result.result);
                if (WLAN_OPER_DNS_RESOLVE == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->evt_tcpip_dns_gethostbyname_result.result){
                        wifi_oper_ctrl.error = pck->evt_tcpip_dns_gethostbyname_result.result;
                    } else {
                        wifi_oper_ctrl.error = 0;
                        *(uint32_t *)wifi_oper_ctrl.output = pck->evt_tcpip_dns_gethostbyname_result.address.u;
                    }

                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_tcpip_tcp_connect_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_tcpip_tcp_connect_id), pck->rsp_tcpip_tcp_connect.result);
                if (WLAN_OPER_TCP_CONNECT == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_tcpip_tcp_connect.result){
                        wifi_oper_ctrl.error = pck->rsp_tcpip_tcp_connect.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_rsp_tcpip_tls_connect_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_tcpip_tls_connect_id), pck->rsp_tcpip_tls_connect.result);
                if (WLAN_OPER_TLS_CONNECT == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_tcpip_tls_connect.result){
                        wifi_oper_ctrl.error = pck->rsp_tcpip_tls_connect.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_evt_endpoint_status_id:
                WiFi_DbgPrintln("recv %s active=%d", DESC(wifi_evt_endpoint_status_id), pck->evt_endpoint_status.active);
                wifi_endpoint_status_change(pck->evt_endpoint_status.endpoint, pck->evt_endpoint_status.active);
                if (WLAN_OPER_TCP_DISCONNECT == wifi_oper_ctrl.type && 0 == pck->evt_endpoint_status.active) {
                    if (pck->evt_endpoint_status.endpoint == *(uint8_t *)wifi_oper_ctrl.input) {
                        wifi_oper_ctrl.done = true;
                    }
                } else if (( (WLAN_OPER_TCP_CONNECT == wifi_oper_ctrl.type) ||
                             (WLAN_OPER_UDP_LISTEN == wifi_oper_ctrl.type) ||
                             (WLAN_OPER_UDP_CONNECT == wifi_oper_ctrl.type) ||
                             (WLAN_OPER_TLS_CONNECT == wifi_oper_ctrl.type))
                      && 1 == pck->evt_endpoint_status.active) {
                    wifi_oper_ctrl.error = 0;
                    *(uint8_t *)wifi_oper_ctrl.output = pck->evt_endpoint_status.endpoint;
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_endpoint_close_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_endpoint_close_id), pck->rsp_endpoint_close.result);
                if (WLAN_OPER_TCP_DISCONNECT == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_endpoint_close.result 
                        && pck->rsp_endpoint_close.endpoint == *(uint8_t *)wifi_oper_ctrl.input){
                        wifi_oper_ctrl.error = pck->rsp_endpoint_close.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_rsp_endpoint_send_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_endpoint_send_id), pck->rsp_endpoint_send.result);
                if (WLAN_OPER_TCP_WRITE == wifi_oper_ctrl.type) {
                    if (*(uint8_t *)wifi_oper_ctrl.input == pck->rsp_endpoint_send.endpoint) {
                        if (wifi_err_success != pck->rsp_endpoint_send.result){
                            wifi_oper_ctrl.error = pck->rsp_endpoint_send.result;
                        } else {
                            wifi_oper_ctrl.error = 0;
                        }
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_evt_endpoint_closing_id:
                WiFi_DbgPrintln("recv %s endpoint=%d", DESC(wifi_evt_endpoint_closing_id), pck->evt_endpoint_closing.endpoint);
                wifi_cmd_endpoint_close(pck->evt_endpoint_closing.endpoint);
                break;
            case wifi_evt_endpoint_data_id:
                WiFi_DbgPrintln("recv %s endpoint=%d len=%d", DESC(wifi_evt_endpoint_data_id), 
                                                              pck->evt_endpoint_data.endpoint,
                                                              pck->evt_endpoint_data.data.len);
                wifi_endpoint_data_enqueue(pck->evt_endpoint_data.endpoint, pck->evt_endpoint_data.data.data, pck->evt_endpoint_data.data.len);
                break;
            case wifi_evt_tcpip_udp_data_id:
                WiFi_DbgPrintln("recv %s endpoint=%d len=%d", DESC(wifi_evt_tcpip_udp_data_id), 
                                                              pck->evt_tcpip_udp_data.endpoint,
                                                              pck->evt_tcpip_udp_data.data.len);
                wifi_udpdata_enqueue(pck->evt_tcpip_udp_data.endpoint, 
                                     pck->evt_tcpip_udp_data.data.data, 
                                     pck->evt_tcpip_udp_data.data.len,
                                     pck->evt_tcpip_udp_data.source_address.u,
                                     pck->evt_tcpip_udp_data.source_port);
                break;
            case wifi_rsp_tcpip_start_udp_server_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_tcpip_start_udp_server_id), pck->rsp_tcpip_start_udp_server.result);
                if (WLAN_OPER_UDP_LISTEN == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_tcpip_start_udp_server.result){
                        wifi_oper_ctrl.error = pck->rsp_tcpip_start_udp_server.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break; 
            case wifi_rsp_tcpip_udp_connect_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_tcpip_udp_connect_id), pck->rsp_tcpip_udp_connect.result);
                if (WLAN_OPER_UDP_CONNECT == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_tcpip_udp_connect.result){
                        wifi_oper_ctrl.error = pck->rsp_tcpip_udp_connect.result;
                        wifi_oper_ctrl.done = true;
                    }
                }
                break;
            case wifi_rsp_tcpip_udp_bind_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_tcpip_udp_bind_id), pck->rsp_tcpip_udp_bind.result);
                if (WLAN_OPER_UDP_BIND == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_tcpip_udp_bind.result){
                        wifi_oper_ctrl.error = pck->rsp_tcpip_udp_bind.result;
                    }
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_tcpip_multicast_join_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_tcpip_multicast_join_id), pck->rsp_tcpip_multicast_join.result);
                if (WLAN_OPER_MULTICAST_JOIN == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_tcpip_multicast_join.result){
                        wifi_oper_ctrl.error = pck->rsp_tcpip_multicast_join.result;
                    }
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_endpoint_set_transmit_size_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_endpoint_set_transmit_size_id), pck->rsp_endpoint_set_transmit_size.result);
                if (WLAN_OPER_UDP_TRANSFER_SIZE == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_endpoint_set_transmit_size.result){
                        wifi_oper_ctrl.error = pck->rsp_endpoint_set_transmit_size.result;
                    }
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_tcpip_tls_set_user_certificate_id:
                WiFi_DbgPrintln("recv %s result=%d", DESC(wifi_rsp_tcpip_tls_set_user_certificate_id), pck->rsp_tcpip_tls_set_user_certificate.result);
                if (WLAN_OPER_TLS_SET_CERT == wifi_oper_ctrl.type) {
                    if (wifi_err_success != pck->rsp_tcpip_tls_set_user_certificate.result){
                        wifi_oper_ctrl.error = pck->rsp_tcpip_tls_set_user_certificate.result;
                    }
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_rsp_tcpip_tls_set_authmode_id:
                WiFi_DbgPrintln("recv %s", DESC(wifi_rsp_tcpip_tls_set_authmode_id));
                if (WLAN_OPER_TLS_SET_AUTHMODE == wifi_oper_ctrl.type) {
                    wifi_oper_ctrl.done = true;
                }
                break;
            case wifi_evt_tcpip_tls_verify_result_id:
                WiFi_DbgPrintln("recv %s flags=%d", DESC(wifi_evt_tcpip_tls_verify_result_id), pck->evt_tcpip_tls_verify_result.flags);
                break;
            case wifi_evt_flash_ps_key_changed_id:
            case wifi_evt_tcpip_endpoint_status_id:
            case wifi_evt_sme_connect_retry_id:
            case wifi_evt_tcpip_dns_configuration_id:
                break;
            default:
                WiFi_DbgPrintln("recv unknown msg %lx", BGLIB_MSG_ID(g_uartrx_buffer));
                break;
        }
    }
}

int wifi_init()
{
    int     ret = 0;
    hal_os_thread_param_t   param;

    g_wifi_mutex = HAL_MutexCreate();
    if (NULL == g_wifi_mutex) { 
        WiFi_ErrPrintln("create wifi mutex failed");
        return WLAN_ERR_OS;
    }

    g_recv_mutex = HAL_MutexCreate();
    if (NULL == g_recv_mutex) { 
        WiFi_ErrPrintln("create recv mutex failed");
        HAL_MutexDestroy(g_wifi_mutex);
        return WLAN_ERR_OS;
    }
    
    ret = wifi_uart_init();
    if (0 != ret) {
        WiFi_ErrPrintln("open console fail");
        HAL_MutexDestroy(g_recv_mutex);
        HAL_MutexDestroy(g_wifi_mutex);
        return ret;
    }

    memset(&param, 0, sizeof(param));
    param.priority = 6;
    param.stack_size = 2048;
    param.name = "wifi";

    g_wifi_state = WLAN_STATE_IDLE;
    BGLIB_INITIALIZE(wifi_on_message_send);
    ret = HAL_ThreadCreate(&g_wifi_taskid,
                           wifi_task,
                           NULL,
                           NULL, //&param,
                           NULL);
    if (0 != ret) {
        WiFi_ErrPrintln("create wifi task fail");
        HAL_MutexDestroy(g_recv_mutex);
        HAL_MutexDestroy(g_wifi_mutex);
        wifi_uart_deinit();
        return ret;
    }

    while (true != wifi_is_running()) {
        HAL_SleepMs(100);
    }

    WiFi_DbgPrintln("wifi is running");

    return 0;
}

uint32_t wifi_get_local_ipaddr()
{
    return g_local_ipaddr;
}

int wifi_get_local_mac(uint8_t mac[6])
{
    memcpy(mac, g_local_mac, 6);
    return 0;
}

int wifi_reset()
{
    int      ret = WLAN_ERR_NONE;
    uint64_t start;

    g_wifi_state = WLAN_STATE_IDLE;
    wifi_cmd_system_reset(0);

    /*wait wifi running  */
    start = HAL_UptimeMs();
    while (1) {
        if (true == wifi_is_running()) {
            break;
        }
        
        if (HAL_UptimeMs() - start > WIFI_DEFAULT_TIMEOUT) {
            ret = WLAN_ERR_TIMEOUT;
            WiFi_ErrPrintln("Wifi timeout");
            break;
        }

        HAL_SleepMs(1);
    }

    return ret;
}

int wifi_sync_wait_done(int timeout_ms)
{
    int      ret = WLAN_ERR_NONE;
    uint64_t start;

    start = HAL_UptimeMs();
    while (!wifi_oper_ctrl.done) {
        if (HAL_UptimeMs() - start > timeout_ms) {
            ret = WLAN_ERR_TIMEOUT;
            WiFi_ErrPrintln("Wifi timeout, oper=%d", wifi_oper_ctrl.type);
            break;
        }

        HAL_SleepMs(1);
    }

    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    wifi_oper_ctrl.type = WLAN_OPER_INVALID;
    wifi_oper_ctrl.done = true;
    return ret;
}

int wifi_erase_alldata()
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_ERASE_ALL;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_flash_ps_erase_all();

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_set_operation_mode(WLAN_OPER_MODE mode)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }

    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_SET_MODE;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_sme_set_operating_mode(mode);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_set_wifi_on()
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_SET_ON;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_sme_wifi_on();

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_set_ap_passwd(char *passwd)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_SET_AP_PASSWD;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_sme_set_ap_password(strlen(passwd), passwd);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_start_ap_mode(uint8_t channel, WLAN_AP_SECURITY security, char *ssid)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_START_AP;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_sme_start_ap_mode(channel, security, strlen(ssid), ssid);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_stop_ap_mode()
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_STOP_AP;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_sme_stop_ap_mode();

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_set_ap_hidden(uint8_t hidden)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_SET_AP_HIDDEN;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_sme_set_ap_hidden(hidden);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_set_ap_server(uint8_t http, uint8_t dhcp, uint8_t dns)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_SET_AP_SRV;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_https_enable(http, dhcp, dns);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_start_ap(char *ssid, char *passwd, uint8_t hide)
{
    int      ret = WLAN_ERR_NONE;

    WiFi_DbgPrintln("ssid[%s] passwd[%s] hide[%d]", ssid, passwd, hide);

    ret = wifi_reset();
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("reset fail");
        return ret;
    }

    ret = wifi_set_operation_mode(WLAN_OPER_MODE_AP);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("set operation mode fail");
        return ret;
    }

    ret = wifi_set_wifi_on();
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("set wifi on fail");
        return ret;
    }

    if (1 == hide) {
        ret = wifi_set_ap_hidden(hide);
        if (WLAN_ERR_NONE != ret) {
            WiFi_ErrPrintln("set ap hidden fail");
            return ret;
        }
    }

    ret = wifi_set_ap_passwd(passwd);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("set ap password fail");
        return ret;
    }

    ret = wifi_set_ap_server(0, 1, 0);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("set ap dhcp fail");
        return ret;
    }

    ret = wifi_start_ap_mode(WLAN_AP_DEFAULT_CHANNEL, WLAN_AP_SECURITY_NONE, ssid);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("start ap mode fail");
        return ret;
    }

    return WLAN_ERR_NONE;
}

int wifi_stop_ap()
{
    int      ret = WLAN_ERR_NONE;
    
    ret = wifi_reset();
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("reset fail");
        return ret;
    }

    return WLAN_ERR_NONE;
}

int wifi_set_sta_passwd(char *passwd)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_SET_STA_PASSWD;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_sme_set_password(strlen(passwd), passwd);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_connect_ssid(char *ssid)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_running()) {
        WiFi_ErrPrintln("Wifi not running");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_CONNECT_SSID;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_sme_connect_ssid(strlen(ssid), ssid);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_connect(const char *pssid, const char *ppasswd, int timeout_ms)
{
    int ret = WLAN_ERR_NONE;
    uint64_t start;

    WiFi_ErrPrintln("connect ssid[%s] password[%s]", pssid, ppasswd);

    ret = wifi_reset();
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("reset fail");
        return ret;
    }

    ret = wifi_set_wifi_on();
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("set wifi on fail");
        return ret;
    }

    ret = wifi_set_sta_passwd((char *)ppasswd);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("set passwd fail");
        return ret;
    }

    ret = wifi_connect_ssid((char *)pssid);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("connect fail");
        return ret;
    }

    start = HAL_UptimeMs();
    while (true != wifi_is_connected()) {
        if (HAL_UptimeMs() - start > timeout_ms) {
                ret = WLAN_ERR_TIMEOUT;
                WiFi_ErrPrintln("Wifi timeout");
                break;
            }
        HAL_SleepMs(10);
    }

    ret = wifi_tls_set_auth_mode(TLS_AUTH_OPTIONAL);
    if (0 != ret) {
        WiFi_ErrPrintln("set tls auth mode fail");
        return ret;
    }

    return ret;
}

int wifi_connect_withsaveddata(int timeout_ms)
{
    return wifi_connect(g_wifi_ssid, g_wifi_passwd, timeout_ms);
}

int wifi_hostname_resolve(const char *hostname, uint32_t *p_ipaddr)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_DNS_RESOLVE;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_dns_gethostbyname(strlen(hostname), hostname);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    if (WLAN_ERR_NONE == ret) {
        *p_ipaddr = *(uint32_t *)wifi_oper_ctrl.output;
        WiFi_DbgPrintln("host[%s]-->%d.%d.%d.%d", hostname, *p_ipaddr & 0xFF, 
                                                            (*p_ipaddr >> 8) & 0xFF, 
                                                            (*p_ipaddr >> 16) & 0xFF, 
                                                            (*p_ipaddr >> 24) & 0xFF);
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_tcpip_tcp_connect_byip(uint32_t ipaddr, uint16_t port, uint8_t *pendpoint)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_TCP_CONNECT;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_tcp_connect(ipaddr, port, -1);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    if (WLAN_ERR_NONE == ret) {
        *pendpoint = *(uint8_t *)wifi_oper_ctrl.output;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_tcpip_tcp_connect_byhostname(const char *phostname, uint16_t port, uint8_t *pendpoint)
{
    int      ret;
    uint32_t ipaddr;

    ret = wifi_hostname_resolve(phostname, &ipaddr);
    if (0 != ret) {
        return ret;
    }

    ret = wifi_tcpip_tcp_connect_byip(ipaddr, port, pendpoint);
    if (0 != ret) {
        return ret;
    }

    return 0;
}

int wifi_tcpip_disconnect(uint8_t endpoint)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_TCP_DISCONNECT;
    *(uint8_t *)wifi_oper_ctrl.input = endpoint;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_endpoint_close(endpoint);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;    
}

int wifi_tcpip_write_onemtu(uint8_t endpoint, uint8_t *pdata, int len, int timeout_ms)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }

    //WiFi_DbgPrintln("tcp write %d bytes", len);
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_TCP_WRITE;
    *(uint8_t *)wifi_oper_ctrl.input = endpoint;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_endpoint_send(endpoint, len, pdata);

    ret = wifi_sync_wait_done(timeout_ms);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;      
}

int wifi_tcpip_write(uint8_t endpoint, uint8_t *pdata, int len, int timeout_ms)
{
    int      offset = 0;
    int      send_size = 0;
    int      retry = 0;
    bool     done = false;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }

    WiFi_DbgPrintln("write %d bytes ep=%d timeout_ms=%d", len, endpoint, timeout_ms);
    if (0 == timeout_ms) {
        timeout_ms = WIFI_DEFAULT_TIMEOUT;
    } else if (timeout_ms < 1000) {
        timeout_ms = 1000;
    }

    while (offset < len) {
        send_size = len - offset;
        if (128 < send_size) {
            send_size = 128;
        }

        retry = 0;
        done = false;
        while (retry++ < 3 && true != done) {
            if (0 == wifi_tcpip_write_onemtu(endpoint, pdata + offset, send_size, timeout_ms)) {
                offset += send_size;
                retry = 0;
                done = true;
                break;
            }

            //wifi_cmd_system_hello();
        }

        if (true != done) {
            break;
        }
    }
    
    return offset;        
}

int wifi_tcpip_read(uint8_t endpoint, uint8_t *pdata, int len, int timeout_ms)
{
    int      cnt = 0;
    int      size = 0;
    uint64_t start;

    if (0 == timeout_ms) {
        timeout_ms = WIFI_DEFAULT_TIMEOUT;
    }

    start = HAL_UptimeMs();
    while (cnt < len) {
        if (HAL_UptimeMs() - start > timeout_ms) {
            //WiFi_ErrPrintln("Wifi timeout");
            break;
        }

        size= wifi_endpoint_data_dequeue(endpoint, pdata + cnt, len - cnt);
        if (size <= 0) {
            HAL_SleepMs(1);
            continue;
        } else {
            cnt += size;
        }
    }

    #if  0
    WiFi_DbgPrintln("read %d bytes len=%d ep=%d timeout_ms=%d", cnt, len, endpoint, timeout_ms);
    int i;
    WiFi_DbgPrint("TCP Read:");
    for (i = 0; i < cnt; i++) {
        WiFi_DbgPrint("%02X ", pdata[i]);
    }
    WiFi_DbgPrint("\r\n");
    #endif /* #if 0 */ 
    return cnt;
}

int wifi_tls_set_user_cert(const char *pcert)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_TLS_SET_CERT;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_tls_set_user_certificate(strlen(pcert), pcert);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }
    
    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_tls_set_auth_mode(uint8_t mode)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_TLS_SET_AUTHMODE;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_tls_set_authmode(mode);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }
    
    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_tcpip_tls_connect_byip(uint32_t ipaddr, uint16_t port, uint8_t *pendpoint)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_TLS_CONNECT;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_tls_connect(ipaddr, port, -1);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    if (WLAN_ERR_NONE == ret) {
        *pendpoint = *(uint8_t *)wifi_oper_ctrl.output;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_tcpip_tls_connect_byhostname(const char *phostname, uint16_t port, uint8_t *pendpoint)
{
    int      ret;
    uint32_t ipaddr;

    ret = wifi_hostname_resolve(phostname, &ipaddr);
    if (0 != ret) {
        return ret;
    }

    ret = wifi_tcpip_tls_connect_byip(ipaddr, port, pendpoint);
    if (0 != ret) {
        return ret;
    }

    return 0;
}

int wifi_udp_listen(uint16_t port, uint8_t *p_endpoint)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_UDP_LISTEN;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_start_udp_server(port, -1);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    if (WLAN_ERR_NONE == ret) {
        *p_endpoint = *(uint8_t *)wifi_oper_ctrl.output;
    }

    g_local_udpport = port;
    HAL_MutexUnlock(g_wifi_mutex);
    
    return ret;
}

int wifi_udp_read(uint8_t endpoint, uint8_t *pdata, int len, int timeout_ms, UDP_Addr *psrc)
{
    int      cnt = 0;
    uint64_t start;

    start = HAL_UptimeMs();
    while (cnt < len) {
        cnt += wifi_udpdata_dequeue(endpoint, pdata + cnt, len - cnt, &(psrc->ipaddr), &(psrc->port));

        if (HAL_UptimeMs() - start > timeout_ms) {
            break;
        } else {
            HAL_SleepMs(1);
        }
    }

    return cnt;
}

int wifi_udp_connect(uint32_t ipaddr, uint16_t port, uint8_t *p_endpoint, int timeout_ms)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    if (0 == timeout_ms) {
        timeout_ms = WIFI_DEFAULT_TIMEOUT;
    } else if (timeout_ms < 3000) {
        timeout_ms = 3000;
    }

    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_UDP_CONNECT;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_udp_connect(ipaddr, port, -1);

    ret = wifi_sync_wait_done(timeout_ms);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    if (WLAN_ERR_NONE == ret) {
        *p_endpoint = *(uint8_t *)wifi_oper_ctrl.output;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_udp_bind(uint16_t port, uint8_t endpoint, int timeout_ms)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }

    if (0 == timeout_ms) {
        timeout_ms = WIFI_DEFAULT_TIMEOUT;
    } else if (timeout_ms < 3000) {
        timeout_ms = 3000;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_UDP_BIND;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_udp_bind(endpoint, port);

    ret = wifi_sync_wait_done(timeout_ms);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_udp_set_transfersize(uint8_t endpoint, int size, int timeout_ms)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }

    if (0 == timeout_ms) {
        timeout_ms = WIFI_DEFAULT_TIMEOUT;
    } else if (timeout_ms < 3000) {
        timeout_ms = 3000;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_UDP_TRANSFER_SIZE;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_endpoint_set_transmit_size(endpoint, size);

    ret = wifi_sync_wait_done(timeout_ms);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    wifi_oper_ctrl.type = WLAN_OPER_INVALID;
    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}

int wifi_udp_write(uint8_t endpoint, uint8_t *pdata, int len, int timeout_ms, UDP_Addr *premote)
{
    int      ret = WLAN_ERR_NONE;
    uint8_t  endpoint_new = 255;

    (void)endpoint;

    WiFi_DbgPrintln("wifi_udp_connect to port %d", premote->port);
    ret = wifi_udp_connect(premote->ipaddr, premote->port, &endpoint_new, timeout_ms);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("wifi_udp_connect fail");
        return -1;
    }

    WiFi_DbgPrintln("wifi_udp_bind ep%d to port %d", endpoint_new, g_local_udpport);
    ret = wifi_udp_bind(g_local_udpport, endpoint_new, timeout_ms);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("wifi_udp_bind fail");
        return -1;
    }

    WiFi_DbgPrintln("wifi_udp_set_transfersize ep%d to len %d", endpoint_new, len);
    ret = wifi_udp_set_transfersize(endpoint_new, len, timeout_ms);
    if (WLAN_ERR_NONE != ret) {
        WiFi_ErrPrintln("wifi_udp_set_transfersize fail");
        return -1;
    }

    WiFi_DbgPrintln("wifi_tcpip_udp_write %d bytes to ep%d", len, endpoint_new);
    ret = wifi_tcpip_write(endpoint_new, pdata, len, timeout_ms);
    if (ret <= 0) {
        WiFi_ErrPrintln("wifi_tcpip_write fail");
        return -1;
    }

    wifi_tcpip_disconnect(endpoint_new);
    
    /*WiFi_DbgPrintln("write %d bytes ep=%d", len, endpoint);  */
    return ret;
}

int wifi_tcpip_udp_close(uint8_t endpoint)
{
    if (0 != wifi_tcpip_disconnect(endpoint)) {
        WiFi_ErrPrintln("disconnect fail");
    }

    wifi_empty_udpdata_buf(endpoint);
    return 0;
}

int wifi_tcpip_multicast_join(uint32_t ipaddr)
{
    int      ret = WLAN_ERR_NONE;

    if (!wifi_is_connected()) {
        WiFi_ErrPrintln("Wifi not connected");
        return WLAN_ERR_HW;
    }
    
    HAL_MutexLock(g_wifi_mutex);

    wifi_oper_ctrl.type = WLAN_OPER_MULTICAST_JOIN;
    wifi_oper_ctrl.error = WLAN_ERR_NONE;
    wifi_oper_ctrl.done = false;
    wifi_cmd_tcpip_multicast_join(ipaddr);

    ret = wifi_sync_wait_done(WIFI_DEFAULT_TIMEOUT);
    if (WLAN_ERR_TIMEOUT != ret) {
        ret = wifi_oper_ctrl.error;
    }

    HAL_MutexUnlock(g_wifi_mutex);
    return ret;
}


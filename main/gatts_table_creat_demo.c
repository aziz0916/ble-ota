/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This demo showcases creating a GATT database using a predefined attribute table.
* It acts as a GATT server and can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the  
  att_server_service_table demo.
* Client demo will enable GATT server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"

// for ota 
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "errno.h"

// for light sleep
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "light_sleep_example.h"

// for spiffs
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_spiffs.h"

// for json
#include "cJSON.h"

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "OTA-BLE"
#define SVC_INST_ID                 0
#define SVC_INST_ID1                1 // add the instance id of the service for new service

/* 
   The max length of characteristic value. When the GATT client performs a write or prepare write  
   operation, the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define BLE_WAKEUP_GPIO                   9
#define TEMPERATURE_WAKEUP_GPIO1          10
#define TEMPERATURE_WAKEUP_GPIO2          11
#define TEMPERATURE_WAKEUP_GPIO3          12
#define LOW_BATTERY_WAKEUP_GPIO           13
#define DEVICE_ABNORMAL_WAKEUP_GPIO       14

static uint8_t adv_config_done       = 0;

uint16_t ota_handle_table[HRS_IDX_NB];
uint16_t temperature_handle_table[HRS_IDX_NB2];// add the handle table for new service

static uint8_t wakeup_gpio;

/* 為了處理長特徵值寫入，定義並實例化了一個準備緩衝區結構 */
typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

//#define CONFIG_SET_RAW_ADV_DATA
// 直接定義廣播封包與廣播掃描回應封包內容
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x0f, 0x09, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D','E', 'M', 'O'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else
esp_err_t err;
/* 
   update handle : set by esp_ota_begin(), must be freed via esp_ota_end(); esp_ota_begin()回傳的hundle，後續用於esp_ota_write()和esp_ota_end() 
*/
esp_ota_handle_t update_handle = 0 ;
/*update_partition: 要進行放置更新韌體的分區*/
const esp_partition_t *update_partition = NULL;
	
// 利用參數設定來產生廣播封包與廣播掃描回應封包內容
/*
   Type: com.silabs.service.ota
   UUID: 1D14D6EE-FD63-4FA1-BFA4-8F47B42119F0
*/
/* Silicon Labs OTA service UUID */
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    //0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
    0xf0, 0x19, 0x21, 0xb4, 0x47, 0x8f, 0xa4, 0xbf, 0xa1, 0x4f, 0x63, 0xfd, 0xee, 0xd6, 0x14, 0x1d,
};

/*
   Type: com.silabs.characteristic.ota_control
   UUID: F7BF3564-FB6D-4E53-88A4-5E37E0326063
   Silicon Labs OTA Control.
   Property requirements: 
	   Notify - Excluded
	   Read - Excluded
	   Write Without Response - Excluded
	   Write - Mandatory
	   Reliable write - Excluded
	   Indicate - Excluded
*/
/* Silicon Labs OTA characteristic UUID: OTA Control Point Attribute */
static uint8_t char_ota_control_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    //0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
    0x63, 0x60, 0x32, 0xe0, 0x37, 0x5e, 0xa4, 0x88, 0x53, 0x4e, 0x6d, 0xfb, 0x64, 0x35, 0xbf, 0xf7,
};


/*
   Type: com.silabs.characteristic.ota_data
   UUID: 984227F3-34FC-4045-A5D0-2C581F81A153
   Silicon Labs OTA Data.
   Property requirements: 
	   Notify - Excluded
	   Read - Excluded
	   Write Without Response - Mandatory
	   Write - Mandatory
	   Reliable write - Excluded
	   Indicate - Excluded
*/
/* Silicon Labs OTA characteristic UUID: OTA Data Attribute */
static uint8_t char_ota_data_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    //0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
    0x53, 0xa1, 0x81, 0x1f, 0x58, 0x2c, 0xd0, 0xa5, 0x45, 0x40, 0xfc, 0x34, 0xf3, 0x27, 0x42, 0x98,
};

// 定义BLE广播中的广播数据
/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,// 是否为扫描响应数据，这里设置为广播数据
    .include_name        = true,// 是否包含设备名称
    .include_txpower     = true,// 是否包含广播信号强度值
    .min_interval        = 0x0006,// 从设备在两次广播之间最短的时间间隔，单位为 1.25 毫秒，这里设置为 7.5 毫秒
    .max_interval        = 0x0010,// 从设备在两次广播之间最长的时间间隔，单位为 1.25 毫秒，这里设置为 20 毫秒
    .appearance          = 0x00,// 设备外观，这里设置为默认值 0
    .manufacturer_len    = 0,// 厂商数据长度，这里设置为 0，TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,// 厂商数据指针，这里设置为 NULL
    .service_data_len    = 0,// 服务数据长度，这里设置为 0
    .p_service_data      = NULL,// 服务数据指针，这里设置为 NULL
    .service_uuid_len    = sizeof(service_uuid),// 服务 UUID 长度，这里设置为Silicon Labs OTA service UUID的长度
    .p_service_uuid      = service_uuid,// 服务 UUID 指针，这里设置为Silicon Labs OTA service UUID
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),// 广播标志，这里设置为一般发现模式和不支持 BR/EDR
};

// 定义BLE广播中的扫描响应数据
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,// 标记这个结构体是否用于扫描响应数据（true 为扫描响应数据，false 为广播数据）
    .include_name        = true,// 是否在广播中包含设备名
    .include_txpower     = true,// 是否在广播中包含发送功率
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,// 设备的外观类别
    .manufacturer_len    = 0,// 制造商数据的长度（单位为字节）
    .p_manufacturer_data = NULL,// 指向包含制造商
    .service_data_len    = 0,// 服务数据的长度（单位为字节）
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),// 服务 UUID 长度，这里设置为Silicon Labs OTA service UUID的长度
    .p_service_uuid      = service_uuid,// 服务 UUID 指针，这里设置为Silicon Labs OTA service UUID
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),// 广播标志，可以指定多个广播标志（如通用发现标志和 BR/EDR 不支持标志）。
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

// 配置蓝牙广播的参数
static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,// 广播的最短时间间隔，单位0.625毫秒
    .adv_int_max         = 0x40,// 广播的最长时间间隔
    .adv_type            = ADV_TYPE_IND,// 广播的类型，这里为 ADV_TYPE_IND，表示非定向广播
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,// 是广播自身的地址类型，这里为 BLE_ADDR_TYPE_PUBLIC，表示使用公共地址
    .channel_map         = ADV_CHNL_ALL,// 表示广播的通道，这里为 ADV_CHNL_ALL，表示使用所有通道
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,// 表示广播过滤策略，这里为 ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY，表示不进行任何筛选，允许任何扫描设备或连接设备接收到广播包
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;// GATT Server事件回调函数
    uint16_t gatts_if;// GATT Server接口ID
    uint16_t app_id;// 应用程序ID
    uint16_t conn_id;// 连接ID
    uint16_t service_handle;// 服务句柄
    esp_gatt_srvc_id_t service_id;// 服务ID
    uint16_t char_handle;// 特征句柄
    esp_bt_uuid_t char_uuid;// 特征UUID
    esp_gatt_perm_t perm;// 特征属性权限
    esp_gatt_char_prop_t property;// 特征属性
    uint16_t descr_handle;// 特征描述符句柄
    esp_bt_uuid_t descr_uuid;// 特征描述符UUID
};

// 宣告GATT Profile的事件处理程序
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/*
  结构体数组，包含 gatts_profile_inst 结构体元素，每个元素对应一个 GATT 服务
*/
/* 
  One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT 
*/
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        // 回调函数，用于处理与该服务相关的事件
        // gatts-if參數初始化爲ESP_GATT_IF_NONE，這意味着Application Profile還沒有連接任何客戶端
        .gatts_if = ESP_GATT_IF_NONE,// 表示与该服务关联的 GATT 接口标识符（IF）的整数值。初始化时，该值被设置为 ESP_GATT_IF_NONE，表示尚未获取 GATT IF
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST2      = 0x00EE;// add the service uuid for new service
static const uint16_t GATTS_CHAR_UUID_TEST_A2       = 0xEE01;// add the characteristic uuid for new service's characteristic A2
static const uint16_t GATTS_CHAR_UUID_TEST_B2       = 0xEE02;// add the characteristic uuid for new service's characteristic B2

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;// add the characteristic descriptor uuid for new service's characteristic A2
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;// add this for new service's characteristic A2
static const uint8_t char_prop_read_write          =  ESP_GATT_CHAR_PROP_BIT_READ | 
ESP_GATT_CHAR_PROP_BIT_WRITE;// add this for new service's characteristic B2
static const uint8_t char_prop_write_writenorsp    =  ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t temperature_measurement_ccc[2]      = {0x00, 0x00};// add this for new service's characteristic A2
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

bool create_tab = false;// add this for new service

/* Full Database Description - Used to add attributes into the database */
/* 透過定義esp_gatts_attr_db_t類型的數組來定義GATT服務 */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    // Service Declaration
    /*
       [IDX_SVC]: 屬性索引
       ESP_GATT_AUTO_RSP: BLE堆棧在讀取或寫入事件到達時自動進行響應
       ESP_UUID_LEN_16: UUID長度設置為16-bit
       (uint8_t *)&primary_service_uuid: 將服務標示為主要服務的UUID(0x2800)
       ESP_GATT_PERM_READ: 服務的讀取權限
       sizeof(service_uuid): 服務UUID的最大長度，此處為Silicon Labs OTA service UUID的長度
       sizeof(service_uuid): 當前服務長度設置為Silicon Labs OTA service UUID的長度
       (uint8_t *)&service_uuid: 服務屬性值設置為Silicon Labs OTA service UUID
    */
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(service_uuid), sizeof(service_uuid), (uint8_t *)&service_uuid}},

    /* Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_writenorsp}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&char_ota_control_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},


    /* Characteristic Declaration */
    [IDX_CHAR_B]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_writenorsp}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&char_ota_data_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

};

/*
  the new service
*/
static const esp_gatts_attr_db_t gatt_db2[HRS_IDX_NB2] = 
{
    // Service Declaration
    [IDX_SVC2]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, 
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST2), (uint8_t *)&GATTS_SERVICE_UUID_TEST2}},

    /* Characteristic Declaration */
    [IDX_CHAR_A2]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,   
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A2] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_A2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(temperature_measurement_ccc), (uint8_t *)temperature_measurement_ccc}},

    /* Characteristic Declaration */
    [IDX_CHAR_B2]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, 
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B2]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

};

/*
  处理 BLE GAP事件
*/
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        //广播数据设置完成事件标志
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        // 广播数据设置完毕，当通过 esp_ble_gap_config_adv_data_raw() 设置原始广播数据时，ESP32 蓝牙栈会向应用程序发送这个事件，表示数据已经成功设置到芯片上
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                // 启用广播
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        //广播扫描相应设置完成标志
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        // 设置原始扫描响应数据完成的事件。当使用原始数据格式来设置扫描响应数据时，会触发该事件
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                // 启用广播
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        //广播数据设置完成事件标志
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // 表示广播数据设置完毕的事件
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                // 启用广播
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        //广播扫描相应设置完成标志
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        // 扫描响应数据已成功设置完毕
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                // 启用广播
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        //开始广播事件标志
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            // 通知 BLE 底层主机（host）设备开始广播操作的结果。此事件可以指示广播是否成功启动或失败
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        //停止广播事件标志
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        // 表示停止广播的完成事件
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        //设备连接事件,可获取当前连接的设备信息
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        // 指示 BLE 连接参数已更新的事件
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

/*
  实现 BLE GATT Server 在接收到 characteristic write请求时的处理函数
*/
/* 長特徵值寫入 */
/* 
  客戶端透過發送執行寫入請求來完成長寫入序列。該命令會觸發一個ESP_GATTS_EXEC_WRITE_EVT事件。伺服器透過發送回應並執行函數來處理此事件example_exec_write_event_env()
*/
void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        //為了使用準備緩衝區，為其分配了一些內存空間
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        //如果由於內存不足導致分配失敗，則會打印錯誤
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true 
    發送寫請求或準備寫請求時，服務器是否響應*/
    if (param->write.need_rsp){
        /*準備esp_gatt_rsp_t要發送回客戶端的類型響應。它使用寫入請求的相同參數構造的響應，例如長度、句柄和偏移量。*/
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;//寫入該特性所需的GATT認證類型設置為ESP_GATT_AUTH_REQ_NONE，這意味著客戶端可以寫入該特性而無需先進行身分驗證。
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);//一旦發送響應，分配給它使用的內存就會被釋放。
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    /*最後，傳入的數據被複製到創建的準備緩衝區中，其長度按偏移量遞增*/
    memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value, param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

/*
  处理BLE GATT协议中的Prepare Write操作的回调函数
*/
/* 執行寫入用於確認或取消先前由長特徵寫入過程完成的寫入過程 */
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    //此函數檢查exec_write_flag隨事件接收的參數中的。如果該標誌等於表示的執行標誌exec_write_flag，則確認寫入並將準備緩衝區列印在日誌中；如果不是，則表示取消寫入，並且刪除所有已寫入的資料。
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    // 将之前分配的缓存释放掉，并将缓存长度清零
    //最後，為存儲來自長寫操作的數據塊而創建的準備緩衝區結構進行釋放，並將其指針設置為NULL，以使其為下一個長寫過程做好準備。
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

/*
  GATT Profile的事件处理程序，处理来自 BLE GATT stack 的事件和操作
  @param event: 事件类型
  @param gatts_if: GATT 接口标识符
  @param param: BLE GATT 事件回调参数的指针
*/
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        // 注册事件
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            // 设置原始广播数据
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            // 设置原始扫描响应数据
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            // 设置原始广播数据
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            // 设置原始扫描响应数据
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            // 創建一個服務Attribute表，包含服務與特徵等
            /*
              使用esp_ble_gatts_create_attr_tab來創建屬性表，執行成功後，會進行ESP_GATTS_CREAT_ATTR_TAB_EVT事件，將屬性句柄保存在數組heart_rate_handle_table中。
            */
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        // 读取事件，从外设读取数据
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        // 写请求事件，GATT写事件，手机给开发板的发送数据
        case ESP_GATTS_WRITE_EVT:
            // 不是寫入長特徵值
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                //esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);

				if (ota_handle_table[IDX_CHAR_VAL_A] == param->write.handle && param->write.len == 1){
                    uint8_t value = param->write.value[0];
					ESP_LOGI(GATTS_TABLE_TAG, "ota-control = %d",value);
					if(0x00 == value){
						ESP_LOGI(GATTS_TABLE_TAG, "======beginota======");
                         // 讀取目前正在運行partition內容，例如: type: 0, subtype 16, address: 10000, size: 180000, erase size: 1000, label: ota_0, encrypted: 0
                        //  update_partition = esp_ota_get_running_partition();
                        //  ESP_LOGI(GATTS_TABLE_TAG, "type: %d, subtype %d, address: %lx, size: %lx, erase size: %lx, label: %s, encrypted: %u", update_partition->type, update_partition->subtype, update_partition->address, update_partition->size, update_partition->erase_size, update_partition->label, update_partition->encrypted);
						 // 讀取下一個partition內容，例如: type: 0, subtype 17, address: 190000, size: 180000, erase size: 1000, label: ota_1, encrypted: 0
                         update_partition = esp_ota_get_next_update_partition(NULL);
                         ESP_LOGI(GATTS_TABLE_TAG, "type: %d, subtype %d, address: %lx, size: %lx, erase size: %lx, label: %s, encrypted: %u", update_partition->type, update_partition->subtype, update_partition->address, update_partition->size, update_partition->erase_size, update_partition->label, update_partition->encrypted);
                        /*
                          update_partition->subtype: partition subtype;
                          update_partition->address: starting address of the partition in flash
                        */
 					     ESP_LOGI(GATTS_TABLE_TAG, "Writing to partition subtype %d at offset 0x%lx", update_partition->subtype, update_partition->address);
 					     assert(update_partition != NULL);
                         // OTA_WITH_SEQUENTIAL_WRITES: Used for esp_ota_begin() if new image size is unknown and erase can be done in incremental manner (assuming write operation is in continuous sequence)
						 err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
						  if (err != ESP_OK) {
		                        ESP_LOGE(GATTS_TABLE_TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
		                        esp_ota_abort(update_handle);
		                  }
					}
					else if(0x03 == value){
						ESP_LOGI(GATTS_TABLE_TAG, "======endota======");
						err = esp_ota_end(update_handle);
					    if (err != ESP_OK) {
					        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
					            ESP_LOGE(GATTS_TABLE_TAG, "Image validation failed, image is corrupted");
					        }
					        ESP_LOGE(GATTS_TABLE_TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
					    }

					    err = esp_ota_set_boot_partition(update_partition);
					    if (err != ESP_OK) {
					        ESP_LOGE(GATTS_TABLE_TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
	
					    }
					    ESP_LOGI(GATTS_TABLE_TAG, "Prepare to restart system!");
					    esp_restart();
					    return ;
					}
					
                }
				if (ota_handle_table[IDX_CHAR_VAL_B] == param->write.handle){
                    uint16_t length = param->write.len;// modify uint8_t to uint16_t when mtu larger than 255
					ESP_LOGI(GATTS_TABLE_TAG, "ota-data = %d",length);
					err = esp_ota_write( update_handle, (const void *)param->write.value, length);
		            if (err != ESP_OK) {
		                esp_ota_abort(update_handle);
						ESP_LOGI(GATTS_TABLE_TAG, "esp_ota_write error!");
		            }
                }
                // add notification for new service's characteristic A2
                if (temperature_handle_table[IDX_CHAR_CFG_A2] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        // the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, temperature_handle_table[IDX_CHAR_VAL_A2], sizeof(notify_data), notify_data, false);
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        // the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, temperature_handle_table[IDX_CHAR_VAL_A2], sizeof(indicate_data), indicate_data, true);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                    }
                }

				/* send response when param->write.need_rsp is true */
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            // 寫入長特徵值
            }else{
                /* 
                  handle prepare write 
                  如果寫入常特徵值則執行example_prepare_write_event_env
                */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        // GATT写事件，手机给开发板的发送数据，收到远程设备的Prepare Write Request后，当远程设备完成所有Write请求并发送Execute Write Request时触发的事件
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        // 当GATT客户端和服务器连接并协商MTU大小时的事件
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        // GATT配置事件
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        // GATT 通用属性 服务器成功启动
        case ESP_GATTS_START_EVT:
            if (create_tab == false){
                ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT1, status %d, service_handle %d", param->start.status, param->start.service_handle);
                esp_err_t create_attr_ret1 = esp_ble_gatts_create_attr_tab(gatt_db2, gatts_if, HRS_IDX_NB2, SVC_INST_ID1);
                if (create_attr_ret1){
                    ESP_LOGE(GATTS_TABLE_TAG, "create attr table2 failed, error code = %x", create_attr_ret1);
                }
                create_tab = true;
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT2, status %d, service_handle %d", param->start.status, param->start.service_handle);
            }           
            break;
        // GATT服务器已确认对于某个客户端发送的数据的接收。
        // 表示有一个BLE中央设备连接到该 GATT 服务器
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* 
              For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. 
            */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 1000;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);

            break;
        // 断开连接事件
        // 一个客户端设备断开
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            // esp_ble_gap_start_advertising(&adv_params);
            esp_restart();
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (create_tab == false){
                if (param->add_attr_tab.status != ESP_GATT_OK){
                    ESP_LOGE(GATTS_TABLE_TAG, "create attribute table1 failed, error code=0x%x", param->add_attr_tab.status);
                }
                else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
                    ESP_LOGE(GATTS_TABLE_TAG, "create attribute table1 abnormally, num_handle (%d) \
                            doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
                }
                else {
                    ESP_LOGI(GATTS_TABLE_TAG, "create attribute table1 successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                    memcpy(ota_handle_table, param->add_attr_tab.handles, sizeof(ota_handle_table));
                    esp_ble_gatts_start_service(ota_handle_table[IDX_SVC]);
                }
            }else{
                if (param->add_attr_tab.status != ESP_GATT_OK){
                    ESP_LOGE(GATTS_TABLE_TAG, "create attribute table2 failed, error code=0x%x", param->add_attr_tab.status);
                }
                else if (param->add_attr_tab.num_handle != HRS_IDX_NB2){
                    ESP_LOGE(GATTS_TABLE_TAG, "create attribute table2 abnormally, num_handle (%d) \
                            doesn't equal to HRS_IDX_NB2(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB2);
                }
                else {
                    ESP_LOGI(GATTS_TABLE_TAG, "create attribute table2 successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                    memcpy(temperature_handle_table, param->add_attr_tab.handles, sizeof(temperature_handle_table));
                    esp_ble_gatts_start_service(temperature_handle_table[IDX_SVC2]);
                }
            }
            
            break;
        }
        // GATT 服务器停止
        case ESP_GATTS_STOP_EVT:
        // 表示一个新的客户端连接
        case ESP_GATTS_OPEN_EVT:
        // GATT服务器已经取消连接请求
        case ESP_GATTS_CANCEL_OPEN_EVT:
        // 表示 GATT 服务器已经关闭了一个连接
        case ESP_GATTS_CLOSE_EVT:
        // GATT 已经开始监听请求
        case ESP_GATTS_LISTEN_EVT:
        // GATT因为传输过多数据而处于拥塞状态
        case ESP_GATTS_CONGEST_EVT:
        // GATT 服务器已被注销
        case ESP_GATTS_UNREG_EVT:
        // 删除GATT服务器的服务或属性
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

/*
  将 GATT 服务器事件分发到相应的处理函数中
*/
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    
    // 如果事件类型是注册事件（ESP_GATTS_REG_EVT），则将每个 profile 的 gatts_if 存储起来
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* 
              ESP_GATT_IF_NONE, 调用所有 profile 的回调函数，否则只调用与当前 gatts_if 对应的 profile 的回调函数 
            */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

/*
  將資料寫入SPIFFS
*/
void write_to_spiffs(uint8_t gpio)
{
    vTaskDelay(pdMS_TO_TICKS(2500));
    ESP_LOGI(GATTS_TABLE_TAG, "Initializing SPIFFS");

    // 對spiffs進行配置
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",// 文件系统的目录地址
      .partition_label = NULL,// 在.csv文件中的标签，如果设置为NULL则使用spiffs
      .max_files = 5,// 同时可以打开最大的文件数
      .format_if_mount_failed = true// 如果挂载失败，则格式化文件系统
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    // 注册函数将spiffs 挂载并注册到vfs中
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(GATTS_TABLE_TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(GATTS_TABLE_TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

#ifdef CONFIG_EXAMPLE_SPIFFS_CHECK_ON_START
    ESP_LOGI(GATTS_TABLE_TAG, "Performing SPIFFS_check().");
    ret = esp_spiffs_check(conf.partition_label);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(GATTS_TABLE_TAG, "SPIFFS_check() successful");
    }
#endif

    // 查看spiffs 的信息
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
        esp_spiffs_format(conf.partition_label);
        return;
    } else {
        ESP_LOGI(GATTS_TABLE_TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Check consistency of reported partiton size info.
    if (used > total) {
        ESP_LOGW(GATTS_TABLE_TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
        ret = esp_spiffs_check(conf.partition_label);
        // Could be also used to mend broken files, to clean unreferenced pages, etc.
        // More info at https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
        if (ret != ESP_OK) {
            ESP_LOGE(GATTS_TABLE_TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
            return;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "SPIFFS_check() successful");
        }
    }
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(GATTS_TABLE_TAG, "Opening file");
    FILE* f = fopen("/spiffs/data.json", "a");
    if (f == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to open file for writing");
        return;
    }
    char *str = "";
    switch (gpio) {
        case TEMPERATURE_WAKEUP_GPIO1:
            str = "Temperature1 ≤ 1°C";
            break;
        case TEMPERATURE_WAKEUP_GPIO2:
            str = "Temperature2 ≤ -1°C";
            break;
        case TEMPERATURE_WAKEUP_GPIO3:
            str = "Temperature3 ≤ -5°C";
            break;
        case LOW_BATTERY_WAKEUP_GPIO:
            str = "Low Battery";
            break;
        case DEVICE_ABNORMAL_WAKEUP_GPIO:
            str = "Device Abnormal";
            break;
        default:
            break;
    }
    // 获取自启动以来经过的时间
    int64_t timestamp = esp_timer_get_time();
    cJSON *json_data = cJSON_CreateObject();
    cJSON_AddStringToObject(json_data, "event", str);
    cJSON_AddNumberToObject(json_data, "time after startup(ms)", timestamp / 1000);
    char *str_data = cJSON_Print(json_data);
    fprintf(f, "%s\n", str_data);
    fclose(f);
    cJSON_Delete(json_data);
    free(str_data);
    ESP_LOGI(GATTS_TABLE_TAG, "File written");


     // Open renamed file for reading
    ESP_LOGI(GATTS_TABLE_TAG, "Reading file");
    f = fopen("/spiffs/data.json", "r");
    if (f == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    // 多行讀取
    while (fgets(line, sizeof(line), f) != NULL){
        // strip newline
        char* pos = strchr(line, '\n');
        if (pos) {
            *pos = '\0';
        }
        ESP_LOGI(GATTS_TABLE_TAG, "Read from file: '%s'", line);
    }
    fclose(f);

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(GATTS_TABLE_TAG, "SPIFFS unmounted");
}

void app_main(void)
{
    /* Enable wakeup from light sleep by gpio */
    example_register_gpio_wakeup(); 
    
    printf("Entering light sleep\n");

    // 等待 UART tx 記憶體清空並且最後一個字元發送成功（輪詢模式）
    uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Enter sleep mode */
    esp_light_sleep_start();

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO){
        /* Waiting for the gpio inactive, or the chip will continously trigger wakeup */
        wakeup_gpio = example_wait_gpio_inactive();
        printf("wakeup_gpio = %u\n", wakeup_gpio);
        
        if (wakeup_gpio == BLE_WAKEUP_GPIO){
            esp_err_t ret;

            // 初始化 NVS.
            ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ret = nvs_flash_init();
            }
            ESP_ERROR_CHECK( ret );

            // 释放经典蓝牙模式下的内存
            ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

            // 初始化蓝牙控制器
            esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
            ret = esp_bt_controller_init(&bt_cfg);
            if (ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
                return;
            }

            // 启用蓝牙控制器的 BLE 模式
            ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
            if (ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
                return;
            }

            // 初始化蓝牙协议栈
            ret = esp_bluedroid_init();
            if (ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
                return;
            }

            // 启用蓝牙协议栈
            ret = esp_bluedroid_enable();
            if (ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
                return;
            }

            // 注册 GATT 事件回调函数
            ret = esp_ble_gatts_register_callback(gatts_event_handler);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
                return;
            }

            // 注册 GAP 事件回调函数
            ret = esp_ble_gap_register_callback(gap_event_handler);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
                return;
            }

            // 注册 GATT 应用程序
            ret = esp_ble_gatts_app_register(ESP_APP_ID);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
                return;
            }

            // 配置 GATT 层的 MTU
            esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
            if (local_mtu_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
            }
        }else if (wakeup_gpio == TEMPERATURE_WAKEUP_GPIO1) {
            printf("Temperature1 is equal to or below 1°C\n");
            write_to_spiffs(wakeup_gpio);
            esp_restart();
        }else if (wakeup_gpio == TEMPERATURE_WAKEUP_GPIO2) {
            printf("Temperature2 is equal to or below -1°C\n");
            write_to_spiffs(wakeup_gpio);
            esp_restart();
        }else if (wakeup_gpio == TEMPERATURE_WAKEUP_GPIO3) {
            printf("Temperature3 is equal to or below -5°C\n");
            write_to_spiffs(wakeup_gpio);
            esp_restart();
        }else if (wakeup_gpio == LOW_BATTERY_WAKEUP_GPIO) {
            printf("Low Battery\n");
            write_to_spiffs(wakeup_gpio);
            esp_restart();
        }else if (wakeup_gpio == DEVICE_ABNORMAL_WAKEUP_GPIO) {
            printf("Device Abnormal\n");
            write_to_spiffs(wakeup_gpio);
            esp_restart();
        }else {
            esp_restart();
        }       
    }
}

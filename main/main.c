#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
/* FreeRTOS 相关头文件，用于任务创建和延时。 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* ESP-IDF 基础系统和日志接口。 */
#include "esp_system.h"
#include "esp_log.h"
/* 非易失存储，用来保存蓝牙栈需要的配置数据。 */
#include "nvs_flash.h"
/* 蓝牙控制器相关接口。 */
#include "esp_bt.h"

/* BLE GATT/GAP 相关接口。 */
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
/* 项目自定义的心率逻辑和 LED 控制接口。 */
#include "heart_rate.h"
#include "led.h"
#include "respiration.h"
#include "uart.h"
#include "radar_receiver.h"
#include "buzzer.h"
#include "alarm.h"
#include "key.h"
#include "ota.h"
#include "esp_ota_ops.h"

#define PROFILE_NUM 3
#define HEART_PROFILE_APP_ID 0
#define AUTO_IO_PROFILE_APP_ID 1 //自动 IO 服务的应用 ID，区分不同服务实例
#define RESPIRATION_PROFILE_APP_ID 2 //呼吸服务的应用 ID
#define HEART_RATE_SVC_UUID 0x180D //心率服务 UUID，协议规定为 0x180D
#define HEART_RATE_CHAR_UUID 0x2A37 //心率测量特征 UUID，协议规定为 0x2A37
#define HEART_NUM_HANDLE 4 //心率服务的句柄数量，根据服务、特征值和描述符的数量预留
#define AUTO_IO_SVC_UUID 0x1815 //自动 IO 服务 UUID，协议规定为 0x1815
#define AUTO_IO_NUM_HANDLE 3 //自动 IO 服务的句柄数量，根据服务和特征值的数量预留
#define RESPIRATION_NUM_HANDLE 4 //呼吸服务的句柄数量

#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

/* GATT profile 实例结构体，用来保存每个服务实例的运行时状态。 */
struct gatts_profile_inst 
{
    esp_gatts_cb_t gatts_cb;            // 当前 profile 的事件回调
    uint16_t gatts_if;                  // 该 profile 注册后得到的接口 ID

    uint16_t app_id;                    // 应用 ID
    uint16_t conn_id;                   // 连接 ID

    uint16_t service_handle;            // 服务句柄
    esp_gatt_srvc_id_t service_id;      // 服务 ID

    uint16_t char_handle;               // 特征值句柄
    esp_bt_uuid_t char_uuid;            // 特征值 UUID
    esp_gatt_perm_t perm;               // 特征值权限
    esp_gatt_char_prop_t property;      //特征值属性，如可读、可写、可指示等

    uint16_t descr_handle;              //描述符句柄，如果特征值需要描述符的话
    esp_bt_uuid_t descr_uuid;           //描述符 UUID，通常是 0x2902 用于 CCCD
};

/* 先声明静态函数，避免后面互相调用时出现未定义。 */
static void heart_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void auto_io_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void respiration_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void example_write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const char *GATTS_TAG = "Radar_Communication";// 日志标签，方便在串口输出中区分本示例的信息
static esp_gatt_char_prop_t heart_property = 0;//心率特征属性，后续根据能力组合读/指示等位
static esp_gatt_char_prop_t auto_io_property = 0;//LED 控制特征属性，这里主要用于写入
static esp_gatt_char_prop_t respiration_property = 0;//呼吸特征属性，支持读和指示
static uint8_t heart_rate_val[2] = {0};// 心率值缓存，两个字节分别保存协议中的固定头部和实时心率
static uint8_t led_status[2] = {0};//LED 状态缓存
static uint8_t respiration_val[2] = {0};//呼吸频率值缓存
static bool indicate_enabled = false;// 是否允许发送 indicate
static bool respiration_indicate_enabled = false;// 是否允许发送呼吸频率 indicate
static bool hrs_create_cmpl = false;// 心率服务是否已经创建完成
static bool rsp_create_cmpl = false;// 呼吸服务是否已经创建完成
static uint8_t adv_config_done = 0;//广播配置是否完成

//心率特征的默认属性值，后续会挂到 GATT 属性表里
static esp_attr_value_t heart_rate_attr = 
{
    .attr_max_len = 2,//协议规定心率特征值长度为 2 字节
    .attr_len     = sizeof(heart_rate_val),//当前心率值的实际长度
    .attr_value   = heart_rate_val,//指向心率值缓存的指针，GATT 读/指示时会从这里取数据
};

// LED 特征的默认属性值
static esp_attr_value_t led_status_attr = 
{
    .attr_max_len = 2,
    .attr_len     = sizeof(led_status),
    .attr_value   = led_status,
};

// 呼吸频率特征的默认属性值
static esp_attr_value_t respiration_attr = 
{
    .attr_max_len = 2,
    .attr_len     = sizeof(respiration_val),
    .attr_value   = respiration_val,
};

// 自定义 LED 特征 UUID，使用 128 位 UUID
static const uint8_t led_chr_uuid[] = 
{
    0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00
};

// 自定义呼吸服务 128 位 UUID
static const uint8_t respiration_svc_uuid[] = 
{
    0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x26, 0x15, 0x00, 0x00
};

// 自定义呼吸频率特征 128 位 UUID
static const uint8_t respiration_chr_uuid[] = 
{
    0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x26, 0x16, 0x00, 0x00
};

// 广播数据，包含设备名等基础信息
static esp_ble_adv_data_t adv_data = 
{
    .set_scan_rsp = false,//这部分数据不是扫描响应，而是直接的广播数据
    .include_name = true,//广播数据里包含设备名称，方便手机等客户端识别
    .include_txpower = false,//不包含 TX 功率，节省广播数据空间
    .min_interval = 0x0006,//协议规定的最小连接间隔，单位是 1.25ms，这里设置为 7.5ms
    .max_interval = 0x0010,//协议规定的最大连接间隔，这里设置为 20ms
    .appearance = 0x00,//设备外观，这里不设置，默认为 0
    .manufacturer_len = 0,//不包含厂商数据
    .p_manufacturer_data =  NULL,//厂商数据指针，既然不包含就设置为 NULL
    .service_data_len = 0,//不包含服务数据
    .p_service_data = NULL,//服务数据指针，既然不包含就设置为 NULL
    .service_uuid_len = 0,//不包含服务 UUID 列表
    .p_service_uuid = NULL,//服务 UUID 列表指针，既然不包含就设置为 NULL
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),//广播标志，表示这是一个通用可发现的 BLE 设备，并且不支持 BR/EDR（经典蓝牙）
};

/* 广播参数，决定广播间隔、类型和过滤策略。 */
static esp_ble_adv_params_t adv_params = 
{
    .adv_int_min        = 0x20,  /* 20ms。 */
    .adv_int_max        = 0x40,  /* 40ms。 */
    .adv_type           = ADV_TYPE_IND,//可连接的非定向广播，允许手机等客户端连接
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,//使用公共地址作为设备地址
    .channel_map        = ADV_CHNL_ALL,//在所有三个广告信道上广播，增加被扫描到的概率
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,//允许任何设备扫描和连接，不使用白名单过滤
};

/* 两个 profile 的运行时表，分别对应心率服务和 LED 服务。 */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = 
{//每个 profile 都有自己的事件回调和初始接口状态
    [HEART_PROFILE_APP_ID] = 
    {
        .gatts_cb = heart_gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* 还没注册到具体接口时，先用 NONE。 */
    },
    [AUTO_IO_PROFILE_APP_ID] = 
    {
        .gatts_cb = auto_io_gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* 还没注册到具体接口时，先用 NONE。 */
    },
    [RESPIRATION_PROFILE_APP_ID] = 
    {
        .gatts_cb = respiration_gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* 还没注册到具体接口时，先用 NONE。 */
    },
};

/* 心率更新任务，周期性修改心率值并把新值写入 GATT 属性。 */
static void heart_rate_task(void *param)
{
    (void)param;
    ESP_LOGI(GATTS_TAG, "Heart Rate Task Start");

    /* 任务一直运行，模拟持续变化的心率数据。 */
    while (1) 
    {
        if (hrs_create_cmpl) 
        {
            /* 先更新业务层心率数据，再同步到 GATT 属性值。 */
            update_heart_rate();
            ESP_LOGI(GATTS_TAG, "Heart Rate updated to %d", get_heart_rate());

            heart_rate_val[1] = get_heart_rate();
            esp_ble_gatts_set_attr_value(gl_profile_tab[HEART_PROFILE_APP_ID].char_handle, 2, heart_rate_val);//把新的心率值写入 GATT 属性，这样手机等客户端读或者接收指示时就能拿到最新的心率数据
        }

        /* 每秒刷新一次。 */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* 呼吸频率更新任务，周期性修改呼吸频率值并把新值写入 GATT 属性。 */
static void respiration_task(void *param)
{
    (void)param;
    ESP_LOGI(GATTS_TAG, "Respiration Rate Task Start");

    while (1) 
    {
        if (rsp_create_cmpl) 
        {
            update_respiration_rate();
            ESP_LOGI(GATTS_TAG, "Respiration Rate updated to %d", get_respiration_rate());

            respiration_val[1] = get_respiration_rate();
            esp_ble_gatts_set_attr_value(gl_profile_tab[RESPIRATION_PROFILE_APP_ID].char_handle, 2, respiration_val);
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

/* GAP 事件处理：负责广播数据设置、开始广播和连接参数通知。 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) 
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT://广播数据配置完成事件，通知配置状态
        ESP_LOGI(GATTS_TAG, "Advertising data set, status %d", param->adv_data_cmpl.status);//广播数据配置完成后会触发，通知配置状态
        adv_config_done &= (~ADV_CONFIG_FLAG);//清除广播数据，配置完成标志位
        if (adv_config_done == 0) 
        {
            esp_ble_gap_start_advertising(&adv_params);//广播数据配置完成后，清掉对应标志位，并开始广播
        }
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT://扫描响应数据配置完成事件，通知配置状态
        ESP_LOGI(GATTS_TAG, "Scan response data set, status %d", param->scan_rsp_data_cmpl.status);
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);//清除扫描响应配置完成标志位
        if (adv_config_done == 0) 
        {
            esp_ble_gap_start_advertising(&adv_params);//扫描响应配置完成后，清掉对应标志位，并开始广播
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT://广播开始完成事件，通知广播启动状态
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) 
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT://连接参数更新事件，手机等客户端连接后会触发，通知当前的连接参数状态
        ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT://数据包长度设置完成事件，通知当前的最大传输单元（MTU）大小
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                 param->pkt_data_length_cmpl.status,
                 param->pkt_data_length_cmpl.params.rx_len,
                 param->pkt_data_length_cmpl.params.tx_len);
        break;

    default:
        break;
    }
}

/* 心率服务的 GATT 事件处理函数。 */
static void heart_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) 
    {
    case ESP_GATTS_REG_EVT://GATT 服务注册事件，服务注册完成后会触发，通知注册状态和分配的接口 ID
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d", param->reg.status, param->reg.app_id);
        
        gl_profile_tab[HEART_PROFILE_APP_ID].service_id.is_primary = true;//设置服务为主服务
        gl_profile_tab[HEART_PROFILE_APP_ID].service_id.id.inst_id = 0x00;//实例 ID，区分同一 UUID 的不同服务实例，这里只有一个实例，所以设置为 0   
        gl_profile_tab[HEART_PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;//服务 UUID 长度为 16 位
        gl_profile_tab[HEART_PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = HEART_RATE_SVC_UUID;//服务 UUID，心率服务的标准 UUID 是 0x180D

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);//配置广播数据，设置完成后会触发 ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT 事件
        if (ret) 
        {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            break;
        }

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[HEART_PROFILE_APP_ID].service_id, HEART_NUM_HANDLE);//创建心率服务，句柄数量提前按需要预留，后续会根据服务、特征值和描述符的数量自动分配具体句柄值
        break;

    case ESP_GATTS_CREATE_EVT://GATT 服务创建事件，服务创建完成后会触发，通知创建状态、服务句柄和服务 ID 等信息
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);

        gl_profile_tab[HEART_PROFILE_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[HEART_PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[HEART_PROFILE_APP_ID].char_uuid.uuid.uuid16 = HEART_RATE_CHAR_UUID;
        esp_ble_gatts_start_service(gl_profile_tab[HEART_PROFILE_APP_ID].service_handle);

        heart_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE;//特征值属性，支持读和指示两种操作
        ret = esp_ble_gatts_add_char(gl_profile_tab[HEART_PROFILE_APP_ID].service_handle, &gl_profile_tab[HEART_PROFILE_APP_ID].char_uuid,
                            ESP_GATT_PERM_READ,//特征值权限：只读
                            heart_property, //特征值属性，支持读和指示两种操作
                            &heart_rate_attr, NULL);//特征值属性，包含默认值和长度等信息
        if (ret) 
        {
            ESP_LOGE(GATTS_TAG, "add char failed, error code = %x", ret);
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT://特征值添加事件，特征值添加完成后会触发，通知添加状态、属性句柄和 UUID 等信息
        ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, char_uuid %x",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.char_uuid.uuid.uuid16);
        /* 保存特征值句柄，后面发送指示时要用。 */

        gl_profile_tab[HEART_PROFILE_APP_ID].char_handle = param->add_char.attr_handle;//保存特征值句柄，后面发送指示时要用
        gl_profile_tab[HEART_PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;//描述符 UUID 长度为 16 位
        gl_profile_tab[HEART_PROFILE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;//描述符 UUID，客户端配置描述符的标准 UUID 是 0x2902

        ESP_LOGI(GATTS_TAG, "heart rate char handle %d", param->add_char.attr_handle);
        ret = esp_ble_gatts_add_char_descr(gl_profile_tab[HEART_PROFILE_APP_ID].service_handle, &gl_profile_tab[HEART_PROFILE_APP_ID].descr_uuid,
                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);//添加客户端配置描述符，权限支持读写，属性值由系统维护，不需要提供初始值
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT://描述符添加事件，描述符添加完成后会触发，通知添加状态、属性句柄和 UUID 等信息
        ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %u",
                 param->add_char_descr.status, param->add_char_descr.attr_handle);
        /* 保存描述符句柄，客户端写 CCCD 时会用到。 */
        gl_profile_tab[HEART_PROFILE_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        hrs_create_cmpl = true;
        break;

    case ESP_GATTS_READ_EVT://GATT 读事件，客户端读取特征值时会触发，通知读取句柄和连接信息等
        ESP_LOGI(GATTS_TAG, "Characteristic read");
        /* 读取时把当前心率值组装成响应返回。 */
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 2;
        memcpy(rsp.attr_value.value, heart_rate_val, sizeof(heart_rate_val));
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;

    case ESP_GATTS_WRITE_EVT://GATT 写事件，客户端写特征值或者描述符时会触发，通知写入句柄、数据和连接信息等
        ESP_LOGI(GATTS_TAG, "Characteristic write, value len %u, value ", param->write.len);
        ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);

        if (gl_profile_tab[HEART_PROFILE_APP_ID].descr_handle == param->write.handle && param->write.len == 2) 
        {
            /* 客户端对 CCCD 的写入，用于打开/关闭 notify 或 indicate。 */
            uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
            if (descr_value == 0x0001) 
            {
                if (heart_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) 
                {
                    ESP_LOGI(GATTS_TAG, "Notification enable");
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); i++) 
                    {
                        notify_data[i] = i % 0xff;
                    }
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[HEART_PROFILE_APP_ID].char_handle,
                                            sizeof(notify_data), notify_data, false);//发送通知给客户端，参数最后一个 false 表示这是通知而不是指示，不需要客户端确认。注意通知数据长度要小于当前 MTU，否则发送会失败。
                }
            } 
            else if (descr_value == 0x0002) 
            {
                if (heart_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                {
                    ESP_LOGI(GATTS_TAG, "Indication enable");
                    indicate_enabled = true;
                    uint8_t indicate_data[15];
                    for (int i = 0; i < sizeof(indicate_data); i++) 
                    {
                        indicate_data[i] = i % 0xff;
                    }
                    /* 指示数据长度也要小于当前 MTU。 */
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[HEART_PROFILE_APP_ID].char_handle,
                                            sizeof(indicate_data), indicate_data, true);
                }
            } 
            else if (descr_value == 0x0000) 
            {
                /* 客户端关闭订阅。 */
                indicate_enabled = false;
                ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
            } 
            else 
            {
                ESP_LOGE(GATTS_TAG, "Invalid descriptor value");
                ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
            }
        }
        /* 统一把写事件交给通用应答函数处理。 */
        example_write_event_env(gatts_if, param);
        break;

    case ESP_GATTS_DELETE_EVT://GATT 服务删除事件，服务被删除后会触发，通知删除状态和服务 ID 等信息
        break;

    case ESP_GATTS_START_EVT://GATT 服务启动事件，服务启动完成后会触发，通知启动状态和服务句柄等信息
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_STOP_EVT:
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[HEART_PROFILE_APP_ID].conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        /* 断开后允许再次发送指示。 */
        indicate_enabled = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK) 
        {
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;

    case ESP_GATTS_SET_ATTR_VAL_EVT:
        ESP_LOGI(GATTS_TAG, "Attribute value set, status %d", param->set_attr_val.status);
        if (indicate_enabled) 
        {
            /* 属性值被更新后，如果已订阅，就再推送一次指示给客户端。 */
            uint8_t indicate_data[2] = {0};
            memcpy(indicate_data, heart_rate_val, sizeof(heart_rate_val));
            esp_ble_gatts_send_indicate(gatts_if, gl_profile_tab[HEART_PROFILE_APP_ID].conn_id, gl_profile_tab[HEART_PROFILE_APP_ID].char_handle, sizeof(indicate_data), indicate_data, true);
        }
        break;

    default:
        break;
    }
}

/* LED 控制服务的 GATT 事件处理函数。 */
static void auto_io_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT://GATT 服务注册事件，服务注册完成后会触发，通知注册状态和分配的接口 ID
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d", param->reg.status, param->reg.app_id);
        /* 设置 128 位自定义 LED 服务 UUID。 */
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.is_primary = true;
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = AUTO_IO_SVC_UUID;
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id, AUTO_IO_NUM_HANDLE);
        break;
    case ESP_GATTS_CREATE_EVT:
        /* 服务创建完成后，继续添加特征值。 */
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid.uuid.uuid128, led_chr_uuid, ESP_UUID_LEN_128);

        esp_ble_gatts_start_service(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle);
        /* LED 特征主要支持写入。 */
        auto_io_property = ESP_GATT_CHAR_PROP_BIT_WRITE ;
        esp_err_t ret = esp_ble_gatts_add_char(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle, &gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid,
                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE ,
                            auto_io_property,
                            &led_status_attr, NULL);
        if (ret) 
        {
            ESP_LOGE(GATTS_TAG, "add char failed, error code = %x", ret);
        }
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, char_uuid %x",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.char_uuid.uuid.uuid16);
        /* 保存 LED 特征句柄。 */
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATTS_TAG, "Descriptor add, status %d", param->add_char_descr.status);
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic read");
        /* 读取 LED 状态时返回一个固定值。 */
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = 0x02;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic write, value len %u, value ", param->write.len);
        ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
        if (param->write.len > 0) {
            if (param->write.value[0]) 
            {
                /* 写入非 0，打开 LED。 */
                ESP_LOGI(GATTS_TAG, "LED ON!");
                led_on();
            } else 
            {
                /* 写入 0，关闭 LED。 */
                ESP_LOGI(GATTS_TAG, "LED OFF!");
                led_off();
            }
        } else {
            ESP_LOGW(GATTS_TAG, "Empty write data received");
        }
        /* 写事件统一由通用应答函数补发 ACK。 */
        example_write_event_env(gatts_if, param);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        /* 建立连接后，主动请求更新连接参数。 */
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;// 连接延迟为 0
        conn_params.max_int = 0x20;
        conn_params.min_int = 0x10;
        conn_params.timeout = 400;
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].conn_id = param->connect.conn_id;
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK) {
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    default:
        break;
    }
}

/* 呼吸服务的 GATT 事件处理函数。 */
static void respiration_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_err_t ret;
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration GATT server register, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_id.is_primary = true;
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_id.id.uuid.uuid.uuid128, respiration_svc_uuid, ESP_UUID_LEN_128);
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_id, RESPIRATION_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile_tab[RESPIRATION_PROFILE_APP_ID].char_uuid.uuid.uuid128, respiration_chr_uuid, ESP_UUID_LEN_128);
        esp_ble_gatts_start_service(gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_handle);

        respiration_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE;
        ret = esp_ble_gatts_add_char(gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_handle, &gl_profile_tab[RESPIRATION_PROFILE_APP_ID].char_uuid,
                            ESP_GATT_PERM_READ,
                            respiration_property,
                            &respiration_attr, NULL);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "respiration add char failed, error code = %x", ret);
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration characteristic add, status %d, attr_handle %d",
                 param->add_char.status, param->add_char.attr_handle);
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        ret = esp_ble_gatts_add_char_descr(gl_profile_tab[RESPIRATION_PROFILE_APP_ID].service_handle, &gl_profile_tab[RESPIRATION_PROFILE_APP_ID].descr_uuid,
                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration descriptor add, status %d, attr_handle %u",
                 param->add_char_descr.status, param->add_char_descr.attr_handle);
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        rsp_create_cmpl = true;
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration characteristic read");
        {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 2;
            memcpy(rsp.attr_value.value, respiration_val, sizeof(respiration_val));
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration characteristic write, value len %u, value ", param->write.len);
        ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
        if (gl_profile_tab[RESPIRATION_PROFILE_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
        {
            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            if (descr_value == 0x0002)
            {
                if (respiration_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                {
                    ESP_LOGI(GATTS_TAG, "Respiration indication enable");
                    respiration_indicate_enabled = true;
                    uint8_t indicate_data[2] = {0};
                    memcpy(indicate_data, respiration_val, sizeof(respiration_val));
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[RESPIRATION_PROFILE_APP_ID].char_handle,
                                            sizeof(indicate_data), indicate_data, true);
                }
            }
            else if (descr_value == 0x0000)
            {
                respiration_indicate_enabled = false;
                ESP_LOGI(GATTS_TAG, "Respiration indication disable");
            }
        }
        example_write_event_env(gatts_if, param);
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[RESPIRATION_PROFILE_APP_ID].conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        respiration_indicate_enabled = false;
        break;

    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK)
        {
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;

    case ESP_GATTS_SET_ATTR_VAL_EVT:
        ESP_LOGI(GATTS_TAG, "Respiration attribute value set, status %d", param->set_attr_val.status);
        if (respiration_indicate_enabled)
        {
            uint8_t indicate_data[2] = {0};
            memcpy(indicate_data, respiration_val, sizeof(respiration_val));
            esp_ble_gatts_send_indicate(gatts_if, gl_profile_tab[RESPIRATION_PROFILE_APP_ID].conn_id, gl_profile_tab[RESPIRATION_PROFILE_APP_ID].char_handle, sizeof(indicate_data), indicate_data, false);// 发送指示
        }
        break;

    default:
        break;
    }
}

/* 总入口事件分发器，把事件转发给具体 profile。 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) 
    {
        if (param->reg.status == ESP_GATT_OK) 
        {
            /* 记录每个 app 注册得到的 gatts_if。 */
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;

        } 
        else 
        {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* 遍历所有 profile，根据 gatts_if 选择对应回调。 */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) 
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* 没指定具体接口时，所有 profile 都要收事件。 */
                    gatts_if == gl_profile_tab[idx].gatts_if) 
                    {
                if (gl_profile_tab[idx].gatts_cb) 
                {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } 
    while (0);
}


/* 应用入口，初始化蓝牙、注册服务并启动心率任务。 */
void app_main(void)
{
    esp_err_t ret;

    /* 确认当前 OTA 固件有效，防止 bootloader 回滚。 */
    esp_ota_mark_app_valid_cancel_rollback();

    /* 初始化 LED 硬件。 */
    led_init();

    /* 初始化按键。 */
    key_init();

    /* 初始化 OTA 按键监控 (KEY3 长按 >3s 触发 OTA 升级)。 */
    ota_init();

    /* 初始化蜂鸣器 (GPIO37, 低电平触发)。 */
    buzzer_init();

    /* 初始化报警模块。 */
    alarm_init();

    /* 初始化 UART1，GPIO17=TX, GPIO18=RX, 波特率115200。 */
    uart1_init();

    /* 初始化雷达数据接收器 (均值滤波 + 5 帧队列)。 */
    radar_receiver_init();

    /* 初始化 NVS，蓝牙栈依赖它存储部分运行数据。 */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 释放经典蓝牙占用的控制器内存，只保留 BLE。 */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* 初始化蓝牙控制器。 */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* 启动 BLE 模式。 */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* 初始化 Bluedroid 协议栈。 */
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* 使能 Bluedroid 协议栈。 */
    ret = esp_bluedroid_enable();
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }


    esp_err_t name_ret = esp_ble_gap_set_device_name("Health_Device");
    if(name_ret != ESP_OK)
    {
        ESP_LOGE(GATTS_TAG, "Set device name failed");
    }

    /* 注册 GAP 事件回调。 */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }

    /* 注册 GATT Server 事件回调。 */
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    /* 注册心率服务 profile。 */
    ret = esp_ble_gatts_app_register(HEART_PROFILE_APP_ID);
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "app register error, error code = %x", ret);
        return;
    }

    /* 注册 LED 控制服务 profile。 */
    ret = esp_ble_gatts_app_register(AUTO_IO_PROFILE_APP_ID);
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "app register error, error code = %x", ret);
        return;
    }

    /* 注册呼吸服务 profile。 */
    ret = esp_ble_gatts_app_register(RESPIRATION_PROFILE_APP_ID);
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "respiration app register error, error code = %x", ret);
        return;
    }

    /* 调整本地 MTU，给后续特征通信留出更大空间。 */
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) 
    {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", ret);
    }

    /* 启动心率更新任务。 */
    xTaskCreate(heart_rate_task, "Heart Rate", 2 * 1024, NULL, 5, NULL);

    /* 启动呼吸频率更新任务。 */
    xTaskCreate(respiration_task, "Respiration", 2 * 1024, NULL, 5, NULL);

    /* 启动报警监控任务 (KEY2 开关 + 心率/呼吸异常检测 + 蜂鸣器控制)。 */
    xTaskCreate(alarm_task, "Alarm", 2 * 1024, NULL, 5, NULL);
}

/* 写事件的通用处理：如果客户端需要响应，就回一个 ACK。 */
void example_write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp) 
    {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
    }
}
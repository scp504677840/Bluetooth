/* ESP32-S3 BLE GATT Server 完整示例
 * 基于 ESP-IDF v5.5.1
 * 功能: BLE 服务器，支持读写特征
 */

#include "esp_log.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#define GATTS_TAG "BLE_DEMO"
#define DEVICE_NAME "ESP32-S3-DEV"
#define LED_GPIO GPIO_NUM_38  // ESP32-S3-DevKitC-1 板载 RGB LED

// 服务和特征 UUID
static const uint16_t GATTS_SERVICE_UUID = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TX = 0xFF01;  // 用于发送数据（notify）
static const uint16_t GATTS_CHAR_UUID_RX = 0xFF02;  // 用于接收数据（write）

// GATT 数据库索引
enum {
    IDX_SVC,
    IDX_CHAR_TX,
    IDX_CHAR_VAL_TX,
    IDX_CHAR_CFG_TX,        // 客户端配置描述符（用于 notify）
    IDX_CHAR_RX,
    IDX_CHAR_VAL_RX,
    HRS_IDX_NB,
};

static uint16_t heart_rate_handle_table[HRS_IDX_NB];
static uint16_t conn_id = 0xFFFF;
static esp_gatt_if_t gatts_if_global = ESP_GATT_IF_NONE;

// UUID 定义
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 64

// GATT 属性数据库
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
    // 服务声明
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
         ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID),
         (uint8_t *)&GATTS_SERVICE_UUID}
    },

    // TX 特征声明
    [IDX_CHAR_TX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
         ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t),
         (uint8_t *)&char_prop_read_notify}
    },

    // TX 特征值
    [IDX_CHAR_VAL_TX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TX,
         ESP_GATT_PERM_READ, GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, NULL}
    },

    // TX 客户端配置描述符（用于启用通知）
    [IDX_CHAR_CFG_TX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), 0, NULL}
    },

    // RX 特征声明
    [IDX_CHAR_RX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
         ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t),
         (uint8_t *)&char_prop_write}
    },

    // RX 特征值
    [IDX_CHAR_VAL_RX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_RX,
         ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, NULL}
    },
};

// 广播数据
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(GATTS_SERVICE_UUID),
    .p_service_uuid = (uint8_t *)&GATTS_SERVICE_UUID,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// 广播参数
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// LED 初始化
static void led_init(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI(GATTS_TAG, "LED initialized on GPIO %d", LED_GPIO);
}

// 发送通知函数
static void send_notify(uint8_t *data, uint16_t len)
{
    if (conn_id != 0xFFFF && gatts_if_global != ESP_GATT_IF_NONE) {
        esp_ble_gatts_send_indicate(
            gatts_if_global,
            conn_id,
            heart_rate_handle_table[IDX_CHAR_VAL_TX],
            len,
            data,
            false
        );
        ESP_LOGI(GATTS_TAG, "Notification sent, len=%d", len);
    }
}

// GAP 事件处理
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Advertisement data set, starting advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(GATTS_TAG, "Advertising started successfully");
        } else {
            ESP_LOGE(GATTS_TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Advertising stopped");
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "Connection params updated: "
                 "min_int=%d, max_int=%d, latency=%d, timeout=%d",
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    default:
        break;
    }
}

// GATTS 事件处理
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server registered, status=%d, app_id=%d",
                 param->reg.status, param->reg.app_id);
        
        // 设置设备名称
        esp_ble_gap_set_device_name(DEVICE_NAME);
        
        // 配置广播数据
        esp_ble_gap_config_adv_data(&adv_data);
        
        // 创建属性表
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            ESP_LOGI(GATTS_TAG, "Attribute table created, num_handle=%d",
                     param->add_attr_tab.num_handle);
            
            // 保存句柄表
            memcpy(heart_rate_handle_table, param->add_attr_tab.handles,
                   sizeof(heart_rate_handle_table));
            
            // 启动服务
            esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
            
            gatts_if_global = gatts_if;
        } else {
            ESP_LOGE(GATTS_TAG, "Attribute table creation failed: %d",
                     param->add_attr_tab.status);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Client connected, conn_id=%d, remote "ESP_BD_ADDR_STR"",
                 param->connect.conn_id,
                 ESP_BD_ADDR_HEX(param->connect.remote_bda));
        
        conn_id = param->connect.conn_id;
        
        // 更新连接参数
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20;
        conn_params.min_int = 0x10;
        conn_params.timeout = 400;
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Client disconnected, reason=0x%x",
                 param->disconnect.reason);
        conn_id = 0xFFFF;
        
        // 重新开始广播
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep) {
            ESP_LOGI(GATTS_TAG, "GATT write event, handle=%d, len=%d",
                     param->write.handle, param->write.len);
            
            // 打印接收到的数据
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
            
            // 如果是 RX 特征，处理 LED 控制命令
            if (param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_RX]) {
                if (param->write.len > 0) {
                    uint8_t cmd = param->write.value[0];
                    
                    if (cmd == 0x01) {
                        // LED ON
                        gpio_set_level(LED_GPIO, 1);
                        ESP_LOGI(GATTS_TAG, "LED turned ON");
                        
                        // 发送响应
                        uint8_t response[] = "LED ON";
                        send_notify(response, sizeof(response) - 1);
                        
                    } else if (cmd == 0x00) {
                        // LED OFF
                        gpio_set_level(LED_GPIO, 0);
                        ESP_LOGI(GATTS_TAG, "LED turned OFF");
                        
                        // 发送响应
                        uint8_t response[] = "LED OFF";
                        send_notify(response, sizeof(response) - 1);
                    }
                }
            }
            
            // 如果需要响应
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                           param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "GATT read event, handle=%d", param->read.handle);
        
        // 自动响应已在属性表中配置
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "MTU exchange, MTU=%d", param->mtu.mtu);
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service started");
        break;

    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(GATTS_TAG, "ESP32-S3 BLE GATT Server Starting...");

    // 初始化 LED
    led_init();

    // 初始化 NVS（非易失性存储）
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(GATTS_TAG, "NVS initialized");

    // 释放经典蓝牙内存（ESP32-S3 仅支持 BLE）
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    // 启用 BLE 模式
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(GATTS_TAG, "Bluetooth controller enabled");

    // 初始化 Bluedroid 协议栈
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(GATTS_TAG, "Bluedroid enabled");

    // 注册 GAP 回调
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return;
    }

    // 注册 GATTS 回调
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GATTS callback register failed: %s", esp_err_to_name(ret));
        return;
    }

    // 注册应用
    ret = esp_ble_gatts_app_register(0);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return;
    }

    // 设置 MTU
    ret = esp_ble_gatt_set_local_mtu(517);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(GATTS_TAG, "BLE GATT Server started successfully!");
    ESP_LOGI(GATTS_TAG, "Device name: %s", DEVICE_NAME);
    ESP_LOGI(GATTS_TAG, "Service UUID: 0x%04X", GATTS_SERVICE_UUID);
    ESP_LOGI(GATTS_TAG, "TX Characteristic UUID: 0x%04X (Notify)", GATTS_CHAR_UUID_TX);
    ESP_LOGI(GATTS_TAG, "RX Characteristic UUID: 0x%04X (Write)", GATTS_CHAR_UUID_RX);
    ESP_LOGI(GATTS_TAG, "Send 0x01 to RX characteristic to turn LED ON");
    ESP_LOGI(GATTS_TAG, "Send 0x00 to RX characteristic to turn LED OFF");
}
/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"
#include "esp_pm.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "ble50_sec_gatts_demo.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#ifndef _LB
    #define _LB(val) ( (uint8_t)((val) & 0xFF) )
#endif
#ifndef _HB
    #define _HB(val) ( (uint8_t)(((val) >> (8 * (sizeof(val) - 1))) & 0xFF) )
#endif

#define SEC_SERVICE_UUID ESP_GATT_UUID_ENVIRONMENTAL_SENSING_SVC
#define BT_NAME_MAX_LEN 32

#define SENSING_PROFILE_NUM                       1
#define SENSING_PROFILE_APP_IDX                   0
#define SENSING_APP_ID                            0x55
#define EXT_ADV_HANDLE                            0
#define NUM_EXT_ADV_SET                           1
#define EXT_ADV_DURATION                          0
#define EXT_ADV_MAX_EVENTS                        0

struct my_connection
{
    uint16_t id;
    bool active;
} typedef my_connection_t;

static const char TAG[] = "MY_BLE";

static my_connection_t last_conn = { 0, 0 };
static uint16_t profile_handle_table[HRS_IDX_NB];
float measured_value = 0;

#define EXT_ADV_DATA_NAME_LENGTH_IDX 10
#define EXT_ADV_DATA_NAME_IDX 12
static const uint8_t ext_adv_raw_template[] = {
    0x02, 0x01, 0x06,
    0x02, 0x0a, 0xeb,
    0x03, 0x03, _HB(SEC_SERVICE_UUID), _LB(SEC_SERVICE_UUID),
    0x0B, 0X09, 'M', 'S', 'U', '_', 'E', 'n', 'd', 'o', 'H', '2'
};
static uint8_t* ext_adv_raw_data = NULL;
static size_t ext_adv_raw_data_len = 0;

static esp_ble_gap_ext_adv_t ext_adv[1] = {
    [0] = {EXT_ADV_HANDLE, EXT_ADV_DURATION, EXT_ADV_MAX_EVENTS},
};

esp_ble_gap_ext_adv_params_t ext_adv_params_2M = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE,
    .interval_min = 0x20,
    .interval_max = 0x20,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_2M,
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst sensing_profile_tab[SENSING_PROFILE_NUM] = {
    [SENSING_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};
static esp_gatt_if_t get_gatt_if()
{
    return sensing_profile_tab[SENSING_PROFILE_APP_IDX].gatts_if;
}

/* Service */
static const uint16_t sensing_svc_uuid = SEC_SERVICE_UUID;
static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
//static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
//static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint16_t sensing_measure_uuid = 0x2BCF; //Ammonia
static const uint16_t sensing_desc_uuid = ESP_GATT_UUID_ENV_SENSING_MEASUREMENT_DESCR;
static const uint16_t sensing_ccc_desc_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t sensing_pres_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;
static uint8_t sensing_meas_attrs[11] = { 
    0x00, 0x00, //Flags, reserved
    0x02, //Sampling = mean
    0x00, 0x00, 0x01, //Sampling interval
    0x00, 0x00, 0x01, //Update interval
    0x01, //Application = air
    0x02 //Uncertainty = 2x0.5%
 };
 #define MY_UNIT_UUID 0x271A //mol/m^3
 static uint8_t sensing_pres_attr[] = {
     0x14, //IEEE-754
    0x00, //Exponent
    _HB(MY_UNIT_UUID), _LB(MY_UNIT_UUID), //Unit UUID
    0x01, //Description namespace (Bluetooth SIG)
    0x00, 0x00 //Description index (??)
 };
static uint8_t sensing_ccc[2] = { 0x00, 0x00 };
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))
#define SVC_INST_ID                 0


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
// Service Declaration
        [IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {
                ESP_UUID_LEN_16, 
                (uint8_t *)&primary_service_uuid, 
                ESP_GATT_PERM_READ, 
                sizeof(sensing_svc_uuid), 
                sizeof(sensing_svc_uuid), 
                (uint8_t *)&sensing_svc_uuid
                }
            },

        //Measurement Characteristic Declaration
        [ATTR_IDX_MEASURE_DECL] =
            {{ESP_GATT_AUTO_RSP}, {
                ESP_UUID_LEN_16, 
                (uint8_t *)&character_declaration_uuid, 
                ESP_GATT_PERM_READ, 
                CHAR_DECLARATION_SIZE, 
                CHAR_DECLARATION_SIZE, 
                (uint8_t *)&char_prop_read_notify
                }
            },

        //Measurement Characteristic Value
        [ATTR_IDX_MEASURE_VAL] =
            {
                { ESP_GATT_RSP_BY_APP },
                {
                    ESP_UUID_LEN_16,
                    (uint8_t*)&sensing_measure_uuid,
                    ESP_GATT_PERM_READ_ENCRYPTED,
                    sizeof(measured_value),
                    sizeof(measured_value),
                    (uint8_t*)&measured_value
                }
            },

        //Measurement Characteristic Descriptor
        [ATTR_IDX_MEASURE_DESC] =
            {{ESP_GATT_AUTO_RSP}, {
                ESP_UUID_LEN_16,
                (uint8_t *)&sensing_desc_uuid,
                ESP_GATT_PERM_READ_ENCRYPTED,
                sizeof(sensing_meas_attrs),
                sizeof(sensing_meas_attrs),
                sensing_meas_attrs
                }
            },

        [ATTR_IDX_MEASURE_CCC] =
            {{ESP_GATT_AUTO_RSP}, {
                ESP_UUID_LEN_16,
                (uint8_t *)&sensing_ccc_desc_uuid,
                ESP_GATT_PERM_WRITE_ENCRYPTED | ESP_GATT_PERM_READ_ENCRYPTED,
                sizeof(sensing_ccc),
                sizeof(sensing_ccc),
                sensing_ccc
                }
            },

        [ATTR_IDX_MEASURE_PRES] = 
            {{ESP_GATT_AUTO_RSP}, {
                ESP_UUID_LEN_16,
                (uint8_t *)&sensing_pres_uuid,
                ESP_GATT_PERM_READ,
                sizeof(sensing_pres_attr),
                sizeof(sensing_pres_attr),
                sensing_pres_attr
                }
            }
};

static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   char *key_str = NULL;
   switch(key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

   }

   return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
   char *auth_str = NULL;
   switch(auth_req) {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
   }

   return auth_str;
}

void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    ESP_LOGI(TAG, "Bonded devices number : %d", dev_num);
    if (dev_num == 0) return;
    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_log_buffer_hex(TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

void remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        ESP_LOGI(TAG,"ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT status %d",  param->ext_adv_set_params.status);
        esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, ext_adv_raw_data_len, ext_adv_raw_data
            /*sizeof(ext_adv_raw_template), &ext_adv_raw_template[0]*/);
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
         ESP_LOGI(TAG,"ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT status %d",  param->ext_adv_data_set.status);
         esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
         break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
         ESP_LOGI(TAG, "ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT, status = %d", param->ext_adv_data_set.status);
        break;
    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_TERMINATED_EVT, status = %d", param->adv_terminate.status);
        if(param->adv_terminate.status == 0x00) {
            ESP_LOGI(TAG, "ADV successfully ended with a connection being created");
        }
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
        /* Call the following function to input the passkey which is displayed on the remote device */
        //esp_ble_passkey_reply(sensing_profile_tab[SENSING_PROFILE_APP_IDX].remote_bda, true, 0x00);
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT: {
        ESP_LOGI(TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;
    }
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer device.
        ESP_LOGI(TAG, "The passkey Notify number:%06d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(TAG, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
        }
        show_bonded_devices();
        break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
        ESP_LOGD(TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
        ESP_LOGI(TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        esp_log_buffer_hex(TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "------------------------------------");
        break;
    }
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT, tatus = %x", param->local_privacy_cmpl.status);
        esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params_2M);
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
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

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_REG_EVT");
            //generate a resolvable random address
            esp_ble_gap_config_local_privacy(true);
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", 
                param->read.conn_id, param->read.trans_id, param->read.handle);
            if (param->read.handle == profile_handle_table[ATTR_IDX_MEASURE_VAL])
            {
                esp_gatt_rsp_t resp = { 0 };
                resp.attr_value.handle = param->read.handle;
                resp.attr_value.len = sizeof(measured_value);
                for (size_t i = 0; i < resp.attr_value.len; i++)
                {
                    resp.attr_value.value[i] = ((uint8_t*)&measured_value)[i];
                }
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &resp);
                ESP_LOGI(TAG, "Sent response: %f", measured_value);
            }
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT, write value:");
            esp_log_buffer_hex(TAG, param->write.value, param->write.len);
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            break;
        case ESP_GATTS_MTU_EVT:
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT");
            /* start security connect with peer device when receive the connect event sent by the master */
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            last_conn.id = param->connect.conn_id;
            last_conn.active = true;
            esp_ble_gap_ext_adv_stop(NUM_EXT_ADV_SET, &ext_adv[0]); //Stop advertising after connection to save power (guess we won't need >1 peer device)
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            last_conn.active = false;
            /* start advertising again when missing the connect */
            esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            ESP_LOGI(TAG, "The number handle = %x",param->add_attr_tab.num_handle);
            if (param->create.status == ESP_GATT_OK){
                if(param->add_attr_tab.num_handle == HRS_IDX_NB) {
                    memcpy(profile_handle_table, param->add_attr_tab.handles,
                    sizeof(profile_handle_table));
                   esp_ble_gatts_start_service(profile_handle_table[IDX_SVC]);
                }else{
                    ESP_LOGE(TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, HRS_IDX_NB);
                }
            }else{
                ESP_LOGE(TAG, " Create attribute table failed, error code = %x", param->create.status);
            }
            break;
        }

        default:
           break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            sensing_profile_tab[SENSING_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SENSING_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == sensing_profile_tab[idx].gatts_if) {
                if (sensing_profile_tab[idx].gatts_cb) {
                    sensing_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void my_ble_notify_value_changed()
{
    if (!last_conn.active) return; //Nowhere to send the indication to
    esp_ble_gatts_send_indicate(get_gatt_if(), last_conn.id, profile_handle_table[ATTR_IDX_MEASURE_VAL],
                                sizeof(measured_value), (uint8_t *)&measured_value, false);
}

esp_err_t ble_init(const char* name)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    size_t name_len = strnlen(name, BT_NAME_MAX_LEN);
    if (name_len == BT_NAME_MAX_LEN)
    {
        ESP_LOGE(TAG, "Device name too long, limit: %u", BT_NAME_MAX_LEN);
        ext_adv_raw_data_len = sizeof(ext_adv_raw_template) * sizeof(ext_adv_raw_template[0]);
        ext_adv_raw_data = (uint8_t*)malloc(ext_adv_raw_data_len);
        if (!ext_adv_raw_data) return ESP_ERR_NO_MEM;
        memcpy(ext_adv_raw_data, ext_adv_raw_template, ext_adv_raw_data_len);
    }
    else
    {
        //esp_ble_gap_set_device_name(name);
        ext_adv_raw_data_len = EXT_ADV_DATA_NAME_IDX + name_len;
        ext_adv_raw_data = (uint8_t*)malloc(ext_adv_raw_data_len);
        if (!ext_adv_raw_data) return ESP_ERR_NO_MEM;
        memcpy(ext_adv_raw_data, ext_adv_raw_template, EXT_ADV_DATA_NAME_IDX);
        memcpy(ext_adv_raw_data + EXT_ADV_DATA_NAME_IDX, name, name_len);
        ext_adv_raw_data[EXT_ADV_DATA_NAME_LENGTH_IDX] = name_len + 1; //+ entry type byte
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "GATTS callback registered.");
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "GAP callback registered.");
    ret = esp_ble_gatts_app_register(SENSING_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "GATTS app registered.");

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    //set static passkey
    uint32_t passkey = 123456;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    /* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the master;
    If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    return ESP_OK;
}

static const char name[] = "MSU_EndoH2_#0";

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(ble_init(name));

    //Configure sleep
    ESP_LOGI(TAG, "Init sleep modes...");
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(5000000UL)); //5sec
    esp_bt_sleep_enable();
    esp_pm_config_esp32s3_t pm_cfg = 
    {
        .max_freq_mhz = 240,
        .min_freq_mhz = 80,
        .light_sleep_enable = true
    };
    esp_pm_configure(&pm_cfg);
    show_bonded_devices();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (++measured_value > 10) measured_value = 0;
        my_ble_notify_value_changed();
        //vTaskDelay(pdMS_TO_TICKS(10000));
        //esp_light_sleep_start();
    }
}

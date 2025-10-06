#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"

#include "usb/usb_host.h"
#include "errno.h"
#include "driver/gpio.h"

#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"

#include "nvs_flash.h"

#include "hid_dev.h"

#include "led_strip.h"

/* =================================================================================================
   MACROS
   ================================================================================================= */
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
#define HID_KEYBOARD_IN_RPT_LEN 8
#define HID_CC_IN_RPT_LEN 2
#define BLE_HID_DEVICE_NAME "BLE HID"

#define LED_GPIO_PIN 21
#define LED_RMT_RES_HZ (10 * 1000 * 1000)
#define LED_BRIGHTNESS 25

/* =================================================================================================
   VARIABLES, STRUCTS, ENUMS
   ================================================================================================= */

// BLE HID
static uint16_t ble_hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;

static const char *TAG_BLE = "BLE";

static uint8_t ble_hid_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0x12,
    0x18,
    0x00,
    0x00,
};

static esp_ble_adv_data_t ble_hid_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,   // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(ble_hid_service_uuid128),
    .p_service_uuid = ble_hid_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t ble_hid_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// USB HOST HID
static const char *TAG_HID = "HID";
static const char *TAG_KEYBOARD = "HID Keyboard";
static const char *TAG_GENERIC = "HID Generic";

QueueHandle_t app_event_queue = NULL;

/**
 * @brief APP event group
 *
 * Application logic can be different. There is a one among other ways to distingiush the
 * event by application event group.
 * In this example we have two event groups:
 * APP_EVENT            - General event, which is APP_QUIT_PIN press event (Generally, it is IO0).
 * APP_EVENT_HID_HOST   - HID Host Driver event, such as device connection/disconnection or input report.
 */
typedef enum
{
  APP_EVENT = 0,
  APP_EVENT_HID_HOST
} app_event_group_t;

/**
 * @brief APP event queue
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct
{
  app_event_group_t event_group;
  /* HID Host - Device related info */
  struct
  {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t event;
    void *arg;
  } hid_host_device;
} app_event_queue_t;

/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {
    "NONE",
    "KEYBOARD",
};

app_event_queue_t evt_queue;

// LED
led_strip_handle_t led_strip;

static const char *TAG_LED = "LED";

/* =================================================================================================
   FUNCTION PROTOTYPES
   ================================================================================================= */

// BLE HID
static void ble_hid_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// USB HOST HID
void printBinary(uint8_t value);
static void hid_host_keyboard_report_callback(hid_host_device_handle_t hid_device_handle, uint8_t *data, int length);
static void hid_host_generic_report_callback(const uint8_t *const data, const int length);
void usb_hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_interface_event_t event,
                                     void *arg);
void usb_hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                               const hid_host_driver_event_t event,
                               void *arg);
static void usb_lib_task(void *arg);
void usb_hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg);

// LED
led_strip_handle_t configure_led(void);
void set_led_color();

/* =================================================================================================
   BLE HID
   ================================================================================================= */

static void ble_hid_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
  switch (event)
  {
  case ESP_HIDD_EVENT_REG_FINISH:
  {
    if (param->init_finish.state == ESP_HIDD_INIT_OK)
    {
      // esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
      esp_ble_gap_set_device_name(BLE_HID_DEVICE_NAME);
      esp_ble_gap_config_adv_data(&ble_hid_adv_data);
    }
    break;
  }
  case ESP_BAT_EVENT_REG:
  {
    break;
  }
  case ESP_HIDD_EVENT_DEINIT_FINISH:
    break;
  case ESP_HIDD_EVENT_BLE_CONNECT:
  {
    ESP_LOGI(TAG_BLE, "ESP_HID_EVENT_BLE_CONNECT");
    ble_hid_conn_id = param->connect.conn_id;
    break;
  }
  case ESP_HIDD_EVENT_BLE_DISCONNECT:
  {
    sec_conn = false;
    ESP_LOGI(TAG_BLE, "ESP_HID_EVENT_BLE_DISCONNECT");
    esp_ble_gap_start_advertising(&ble_hid_adv_params);
    set_led_color();
    break;
  }
  case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
  {
    ESP_LOGI(TAG_BLE, "%s, ESP_HID_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
    ESP_LOG_BUFFER_HEX(TAG_BLE, param->vendor_write.data, param->vendor_write.length);
    break;
  }
  case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
  {
    ESP_LOGI(TAG_BLE, "ESP_HID_EVENT_BLE_LED_REPORT_WRITE_EVT");
    if (evt_queue.hid_host_device.handle)
    {
      ESP_ERROR_CHECK(hid_class_request_set_report(evt_queue.hid_host_device.handle, HID_REPORT_TYPE_OUTPUT, 0, param->led_write.data, param->led_write.length));
    }
    ESP_LOG_BUFFER_HEX(TAG_BLE, param->led_write.data, param->led_write.length);
    printBinary(param->led_write.data[0]);
    break;
  }
  default:
    break;
  }
  return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  switch (event)
  {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    esp_ble_gap_start_advertising(&ble_hid_adv_params);
    break;
  case ESP_GAP_BLE_SEC_REQ_EVT:
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
    {
      ESP_LOGD(TAG_BLE, "%x:", param->ble_security.ble_req.bd_addr[i]);
    }
    esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
    break;
  case ESP_GAP_BLE_AUTH_CMPL_EVT:
    sec_conn = true;
    esp_bd_addr_t bd_addr;
    memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
    ESP_LOGI(TAG_BLE, "remote BD_ADDR: %08x%04x",
             (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
             (bd_addr[4] << 8) + bd_addr[5]);
    ESP_LOGI(TAG_BLE, "address type = %d", param->ble_security.auth_cmpl.addr_type);
    ESP_LOGI(TAG_BLE, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
    if (!param->ble_security.auth_cmpl.success)
    {
      ESP_LOGE(TAG_BLE, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
    }
    else
    {
      set_led_color();
    }
    break;
  default:
    break;
  }
}

/* =================================================================================================
   USB HID HOST
   ================================================================================================= */

/**
 * @brief Print binary value
 * @param[in] value  Value to print
 */
void printBinary(uint8_t value)
{
  for (int i = 7; i >= 0; --i)
  {                                          // Iterate over each bit (from MSB to LSB)
    putchar((value & (1 << i)) ? '1' : '0'); // Print '1' if the bit is set, else '0'
  }
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(hid_host_device_handle_t hid_device_handle, uint8_t *data, int length)
{

  hid_dev_send_report(hidd_le_env.gatt_if, ble_hid_conn_id, HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT, HID_KEYBOARD_IN_RPT_LEN, data);

  hid_keyboard_input_report_boot_t *kb_report = (hid_keyboard_input_report_boot_t *)data;

  if (length < sizeof(hid_keyboard_input_report_boot_t))
  {
    return;
  }

  if (kb_report->key[0] > 0 || kb_report->modifier.val > 0)
  {
    putchar('\n');
    if (kb_report->modifier.val > 0)
    {
      printf("Modifier: ");
      printBinary(kb_report->modifier.val);
      putchar('\n');
    }
    if (kb_report->key[0] > 0)
    {
      printf("Keys: ");
      putchar('\n');
      for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++)
      {
        printf("%02X ", kb_report->key[i]);
      }
      putchar('\n');
    }
  }
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data, const int length)
{
  int report_length_without_report_id = length - 1;
  if (report_length_without_report_id <= 2)
  {
    uint8_t report_data_without_report_id[2] = {0, 0};
    memcpy(report_data_without_report_id, &data[1], report_length_without_report_id);
    printf("Maybe Consumer Report\n");
    hid_dev_send_report(hidd_le_env.gatt_if, ble_hid_conn_id, HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT, report_length_without_report_id, report_data_without_report_id);
  }
  for (int i = 0; i < length; i++)
  {
    printf("%02X ", data[i]);
  }
  putchar('\n');
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void usb_hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_interface_event_t event,
                                     void *arg)
{
  uint8_t data[64] = {0};
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  putchar('\n');
  ESP_LOGI(TAG_HID, "Interface: %d", dev_params.iface_num);

  // size_t report_desc_len = 0;
  // uint8_t *report_desc = hid_host_get_report_descriptor(hid_device_handle, &report_desc_len);

  // putchar('\n');
  // ESP_LOGI(TAG_HID, "Report descriptor:");
  // for (size_t i = 0; i < report_desc_len; i++)
  // {
  //   printf("%02X ", report_desc[i]);
  // }
  // putchar('\n');

  switch (event)
  {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                              data,
                                                              64,
                                                              &data_length));

    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class && HID_PROTOCOL_KEYBOARD == dev_params.proto)
    {
      ESP_LOGI(TAG_KEYBOARD, "Keyboard Event");
      hid_host_keyboard_report_callback(hid_device_handle, data, data_length);
    }
    else
    {
      ESP_LOGI(TAG_GENERIC, "Generic Event");
      hid_host_generic_report_callback(data, data_length);
    }

    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    ESP_LOGI(TAG_HID, "HID Device, interface %d protocol '%s' DISCONNECTED",
             dev_params.iface_num, hid_proto_name_str[dev_params.proto]);
    ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
    evt_queue.hid_host_device.handle = NULL;
    set_led_color();
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    ESP_LOGI(TAG_HID, "HID Device, interface %d protocol '%s' TRANSFER_ERROR",
             dev_params.iface_num, hid_proto_name_str[dev_params.proto]);
    break;
  default:
    ESP_LOGE(TAG_HID, "HID Device, interface %d protocol '%s' Unhandled event",
             dev_params.iface_num, hid_proto_name_str[dev_params.proto]);
    break;
  }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, (not used)
 */
void usb_hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                               const hid_host_driver_event_t event,
                               void *arg)
{
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event)
  {
  case HID_HOST_DRIVER_EVENT_CONNECTED:
    ESP_LOGI(TAG_HID, "HID Device");
    printf("address: %d, interface: %d, subclass: %d, protocol: %d %s\n",
           dev_params.addr, dev_params.iface_num, dev_params.sub_class, dev_params.proto, hid_proto_name_str[dev_params.proto]);

    const hid_host_device_config_t dev_config = {
        .callback = usb_hid_host_interface_callback,
        .callback_arg = NULL};

    ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class)
    {
      // ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT));
      ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_REPORT));
      if (HID_PROTOCOL_KEYBOARD == dev_params.proto)
      {
        ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
      }
    }
    ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
    set_led_color();
    break;
  default:
    break;
  }
}

/**
 * @brief Start USB Host install and handle common USB host library events while app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_lib_task(void *arg)
{
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };

  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xTaskNotifyGive(arg);

  while (true)
  {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    // In this example, there is only one client registered
    // So, once we deregister the client, this call must succeed with ESP_OK
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
    {
      ESP_ERROR_CHECK(usb_host_device_free_all());
      break;
    }
  }

  ESP_LOGI(TAG_HID, "USB shutdown");
  // Clean up USB Host
  vTaskDelay(10); // Short delay to allow clients clean-up
  ESP_ERROR_CHECK(usb_host_uninstall());
  vTaskDelete(NULL);
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void usb_hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg)
{
  const app_event_queue_t evt_queue = {
      .event_group = APP_EVENT_HID_HOST,
      // HID Host Device related info
      .hid_host_device.handle = hid_device_handle,
      .hid_host_device.event = event,
      .hid_host_device.arg = arg};

  if (app_event_queue)
  {
    xQueueSend(app_event_queue, &evt_queue, 0);
  }
}

/* =================================================================================================
   LED
   ================================================================================================= */

led_strip_handle_t configure_led(void)
{
  // LED strip general initialization, according to your led board design
  led_strip_config_t strip_config = {
      .strip_gpio_num = LED_GPIO_PIN,                              // The GPIO that connected to the LED strip's data line
      .max_leds = 1,                                               // The number of LEDs in the strip,
      .led_model = LED_MODEL_WS2812,                               // LED strip model
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB, // The color order of the strip: GRB
      .flags = {
          .invert_out = false, // don't invert the output signal
      }};

  // LED strip backend configuration: RMT
  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,  // different clock source can lead to different power consumption
      .resolution_hz = LED_RMT_RES_HZ, // RMT counter clock frequency
      .mem_block_symbols = 64,         // the memory size of each RMT channel, in words (4 bytes)
      .flags = {
          .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
      }};

  // LED Strip object handle
  led_strip_handle_t led_strip;
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  ESP_ERROR_CHECK(led_strip_clear(led_strip));
  ESP_LOGI(TAG_LED, "Created LED strip object with RMT backend");
  return led_strip;
}

/**
 * @brief Set LED color based on USB and BLE HID connection status
 */
void set_led_color()
{
  printf("USB HID: %d, BLE HID: %d\n", evt_queue.hid_host_device.handle != NULL ? true : false, sec_conn);
  if (evt_queue.hid_host_device.handle && sec_conn)
  {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS));
  }
  else if (evt_queue.hid_host_device.handle)
  {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, LED_BRIGHTNESS, 0));
  }
  else if (sec_conn)
  {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, LED_BRIGHTNESS));
  }
  else
  {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, LED_BRIGHTNESS, 0, 0));
  }
  ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void app_main(void)
{
  esp_err_t ret;

  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret)
  {
    ESP_LOGE(TAG_BLE, "%s initialize controller failed", __func__);
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret)
  {
    ESP_LOGE(TAG_BLE, "%s enable controller failed", __func__);
    return;
  }

  ret = esp_bluedroid_init();
  if (ret)
  {
    ESP_LOGE(TAG_BLE, "%s init bluedroid failed", __func__);
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret)
  {
    ESP_LOGE(TAG_BLE, "%s init bluedroid failed", __func__);
    return;
  }

  if ((ret = esp_hidd_profile_init()) != ESP_OK)
  {
    ESP_LOGE(TAG_BLE, "%s init bluedroid failed", __func__);
  }

  /// register the callback function to the gap module
  esp_ble_gap_register_callback(gap_event_handler);
  esp_hidd_register_callbacks(ble_hid_event_callback);

  /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;       // set the IO capability to No output No input
  uint8_t key_size = 16;                          // the key size should be 7~16 bytes
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
  and the response key means which key you can distribute to the Master;
  If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
  and the init key means which key you can distribute to the slave. */
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

  BaseType_t task_created;
  ESP_LOGI(TAG_HID, "HID Host example");

  /*
   * Create usb_lib_task to:
   * - initialize USB Host library
   * - Handle USB Host events while APP pin in in HIGH state
   */
  task_created = xTaskCreatePinnedToCore(usb_lib_task,
                                         "usb_events",
                                         4096,
                                         xTaskGetCurrentTaskHandle(),
                                         2, NULL, 0);
  assert(task_created == pdTRUE);

  // Wait for notification from usb_lib_task to proceed
  ulTaskNotifyTake(false, 1000);

  /*
   * HID host driver configuration
   * - create background task for handling low level event inside the HID driver
   * - provide the device callback to get new HID Device connection event
   */
  const hid_host_driver_config_t hid_host_driver_config = {
      .create_background_task = true,
      .task_priority = 5,
      .stack_size = 4096,
      .core_id = 0,
      .callback = usb_hid_host_device_callback,
      .callback_arg = NULL};

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  // Create queue
  app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));

  ESP_LOGI(TAG_HID, "Waiting for HID Device to be connected");

  led_strip = configure_led();
  set_led_color();

  while (1)
  {
    // Wait queue
    if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY))
    {
      if (APP_EVENT_HID_HOST == evt_queue.event_group)
      {
        usb_hid_host_device_event(evt_queue.hid_host_device.handle,
                                  evt_queue.hid_host_device.event,
                                  evt_queue.hid_host_device.arg);
      }
    }
  }

  ESP_LOGI(TAG_HID, "HID Driver uninstall");
  ESP_ERROR_CHECK(hid_host_uninstall());
  xQueueReset(app_event_queue);
  vQueueDelete(app_event_queue);
}

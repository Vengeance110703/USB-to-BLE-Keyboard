/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "errno.h"
#include "driver/gpio.h"

#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"

/* GPIO Pin number for quit from example logic */
#define APP_QUIT_PIN GPIO_NUM_0

static const char *TAG = "example";
static const char *TAG_HID = "hid";
static const char *TAG_KEYBOARD = "keyboard";
static const char *TAG_GENERIC = "generic";

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

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
  static hid_protocol_t prev_proto_output = -1;

  if (prev_proto_output != proto)
  {
    prev_proto_output = proto;
    printf("\r\n");
    if (proto == HID_PROTOCOL_KEYBOARD)
    {
      printf("Keyboard\r\n");
    }
    else
    {
      printf("Generic\r\n");
    }
    fflush(stdout);
  }
}

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
static void hid_host_keyboard_report_callback(hid_host_device_handle_t hid_device_handle, const uint8_t *const data, const int length)
{
  hid_keyboard_input_report_boot_t *kb_report = (hid_keyboard_input_report_boot_t *)data;

  if (length < sizeof(hid_keyboard_input_report_boot_t))
  {
    return;
  }

  static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
  static bool caps_lock = false;

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
        if (kb_report->key[i] == HID_KEY_CAPS_LOCK)
        {
          caps_lock = !caps_lock;
          // ESP_ERROR_CHECK(hid_class_request_set_report(hid_device_handle, HID_REPORT_TYPE_OUTPUT, 0, ));
          printf("CAPS_LOCK: %s\n", caps_lock ? "ON" : "OFF");
        }
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
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
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
      hid_host_keyboard_report_callback(data, data_length);
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
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event,
                           void *arg)
{
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event)
  {
  case HID_HOST_DRIVER_EVENT_CONNECTED:
    ESP_LOGI(TAG, "HID Device");
    printf("address: %d, interface: %d, subclass: %d, protocol: %d %s\n",
           dev_params.addr, dev_params.iface_num, dev_params.sub_class, dev_params.proto, hid_proto_name_str[dev_params.proto]);

    const hid_host_device_config_t dev_config = {
        .callback = hid_host_interface_callback,
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

  ESP_LOGI(TAG, "USB shutdown");
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
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
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

void app_main(void)
{
  BaseType_t task_created;
  app_event_queue_t evt_queue;
  ESP_LOGI(TAG, "HID Host example");

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
      .callback = hid_host_device_callback,
      .callback_arg = NULL};

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  // Create queue
  app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));

  ESP_LOGI(TAG, "Waiting for HID Device to be connected");

  while (1)
  {
    // Wait queue
    if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY))
    {
      if (APP_EVENT_HID_HOST == evt_queue.event_group)
      {
        hid_host_device_event(evt_queue.hid_host_device.handle,
                              evt_queue.hid_host_device.event,
                              evt_queue.hid_host_device.arg);
      }
    }
  }

  ESP_LOGI(TAG, "HID Driver uninstall");
  ESP_ERROR_CHECK(hid_host_uninstall());
  xQueueReset(app_event_queue);
  vQueueDelete(app_event_queue);
}

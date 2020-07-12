/*********
 * Modification of app.c file from empty-soc project a dtm project file
 * from silicon labs.
 *
 */

/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"

#include "app.h"

#define TX

#define ONE_S (32768U)
#define TEST_DURATION_TIMER (1)

#define RESPONSE_LEN_BYTES (2)
#define PHY_NUMBER     (5) /* actually there only 4 options but the enumeration starts at 1 */

#define BUTTON_PRESSED (1 << 0)
#define BUTTON_RELEASED (1 << 1)

struct dtm_data{
    // eDTM_TYPE eType; /* Best would be to use a status word/register type of thing */
    // union dmt_data_status status;

    uint8 result[RESPONSE_LEN_BYTES];
    uint32 test_duration;
    struct gecko_msg_test_dtm_completed_evt_t event;
    struct gecko_msg_test_dtm_rx_cmd_t rx_cmd;
    struct gecko_msg_test_dtm_rx_rsp_t rx_rsp;
    struct gecko_msg_test_dtm_tx_cmd_t tx_cmd;
    struct gecko_msg_test_dtm_tx_rsp_t tx_rsp;
};

static struct dtm_data dtm_test_data;
static const char* phy_desc[PHY_NUMBER] = { "invalid",
                                            "1M",
                                            "2M",
                                            "125K",
                                            "500k" };

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);
void do_test();
void init_GPIO(void);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  void * response_hndl;
  bool started=false;
  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();

  /* Init LEDs */
  GPIO_PinModeSet(BSP_LED0_PORT,BSP_LED0_PIN,gpioModePushPull,HAL_GPIO_DOUT_HIGH);
  GPIO_PinOutClear(BSP_LED0_PORT, BSP_LED0_PIN);

  /* Init Button */
  init_GPIO();
  /* Initialize stack */
  gecko_init(pconfig);


  printLog("PrintLog\r\n");
  printf("printf\r\n");
  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* if there are no events pending then the next call to gecko_wait_event() may cause
     * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
    if (!gecko_event_pending()) {
      flushLog();
    }

    /* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
        */
      case gecko_evt_system_boot_id:

        bootMessage(&(evt->data.evt_system_boot));
#ifdef TX
        gecko_bgapi_class_cte_transmitter_init();
#else
        gecko_bgapi_class_cte_receiver_init();
#endif
        printf("Ready to start test\r\n");
        break;

      case gecko_evt_hardware_soft_timer_id:
          printf("\rSoft Timer evt Handle : %d\n\r",
                 evt->data.evt_hardware_soft_timer.handle );

          if( TEST_DURATION_TIMER == evt->data.evt_hardware_soft_timer.handle )
          {
              /* Stop the test */
        	  printf("Stop the test\r\n");
        	  // Turn off LED
              GPIO_PinOutClear(BSP_LED0_PORT, BSP_LED0_PIN);

              /* End test, handle the response ! */
              response_hndl = (void*)gecko_cmd_test_dtm_end();

              if( 0 != ((struct gecko_msg_test_dtm_end_rsp_t*)response_hndl)->result )
              {
            	  printf("gecko_cmd_test_dtm_end failed!\r\n");
              }
          }
          break;

      case gecko_evt_test_dtm_completed_id:
    	  if(!started) {
    		  started = true;
#ifdef TX
    		  printf("Starting TX test\r\n");
#else
    		  printf("Starting RX test\r\n");
#endif
    	  } else {
              dtm_test_data.event = evt->data.evt_test_dtm_completed;
#ifdef TX
    		  printf("Done with TX test\r\n");
              printf("\rNumber of packets transmitted : %d\n\r",
                      dtm_test_data.event.number_of_packets );
#else
			  printf("Done with RX test\r\n");
              printf("\rNumber of packets received : %d\n\r",
                      dtm_test_data.event.number_of_packets );
#endif
              started = false;
    	  }

    	  break;

      case gecko_evt_system_external_signal_id:
	        if (evt->data.evt_system_external_signal.extsignals & BUTTON_PRESSED) {
	        	printLog("Button Pressed\r\n");
	        }

	        if (evt->data.evt_system_external_signal.extsignals & BUTTON_RELEASED) {
	        	printLog("Button released\r\n");
	        	do_test();
	        }
	        break;

      case gecko_evt_cte_receiver_connection_iq_report_id:
    	  printf("Got recvr connection iq report\r\n");
    	  break;

      case gecko_evt_cte_receiver_dtm_iq_report_id: {
    	  struct gecko_msg_cte_receiver_dtm_iq_report_evt_t report = evt->data.evt_cte_receiver_dtm_iq_report;
    	  // printf("Got dtm iq report\r\n");
			printf("status: %d, ch: %d, rssi: %d, ant:%d, cte:%d, duration:%d, len:%d\r\n", report.status,
					report.channel, report.rssi, report.rssi_antenna_id, report.cte_type, report.slot_durations,
					report.samples.len);
			for (int i=0; i<report.samples.len; i++) {
				RETARGET_WriteChar(report.samples.data[i]);
			}
			printf("\r\n");
    	  break;
      }
      case gecko_evt_le_connection_opened_id:

        printLog("connection opened\r\n");

        break;

      case gecko_evt_le_connection_closed_id:

        printLog("connection closed, reason: 0x%2.2x\r\n", evt->data.evt_le_connection_closed.reason);

        /* Check if need to boot to OTA DFU mode */
        if (boot_to_dfu) {
          /* Enter to OTA DFU mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        }
        break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:

        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      /* Add additional event handlers as your application requires */

      default:
    	printf("Unhandled event %lx\r\n", BGLIB_MSG_ID(evt->header));
        break;
    }
  }
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++) {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}


static void button_callback(const uint8_t pin)
{
  if (pin == BSP_BUTTON0_PIN) {
    /* when input is high, the button was released */
    if (GPIO_PinInGet(BSP_BUTTON0_PORT,BSP_BUTTON0_PIN)) {
        gecko_external_signal(BUTTON_RELEASED);
    }
    /* when input is low, the button was pressed*/
    else {
        gecko_external_signal(BUTTON_PRESSED);
    }
  }
}

void init_GPIO(void) {
	/* Initialize GPIO interrupt handler */
	GPIOINT_Init();

	/* Set the pin of Push Button 0 as input with pull-up resistor */
	GPIO_PinModeSet( BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, HAL_GPIO_MODE_INPUT_PULL_FILTER, HAL_GPIO_DOUT_HIGH );

	/* Enable interrupt on Push Button 0 */
	GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, true, true);

	/* Register callback for Push Button 0 */
	GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, button_callback);
}

void do_test() {
	uint8 ch = 1;
	uint8 phy = 1;
    //  Set up tx and rx test parameters
    dtm_test_data.test_duration = 10*ONE_S;
    dtm_test_data.tx_cmd.packet_type = 1;
    dtm_test_data.tx_cmd.length = 8;
    dtm_test_data.tx_cmd.channel = ch;
    dtm_test_data.tx_cmd.phy = phy;
    dtm_test_data.rx_cmd.channel = ch;
    dtm_test_data.rx_cmd.phy = phy;

        uint8 cte_length = 0x02;
        uint8 cte_type = 0;
        uint8 duration = 1;
        uint8 pattern_len = 1;
        uint8 pattern[1] = {0};
        uint16 response;
#ifdef TX
        response = (gecko_cmd_cte_transmitter_set_dtm_parameters(cte_length,
        		cte_type,
				pattern_len,
				pattern))->result;
		printf("cte transmitter_set dtm result 0x%x\r\n", response);
#else
		response = (gecko_cmd_cte_receiver_set_dtm_parameters(cte_length,
        		cte_type,
				duration,
				pattern_len,
				pattern))->result;
		printf("cte receiver_set dtm result 0x%x\r\n", response);
#endif
        gecko_cmd_hardware_set_soft_timer( dtm_test_data.test_duration,
                                           TEST_DURATION_TIMER,
                                           true );

        /* Switch on LED 0 */
        GPIO_PinOutSet(BSP_LED0_PORT, BSP_LED0_PIN);
#ifdef TX
        printf("\rTX args :\n\rpkt_type -> %d; length -> %d; channel -> %d; phy -> %s\n\r",
        		dtm_test_data.tx_cmd.packet_type,
				dtm_test_data.tx_cmd.length,
				dtm_test_data.tx_cmd.channel,
				phy_desc[dtm_test_data.tx_cmd.phy]);

        gecko_cmd_test_dtm_tx( dtm_test_data.tx_cmd.packet_type,
        		dtm_test_data.tx_cmd.length,
				dtm_test_data.tx_cmd.channel,
				dtm_test_data.tx_cmd.phy );
#else
        printf("\rRX args :\n\rchannel -> %d; phy -> %s\n\r",
				dtm_test_data.tx_cmd.channel,
				phy_desc[dtm_test_data.tx_cmd.phy]);
        gecko_cmd_test_dtm_rx( dtm_test_data.rx_cmd.channel,
                               dtm_test_data.rx_cmd.phy );
#endif
}

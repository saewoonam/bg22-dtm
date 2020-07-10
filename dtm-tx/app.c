/***************************************************************************//**
 * \file   app.c
 * \brief  Silicon Labs DTM command interface.
 *
 * Description : The dtm application
 *******************************************************************************
 * <b> (C) Copyright 2018 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 ******************************************************************************/

/* Bluetooth stack declarations */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Debug headers */
#include "stdio.h"

/* LED declarations */
#include "em_gpio.h"

/*******************************************************************************
**                             MACROS AND ENUMS                               **
*******************************************************************************/
#define PHY_NUMBER     (5) /* actually there only 4 options but the enumeration starts at 1 */

#define ONE_S (32768U)
#define TEST_DURATION_TIMER (1)

#define RESPONSE_LEN_BYTES (2)

/*******************************************************************************
**                         Data struct definitions                            **
*******************************************************************************/
/* DTM state machine definition :
**  The DTM functionality is implemented using a slave state machine and a BLE
**  stack event manager */
typedef enum {
    eDTM_IDLE = 0,
    eDTM_ADVERTISING,
    eDTM_CONNECTED,
    eDTM_TEST_STARTING,
    eDTM_TEST_RUNNING,
    eDTM_TEST_STOPPING,
    eDTM_TEST_PUBLISHING

} eDTM_STATE;

typedef enum {
    eDTM_TYPE_EMPTY = 0,
    eDTM_TYPE_TX,
    eDTM_TYPE_RX

} eDTM_TYPE;

union connection_data_status{
    struct {
        uint8 params_set:1;
        uint8 phy_set:1;
        uint8 mtu_exch_set:1;
        uint8 connection_active:1;
        uint8 connection_established:1;
        uint8 reserverd:3;
    } bit;
    uint8 byte;
};

struct connection_data{
    union connection_data_status status; /* Best would be to use a status word/register type of thing */

    struct gecko_msg_le_connection_opened_evt_t open;
    struct gecko_msg_le_connection_parameters_evt_t parameters;
    struct gecko_msg_le_connection_phy_status_evt_t phy_status;
    struct gecko_msg_gatt_mtu_exchanged_evt_t mtu_exchange;
    struct gecko_msg_le_connection_closed_evt_t close;
};

union dmt_data_status{
    struct {
        uint8 channel:1;
        uint8 phy:1;
        uint8 length:1;
        uint8 packet_type:1;
        uint8 cmd_ready:1;
        uint8 reserverd:3;
    } bit;
    uint8 byte;
};

struct dtm_data{
    eDTM_TYPE eType; /* Best would be to use a status word/register type of thing */
    union dmt_data_status status;

    uint8 result[RESPONSE_LEN_BYTES];
    uint32 test_duration;
    struct gecko_msg_test_dtm_completed_evt_t event;
    struct gecko_msg_test_dtm_rx_cmd_t rx_cmd;
    struct gecko_msg_test_dtm_rx_rsp_t rx_rsp;
    struct gecko_msg_test_dtm_tx_cmd_t tx_cmd;
    struct gecko_msg_test_dtm_tx_rsp_t tx_rsp;
};

/*******************************************************************************
**                             LOCAL VARIABLES                                **
*******************************************************************************/
// Flag for indicating DFU Reset must be performed
/* Event pointer for handling events */
static eDTM_STATE eState;
static struct connection_data dtm_connection_data;
static struct dtm_data dtm_test_data;
static const char* phy_desc[PHY_NUMBER] = { "invalid",
                                            "1M",
                                            "2M",
                                            "125K",
                                            "500k" };

/*******************************************************************************
**                              LOCAL ROUTINES                                **
*******************************************************************************/
static eDTM_STATE event_manager(void);
static void advertise(void);
static void publish_connection_data(void);
static void publish_test_result(void);
static void parse_tx_args(struct gecko_msg_gatt_server_user_write_request_evt_t *ptr_usr_write_req, uint8 gatt_hndl);
static void parse_rx_args(struct gecko_msg_gatt_server_user_write_request_evt_t *ptr_usr_write_req, uint8 gatt_hndl);

/*******************************************************************************
**                                 API DEFNS                                  **
*******************************************************************************/
void appMain(gecko_configuration_t *pconfig)
{
  void * response_hndl;

  /* Initialise global variables */
  eState = eDTM_IDLE;

  /* Init LEDs */
  GPIO_PinModeSet(BSP_LED0_PORT,BSP_LED0_PIN,gpioModePushPull,HAL_GPIO_DOUT_HIGH);
  // GPIO_PinModeSet(BSP_LED1_PORT,BSP_LED1_PIN,gpioModePushPull,HAL_GPIO_DOUT_HIGH);

  /* General reset connection and dtm data structure */
  memset(&dtm_connection_data,0x00,sizeof(struct connection_data));
  memset(&dtm_test_data,0x00,sizeof(struct dtm_data));

  /* Initialize stack */
  gecko_init(pconfig);
  printf("printf\r\n");
  // printLog("hello\r\n");
  while (1) {
    /* Initialise locals */
    response_hndl = NULL;

    /* Check for stack event. */
    eState = event_manager();

        /* Handle events */
    switch (eState) {
      case eDTM_IDLE:
        /* Nothing to do for the moment */
        break;

      case eDTM_CONNECTED:
          /* Just received a "connected" event from the stack. Some more events will come, mark connection as active */

          /* See whether we have a fully established connection or not */
          if( (1 == dtm_connection_data.status.bit.connection_established) )
          {
              publish_connection_data();

              /* reset flag */
              dtm_connection_data.status.bit.connection_established = 0;
          }

          break;

      case eDTM_TEST_STARTING:
          /* Kick off the appropriate test */

          /* Trigger timer */
    	  printf("Test Starting\r\n");
          gecko_cmd_hardware_set_soft_timer( dtm_test_data.test_duration,
                                             TEST_DURATION_TIMER,
                                             true );

          if(   ( dtm_test_data.eType == eDTM_TYPE_TX )
             && ( dtm_test_data.status.bit.cmd_ready == 1 )
            )
          {
              /* Switch on LED 1 */
              //GPIO_PinOutClear(BSP_LED1_PORT, BSP_LED1_PIN);

              printf("\rTX args :\n\rpkt_type -> %d; length -> %d; channel -> %d; phy -> %d\n\r",
                     dtm_test_data.tx_cmd.packet_type,
                     dtm_test_data.tx_cmd.length,
                     dtm_test_data.tx_cmd.channel,
                     dtm_test_data.tx_cmd.phy);

              /* Handle the response */
              gecko_cmd_test_dtm_tx( dtm_test_data.tx_cmd.packet_type,
                                     dtm_test_data.tx_cmd.length,
                                     dtm_test_data.tx_cmd.channel,
                                     dtm_test_data.tx_cmd.phy );
          }

          if(   ( dtm_test_data.eType == eDTM_TYPE_RX )
             && ( dtm_test_data.status.bit.cmd_ready == 1 )
            )
          {
              /* Switch on LED 0 */
              GPIO_PinOutClear(BSP_LED0_PORT, BSP_LED0_PIN);

              printf("\rRX args :\n\rchannel -> %d; phy -> %d\n\r",
                      dtm_test_data.rx_cmd.channel,
                      dtm_test_data.rx_cmd.phy);

              /* Handle the response */
              gecko_cmd_test_dtm_rx( dtm_test_data.rx_cmd.channel,
                                     dtm_test_data.rx_cmd.phy );
          }

          if(   ( dtm_test_data.eType == eDTM_TYPE_EMPTY )
             || ( dtm_test_data.status.bit.cmd_ready == 0 )
            )
          {
                /* Go back advertising in that case */
                gecko_cmd_test_dtm_end();
          }

          break;

      case eDTM_TEST_RUNNING:
          /* Test is happily running, indicate the user via the console */
          break;

      case eDTM_TEST_STOPPING:
          // GPIO_PinOutSet(BSP_LED1_PORT, BSP_LED1_PIN);
          GPIO_PinOutSet(BSP_LED0_PORT, BSP_LED0_PIN);

          /* End test, handle the response ! */
          response_hndl = (void*)gecko_cmd_test_dtm_end();

          /* If the response is positive, go back to the connected state */
          if( 0 != ((struct gecko_msg_test_dtm_end_rsp_t*)response_hndl)->result )
          {
              /* Failure, report it and operate transition to the appropriate state */
          }

          break;

      case eDTM_TEST_PUBLISHING:
          publish_test_result();

          /* Go back to advertising state */
          eState = eDTM_ADVERTISING;

      case eDTM_ADVERTISING:
          /* BLE stack is ready to operate. It is now time to advertise  */
          advertise();

          break;

      default:
          /* Nothing to do in particular for the moment */
          break;
    }
  }
}

/*******************************************************************************
**                           LOCAL ROUTNINE DEFNS                             **
*******************************************************************************/
static eDTM_STATE event_manager(void)
{
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;
    eDTM_STATE ret_val;

    /* Initialize locals */
    ret_val = eState; /* remember the state we are in */
    evt = gecko_wait_event(); /* Check for stack event. Always keep in mind this is a blocking call */

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
        ret_val = eDTM_ADVERTISING; /* Now ready to operate, we can advertise. */
        break;

      case gecko_evt_le_connection_opened_id: /* Now need to handle next events comming up from the stack */
          dtm_connection_data.open = evt->data.evt_le_connection_opened;
          dtm_connection_data.status.bit.connection_active = 1;
          ret_val = eDTM_CONNECTED;
          break;
      case gecko_evt_le_connection_parameters_id:
          dtm_connection_data.parameters = evt->data.evt_le_connection_parameters;
          dtm_connection_data.status.bit.params_set = 1;

          if(    ( 1 == dtm_connection_data.status.bit.connection_active )
              && ( 1 == dtm_connection_data.status.bit.mtu_exch_set )
              && ( 1 == dtm_connection_data.status.bit.params_set )
              && ( 1 == dtm_connection_data.status.bit.phy_set ) )
          {
              /* Mark connection as fully established, we can print the report */
              dtm_connection_data.status.bit.connection_established = 1;
          }

          /* No slave state machine update */

          break;
      case gecko_evt_le_connection_phy_status_id:
          dtm_connection_data.phy_status = evt->data.evt_le_connection_phy_status;
          dtm_connection_data.status.bit.phy_set = 1;
          /* No slave state machine update */
          break;
      case gecko_evt_gatt_mtu_exchanged_id:
          dtm_connection_data.mtu_exchange = evt->data.evt_gatt_mtu_exchanged;
          dtm_connection_data.status.bit.mtu_exch_set = 1;
          /* No slave state machine update */
          break;
      case gecko_evt_gatt_server_user_write_request_id:
          /* ********************** Control characteristics ************************ */
          if ( evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_dtm_timer )
          {
              /* Do some timer related stuff */
              dtm_test_data.test_duration = ((*(evt->data.evt_gatt_server_user_write_request.value.data))*(ONE_S));

              printf("\r Timer setting :\n\rtick(s) -> %ld; second(s) -> %ld;\n\r",
                     dtm_test_data.test_duration,
                     (dtm_test_data.test_duration/ONE_S));
          }

          /* ********************** Transmit arguments ************************ */
          if (   ( evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_dtm_tx_packet_type )
              || ( evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_dtm_tx_length )
              || ( evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_dtm_tx_channel )
              || ( evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_dtm_tx_phy )
             )
          {
              /* We are now receiving the arguments, no slave state machine transition operated here */
              dtm_test_data.eType = eDTM_TYPE_TX;

              parse_tx_args( &evt->data.evt_gatt_server_user_write_request,
                             evt->data.evt_gatt_server_user_write_request.characteristic );
          }

          /* ********************** Receive arguments ************************ */
          if (   ( evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_dtm_rx_channel )
              || ( evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_dtm_rx_phy )
             )
          {
              /* We are now receiving the arguments, no slave state machine transition operated here */
              dtm_test_data.eType = eDTM_TYPE_RX;

              parse_rx_args( &evt->data.evt_gatt_server_user_write_request,
                             evt->data.evt_gatt_server_user_write_request.characteristic );
          }

          /* acknowledge */
          gecko_cmd_gatt_server_send_user_write_response( evt->data.evt_gatt_server_user_write_request.connection,
                                                          evt->data.evt_gatt_server_user_write_request.characteristic,
                                                          0 );
          break;

      case gecko_evt_gatt_server_user_read_request_id:
          if ( evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_dtm_result )
          {
              /* Give back result */
              printf("\rRead request :\n\rcharacteritic -> %d; att_opcode -> %d;\n\r",
                     evt->data.evt_gatt_server_user_read_request.characteristic,
                     evt->data.evt_gatt_server_user_read_request.att_opcode);

             /* answer the request */
             gecko_cmd_gatt_server_send_user_read_response( evt->data.evt_gatt_server_user_read_request.connection,
                                                            evt->data.evt_gatt_server_user_read_request.characteristic,
                                                            0,
                                                            RESPONSE_LEN_BYTES,
                                                            &dtm_test_data.result[0] );
          }

          break;
      case gecko_evt_test_dtm_completed_id:
          /* Get event atomically */
          dtm_test_data.event = evt->data.evt_test_dtm_completed;

          /* This event indicates that the radio has processed a test start or end command. */
          if( eState == eDTM_TEST_STARTING )
          {
              if( dtm_test_data.eType == eDTM_TYPE_EMPTY )
              {
                  ret_val = eDTM_ADVERTISING;
              }
              else
              {
                  /* test is now running */
                  ret_val = eDTM_TEST_RUNNING;
              }
          }

          if(   ( eState == eDTM_TEST_RUNNING )
             || ( eState == eDTM_TEST_STOPPING ) )
          {
              /* The test was either running or has been stopped, publish the result */
              ret_val = eDTM_TEST_PUBLISHING;
          }

          break;

      case gecko_evt_le_connection_closed_id:
          /* Zero out the connection status byte */
          dtm_connection_data.status.byte = 0x00;

          if(   ( eState == eDTM_CONNECTED )
        	 && ( dtm_test_data.status.bit.cmd_ready == 1 )
            )
          {
              /* At this state, the master closes the connection and we proceed to the testing */
              ret_val = eDTM_TEST_STARTING;
          }
          else
          {
              /* Connection is closed - nobody to talk to anymore. Back to advertising */
              ret_val = eDTM_ADVERTISING;
          }

          break;

      /* Timer Event */
      case gecko_evt_hardware_soft_timer_id:
          printf("\rHandle : %d\n\r",
                 evt->data.evt_hardware_soft_timer.handle );

          if( TEST_DURATION_TIMER == evt->data.evt_hardware_soft_timer.handle )
          {
              /* Stop the test */
              ret_val = eDTM_TEST_STOPPING;
          }

          break;

      case gecko_evt_gatt_server_characteristic_status_id :
          /* Nothing to do in particular */
          break;

      default:
          /* Reroute slave state machine to idle. The stack is potentially not able operate. */
          ret_val = eDTM_IDLE;
          break;
    }

    return ret_val;
}

static void advertise(void)
{
    /* Always check the responses */

    /* Set advertising parameters. 100ms advertisement interval.
     * The first parameter is advertising set handle
     * The next two parameters are minimum and maximum advertising interval, both in
     * units of (milliseconds * 1.6).
     * The last two parameters are duration and maxevents left as default. */
    gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

    /* Start general advertising and enable connections. */
    gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);

    /* Need to handle all the things that happen prior to open the connection */

    return;
}

static void publish_connection_data(void)
{
    printf("\rConnection properties :\n\rHandle -> %d; State -> %s, @ -> %.2x:%.2x:%.2x:%.2x:%.2x:%.2x, @ type -> %d, bonding -> %2x, master -> %d\n\r",
            dtm_connection_data.open.connection,
            (dtm_connection_data.status.bit.connection_active?"active":"inactive"),
            dtm_connection_data.open.address.addr[5],
            dtm_connection_data.open.address.addr[4],
            dtm_connection_data.open.address.addr[3],
            dtm_connection_data.open.address.addr[2],
            dtm_connection_data.open.address.addr[1],
            dtm_connection_data.open.address.addr[0],
            dtm_connection_data.open.address_type,
            dtm_connection_data.open.bonding,
            dtm_connection_data.open.master);

    printf("\rParameters :\n\rinterval -> %d; latency -> %d, security mode -> %d, timeout -> %d, txsize -> %d\n\r",
            dtm_connection_data.parameters.interval,
            dtm_connection_data.parameters.latency,
            dtm_connection_data.parameters.security_mode,
            dtm_connection_data.parameters.timeout,
            dtm_connection_data.parameters.txsize);
    return;
}

static void publish_test_result(void)
{
    if( 0 == dtm_test_data.event.result )
    {
        printf("\rSuccess ! (return 0)\n\r");

        if( eDTM_TYPE_RX == dtm_test_data.eType )
        {
            printf("\rNumber of packets received : %d\n\r",
                    dtm_test_data.event.number_of_packets );

            printf("\rChannel : %d\n\r",
                    dtm_test_data.rx_cmd.channel );

            printf("\rPHY used : %s\n\r",
                    phy_desc[dtm_test_data.rx_cmd.phy] );
        }

        if( eDTM_TYPE_TX == dtm_test_data.eType )
        {
            printf("\rNumber of packets transmitted : %d\n\r",
                    dtm_test_data.event.number_of_packets );

            printf("\rLength : %d\n\r",
                    dtm_test_data.tx_cmd.length );

            printf("\rPacket type : %d\n\r",
                    dtm_test_data.tx_cmd.packet_type );

            printf("\rPHY used : %s\n\r",
                    phy_desc[dtm_test_data.tx_cmd.phy] );
        }

        /* Fill in result */
        dtm_test_data.result[0] = (uint8)((dtm_test_data.event.number_of_packets & 0xFF00)>>8);
        dtm_test_data.result[1] = (uint8)(dtm_test_data.event.number_of_packets & 0x00FF);
    }

    /* Oops, need to shut down the test :( */
    dtm_test_data.eType = eDTM_TYPE_EMPTY;

    return;
}

static void parse_tx_args( struct gecko_msg_gatt_server_user_write_request_evt_t *ptr_usr_write_req,
                           uint8 gatt_hndl )
{
    /* Need some basic sanity check */
    if( ptr_usr_write_req != NULL )
    {
        switch(gatt_hndl)
        {
            case gattdb_dtm_tx_packet_type:
                dtm_test_data.tx_cmd.packet_type = *(ptr_usr_write_req->value.data);
                dtm_test_data.status.bit.packet_type = 1;
                break;
            case gattdb_dtm_tx_length:
                dtm_test_data.tx_cmd.length = *(ptr_usr_write_req->value.data);
                dtm_test_data.status.bit.length = 1;
                break;
            case gattdb_dtm_tx_channel:
                dtm_test_data.tx_cmd.channel = *(ptr_usr_write_req->value.data);
                dtm_test_data.status.bit.channel = 1;
                break;
            case gattdb_dtm_tx_phy:
                dtm_test_data.tx_cmd.phy = *(ptr_usr_write_req->value.data);
                dtm_test_data.status.bit.phy = 1;
                break;
            default:
                /* Do nothing */
                break;
        }
    }

    /* Display info in case the comment is complete */
    if (   ( dtm_test_data.status.bit.packet_type == 1 )
           && ( dtm_test_data.status.bit.length == 1 )
           && ( dtm_test_data.status.bit.channel == 1 )
           && ( dtm_test_data.status.bit.phy == 1 )
       )
    {
        dtm_test_data.status.bit.cmd_ready = 1;
    }

    return;
}

static void parse_rx_args( struct gecko_msg_gatt_server_user_write_request_evt_t *ptr_usr_write_req,
                           uint8 gatt_hndl )
{
    /* Need some basic sanity check */
    if( ptr_usr_write_req != NULL )
    {
        switch(gatt_hndl)
        {
            case gattdb_dtm_rx_channel:
                dtm_test_data.rx_cmd.channel = *(ptr_usr_write_req->value.data);
                dtm_test_data.status.bit.channel = 1;
                break;
            case gattdb_dtm_rx_phy:
                dtm_test_data.rx_cmd.phy = *(ptr_usr_write_req->value.data);
                dtm_test_data.status.bit.phy = 1;
                break;
            default:
                /* Do nothing */
                break;
        }
    }

    /* Display info in case the comment is complete */
    if (   ( dtm_test_data.status.bit.channel == 1 )
        && ( dtm_test_data.status.bit.phy == 1 )
       )
    {
        dtm_test_data.status.bit.cmd_ready = 1;
    }

    return;
}

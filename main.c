/***************************************************************************//**
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
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

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "retargetserial.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/* User command
 Include emlib and user library in here.
 */
#include "graphics.h"
#include "lcd_driver.h"
#include "mesh_data.h"
/***********************************************************************************************//**
 * Define for Led
 *
 **************************************************************************************************/
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
/* LED GPIO is active-low */
#define TURN_LED_OFF   GPIO_PinOutSet
#define TURN_LED_ON    GPIO_PinOutClear
#define LED_DEFAULT_STATE  1
#else
/* LED GPIO is active-high */
#define TURN_LED_OFF   GPIO_PinOutClear
#define TURN_LED_ON    GPIO_PinOutSet
#define LED_DEFAULT_STATE  0
#endif
/***********************************************************************************************//**
 *
 **
 **************************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)
		+ BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities =
GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config = { .bluetooth.max_connections =
MAX_CONNECTIONS, .bluetooth.max_advertisers = MAX_ADVERTISERS, .bluetooth.heap =
		bluetooth_stack_heap, .bluetooth.heap_size =
		sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
		.bluetooth.sleep_clock_accuracy = 100, .bluetooth.linklayer_priorities =
				&linklayer_priorities, .gattdb = &bg_gattdb_data,
		.btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
		.pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
		.pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
		.pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
		.max_timers = 16, };

/* User commnad
 * Define header for LCD Graphic
 */
#define MY_APP_HEADER 		"\nFIRMESH TEAM\nBLE MESH THESIS\n******************\n"
#define MY_APP_HEADER_SIZE 	(sizeof(MY_APP_HEADER))
#define APP_KEY_INDEX	0
//Define clock frequency
#define TIMER_CLOCK_FREQ             (uint32) 32768
#define TIMER_MILLIS_SECONDS(ms)     ((TIMER_CLOCK_FREQ * ms)/1000)
#define TIMER_CHECK_LPN_HEART_BEAT 		(uint32) (15*32768)
#define TIMER_CHECK_GATEWAY_HEART_BEAT		(uint32) (15*32768)
//Timer handlers defines
#define TIMER_ID_RESTART        78
#define TIMER_ID_FACTORY_RESET  77
#define TIMER_ID_BLINK_LED      10
#define TIMER_REPEAT		0
//TODO
/*#define TIMER_ID_CHECK_LPN_HEART_BEAT	79
 #define TIMER_ID_CHECK_GATEWAY_HEAT_BEAT 80*/
#define TIMER_ID_CHECK_HEALTH		79
#define TIMER_ID_SEND_MESSAGE  81
/* Define Response flag when send Mesh data */
#define FLAG_NON_RESPONSE          0x00
#define FLAG_RESPONSE              0x01
/*Define led state*/
#define LED_STATE_OFF    		0
#define LED_STATE_ON 			1
/* Define Retransmit flag */
#define FLAG_RETRANS               0x01
#define FLAG_NON_RETRANS           0x00

#define MAX_TIMEOUT 			3
//Global Variable
///Number of active Bluetooth connections
static uint8 num_connections = 0;
///Handle of the last opened LE connection
static uint8 connection_handle = 0xFF;

//static uint8 gateway_health;
static uint16 this_node_address;
static uint16 primary_element = 0;
static uint16 transaction_id = 0;
static uint16 gateway_address = 1;
mesh_data_t* mesh_data_array;
mesh_status_t* mesh_status_array;
//define Status Array
/*static uint16 transaction_id = 0;
 static uint16 primary_element = 0;
 static uint16 secondary_element = 1;
 static uint16 third_element = 2;
 static uint16 response_flag = 0;
 static uint16 gateway_address = 1;
 typedef struct arr {
 uint8 status;
 uint16 address;
 uint8 mess_count;
 uint8 flag;
 } ;
 struct arr status_arr[MESH_CFG_MAX_FRIENDSHIPS];
 */
//lpn_status lpn_status_arr;
static uint8 index = 0;
static uint8 num_lpn = 0;
//mesh_data_t node_data;
//User function
static void button_init();
static void led_init();
void set_device_name(bd_addr *pAddr);
void factory_reset();
void receive_node_init();

static void pri_level_request(uint16_t model_id, uint16_t element_index,
		uint16_t client_addr, uint16_t server_addr, uint16_t appkey_index,
		const struct mesh_generic_request *request, uint32_t transition_ms,
		uint16_t delay_ms, uint8_t request_flags);

static void pri_level_change(uint16_t model_id, uint16_t element_index,
		const struct mesh_generic_state *current,
		const struct mesh_generic_state *target, uint32_t remaining_ms);

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
void mesh_data_init();
int main() {
	// Initialize device
	initMcu();
	// Initialize board
	initBoard();
	// Initialize application
	initApp();

	// Minimize advertisement latency by allowing the advertiser to always
	// interrupt the scanner.
	linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

	gecko_stack_init(&config);
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();

	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	//gecko_bgapi_class_mesh_lpn_init();
	gecko_bgapi_class_mesh_friend_init();

	gecko_initCoexHAL();

	//Init retarget serial to use printf function
	RETARGET_SerialInit();

	//Init button and led
	button_init();
	led_init();
	mesh_data_init();
	//Print to console
	printf("*\r\n*\r\n*\r\n*\r\n*\r\n");
	printf("Welcome to FIRMESH TEAM\r\n");
	printf("This application will provision a device to Mesh network\r\n");
	printf("------------------------------------------------------------\r\n");

	//Init LCD graphics header
	char header_buffer[MY_APP_HEADER_SIZE + 1];
	snprintf(header_buffer, MY_APP_HEADER_SIZE, MY_APP_HEADER);
	LCD_init(header_buffer);

	while (1) {
		struct gecko_cmd_packet *evt = gecko_wait_event();
		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}
	}
}

static void button_init() {
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);
	GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPull, 1);
}

static void led_init() {
	GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(BSP_LED1_PORT, BSP_LED1_PIN, gpioModePushPull, 0);
}

/*static void LED_set_state(int state) {
 switch (state) {
 case LED_STATE_OFF:
 TURN_LED_OFF(BSP_LED0_PORT, BSP_LED0_PIN);
 TURN_LED_OFF(BSP_LED1_PORT, BSP_LED1_PIN);
 break;
 case LED_STATE_ON:
 TURN_LED_ON(BSP_LED0_PORT, BSP_LED0_PIN);
 TURN_LED_ON(BSP_LED1_PORT, BSP_LED1_PIN);
 break;
 default:
 break;
 }
 }*/
void mesh_data_init() {
	mesh_data_array = (uint16 *) malloc(
			sizeof(uint16) * MESH_CFG_MAX_FRIENDSHIPS + 1);
	if (mesh_data_array == NULL) {
		printf("Out of memory !!! \r\n");
		return;
	}

	mesh_data_array[0].unicast_address = gateway_address;
	//gecko_cmd_mesh_node_get_element_address(primary_element)->result;
	//printf("this node address %x",gecko_cmd_mesh_node_get_element_address(0)->address);
	mesh_data_array[0].heart_beat = 1;
	/*mesh_data_array.max_elements = MESH_CFG_MAX_FRIENDSHIPS+1;
	 mesh_data_array.current_elements = 0;
	 */
	mesh_status_array = (mesh_status_t *) malloc(
			sizeof(mesh_status_t) * (MESH_CFG_MAX_FRIENDSHIPS + 1));
	if (mesh_status_array == NULL) {
		printf("Out of memory !!! \r\n");
		return;
	}
	mesh_status_array[0].packet_count = 0;
	mesh_status_array[0].unicast_address = gateway_address;
	//gecko_cmd_mesh_node_get_element_address(primary_element)->result;
	/*struct gecko_msg_mesh_node_get_element_address_rsp_t *element_address;

	 element_address = gecko_cmd_mesh_node_get_element_address(primary_element);
	 if (element_address->result == 0) {
	 gateway_data.unicast_address = element_address->address;
	 printf("Primary Element, Unicast address = %d \r\n", gateway_data.unicast_address);
	 }
	 else {
	 printf("Get Unicast address from Promary element failed !!! \r\n");
	 }*/
	/*struct gecko_msg_mesh_node_get_element_address_rsp_t *element_address;
	 element_address = gecko_cmd_mesh_node_get_element_address(primary_element);
	 if(element_address->result == 0){
	 printf("result == 0\r\n");

	 mesh_status_array[0].unicast_address =element_address->address;
	 }*/
	/*mesh_status_array.max_elements = MESH_CFG_MAX_FRIENDSHIPS+1;
	 mesh_status_array.current_elements = 0;*/
}
void set_device_name(bd_addr *pAddr) {
	char name[20];
	sprintf(name, "Address: %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

	printf("Bluetooth Mesh Device Address: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
			pAddr->addr[5], pAddr->addr[4], pAddr->addr[3], pAddr->addr[2],
			pAddr->addr[1], pAddr->addr[0]);

	LCD_write(name, LCD_ROW_NAME);
}

void factory_reset() {
	LCD_write("*******", LCD_ROW_INFO);
	LCD_write("FACTORY RESET", LCD_ROW_INFO + 1);
	LCD_write("*******", LCD_ROW_INFO + 2);

	printf("***********************************************\r\n");
	printf("*************FACTORY RESET*********************\r\n");
	printf("***********************************************\r\n");

	gecko_cmd_flash_ps_erase_all();
	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

void receive_node_init() {
	uint16 result;

	mesh_lib_init(malloc, free, 8);

	//Re-init primary and secondary element
	primary_element = 0;

	//load PS store address
	/*uint8* address_array;
	 address_array = gecko_cmd_flash_ps_load(0x1234)->value.data;*/
	/*printf("length %d\r\n", gecko_cmd_flash_ps_load(0x1234)->value.len);
	 uint8 i = 0;
	 for (; i < 2; i++) {
	 printf("%x", address_array[i]);
	 }
	 printf("\r\n");
	 this_node_address = address_array[0] + address_array[1] << 8;
	 printf("this node address = %x\r\n", this_node_address);*/
	//Initialize friend function
	printf("Initialize friend function !!! \r\n");

	result = gecko_cmd_mesh_friend_init()->result;
	if (result) {
		printf("Friend init failed !!! \r\n");
	}
	// define gateway health status and timer that check gateway's heartbeat
	/*gateway_health = 5;
	 gecko_cmd_hardware_set_soft_timer(TIMER_CHECK_GATEWAY_HEART_BEAT,
	 TIMER_ID_CHECK_GATEWAY_HEAT_BEAT, TIMER_REPEAT);*/

	/*lpn_status_arr = (lpn_status*) malloc(
	 sizeof(lpn_status) * (MESH_CFG_MAX_FRIENDSHIPS + 1));
	 //init first index for gateway

	 if (lpn_status_arr == NULL) {
	 printf("Out of memory !!! \r\n");
	 return;
	 }
	 {
	 lpn_status_arr[0].address = 1;
	 lpn_status_arr[0].status = 1;
	 lpn_status_arr[0].timeOut = 0;
	 }*/
	printf("Init gateway status\r\n");
	gecko_cmd_hardware_set_soft_timer(15 * 32768,
	TIMER_ID_CHECK_HEALTH, TIMER_REPEAT);
	/*gecko_cmd_hardware_set_soft_timer(3 * 32768, TIMER_ID_SEND_MESSAGE,
	 TIMER_REPEAT);*/
	mesh_lib_generic_server_register_handler(
	MESH_GENERIC_LEVEL_SERVER_MODEL_ID, primary_element, pri_level_request,
			pri_level_change);

}

static void pri_level_request(uint16_t model_id, uint16_t element_index,
		uint16_t client_addr, uint16_t server_addr, uint16_t appkey_index,
		const struct mesh_generic_request *request, uint32_t transition_ms,
		uint16_t delay_ms, uint8_t request_flags) {
	//printf("evt handle\r\n");
	if (request->kind != mesh_generic_request_level) {
		return;
	}
	//printf("1\r\n");
	clear_packet_count(client_addr, mesh_status_array, num_lpn);
	int i = 0;
	for (; i <= num_lpn; i++) {
		printf("%d\t%d\r\n", mesh_status_array[i].unicast_address,
				mesh_status_array[i].packet_count);
	}
	//get index
	//printf("2\r\n");
	int elem_index = get_index(client_addr, mesh_status_array, num_lpn);
	//printf("3\r\n");
	if (elem_index == -1) {
		printf("can't find address\r\n");
		return;
	}
	//printf("4\r\n");
	mesh_data_array[elem_index] = get_mesh_data(request->level);
	//printf("5\r\n");
	if (elem_index != 0) {
		send_mesh_data(FLAG_NON_RESPONSE, FLAG_NON_RETRANS, elem_index);
	}
	//printf("6\r\n");
	/*mesh_data_array[elem_index].
	 struct mesh_generic_request *req;
	 req->kind = mesh_generic_request_level;*/
	/*//TODO
	 uint8 i = 0;
	 for (; i <= num_lpn; i++) {
	 printf("%d\t%d\t%d\t%d\r\n", i, lpn_status_arr[i].address,lpn_status_arr[i].status, lpn_status_arr[i].timeOut);
	 }
	 */
	/*printf("request->level 1 %x\r\n", request->level);
	 struct mesh_generic_request *req;
	 req->kind = mesh_generic_request_level;
	 refresh_timeOut(client_addr, lpn_status_arr, num_lpn);
	 uint8 i = 0;
	 for (; i <= num_lpn; i++) {
	 printf("%d\t%d\t%d\t%d\r\n", i, lpn_status_arr[i].address,lpn_status_arr[i].status, lpn_status_arr[i].timeOut);
	 }
	 //check address co' trong lpn status array;
	 uint8 result = is_friend_or_gateway_address(client_addr, lpn_status_arr, num_lpn);
	 printf("result: %d", result);
	 if(result){
	 printf("request->level 2 %x\r\n", request->level);
	 uint16 new_elem_index = get_lpn_status_index(client_addr,
	 lpn_status_arr, num_lpn);
	 node_data_arr[new_elem_index] = get_mesh_data(request->level);
	 printf("%d\t%d\t%d\t%d\r\n",node_data_arr[new_elem_index].battery_percent,node_data_arr[new_elem_index].heart_beat,node_data_arr[new_elem_index].unicast_address, node_data_arr[new_elem_index].alarm_signal);

	 node_data_arr[new_elem_index].heart_beat = lpn_status_arr[new_elem_index].status;
	 //req->level = set_mesh_data(node_data_arr[new_elem_index]);
	 if(new_elem_index != 0){
	 //node_data.heart_beat = lpn_status_arr[new_elem_index].status;
	 send_mesh_data(FLAG_NON_RESPONSE, FLAG_NON_RETRANS,new_elem_index);
	 }
	 }
	 */
	//uint16 index = get_lpn_status_index (client_addr, lpn_status_arr, num_lpn);
	/*node_data = get_mesh_data(request->level);*/
	/*if (node_data.alarm_signal != 0) {
	 //printf("message receive level %x\r\n", req->level);
	 /*uint16 new_elem_index = get_lpn_status_index(client_addr,
	 lpn_status_arr, num_lpn);
	 node_data.heart_beat = lpn_status_arr[new_elem_index].status;
	 printf("node_data: %d\t%d\t%d\t%d to %d\r\n", node_data.alarm_signal,
	 node_data.battery_percent, node_data.heart_beat,
	 node_data.unicast_address, gateway_address);
	 //node_data = get_mesh_data(request->level);
	 req->level = set_mesh_data(node_data);
	 if (new_elem_index == 0) {
	 send_mesh_data(FLAG_RESPONSE, FLAG_NON_RETRANS, new_elem_index);
	 //printf("level message: %x\r\n", req->level);
	 } else
	 send_mesh_data(FLAG_RESPONSE, FLAG_NON_RETRANS, new_elem_index);
	 }*/
}
static void pri_level_change(uint16_t model_id, uint16_t element_index,
		const struct mesh_generic_state *current,
		const struct mesh_generic_state *target, uint32_t remaining_ms) {
}
void send_mesh_data(uint8 response_flag, uint8 retransmit, int element_index) {
	uint16 resp;
	uint32_t transition_ms = 0;
	uint16_t delay_ms = 0;
	struct mesh_generic_request req;

	printf("Send Mesh Data Function \r\n");
	printf("***********************\r\n");

	req.kind = mesh_generic_request_level;
	req.level = set_mesh_data(mesh_data_array[element_index]);
	if (element_index == 0) {
		req.level &= 0xFF03;
		printf("this node address %x\r\n",gecko_cmd_mesh_node_get_element_address(0)->address);
		req.level |= ((gecko_cmd_mesh_node_get_element_address(0)->address)	& 0x3F) << 2;
	}

	//uint16 test = set_mesh_data(&node_data_arr[element_index]);
	printf("node data %x \r\n", req.level);
	/* Increase transaction_id after each packet sent with non - retransmition */
	if (retransmit == FLAG_NON_RETRANS) {
		transaction_id++;
	}

	/* If LPN - Friend friendship establish, mesh data wil send to friend node */
	/* Else, mesh data will send to 0x0000 address */

	resp = mesh_lib_generic_client_set(
	MESH_GENERIC_LEVEL_CLIENT_MODEL_ID, element_index, gateway_address,
	APP_KEY_INDEX, transaction_id, &req, transition_ms, delay_ms,
			response_flag);

	if (resp) {
		printf("Send Mesh data failed !!! \r\n");
	} else {
		printf("Mesh data sent %x!!! \r\n", req.level);
	}
}

/*void update_and_check_timeOut(lpn_status* array, uint16 num_lpn) {
 uint8 i = 0;
 for (; i <= num_lpn; i++) {
 array[i].timeOut++;
 if (array[i].timeOut > 2) {
 array[i].status = 0;
 if (i == 0)
 gateway_address = 2;
 else
 send_mesh_data(FLAG_NON_RESPONSE, FLAG_NON_RETRANS, i);
 }
 }
 }
 */
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt) {
	uint16 result;
	char buf[30];

	if (evt == NULL) {
		return;
	}

	switch (evt_id) {
	case gecko_evt_system_boot_id:
		if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0
				|| GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
			factory_reset();
		} else {
			struct gecko_msg_system_get_bt_address_rsp_t *pAddr =
					gecko_cmd_system_get_bt_address();

			set_device_name(&pAddr->address);

			result = gecko_cmd_mesh_node_init()->result;

			if (result) {
				sprintf(buf, "Init Failed");

				printf("Bluetooth Mesh Stack Init Failed !!!!!!\r\n");

				LCD_write(buf, LCD_ROW_ERR);
			}
		}
		break;

	case gecko_evt_hardware_soft_timer_id:
		switch (evt->data.evt_hardware_soft_timer.handle) {
		case TIMER_ID_FACTORY_RESET:
			gecko_cmd_system_reset(0);
			break;

		case TIMER_ID_RESTART:
			gecko_cmd_system_reset(0);
			break;

		case TIMER_ID_BLINK_LED:
			GPIO_PinOutToggle(BSP_LED0_PORT, BSP_LED0_PIN);
			GPIO_PinOutToggle(BSP_LED1_PORT, BSP_LED1_PIN);
			break;
			//TODO
		case TIMER_ID_CHECK_HEALTH: {
			printf("CHECK HEALTH\r\n");
			//check gateway
			mesh_status_array[0].packet_count++;
			if (mesh_status_array[0].packet_count > MAX_TIMEOUT) {
				gateway_address = 2;
			}
			send_mesh_data(FLAG_RESPONSE, FLAG_NON_RETRANS, 0);
			uint16 index = 1;
			for (; index <= num_lpn; index++) {
				mesh_status_array[index].packet_count++;
				if (mesh_status_array[index].packet_count > MAX_TIMEOUT) {
					mesh_data_array[index].heart_beat = 0;
				}
				send_mesh_data(FLAG_NON_RESPONSE, FLAG_NON_RETRANS, index);
			}

		}
			/*uint8 elem_index = 1;
			 for (; elem_index <= num_lpn; elem_index++) {
			 lpn_status_arr[elem_index].timeOut++;
			 if(lpn_status_arr[elem_index].timeOut > MAX_TIMEOUT){
			 node_data_arr[elem_index].heart_beat = 0;
			 }
			 printf("CHECK HEALTH\r\n");
			 send_mesh_data(FLAG_NON_RESPONSE, FLAG_NON_RETRANS, elem_index);
			 }
			 //send_mesh_data of this receiver node
			 send_mesh_data(FLAG_RESPONSE, FLAG_NON_RETRANS, 0);

			 }*/
			break;
		default:
			break;
		}
		break;
	case gecko_evt_mesh_node_initialized_id:
		printf("Node initialized !!! \r\n");

		result = gecko_cmd_mesh_generic_server_init()->result;
		if (result) {
			printf("Generic Sever Init failed !!! \r\n");
		}
		result = gecko_cmd_mesh_generic_client_init()->result;
		if (result) {
			printf("Generic Client Init failed !!! \r\n");
		}
		if (!evt->data.evt_mesh_node_initialized.provisioned) {
			LCD_write("Unprovisioned !!!", LCD_ROW_INFO);

			printf("Device Unprovisioned !!!!!!\r\n");

			// The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
			gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
		} else {
			LCD_write("Provisioned !!!", LCD_ROW_INFO);

			printf("Device Provisioned !!!!!!\r\n");

			receive_node_init();
		}
		break;

	case gecko_evt_mesh_node_provisioning_started_id:
		LCD_write("Provisioning...", LCD_ROW_INFO);

		printf("Provisioning Process !!!!!!\r\n");
		gecko_cmd_hardware_set_soft_timer(TIMER_MILLIS_SECONDS(1000),
		TIMER_ID_BLINK_LED, 0);
		break;

	case gecko_evt_mesh_node_provisioned_id:
		LCD_write("Provisioned !!!", LCD_ROW_INFO);

		printf("Device Provisioned !!!!!!\r\n");
		/*this_node_address = evt->data.evt_mesh_node_provisioned.address;*/
		/*uint8* address_array;
		 address_array[0] = this_node_address;*/
		//address_array[1] = this_node_address >> 8;
		/*printf("this node address: %x \r\n", this_node_address);*/
		/*uint16 resp = gecko_cmd_flash_ps_save(0x1234, 16, address_array)->result;*/
		/*printf("resp %x", resp);*/

		receive_node_init();

		gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_BLINK_LED, 1);
		GPIO_PinOutClear(BSP_LED0_PORT, BSP_LED0_PIN);
		GPIO_PinOutClear(BSP_LED1_PORT, BSP_LED1_PIN);
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		LCD_write("Prov Failed !!!", LCD_ROW_INFO);

		printf("Provisioning Process Failed !!!!!!\r\n");

		gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
		break;

	case gecko_evt_mesh_node_key_added_id:
		printf("New key !!!! \r\n");
		break;

	case gecko_evt_mesh_node_model_config_changed_id:
		printf("Mesh node model config changed !!! \r\n");
		break;

	case gecko_evt_mesh_generic_server_client_request_id:
		printf("Receive message from %d  !!! \r\n",
				evt->data.evt_mesh_generic_server_client_request.client_address);
		/*uint16 client_address =
		 evt->data.evt_mesh_generic_server_client_request.client_address;
		 /*	result = is_friend_or_gateway_address(client_address, lpn_status_arr,
		 num_lpn);*/

		mesh_lib_generic_server_event_handler(evt);
		break;

	case gecko_evt_mesh_generic_client_server_status_id:
		printf("Received response");
		break;

	case gecko_evt_mesh_generic_server_state_changed_id:
		printf("Server state changed !!! \r\n");
		mesh_lib_generic_server_event_handler(evt);
		break;

	case gecko_evt_mesh_node_reset_id:
		printf("Event gecko_evt_mesh_node_reset_id !!! \r\n");
		factory_reset();
		break;

	case gecko_evt_mesh_friend_friendship_established_id:
		LCD_write("FRIEND", LCD_ROW_FRIEND_INFOR);
		printf("Event gecko_evt_mesh_friend_friendship_established !!! \r\n");
		num_lpn++;
		printf("num_lpn %d \r\n", num_lpn);
		uint16 new_friendship_address =
				evt->data.evt_mesh_friend_friendship_established.lpn_address;
		if (num_lpn <= MESH_CFG_MAX_FRIENDSHIPS) {
			//uint16 id_in_array = find_id_by_address(new_friendship_address, STATUS_ARRAY);
			mesh_status_array[num_lpn].unicast_address = new_friendship_address;
			mesh_status_array[num_lpn].packet_count = 0;
			mesh_data_array[num_lpn].unicast_address = new_friendship_address;
			mesh_data_array[num_lpn].heart_beat = 1;
			mesh_data_array[num_lpn].battery_percent = 100;
			mesh_data_array[num_lpn].alarm_signal = 0;
			printf("%d\t%d\r\n", mesh_status_array[num_lpn].unicast_address,
					mesh_status_array[num_lpn].packet_count);
		} else {
			printf("Max number of friendship was established");
		}
		//printf("LPN stats:%d\t %d\t%d\r\n", lpn_status_arr[num_lpn].address, lpn_status_arr[num_lpn].timeOut);
		break;

	case gecko_evt_mesh_friend_friendship_terminated_id:
		printf("Event gecko_evt_mesh_friend_friendship_terminated !!!\r\n");
		LCD_write("NO LPN", LCD_ROW_FRIEND_INFOR);
		gecko_cmd_mesh_friend_deinit();
		//clear_lpn_status_arr(lpn_status_arr, num_lpn);
		uint8 index = 1;
		for (; index <= MESH_CFG_MAX_FRIENDSHIPS; index++) {
			mesh_status_array[index].packet_count = 0;
			mesh_status_array[index].unicast_address = 0;

		}
		num_lpn = 0;
		gecko_cmd_mesh_friend_init();
		break;

	case gecko_evt_le_connection_opened_id:
		printf("Open BLE connection !!! \r\n");
		num_connections++;
		connection_handle = evt->data.evt_le_connection_opened.connection;
		LCD_write("Connected !!!", LCD_ROW_CONNECTION);
		break;

	case gecko_evt_le_connection_closed_id:
		if (boot_to_dfu) {
			gecko_cmd_system_reset(2);
		}

		printf("Close BLE connection !!! \r\n");
		connection_handle = 0xFF;
		if (num_connections > 0) {
			if (--num_connections == 0) {
				LCD_write("", LCD_ROW_CONNECTION);
			}
		}
		break;

	case gecko_evt_le_connection_parameters_id:
		printf("BLE connection parameter: interval %d, timeout %d \r\n",
				evt->data.evt_le_connection_parameters.interval,
				evt->data.evt_le_connection_parameters.timeout);
		break;

	case gecko_evt_le_gap_adv_timeout_id:
		break;

	case gecko_evt_gatt_server_user_write_request_id:
		if (evt->data.evt_gatt_server_user_write_request.characteristic
				== gattdb_ota_control) {
			boot_to_dfu = 1;

			gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control, bg_err_success);

			gecko_cmd_le_connection_close(
					evt->data.evt_gatt_server_user_write_request.connection);
		}
		break;

	default:
		break;

	}
}


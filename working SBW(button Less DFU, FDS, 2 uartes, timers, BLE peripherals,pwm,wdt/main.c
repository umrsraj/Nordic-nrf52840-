/*
  Last change: 11 Nov 2020
  --> Adding Calibration factor to the set Temp.
  --> Setting high priority for BSF over HF, SPF, T2F,and APF.
  --> Added WDT.
  --> ADT time removed for IR trigger command.
  --> Added Validation for all flash write variables.
  **********************************************************************************************
  --> sending SPO2 data as "00" after no SPO2 data for 15 sec
  --> validation for spo2 alerts checking
  --> any high priority alert related to BMD - system will set to manual mode and heater percentage will be ZERO.(No storing in Flash)
  --> Corrected the BMD probes fail codes.
  --> added weight calibration with flash storage(with 3 decimal points and range is -2.500 to 2.500);
  --> Closing all alerts and Vital alerts when switch to manual mode from tab.
  --> loop running when baby in onboarded only(except weight)
  --> storing Baby status in Flash.
  --> waiting for ble connetion to send alarm status.
  --> when ble is disconnected more than 15 secs the (mose=manual, heater=0) and (no Flash storage)
  --> validation on reading Temperatures from bmd.
  --> even ble disconnected the buzzed stops buzzing after 3min-- solved.
  --> Added Mode in sendvitals Json.
  --> waiting for the Ble connection to send alert close command.
  --> Not applying the weight calibration factor when weight is 0 from BMD system
  --> BottonLess DFU with 2 UUID's
  -->(17-Dec-20) Resolving Bug on reading The baby_status (MID:105)
  -->(18-Dec-20) coming back to privious mode after closing the High Priority Alerts
  -->(18-Dec-20) if bmd.mode != mode, bring the bmd mode to system mode.
  -->(18-Dec-20) closing the High Priority Alerts.
  -->(21-DEC-20) Resolved bug - ble disconnects after data recieve from ble, solution - we should not sped more time in interrupt handlers.
  -->(26-Dec-20) Resoloved Bug - In manual Mode BMD values not updates if baby not admitted.
  -->(26-Dec-20) Added the alerts(atl,ath, btl,bth) deciding dynamically through factory setting inputs(commands)
*/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
	/////////
#include <stddef.h>
	///////////
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "app_pwm.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//*********************************************
#include "nrf_drv_gpiote.h"
	///////////////
#include "fds.h"
#include "fds_example.h"
#include "nrf_fstorage.h"
	/////////
#include "nrf_drv_wdt.h"
	//|**************************************DFU ***************************************************************************
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"
#include "nrf_bootloader_info.h"
	//************************************************************************************************************************
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_power.h"
#include "nrf_serial.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_util.h"
#include "boards.h"
#define OP_QUEUES_SIZE 3
#define APP_TIMER_PRESCALER NRF_SERIAL_APP_TIMER_PRESCALER
//|*****************************************************************************************************************
#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */
#define DEVICE_NAME "SBW3.0" /**< Name of device. Will be included in the advertising data. */ 
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL 64 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION 18000 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */ 
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS) /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */ 
#define SLAVE_LATENCY 0 /**< Slave latency. */ 
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */ 
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */ 
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */ 
#define MAX_CONN_PARAMS_UPDATE_COUNT 3 /**< Number of attempts before giving up the connection parameter negotiation. */
#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */ 
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */
#define rs485_RE 15
#define rs485_DE 16
#define LED NRF_GPIO_PIN_MAP(1, 1)	//NRF_GPIO_PIN_MAP(0,13)// NRF_GPIO_PIN_MAP(1,1)
#define FLED NRF_GPIO_PIN_MAP(0, 14)	//NRF_GPIO_PIN_MAP(0,13)// NRF_GPIO_PIN_MAP(1,1)
#define IR_PIN NRF_GPIO_PIN_MAP(1, 2)
//////////////For Birdmeditech MODBUS communication//////////////////
#define DEVICE_ADDR 0x01
#define FC_READ 0x03
#define FC_WRITE 0x06
#define buzz NRF_GPIO_PIN_MAP(0, 12)

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr); /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifier. */ {
  {
    BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE
  }
};
//************************* DFU- Functions  *******************************************************************************

/**@brief Function for handling DFU events
 *
 *@details This function is called when entering buttonless DFU
 *
 *@param[in] event Buttonless DFU event.
 */
static void ble_dfu_buttonless_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
  switch (event)
  {
    case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
      NRF_LOG_INFO("Device is preparing to enter bootloader mode\r\n");
      break;
    case BLE_DFU_EVT_BOOTLOADER_ENTER:
      NRF_LOG_INFO("Device will enter bootloader mode\r\n");
      break;
    case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
      NRF_LOG_ERROR("Device failed to enter bootloader mode\r\n");
      break;
    default:
      NRF_LOG_INFO("Unknown event from ble_dfu.\r\n");
      break;
  }
}

/**@brief Function for handling bootloader power management events
 *
 *@details This function is called to set a persistent register which informs the
 *bootloader it should continue or pass control back to the application
 *
 *@param[in] event Power management event.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
  switch (event)
  {
    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
      NRF_LOG_INFO("Power management wants to reset to DFU mode\r\n");
     	// Change this code to tailor to your reset strategy.
     	// Returning false here means that the device is not ready
     	// to jump to DFU mode yet.
     	//
     	// Here is an example using a variable to delay resetting the device:
     	//
      /*if (!im_ready_for_reset)
      {
      return false;
      }

      */
      break;
    default:
     	// Implement any of the other events available
     	// from the power management module:
     	// -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
     	// -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
     	// -NRF_PWR_MGMT_EVT_PREPARE_RESET
      return true;
  }

  NRF_LOG_INFO("Power management allowed to reset to DFU mode\r\n");
  return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);
//|********************************************************************************************************************************************
char *DID = "SBWXXX";
//char *TDID = "SBIXXX";
char *dfu_uname = "EVOV";
char *dfu_pass = "EVOV321";

float slv = 0.5;	//skin low value
float shv = 0.5;	//skin high vlaue
float alv = 3.0;	//air low value
float ahv = 1.5;	//air high value

uint8_t adt = 10;	//alarm delay time
uint8_t Hadt = 5;	//High priority alarm delay time
//uint8_t sft = 1.0;
//uint8_t amt = 0;   	//alarm mute time
float weight_dt = 4;
uint32_t weight_ticks = 0;
uint32_t weight_send_ticks = 0;
uint32_t bmd_ticks = 0;

float blink_dt = 1.0;
float beep_dt = 0.5;
bool blink_led = false;
bool buz_on = false;
uint8_t buz_cnt = 0;

uint32_t buz_ontime = 0;
uint32_t buz_offtime = 0;

uint32_t led_ontime = 0;
uint32_t led_offtime = 0;
//uint32_t nowtime = 0;
bool blink_state = true;
bool buz_state = true;
uint8_t ble_live = 0;

#define admitted 1
#define discharged 2

uint8_t baby_status = admitted;
bool ble_connected = false;
bool ble_fail_process = true;

uint8_t mode = 2;	//default - skin mode
uint8_t real_mode;
uint8_t real_htr;

#define manual_mode 3
//
#define baby_mode       2
#define air_mode 1
#define skin_mode 2
#define no_error 0
#define air_temp_high 2
#define air_temp_low 3
#define baby_temp_high 4
#define baby_temp_low 5
#define baby_temp_cold 6
#define baby_temp_over_range 7
#define cold_stress_onset 8
#define heat_stress_onset 9
#define fever_onset 10
#define baby_infection 11
#define check_skin_probe 12
#define standby_mode 0
#define bnormal_mode 4
#define no_fail 0
#define air_probe_open 3
#define skin_probe_open 1
#define peri_probe_open 2
#define heater_fail 4
#define undefined_fail 5
#define write_fail 6

/************Alarm Flags and ticks variables ************************/
struct allAlerts
{
  uint8_t bth_alert;
  uint8_t btl_alert;
  uint8_t bto_t1_alert;
  uint8_t bto_t3_alert;
  uint8_t btc_alert;
  uint8_t ath_alert;
  uint8_t atl_alert;
 	//  uint8_t ato_alert;
  uint8_t cso_alert;
  uint8_t hso_alert;
  uint8_t fo_alert;
  uint8_t bi_alert;
  uint8_t csp_alert;
  uint8_t cs_alert;
  uint8_t spf_alert;
  uint8_t t2f_alert;
  uint8_t apf_alert;
  uint8_t hf_alert;
  uint8_t sph_alert;
  uint8_t spl_alert;
  uint8_t prh_alert;
  uint8_t prl_alert;
  uint8_t sd_alert;
  uint8_t lm_alert;
  uint8_t lp_alert;
  uint8_t bsf_alert;	//bmd sys fail
  uint8_t spo_alert;	//spo sys fail
  uint8_t blef_alert;
}

alert;

uint32_t send_ble_ticks = 0;
uint32_t flash_ticks = 0;
uint32_t bsf_ticks = 0;
uint32_t spo_ticks = 0;
uint32_t blef_ticks = 0;
uint32_t bth_ticks = 0;
uint32_t btl_ticks = 0;
uint32_t bto_t1_ticks = 0;
uint32_t bto_t3_ticks = 0;
uint32_t btc_ticks = 0;
uint32_t ath_ticks = 0;
uint32_t atl_ticks = 0;
//uint32_t ato_ticks = 0;
uint32_t cso_ticks = 0;
uint32_t hso_ticks = 0;
uint32_t fo_ticks = 0;
uint32_t bi_ticks = 0;
uint32_t csp_ticks = 0;
uint32_t cs_ticks = 0;

uint32_t spf_ticks = 0;
uint32_t t2f_ticks = 0;
uint32_t apf_ticks = 0;
uint32_t hf_ticks = 0;

uint32_t sph_ticks = 0;
uint32_t spl_ticks = 0;
uint32_t prh_ticks = 0;
uint32_t prl_ticks = 0;

uint32_t sd_ticks = 0;
uint32_t lm_ticks = 0;
uint32_t lp_ticks = 0;

uint8_t temp_mode;
uint8_t bmd_mode = 0;
uint8_t Tfail_code = 0;
uint8_t fail_code = 0;

////////////////flags for commands	/////////////////////////
struct set_commands
{
  uint8_t T1skin_setflag;
  uint8_t T3air_setflag;
  uint8_t htr_setflag;
  uint8_t Tweight_setflag;
  uint8_t standby_setflag;
  uint8_t set_flag;
}

set_com;

struct flash_status
{
  uint8_t sT1_flag;
  uint8_t sT3_flag;
  uint8_t sSPO_Ul_flag;
  uint8_t sSPO_Ll_flag;
  uint8_t sPRT_Ul_flag;
  uint8_t sPRT_Ll_flag;
  uint8_t sHTR_flag;
  uint8_t sADT_flag;
  uint8_t sMODE_flag;
  uint8_t cT1_flag;
  uint8_t cT2_flag;
  uint8_t cT3_flag;
  uint8_t cSPO_flag;
  uint8_t cPRT_flag;
  uint8_t cWT_flag;
  uint8_t sBaby_status_flag;
}F_sta;

struct meditech
{
  float T1_skin;
  float T2_peri;
  float T3_air;
  uint8_t htr;
  float deltaT;
  float weight;
  uint8_t mode;
  uint8_t bmdMode;
  uint8_t sens_fail;
}bmd;

struct SPO2
{
  uint8_t spo;
  uint8_t prt;
  char status[3];
}spo;

struct SET_values
{
  float T1;	//Baby set Temperature  (T1)
  float T3;	//Air set Temperature   (T3)
  float Ul;
  float Ll;
  uint8_t htr;	//Air set Temperature   (H)
  uint8_t spo2_Ul;	//SPO2 set Value Upper Limit (SPO2)
  uint8_t spo2_Ll;	//SPO2 set Value Lower Limit (SPO2)
  uint8_t pRate_Ul;	//Pulse rate set Value Upper Limit
  uint8_t pRate_Ll;	//Pulse rate set Value Lower Limit
}set;

struct calibration
{
  float T1_skin;
  float T2_peri;
  float T3_air;
  float spo;
  float prt;
  float wt;
}cal;

//uint8_t ble_sys_fail_time = 5;    	//time to wait for ble data in minutes
//uint8_t bmd_sys_fail_time = 5;    	//time to wait for bmd data in minutes
//bool ble_ack_flag = false;
//bool bmd_ack_flag = false;
uint8_t spo_send_ready = 0;
uint8_t bsf_flag = 1;

APP_TIMER_DEF(bmb_repeated_timer_id);
APP_PWM_INSTANCE(PWM1, 1);	// Create the instance "PWM1" using TIMER1.
nrf_drv_wdt_channel_id m_channel_id;
/**************************************************************************/
void spo_sensor_data_process(void);
void bmd_sensor_data_process(void);
void check_alerts_with_dt(void);
int getCRC(unsigned char message[], int length);

//*************************************************************************************************************
void do_dfu(void)
{
  uint32_t err_code;

 	//#if LEGACY == true
  NRF_POWER->GPREGRET = 0xB1;
  NVIC_SystemReset();
 	//#else
 	//    err_code = sd_power_gpregret_clr(0, 0xffffffff);
 	//    VERIFY_SUCCESS(err_code);

 	//    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
 	//    VERIFY_SUCCESS(err_code);

 	//   	// Signal that DFU mode is to be enter to the power management module
 	//    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
 	//#endif
}

//*********************************************************************************************************
void ath_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"ATH\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void ath_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"ATH\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void atl_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"ATL\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void atl_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"ATL\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

///////////////////////////////////////////////////////////////////////////////////////
void bto_t3_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTO\",\"s\":\"1\",\"V\":\"T3\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void bto_t3_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTO\",\"s\":\"0\",\"V\":\"T3\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void bto_t1_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTO\",\"s\":\"1\",\"V\":\"T1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void bto_t1_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTO\",\"s\":\"0\",\"V\":\"T1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void bth_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTH\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void bth_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTH\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void btl_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTL\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void btl_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTL\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void btc_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTC\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void btc_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BTC\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
void cso_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"CSO\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void cso_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"CSO\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void hso_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"HSO\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void hso_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"HSO\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void fo_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"FO\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void fo_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"FO\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void bi_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BI\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void bi_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"BI\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void csp_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"CSP\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void csp_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"CSP\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void cs_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"CS\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void cs_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"CS\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
void sph_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"SPH\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void sph_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"SPH\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void spl_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"SPL\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void spl_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"SPL\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//////////////////////////////////////////////////////////////////////////////////////
void prh_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"PRH\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void prh_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"PRH\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void prl_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"PRL\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void prl_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"PRL\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void spf_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"SPF\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void spf_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"SPF\",\"s\":\"0\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void t2f_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"T2F\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void t2f_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"T2F\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void apf_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"APF\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void apf_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"APF\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void hf_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"HF\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void hf_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"HF\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void sd_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"SD\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void sd_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"SD\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void lp_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"LP\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void lp_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"LP\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void lm_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"LM\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void lm_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"LM\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void bsf_open(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"BSF\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

void bsf_close(void)
{
  uint8_t buff[128];
  memset(&buff, 0, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"205\",\"AL\":\"BSF\",\"s\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void ir_open(void)
{
  uint8_t buff[128];
  memset(&buff, NULL, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"206\",\"IR\":\"1\"}", DID);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//**********************************************************************************************************************************

void flash_R_t1_t3_htr(uint32_t R_dat)
{
  uint16_t tt3 = 0, tt1 = 0;
  set.htr = 0x000000ff &R_dat;
  tt3 = (0x000fff00 & R_dat) >> 8;
  tt1 = (0xfff00000 & R_dat) >> 20;
  set.T1 = tt1 / 10.0;
  set.T3 = tt3 / 10.0;
  int k = 0;
}

void flash_R_sp_pr(uint32_t R_dat)
{
 	//uint8_t tt3=0,tt2=0,tt1=0;
  set.pRate_Ll = (0x000000ff & R_dat);
  set.pRate_Ul = (0x0000ff00 & R_dat) >> 8;
  set.spo2_Ll = (0x00ff0000 & R_dat) >> 16;
  set.spo2_Ul = (0xff000000 & R_dat) >> 24;
 	// int k=0;
}

void flash_R_3(uint32_t R_dat)
{
  uint8_t cMode = 0, pol = 0, cT1 = 0, cT2 = 0, cT3 = 0;

  mode = (0x0000000f & R_dat);
  pol = (0x000000f0 & R_dat) >> 4;
  cT3 = (0x0000ff00 & R_dat) >> 8;
  cT2 = (0x00ff0000 & R_dat) >> 16;
  cT1 = (0xff000000 & R_dat) >> 24;

  if (pol & 0x01)
  {
    cal.T1_skin = -1 *((float) cT1 / 10.0);
  }
  else
  {
    cal.T1_skin = (float) cT1 / 10.0;
  }

  if (pol & 0x02)
  {
    cal.T2_peri = -1 *((float) cT2 / 10.0);
  }
  else
  {
    cal.T2_peri = (float) cT2 / 10.0;
  }

  if (pol & 0x04)
  {
    cal.T3_air = -1 *((float) cT3 / 10.0);
  }
  else
  {
    cal.T3_air = (float) cT3 / 10.0;
  }

  int k = 0;
}

void flash_R_4(uint32_t R_dat)
{
 	//uint8_t cSP, uint8_t cPR, uint8_t pol, uint8_t Fadt
  uint8_t cSPO = 0, cPRT = 0, pola = 0, Lbaby_status = 6;
  adt = (0x0000000f & R_dat);
  pola = (0x000000f0 & R_dat) >> 4;
  cPRT = (0x0000ff00 & R_dat) >> 8;
  cSPO = (0x00ff0000 & R_dat) >> 16;
  Lbaby_status = (0x0f000000 & R_dat) >> 24;

  if ((Lbaby_status < 10) && (Lbaby_status > 0))
    baby_status = Lbaby_status;

  if (pola & 0x01)
  {
    cal.spo = -1 *((float) cSPO / 10.0);
  }
  else
  {
    cal.spo = (float) cSPO / 10.0;
  }

  if (pola & 0x02)
  {
    cal.prt = -1 *((float) cPRT / 10.0);
  }
  else
  {
    cal.prt = cPRT / 10;
  }

  int k = 0;
}

void flash_R_5(uint32_t R_dat)
{
 	//uint8_t cSP, uint8_t cPR, uint8_t pol, uint8_t Fadt
  uint16_t cWTT = 0, pola = 0;
  pola = (0x0000000f & R_dat);
  cWTT = (0x0000fff0 & R_dat) >> 4;
  if (pola & 0x01)
  {
    cal.wt = -1 *((float) cWTT / 1000.0);
  }
  else
  {
    cal.wt = (float) cWTT / 1000.0;
  }

 	//int k = 0;
}

void flash_R_6(uint32_t R_dat)
{
  uint8_t Lslv = 0, Lshv = 0, Lalv = 0, Lahv = 0;
  Lahv = (0x000000ff & R_dat);
  Lalv = (0x0000ff00 & R_dat) >> 8;
  Lshv = (0x00ff0000 & R_dat) >> 16;
  Lslv = (0xff000000 & R_dat) >> 24;

  if ((Lahv <= 50) && (Lahv >= 5))
    ahv = ((float) Lahv / 10);

  if ((Lalv <= 50) && (Lalv >= 5))
    alv = ((float) Lalv / 10);

  if ((Lslv <= 50) && (Lslv >= 5))
    slv = ((float) Lslv / 10);

  if ((Lshv <= 50) && (Lshv >= 5))
    shv = ((float) Lshv / 10);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t flash_W_1(uint16_t tT1, uint16_t tT3, uint8_t hHtr)
{
  uint32_t w1_buf = 0;	//FFFFFFFF
  w1_buf = w1_buf | tT1;
  w1_buf = (w1_buf << 12) | tT3;
  w1_buf = (w1_buf << 8) | hHtr;

 	// int k = 0;
  return w1_buf;
}

uint32_t flash_W_2(uint8_t s_ul, uint8_t s_ll, uint8_t p_ul, uint8_t p_ll)
{
  uint32_t w2_buf = 0;	//FFFFFFFF
  w2_buf = w2_buf | s_ul;
  w2_buf = (w2_buf << 8) | s_ll;
  w2_buf = (w2_buf << 8) | p_ul;
  w2_buf = (w2_buf << 8) | p_ll;

 	//int k = 0;
  return w2_buf;
}

uint32_t flash_W_3(uint8_t cT1, uint8_t cT2, uint8_t cT3, uint8_t pol, uint8_t mode)
{
  uint32_t w3_buf = 0;	//FFFFFFFF
  w3_buf = w3_buf | cT1;
  w3_buf = (w3_buf << 8) | cT2;
  w3_buf = (w3_buf << 8) | cT3;
  w3_buf = (w3_buf << 4) | pol;
  w3_buf = (w3_buf << 4) | mode;

 	//int k = 0;
  return w3_buf;
}

uint32_t flash_W_4(uint8_t cSP, uint8_t cPR, uint8_t pol, uint8_t Fadt, uint8_t Lbaby_status)
{
  uint32_t w4_buf = 0;	//FFFFFFFF
  w4_buf = w4_buf | Lbaby_status;
  w4_buf = (w4_buf << 8) | cSP;
  w4_buf = (w4_buf << 8) | cPR;
  w4_buf = (w4_buf << 4) | pol;
  w4_buf = (w4_buf << 4) | Fadt;

 	// int k = 0;
  return w4_buf;
}

uint32_t flash_W_5(uint8_t pol, uint16_t cWTt)
{
  uint32_t w5_buf = 0;	//FFFFFFFF
  w5_buf = w5_buf | cWTt;
  w5_buf = (w5_buf << 4) | pol;
 	// int k = 0;
  return w5_buf;
}

uint32_t flash_W_6(uint8_t Wslv, uint8_t Wshv, uint8_t Walv, uint8_t Wahv)
{
  uint32_t w6_buf = 0;	//FFFFFFFF
  w6_buf = w6_buf | Wslv;
  w6_buf = (w6_buf << 8) | Wshv;
  w6_buf = (w6_buf << 8) | Walv;
  w6_buf = (w6_buf << 8) | Wahv;
 	//int k = 0;
  return w6_buf;
}

//******************************************************************************
//float tf = 33.27;
//uint16_t temp;
//uint8_t write_flag = 0;

static ret_code_t fds_test_init(void)
{
  ret_code_t ret = fds_init();
 	// temp = tf * 10;
  if (ret != FDS_SUCCESS)
  {
    return ret;
  }

  return NRF_SUCCESS;
}

static ret_code_t fds_test_write(uint16_t file_num, uint32_t dataa)
{
  static uint32_t my_data[1];
  my_data[0] = dataa;
 	//static uint32_t const n_data[1] =  {0xFFFFF000};
  fds_record_t record;
  fds_record_desc_t record_desc;

 	// Set up data.
  record.data.p_data = my_data;
  record.data.length_words = 1;
 	// Set up record.
  record.file_id = file_num;
  record.key = file_num;

 	// Set up data.
 	//record2.data.p_data = n_data;
 	//record2.data.length_words = 1;
 	// Set up record.
 	//record2.file_id = FILE_ID2;
 	//record2.key = REC_KEY2;

 	//ret_code_t ret =
  while (fds_record_write(&record_desc, &record) != FDS_SUCCESS);
 	//if (ret != FDS_SUCCESS) {
 	//  return ret;
 	//}

 	//ret_code_t ret2 = fds_record_write(&record_desc2, &record2);
 	//if (ret2 != FDS_SUCCESS) {
 	//  return ret2;
 	//}

 	//NRF_LOG_INFO("Writing Record ID = %d \r\n", record_desc.record_id);
 	//write_flag = 5;
  flash_ticks = app_timer_cnt_get();
  return NRF_SUCCESS;
}

uint32_t fds_read(uint16_t file_num)
{
  uint32_t * data;
  fds_flash_record_t flash_record = { 0 };
  fds_record_desc_t record_desc = { 0 };
  fds_find_token_t ftok = { 0 };	//Important, make sure you zero init the ftok token

 	//fds_flash_record_t flash_record2={0};
 	//fds_record_desc_t record_desc2={0};
 	//fds_find_token_t ftok2 = {0};	//Important, make sure you zero init the ftok token
 	//uint32_t * data;
  uint32_t err_code;

 	//NRF_LOG_PRINTF("Start searching... \r\n");
 	// Loop until all records with the given key and file ID have been found.
  while (fds_record_find(file_num, file_num, &record_desc, &ftok) == FDS_SUCCESS)
  {
    err_code = fds_record_open(&record_desc, &flash_record);
    if (err_code != FDS_SUCCESS)
    {
      return err_code;
    }

    data = (uint32_t*) flash_record.p_data;

   	//for (uint8_t i = 0; i <= flash_record.p_header->length_words; i++) {
   	//  NRF_LOG_INFO("0x%8x ", data[i]);
   	//  data[i]=(uint32_t*)flash_record.p_data[i];
   	//}

    err_code = fds_record_close(&record_desc);
    if (err_code != FDS_SUCCESS)
    {
      return err_code;
    }
  }

 	//while (fds_record_find(FILE_ID2, REC_KEY2, &record_desc2, &ftok2) == FDS_SUCCESS) {
 	//err_code = fds_record_open(&record_desc2, &flash_record2);
 	//if (err_code != FDS_SUCCESS) {
 	//  return err_code;
 	//}

 	//data2 = (uint32_t*)flash_record2.p_data;
 	//NRF_LOG_INFO("\r\n");
 	//// Access the record through the flash_record structure.
 	//// Close the record when done.

 	//err_code = fds_record_close(&record_desc2);
 	//if (err_code != FDS_SUCCESS) {
 	//  return err_code;
 	//}

 	//}

  return (*data);
}

//*************************************************************************************************************

float params_To_temp(uint8_t p0, uint8_t p1)
{
  uint16_t hex_value = p0 << 8 | p1;
  float float_value = hex_value;
  return float_value / 100;
}

float params_To_heat(uint8_t p0, uint8_t p1)
{
  return p0 << 8 | p1;
}

float params_To_weight(uint8_t p0, uint8_t p1)
{
  uint16_t hex_value = p0 << 8 | p1;
 	//t_weight = hex_value;
 	//_weight = t_weight;

  if (hex_value <= 32767)
  {
    float w = hex_value;
    return w / 1000;
  }
  else
  {
    float w = hex_value - 65535;
    w = w / 1000;
    return w;
  }
}

uint8_t hex_To_dec(uint8_t hex)
{
  return ((hex >> 4) *16) + (hex & 0x0F);
}

uint32_t app_timer_ms(uint32_t ticks)
{
  float numerator = ((float) APP_TIMER_CONFIG_RTC_FREQUENCY + 1.0f) *1000.0f;
  float denominator = (float) APP_TIMER_CLOCK_FREQ;
  float ms_per_tick = numerator / denominator;

  uint32_t ms = ms_per_tick * ticks;

  return ms;
}

//|******************************************************************************************************************
static void sleep_handler(void)
{
  __WFE();
  __SEV();
  __WFE();
}
#define pin_rx1 NRF_GPIO_PIN_MAP(1, 13)
#define pin_tx1 NRF_GPIO_PIN_MAP(1, 14)
#define pin_rx NRF_GPIO_PIN_MAP(0, 27)	//RX_PIN_NUMBER//NRF_GPIO_PIN_MAP(1,1)
#define pin_tx NRF_GPIO_PIN_MAP(0, 26)	//TX_PIN_NUMBER//NRF_GPIO_PIN_MAP(1,2)

void uart_event_handler0(nrf_serial_t
  const *p_serial, nrf_serial_event_t event);
void uart_event_handler1(nrf_serial_t
  const *p_serial, nrf_serial_event_t event);

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte0_drv_config,
  pin_rx, pin_tx,
  RTS_PIN_NUMBER, CTS_PIN_NUMBER,
  NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
  NRF_UART_BAUDRATE_19200,
  UART_DEFAULT_CONFIG_IRQ_PRIORITY);

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte1_drv_config,
  pin_rx1, pin_tx1,
  RTS_PIN_NUMBER, CTS_PIN_NUMBER,
  NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
  NRF_UART_BAUDRATE_9600,
  UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 256
#define SERIAL_FIFO_RX_SIZE 256

NRF_SERIAL_QUEUES_DEF(serial0_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_QUEUES_DEF(serial1_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);

#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF(serial0_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial1_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial0_config, NRF_SERIAL_MODE_DMA, &serial0_queues, &serial0_buffs, uart_event_handler0, sleep_handler);

NRF_SERIAL_CONFIG_DEF(serial1_config, NRF_SERIAL_MODE_DMA, &serial1_queues, &serial1_buffs, uart_event_handler1, sleep_handler);

NRF_SERIAL_UART_DEF(serial0_uarte, 0);
NRF_SERIAL_UART_DEF(serial1_uarte, 1);

uint8_t spo2_cnt = 0;
uint8_t spo2_err_cnt = 0;
static uint8_t data_array0[256];
uint8_t index0 = 0;
static uint8_t data_array1[256];
uint8_t index1 = 0;
bool bmd_data_check = false;
char temp_buff0[BLE_NUS_MAX_DATA_LEN];
uint8_t follow_length = 0;
uint8_t cmd_length = 0;
char temp_buff[BLE_NUS_MAX_DATA_LEN];

void check_bmd_data(void)
{
  if (index1 > 6)
  {
    for (int pos = 0; pos < BLE_NUS_MAX_DATA_LEN; pos++)
    {
      if (data_array1[pos] == DEVICE_ADDR && data_array1[pos + 1] == FC_READ)	//FC_READ = 0x03
      {
       	//uint8_t
        follow_length = hex_To_dec(data_array1[pos + 2]);
        cmd_length = index1 - pos;
        if (cmd_length > follow_length + 4)
        {
          if (follow_length > 8)	//to differ weight and temps
          {
           	//readtemp_crc = cal_crc(&data_array[pos],12);
           	// Get T1, T2, T3 and heater%
            if (data_array1[pos] == 0x01 && data_array1[pos + 1] == 0x03 && data_array1[pos + 2] == 0x09)
            {
             	//get air temp(T3)
              float Lt3_air = params_To_temp(data_array1[pos + 3], data_array1[pos + 4]);	//high byte and low byte

              if (Lt3_air < 100.0 && Lt3_air >= 0.0)
              {
                bmd.T3_air = Lt3_air;
                bmd.T3_air = bmd.T3_air - cal.T3_air;
              }

             	//get skin temp(T1)
              float Lt1_skin = params_To_temp(data_array1[pos + 5], data_array1[pos + 6]);
              if (Lt1_skin < 100.00 && Lt1_skin >= 0.0)
              {
                bmd.T1_skin = Lt1_skin;
                bmd.T1_skin = bmd.T1_skin - cal.T1_skin;
              }

             	//get heater %
              uint8_t Lhtr = params_To_heat(data_array1[pos + 7], data_array1[pos + 8]);
              if (Lhtr <= 100 && Lhtr >= 0)
              {
                bmd.htr = Lhtr;
              }

             	//get peripherral temp(T2)
              float Lt2_peri = params_To_temp(data_array1[pos + 9], data_array1[pos + 10]);
              if (Lt2_peri < 100.0 && Lt2_peri >= 0.0)
              {
                bmd.T2_peri = Lt2_peri;
                bmd.T2_peri = bmd.T2_peri - cal.T2_peri;
              }

             	//get mode
              bmd.deltaT = (bmd.T1_skin - bmd.T2_peri);

              temp_mode = data_array1[pos + 11];

              if (temp_mode & 0xf0)
              {
                bmd.bmdMode = standby_mode;
                set_com.standby_setflag = 1;
                set_com.set_flag++;
              }
              else
              {
                if (temp_mode == 0x01)
                  bmd.mode = air_mode;
                else if (temp_mode == 0x02)
                  bmd.mode = skin_mode;
                else if (temp_mode == 0x03)
                  bmd.mode = manual_mode;
                else
                  bmd.bmdMode = bnormal_mode;

                set_com.standby_setflag = 0;
              }

              bmd.sens_fail = no_fail;
              if (alert.spf_alert != 0)
              {
                spf_close();
                alert.spf_alert = 0;
              }

              if (alert.t2f_alert != 0)
              {
                t2f_close();
                alert.t2f_alert = 0;
              }

              if (alert.apf_alert != 0)
              {
                apf_close();
                alert.apf_alert = 0;
              }

              if (alert.hf_alert != 0)
              {
                hf_close();
                alert.hf_alert = 0;
              }

              memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);

              for (uint8_t k = 0; k < strlen(data_array1); k++)
              {
                temp_buff[k] = data_array1[k];
              }

              index1 = 0;
              memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
            }	// receive data of 9 byte
            else if (data_array1[pos] == 0x01 && data_array1[pos + 1] == 0x03 && data_array1[pos + 2] == 0x0a)
            {
             	//get air temp(T3)
              float Lt3_air = params_To_temp(data_array1[pos + 3], data_array1[pos + 4]);	//high byte and low byte

              if (Lt3_air < 100.0 && Lt3_air >= 0.0)
              {
                bmd.T3_air = Lt3_air;
                bmd.T3_air = bmd.T3_air - cal.T3_air;
              }

             	//get skin temp(T1)
              float Lt1_skin = params_To_temp(data_array1[pos + 5], data_array1[pos + 6]);
              if (Lt1_skin < 100.00 && Lt1_skin >= 0.0)
              {
                bmd.T1_skin = Lt1_skin;
                bmd.T1_skin = bmd.T1_skin - cal.T1_skin;
              }

             	//get heater %
              uint8_t Lhtr = params_To_heat(data_array1[pos + 7], data_array1[pos + 8]);
              if (Lhtr <= 100 && Lhtr >= 0)
              {
                bmd.htr = Lhtr;
              }

             	//get peripherral temp(T2)
              float Lt2_peri = params_To_temp(data_array1[pos + 9], data_array1[pos + 10]);
              if (Lt2_peri < 100.0 && Lt2_peri >= 0.0)
              {
                bmd.T2_peri = Lt2_peri;
                bmd.T2_peri = bmd.T2_peri - cal.T2_peri;
              }

              bmd.deltaT = (bmd.T1_skin - bmd.T2_peri);

             	//get mode
              temp_mode = data_array1[pos + 11];

              if (temp_mode & 0xf0)
              {
                bmd.bmdMode = standby_mode;
                set_com.standby_setflag = 1;
                set_com.set_flag++;
              }
              else
              {
                if (temp_mode == 0x01)
                  bmd.mode = air_mode;
                else if (temp_mode == 0x02)
                  bmd.mode = skin_mode;
                else if (temp_mode == 0x03)
                  bmd.mode = manual_mode;
                else
                  bmd.bmdMode = bnormal_mode;

                set_com.standby_setflag = 0;
              }

             	//bmd failure code
              Tfail_code = data_array1[pos + 12];

              if (Tfail_code == 0x84)
              {
                bmd.sens_fail = heater_fail;	//4;
                if (alert.hf_alert == 0)
                {
                  real_mode = mode;
                  mode = manual_mode;
                  set.htr = 0;
                  set_com.htr_setflag = 1;
                  set_com.set_flag++;

                  alert.hf_alert = 1;
                }

                if (alert.spf_alert != 0)
                {
                  spf_close();
                  alert.spf_alert = 0;
                }

                if (alert.t2f_alert != 0)
                {
                  t2f_close();
                  alert.t2f_alert = 0;
                }

                if (alert.apf_alert != 0)
                {
                  apf_close();
                  alert.apf_alert = 0;
                }
              }
              else if (Tfail_code == 0x44)
              {
                bmd.sens_fail = peri_probe_open;	//2;
                if (alert.t2f_alert == 0)
                {
                  real_mode = mode;
                  mode = manual_mode;
                  set.htr = 0;
                  set_com.htr_setflag = 1;
                  set_com.set_flag++;

                  alert.t2f_alert = 1;
                }

                if (alert.spf_alert != 0)
                {
                  spf_close();
                  alert.spf_alert = 0;
                }

                if (alert.apf_alert != 0)
                {
                  apf_close();
                  alert.apf_alert = 0;
                }

                if (alert.hf_alert != 0)
                {
                  hf_close();
                  alert.hf_alert = 0;
                }
              }
              else if (Tfail_code == 0x14)
              {
                bmd.sens_fail = skin_probe_open;	//1;
                if (alert.spf_alert == 0)
                {
                  real_mode = mode;
                  mode = manual_mode;
                  set.htr = 0;
                  set_com.htr_setflag = 1;
                  set_com.set_flag++;

                  alert.spf_alert = 1;
                }

                if (alert.t2f_alert != 0)
                {
                  t2f_close();
                  alert.t2f_alert = 0;
                }

                if (alert.apf_alert != 0)
                {
                  apf_close();
                  alert.apf_alert = 0;
                }

                if (alert.hf_alert != 0)
                {
                  hf_close();
                  alert.hf_alert = 0;
                }
              }
              else if (Tfail_code == 0x24)
              {
                bmd.sens_fail = air_probe_open;	//3;
                if (alert.apf_alert == 0)
                {
                  real_mode = mode;
                  mode = manual_mode;
                  set.htr = 0;
                  set_com.htr_setflag = 1;
                  set_com.set_flag++;

                  alert.apf_alert = 1;
                }

                if (alert.spf_alert != 0)
                {
                  spf_close();
                  alert.spf_alert = 0;
                }

                if (alert.t2f_alert != 0)
                {
                  t2f_close();
                  alert.t2f_alert = 0;
                }

                if (alert.hf_alert != 0)
                {
                  hf_close();
                  alert.hf_alert = 0;
                }
              }
              else
              {
                bmd.sens_fail = undefined_fail;	//5;
              }

              memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);

              for (uint8_t k = 0; k < strlen(data_array1); k++)
              {
                temp_buff[k] = data_array1[k];
              }

              index1 = 0;
              memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
            }	//if receive data of 0x0a bytes
            if (bmd.mode != mode)
            {
              if (mode == air_mode)
              {
                set_com.T3air_setflag = 1;
              }
              else if (mode == skin_mode)
              {
                set_com.T1skin_setflag = 1;
              }
              else if (mode == manual_mode)
              {
                set_com.htr_setflag = 1;
              }
            }
          }	// if temperature read command reply
          else
          {
           	//Get weight
            if (data_array1[pos] == 0x01 && data_array1[pos + 1] == 0x03 && data_array1[pos + 2] == 0x02)
            {
             	//get weight
             	//weight[0] = data_array[pos+3];
             	//weight[1] = data_array[pos+4];
              float Lweight = params_To_weight(data_array1[pos + 3], data_array1[pos + 4]);
              if (Lweight < 50.0 && Lweight > -50.0)
              {
                bmd.weight = Lweight;
                if (bmd.weight != 0)
                  bmd.weight = bmd.weight - cal.wt;
              }

             	//***************************************************
              uint8_t buff[128];
              memset(&buff, NULL, 128);
              sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"202\",\"WT\":\"%.3f\"}", DID, bmd.weight);
              uint16_t length = (uint16_t) strlen(buff);
              ret_code_t ret;
              do {
                ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
              } while (ret != NRF_SUCCESS && ble_live != 0);
             	//***************************************************

              memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);
              for (uint8_t k = 0; k < strlen(data_array1); k++)
              {
                temp_buff[k] = data_array1[k];
              }

              index1 = 0;
              memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
              weight_ticks = app_timer_cnt_get();
            }
          }	//weight
        }	//wait for complete response
      }	//read command
      else if (data_array1[pos] == DEVICE_ADDR && data_array1[pos + 1] == FC_WRITE)	//FC_WRITE = 0x06
      {
       	//set t3 success
        if (data_array1[pos] == 0x01 && data_array1[pos + 1] == 0x06 && data_array1[pos + 2] == 0x00 && data_array1[pos + 3] == 0x00 && baby_status != discharged)
        {
          set_com.T3air_setflag = 3;
          set_com.set_flag--;

          memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);

          for (uint8_t k = 0; k < strlen(data_array1); k++)
          {
            temp_buff[k] = data_array1[k];
          }

          index1 = 0;
          memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
        }
        else if (data_array1[pos] == 0x01 && data_array1[pos + 1] == 0x06 && data_array1[pos + 2] == 0x00 && data_array1[pos + 3] == 0x01)
        {
          set_com.T1skin_setflag = 3;
          set_com.set_flag--;

          memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);

          for (uint8_t k = 0; k < strlen(data_array1); k++)
          {
            temp_buff[k] = data_array1[k];
          }

          index1 = 0;
          memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
        }
        else if (data_array1[pos] == 0x01 && data_array1[pos + 1] == 0x06 && data_array1[pos + 2] == 0x00 && data_array1[pos + 3] == 0x02)
        {
          set_com.htr_setflag = 3;
          set_com.set_flag--;

          memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);

          for (uint8_t k = 0; k < strlen(data_array1); k++)
          {
            temp_buff[k] = data_array1[k];
          }

          index1 = 0;
          memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
        }
        else if (data_array1[pos] == 0x01 && data_array1[pos + 1] == 0x06 && data_array1[pos + 2] == 0x00 && data_array1[pos + 3] == 0x03)
        {
          set_com.Tweight_setflag = 3;
          set_com.set_flag--;

          memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);

          for (uint8_t k = 0; k < strlen(data_array1); k++)
          {
            temp_buff[k] = data_array1[k];
          }

          index1 = 0;
          memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
        }
        else if (data_array1[pos] == 0x01 && data_array1[pos + 1] == 0x06 && data_array1[pos + 2] == 0x00 && data_array1[pos + 3] == 0x04)
        {
          set_com.standby_setflag = 0;
          set_com.set_flag--;

          memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);

          for (uint8_t k = 0; k < strlen(data_array1); k++)
          {
            temp_buff[k] = data_array1[k];
          }

          index1 = 0;
          memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
        }
      }	//write command
      else if (data_array1[pos] == DEVICE_ADDR && data_array1[pos + 1] == 0x86)
      {
        fail_code = write_fail;

        memset(temp_buff, 0, BLE_NUS_MAX_DATA_LEN);

        for (uint8_t k = 0; k < strlen(data_array1); k++)
        {
          temp_buff[k] = data_array1[k];
        }

        index1 = 0;
        memset(data_array1, 0, BLE_NUS_MAX_DATA_LEN);
      }

      bsf_ticks = app_timer_cnt_get();
    }	//for loop
  }	//min 7 bytes
}

void uart_event_handler0(nrf_serial_t
  const *p_serial, nrf_serial_event_t event)
{
  uint32_t err_code;
 	// static uint8_t index = 0;
  char bb;
  switch (event)
  {
    case NRF_SERIAL_EVENT_RX_DATA:
     	//NRF_LOG_INFO("received -0");
      nrf_serial_read(&serial0_uarte, &bb, sizeof(bb), NULL, 100);
      data_array0[index0++] = bb;

      if (index0 > 65)
        index0 = 0;

      if ((data_array0[index0 - 1] == 0x0A) && (data_array0[index0 - 2] == 0x0D))
      {
        if (index0 > 1)
        {
          uint8_t diff = 0;
          uint8_t current_pos = 0;

          char *first_colon;
          char *second_colon;

          uint8_t len = strlen(data_array0);

          for (int i = 0; i < len; i++)
          {
            if (data_array0[i] == 0x00)
            {
              for (int j = i; j < len; j++)
              {
                data_array0[j] = data_array0[j + 1];
              }
            }

            len = strlen(data_array0);
          }

          first_colon = strchr(data_array0, ':');
          second_colon = strchr(first_colon + 1, ':');

          diff = second_colon - first_colon;

          if (diff < 5)
          {
           	//char sp[4],pr[4],st[4];
           	//sp[3] = '\0';
           	//pr[3] = '\0';

           	//sp[0] = data_array0[22];
           	//sp[1] = data_array0[23];
           	//sp[2] = data_array0[24];

           	//pr[0] = data_array0[29];
           	//pr[1] = data_array0[30];
           	//pr[2] = data_array0[31];

           	//st[0] = data_array0[48];
           	//st[1] = data_array0[49];

            char *temp;
            char *temp1;
            char *temp2;
            char *temp3;
            char *temp4;
            char *st;

           	//uint8_t sss = 0;
           	//uint8_t bbb = 0;

            temp = strtok(second_colon + 1, " ");
            temp1 = strtok(NULL, " ");
            temp2 = strtok(NULL, " ");
            temp3 = strtok(NULL, " ");
            temp4 = strtok(NULL, " ");

            uint8_t sss = 0;
            uint8_t bbb = 0;

            sss = atoi(temp1);
            bbb = atoi(temp2);
            st = temp4;
           	//spo2_cnt1++;

           	//sss = atoi((const char*)&sp);
           	//bbb = atoi((const char*)&pr);

            if ((sss > 9) && (bbb > 9))
            {
              spo.spo = sss - cal.spo;
              spo.prt = bbb - cal.prt;
              spo_send_ready = 1;
            }

            memcpy(spo.status, st, sizeof(st));

            if (strstr(spo.status, "SD"))
            {
              if (alert.sd_alert == 0)
              {
                alert.sd_alert = 1;
              }

              if (alert.lm_alert != 0)
              {
                lm_close();
                alert.lm_alert = 0;
              }

              if (alert.lp_alert != 0)
              {
                lp_close();
                alert.lp_alert = 0;
              }
            }
            else if (strstr(spo.status, "LM"))
            {
              if (alert.lm_alert == 0)
              {
                alert.lm_alert = 1;
              }

              if (alert.sd_alert != 0)
              {
                sd_close();
                alert.sd_alert = 0;
              }

              if (alert.lp_alert != 0)
              {
                lp_close();
                alert.lp_alert = 0;
              }
            }
            else if (strstr(spo.status, "LP"))
            {
              if (alert.lp_alert == 0)
              {
                alert.lp_alert = 1;
              }

              if (alert.lm_alert != 0)
              {
                lm_close();
                alert.lm_alert = 0;
              }

              if (alert.sd_alert != 0)
              {
                sd_close();
                alert.sd_alert = 0;
              }
            }
            else
            {
              if (alert.lm_alert != 0)
              {
                lm_close();
                alert.lm_alert = 0;
              }

              if (alert.lp_alert != 0)
              {
                lp_close();
                alert.lp_alert = 0;
              }

              if (alert.sd_alert != 0)
              {
                sd_close();
                alert.sd_alert = 0;
              }
            }

           	//spo2_cnt2++;

           	//spo.status = st;
           	//for(int k=0; k < 4; k++){
           	//spo.status[k] = st[k];
           	//}

           	// int jj = 0;
            memset(temp_buff0, 0, BLE_NUS_MAX_DATA_LEN);

           	//for (uint8_t k = 0; k < strlen(data_array0); k++)
           	//{
           	//  temp_buff0[k] = data_array0[k];
           	//}

            index0 = 0;
            memset(data_array0, 0, BLE_NUS_MAX_DATA_LEN);
           	//spo_send_ready = 1;
            spo2_cnt++;
            spo_ticks = app_timer_cnt_get();
            alert.spo_alert = 0;

           	// ret_code_t ret;

           	//ret = nrf_serial_uninit(&serial0_uarte);
           	//APP_ERROR_CHECK(ret);

           	//ret = nrf_serial_init(&serial0_uarte, &m_uarte0_drv_config, &serial0_config);
           	//APP_ERROR_CHECK(ret);
          }
        }
      }

      break;
    case NRF_SERIAL_EVENT_TX_DONE:
     	//NRF_LOG_INFO("Send-0");
      break;
    case NRF_SERIAL_EVENT_DRV_ERR:
      spo2_cnt++;
      APP_ERROR_CHECK(nrf_serial_uninit(&serial0_uarte));
      ret_code_t ret;
      ret = nrf_serial_init(&serial0_uarte, &m_uarte0_drv_config, &serial0_config);
      APP_ERROR_CHECK(ret);
      spo2_err_cnt++;
      break;
    case NRF_SERIAL_EVENT_FIFO_ERR:
      err_code++;
      break;
    default:
      err_code++;
      break;
  }
}

void uart_event_handler1(nrf_serial_t
  const *p_serial, nrf_serial_event_t event)
{
  uint32_t err_code;
  char bb;
  switch (event)
  {
    case NRF_SERIAL_EVENT_RX_DATA:
     	//NRF_LOG_INFO("received - 1");
      nrf_serial_read(&serial1_uarte, &bb, sizeof(bb), NULL, 2000);
      data_array1[index1++] = bb;
      bmd_data_check = true;
      bsf_flag = 1;

      break;
    case NRF_SERIAL_EVENT_TX_DONE:
     	//NRF_LOG_INFO("Sent-1");
      break;
    case NRF_SERIAL_EVENT_DRV_ERR:
      spo2_cnt++;
      APP_ERROR_CHECK(nrf_serial_uninit(&serial1_uarte));
      ret_code_t ret;
      ret = nrf_serial_init(&serial1_uarte, &m_uarte1_drv_config, &serial1_config);
      APP_ERROR_CHECK(ret);
      spo2_err_cnt++;
      break;
    case NRF_SERIAL_EVENT_FIFO_ERR:
      err_code++;
      break;
    default:
      err_code++;
      break;
  }
}

//|*************************************************************************************************************************

/**@brief Function for assert macro callback.

   @details This function will be called in case of an assert in the SoftDevice.

   @warning This handler is an example only and does not fit a final product. You need to analyse
            how your product is supposed to react in case of Assert.
   @warning On assert from the SoftDevice, the system can only recover on reset.

   @param[in] line_num    Line number of the failing ASSERT call.
   @param[in] p_file_name File name of the failing ASSERT call.
*/
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.

   @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
            the device. It also sets the permissions and appearance.
*/
static void gap_params_init(void)
{
  uint32_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
    (const uint8_t *) DEVICE_NAME,
    strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.

   @details A pointer to this function will be passed to each service which may need to inform the
            application about an error.

   @param[in]   nrf_error   Error code containing information about what went wrong.
*/
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

void close_spo_alarms(void)
{
  if (alert.sph_alert != 0)
  {
    sph_close();
    alert.sph_alert = 0;
  }

  if (alert.spl_alert != 0)
  {
    spl_close();
    alert.spl_alert = 0;
  }

  if (alert.prh_alert != 0)
  {
    prh_close();
    alert.prh_alert = 0;
  }

  if (alert.prl_alert != 0)
  {
    prl_close();
    alert.prl_alert = 0;
  }
}

void close_general_alarms(void)
{
  if (alert.cso_alert != 0)
  {
    cso_close();
    alert.cso_alert = 0;
  }

  if (alert.hso_alert != 0)
  {
    hso_close();
    alert.hso_alert = 0;
  }

  if (alert.fo_alert != 0)
  {
    fo_close();
    alert.fo_alert = 0;
  }

  if (alert.bi_alert != 0)
  {
    bi_close();
    alert.bi_alert = 0;
  }

  if (alert.csp_alert != 0)
  {
    csp_close();
    alert.csp_alert = 0;
  }

  if (alert.cs_alert != 0)
  {
    cs_close();
    alert.cs_alert = 0;
  }
}

void close_vital_alarms(void)
{
  if (alert.ath_alert != 0)
  {
    ath_close();
    alert.ath_alert = 0;
  }

  if (alert.atl_alert != 0)
  {
    atl_close();
    alert.atl_alert = 0;
  }

  if (alert.bto_t3_alert != 0)
  {
    bto_t3_close();
    alert.bto_t3_alert = 0;
  }

 	////////////////////////////////////////////////////////////////////////////////////////////////////
  if (alert.bth_alert != 0)
  {
    bth_close();
    alert.bth_alert = 0;
  }

  if (alert.btl_alert != 0)
  {
    btl_close();
    alert.btl_alert = 0;
  }

  if (alert.btc_alert != 0)
  {
    btc_close();
    alert.btc_alert = 0;
  }

  if (alert.bto_t1_alert != 0)
  {
    bto_t1_close();
    alert.bto_t1_alert = 0;
  }
}	//close_vital_alarms
int8_t ble_data_process(const char *ble_buff, uint8_t length)
{
  char buff[256];
  uint8_t len = length;

  uint8_t current_pos = 0;

  char did[7];
  did[6] = '\n';
  char mid[4];
  int mid_i = 0;
 	//uint8_t bitt = 9;
  char buff1[256];

 	//{"DID": "SBWXXX", "MID":"101", "ADT":"2", "AMT":"5" }

  memset(buff, 0, len);
  memcpy(buff, ble_buff, len);

  for (int i = 0; i < len; i++)
  {
    if (buff[i] == '"')
    {
      for (int j = i; j < len; j++)
      {
        buff[j] = buff[j + 1];
      }
    }

    len = strlen(buff);
  }

  len = strlen(buff);
  int lenn = len - 1;
  char str1[256];
  for (int i = 1; i < lenn; i++)
  {
    str1[i - 1] = buff[i];
  }

  typedef struct data
  {
    char *string;
    char *variable;
    char *value;
  }

  parsing;
  parsing parse[10];

  parse[0].string = strtok(str1, ",");

  int c = 0;

  for (int i = 1; parse[i - 1].string != NULL; i++)
  {
    c = i;
    parse[i].string = strtok(NULL, ",");
  }

  for (int i = 0; i < c; i++)
  {
    parse[i].variable = strtok(parse[i].string, ":");
    parse[i].value = strtok(NULL, ":");
  }

  char tagg = *(parse[2].value);
  if (strstr(parse[0].variable, "NID"))
  {
    for (int j = 0; j < 6; j++)
      DID[j] = parse[0].value[j];
    return 1;
  }
  else if (strstr(parse[0].variable, "UNAME"))
  {
    char *Luname = strstr(parse[0].value, dfu_uname);
    if (Luname == NULL)
      return 0;

    char *Lpass = strstr(parse[1].value, dfu_pass);
    if (Lpass == NULL)
      return 0;

    do_dfu();

  }

  char *tDID = strstr(parse[0].value, DID);
  if (tDID == NULL)
  {
    uint8_t buff[128];
    memset(&buff, NULL, 128);
    sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"203\",\"AL\":\"DIDF\"}", DID);	//DID Fail
    uint16_t length = (uint16_t) strlen(temp_buff);
    ret_code_t ret;
    do {
      ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
    } while (ret != NRF_SUCCESS && ble_live != 0);
    return 0;
  }
  else
  {
    if (strstr(parse[1].value, "101"))	//mid
    {
      uint8_t pol = 0, cSP = 0, cPR = 0, Ladt = 0;
      Ladt = atoi(parse[2].value);

      if (Ladt >= 0 && Ladt <= 10)
      {
        adt = Ladt;

        cSP = abs(cal.spo *10);
        cPR = abs(cal.prt *10);

        if (cal.spo < 0)
          pol = pol | 0x01;
        if (cal.prt < 0)
          pol = pol | 0x02;

        F_sta.sADT_flag = 1;

        fds_test_write(4, flash_W_4(cSP, cPR, pol, adt, baby_status));
        F_sta.sADT_flag = 2;
      }	//adt validation

    }	//if MID 101
    else if (strstr(parse[1].value, "102"))	//mid
    {
      uint8_t cT1 = 0, cT2 = 0, cT3 = 0, pol = 0;

      cT3 = abs(cal.T3_air *10);
      cT2 = abs(cal.T2_peri *10);
      cT1 = abs(cal.T1_skin *10);

      if (cal.T1_skin < 0)
        pol = pol | 0x01;
      if (cal.T2_peri < 0)
        pol = pol | 0x02;
      if (cal.T3_air < 0)
        pol = pol | 0x04;

      if (strstr(parse[2].value, "1"))	//manual mode
      {
        uint8_t Lhtr = atoi(parse[4].value);
        if (Lhtr >= 0 && Lhtr <= 100)
        {
          if (mode != manual_mode)
          {
            close_vital_alarms();
            close_general_alarms();
            blink_led = false;
            nrf_gpio_pin_write(LED, 0);
            led_offtime = app_timer_cnt_get();
          }
          real_mode = manual_mode;
          mode = manual_mode;
        //////////////////////////
          set.htr = Lhtr;
          real_htr = Lhtr;
          set_com.htr_setflag = 1;
          set_com.set_flag++;

          F_sta.sMODE_flag = 1;
          F_sta.sHTR_flag = 1;

          fds_test_write(1, flash_W_1(set.T1 * 10, set.T3 * 10, set.htr));
          F_sta.sHTR_flag = 2;

         	//fds_test_write(3,flash_W_3(cT1,cT2,cT3,pol,mode));
         	//F_sta.sMODE_flag = 2;
        }	//htr validation
      }
      else if (strstr(parse[3].value, "1"))	//baby mode
      {
        float sT3, sT1;
        sT3 = atof(parse[5].value);
        sT1 = atof(parse[6].value);
        if (sT3 != 0.0 && sT3 <= 39.00)	//Air_mode
        {
          if (mode != air_mode)
            close_vital_alarms();

          real_mode = air_mode;
          mode = air_mode;
          set_com.T3air_setflag = 1;
          set.T3 = (sT3 - cal.T3_air);
          set_com.set_flag++;

          F_sta.sT3_flag = 1;
          F_sta.sMODE_flag = 1;

          fds_test_write(1, flash_W_1(set.T1 * 10, set.T3 * 10, set.htr));
          F_sta.sT3_flag = 2;

         	//fds_test_write(3,flash_W_3(cT1,cT2,cT3,pol,mode));
         	//F_sta.sMODE_flag = 2;
        }
        else if (sT1 != 0.0 && sT1 <= 38.00)	//skin_mode
        {
          if (mode != skin_mode)
            close_vital_alarms();

          real_mode = skin_mode;
          mode = skin_mode;
          set_com.T1skin_setflag = 1;
          set.T1 = (sT1 - cal.T1_skin);
          set_com.set_flag++;

          set.Ul = set.T1 + 0.3;
          set.Ll = set.T1 - 0.3;

          F_sta.sT1_flag = 1;
          F_sta.sMODE_flag = 1;

          fds_test_write(1, flash_W_1(set.T1 * 10, set.T3 * 10, set.htr));
          F_sta.sT1_flag = 2;
         	// nrf_delay_ms(200);
         	//fds_test_write(3,flash_W_3(cT1,cT2,cT3,pol,mode));
         	//F_sta.sMODE_flag = 2;
         	//nrf_delay_ms(200);
        }
      }	//baby mode
    }	//102
    else if (strstr(parse[1].value, "103"))	//mid
    {
      uint8_t Lspl = 0, Lspg = 0, Lprl = 0, Lprg = 0;

      Lspl = atoi(parse[2].value);
      Lspg = atoi(parse[3].value);
      Lprl = atoi(parse[4].value);
      Lprg = atoi(parse[5].value);
      if (((Lspl >= 0) && (Lspl <= 250)) &&
        ((Lspg >= 0) && (Lspg <= 250)) &&
        ((Lprl >= 0) && (Lprl <= 250)) &&
        ((Lprg >= 0) && (Lprg <= 250)))
      {
        set.spo2_Ll = Lspl;
        set.spo2_Ul = Lspg;
        set.pRate_Ll = Lprl;
        set.pRate_Ul = Lprg;

        F_sta.sSPO_Ul_flag = 1;
        F_sta.sSPO_Ll_flag = 1;
        F_sta.sPRT_Ul_flag = 1;
        F_sta.sPRT_Ll_flag = 1;

        fds_test_write(2, flash_W_2(set.spo2_Ul, set.spo2_Ll, set.pRate_Ul, set.pRate_Ll));
        F_sta.sSPO_Ul_flag = 2;
        F_sta.sSPO_Ll_flag = 2;
        F_sta.sPRT_Ul_flag = 2;
        F_sta.sPRT_Ll_flag = 2;
      }	//validation
    }	//mid = 103
    else if (strstr(parse[1].value, "104"))	//mid
    {
      if (strstr(parse[2].variable, "TW"))
      {
        set_com.Tweight_setflag = 1;
        set_com.set_flag++;
      }
      else
      {
        uint8_t cT1 = 0, cT2 = 0, cT3 = 0, cSP = 0, cPR = 0, pol = 0;
        cT1 = abs(cal.T1_skin *10);
        cT2 = abs(cal.T2_peri *10);
        cT3 = abs(cal.T3_air *10);

        cPR = abs(cal.prt *10);
        cSP = abs(cal.spo *10);

       	//if(cal.T1_skin < 0)
       	//  pol = pol|0x01
       	//if(cal.T2_peri < 0)
       	//  pol = pol|0x02
       	//if(cal.T3_air < 0)
       	//  pol = pol|0x04

        if (strstr(parse[2].variable, "CPT"))
        {
          float cpt = atof(parse[2].value);
          if ((cpt <= 2.5) && (cpt >= -2.5))
          {
            cal.T2_peri = cpt;
            cT2 = abs(cal.T2_peri *10);

            if (cal.T1_skin < 0)
              pol = pol | 0x01;
            if (cal.T2_peri < 0)
              pol = pol | 0x02;
            if (cal.T3_air < 0)
              pol = pol | 0x04;

            F_sta.cT2_flag = 1;

            fds_test_write(3, flash_W_3(cT1, cT2, cT3, pol, mode));
            F_sta.cT2_flag = 2;
          }	//validation
        }
        else if (strstr(parse[2].variable, "CBT"))
        {
          float cbt = atof(parse[2].value);
          if ((cbt <= 2.5) && (cbt >= -2.5))
          {
            cal.T1_skin = cbt;
            cT1 = abs(cal.T1_skin *10);

            if (cal.T1_skin < 0)
              pol = pol | 0x01;
            if (cal.T2_peri < 0)
              pol = pol | 0x02;
            if (cal.T3_air < 0)
              pol = pol | 0x04;

            F_sta.cT1_flag = 1;

            fds_test_write(3, flash_W_3(cT1, cT2, cT3, pol, mode));
            F_sta.cT1_flag = 2;
          }	//validation
        }
        else if (strstr(parse[2].variable, "CAT"))
        {
          float cat = atof(parse[2].value);
          if ((cat <= 2.5) && (cat >= -2.5))
          {
            cal.T3_air = cat;
            cT3 = abs(cal.T3_air *10);

            if (cal.T1_skin < 0)
              pol = pol | 0x01;
            if (cal.T2_peri < 0)
              pol = pol | 0x02;
            if (cal.T3_air < 0)
              pol = pol | 0x04;

            F_sta.cT3_flag = 1;

            fds_test_write(3, flash_W_3(cT1, cT2, cT3, pol, mode));
            F_sta.cT3_flag = 2;
          }	//validation
        }
        else if (strstr(parse[2].variable, "CSPO"))
        {
          float csp = atof(parse[2].value);
          if ((csp <= 2.5) && (csp >= -2.5))
          {
            cal.spo = csp;
            cSP = abs(cal.spo *10);

            if (cal.spo < 0)
              pol = pol | 0x01;
            if (cal.prt < 0)
              pol = pol | 0x02;

            F_sta.cSPO_flag = 1;

            fds_test_write(4, flash_W_4(cSP, cPR, pol, adt, baby_status));
            F_sta.cSPO_flag = 2;
          }	//validation
        }
        else if (strstr(parse[2].variable, "CPRT"))
        {
          float cpr = atof(parse[2].value);
          if ((cpr <= 2.5) && (cpr >= -2.5))
          {
            cal.prt = cpr;
            cPR = abs(cal.prt *10);

            if (cal.spo < 0)
              pol = pol | 0x01;
            if (cal.prt < 0)
              pol = pol | 0x02;

            F_sta.cPRT_flag = 1;

            fds_test_write(4, flash_W_4(cSP, cPR, pol, adt, baby_status));
            F_sta.cPRT_flag = 2;
          }	//validation
        }	//cprt
        else if (strstr(parse[2].variable, "CWT"))
        {
          float cwt = atof(parse[2].value);
          if ((cwt <= 2.500) && (cwt >= -2.500))
          {
            uint16_t cweight = 0;
            cal.wt = cwt;
            cweight = abs(cal.wt *1000);
            if (cal.wt < 0.000)
              pol = pol | 0x01;
            F_sta.cWT_flag = 1;
            fds_test_write(5, flash_W_5(pol, cweight));
            F_sta.cWT_flag = 2;
          }
        }	//if weight calibration
      }	//else(other than TW)
    }	//mid = 104
    else if (strstr(parse[1].value, "105"))
    {
     	//do_dfu();
      if (tagg == '0')	//Baby Discharged
      {
        if (baby_status != discharged)
          F_sta.sBaby_status_flag = 1;

        baby_status = discharged;

        if (mode != manual_mode)
        {
          close_vital_alarms();
          close_general_alarms();
        }

        mode = manual_mode;

        set.htr = 0;
        set_com.htr_setflag = 1;
        set_com.set_flag++;

        F_sta.sMODE_flag = 1;
        F_sta.sHTR_flag = 1;

        fds_test_write(1, flash_W_1(set.T1 * 10, set.T3 * 10, set.htr));
        F_sta.sHTR_flag = 2;
      }
      else
      {
        if (baby_status != admitted)
          F_sta.sBaby_status_flag = 1;

        baby_status = admitted;
      }
    }	//mid105
    else if (strstr(parse[1].value, "106"))
    {
      float Lslv = atof(parse[2].value);
      if ((Lslv <= 5.0) && (Lslv >= 0.5) && (Lslv != slv))
      {
        slv = Lslv;
      }	//validation

      float Lshv = atof(parse[3].value);
      if ((Lshv <= 5.0) && (Lshv >= 0.5))
      {
        shv = Lshv;
      }	//validation

      float Lalv = atof(parse[4].value);
      if ((Lalv <= 5.0) && (Lalv >= 0.5))
      {
        alv = Lalv;
      }	//validatio

      float Lahv = atof(parse[5].value);
      if ((Lahv <= 5.0) && (Lahv >= 0.5))
      {
        ahv = Lahv;
      }	//validation
      fds_test_write(6, flash_W_6((slv *10), (shv *10), (alv *10), (ahv *10)));
    }	//mid106
  }

 	//bitt = 8;//for test
}	//ble_data_process

/**@brief Function for handling the data from the Nordic UART Service.

   @details This function will process the data received from the Nordic UART BLE Service and send
            it to the UART module.

   @param[in] p_evt       Nordic UART Service event.
*/
/**@snippet[Handling the data received over BLE] */
char ble_data_array[256];
char ble_data_array1[256];
char ble_data_array2[256];
char ble_data_array3[256];
uint8_t ble_data_flag = 0;
uint8_t ble_data_flag1 = 0;
uint8_t ble_data_flag2 = 0;
uint8_t ble_data_flag3 = 0;
/**@snippet[Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t *p_evt)
{
  if (p_evt->type == BLE_NUS_EVT_RX_DATA)
  {
    memset(ble_data_array, 0, sizeof(ble_data_array));
    if (ble_data_flag == 0)
    {
      for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
      {
        ble_data_array[i] = p_evt->params.rx_data.p_data[i];
      }

      ble_data_flag = 1;
    }
    else if (ble_data_flag1 == 0)
    {
      for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
      {
        ble_data_array1[i] = p_evt->params.rx_data.p_data[i];
      }

      ble_data_flag1 = 1;
    }
    else if (ble_data_flag2 == 0)
    {
      for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
      {
        ble_data_array2[i] = p_evt->params.rx_data.p_data[i];
      }

      ble_data_flag2 = 2;
    }
    else if (ble_data_flag3 == 0)
    {
      for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
      {
        ble_data_array3[i] = p_evt->params.rx_data.p_data[i];
      }

      ble_data_flag3 = 1;
    }

   	//ble_rx_ticks = app_timer_cnt_get();
   	//ble_ack_flag = true;

   	//test_alert(ble_data_array, strlen(ble_data_array));
  }
}

/**@snippet[Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
  uint32_t err_code;
  ble_nus_init_t nus_init;
  nrf_ble_qwr_init_t qwr_init = { 0 };

 	// Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

 	// Initialize NUS.
  memset(&nus_init, 0, sizeof(nus_init));

  nus_init.data_handler = nus_data_handler;

  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);

 	// BEGIN Block Added for DFU
 	// ONLY ADD THIS BLOCK TO THE EXISTING FUNCTION
 	// Initialize the DFU service
  ble_dfu_buttonless_init_t dfus_init = { .evt_handler = ble_dfu_buttonless_evt_handler
  };
  err_code = ble_dfu_buttonless_init(&dfus_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.

   @details This function will be called for all events in the Connection Parameters Module
            which are passed to the application.

   @note All this function does is to disconnect. This could have been done by simply setting
         the disconnect_on_fail config parameter, but instead we use the event handler
         mechanism to demonstrate its use.

   @param[in] p_evt  Event received from the Connection Parameters Module.
*/
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
  uint32_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
  {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling errors from the Connection Parameters module.

   @param[in] nrf_error  Error code containing information about what went wrong.
*/
static void conn_params_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
  uint32_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.

   @note This function will not return.
*/
static void sleep_mode_enter(void)
{
  uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

 	// Prepare wakeup buttons.
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

 	// Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.

   @details This function will be called for advertising events which are passed to the application.

   @param[in] ble_adv_evt  Advertising event.
*/
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  uint32_t err_code;

  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_FAST:
      err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
      APP_ERROR_CHECK(err_code);
      break;
    case BLE_ADV_EVT_IDLE:
      sleep_mode_enter();
      break;
    default:
      break;
  }
}

/**@brief Function for handling BLE events.

   @param[in]   p_ble_evt   Bluetooth stack event.
   @param[in]   p_context   Unused.
*/
static void ble_evt_handler(ble_evt_t
  const *p_ble_evt, void *p_context)
{
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Connected");
      ble_connected = true;
      ble_live = 1;
      alert.blef_alert = 0;
      err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
      APP_ERROR_CHECK(err_code);
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected");
      ble_connected = false;
      ble_live = 0;
      alert.blef_alert = 1;
     	// LED indication will be changed when advertising starts.
      m_conn_handle = BLE_CONN_HANDLE_INVALID;
      break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
      {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t
        const phys = { .rx_phys = BLE_GAP_PHY_AUTO,
          .tx_phys = BLE_GAP_PHY_AUTO,
        };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
      }

      break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
     	// Pairing not supported
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
     	// No system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
     	// Disconnect on GATT Client timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
     	// Disconnect on GATT Server timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    default:
     	// No implementation needed.
      break;
  }
}

/**@brief Function for the SoftDevice initialization.

   @details This function initializes the SoftDevice and the BLE event interrupt.
*/
static void ble_stack_init(void)
{
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

 	// Configure the BLE stack using the default settings.
 	// Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

 	// Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

 	// Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t
  const *p_evt)
{
  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
  {
    m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }

  NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
    p_gatt->att_mtu_desired_central,
    p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
  uint32_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = false;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;	//BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = 0;	//APP_ADV_DURATION;
  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).

   @details If there is no pending log operation, then sleep until next the next event occurs.
*/
static void idle_state_handle(void)
{
  UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
  nrf_pwr_mgmt_run();
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
  uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
}

//********************************************************************************************************

//|*******************************************************************************************************************************************
void spo_sensor_data_process(void)
{
  uint8_t spo_value = spo.spo;
  uint8_t prt_value = spo.prt;
  char *st_code = spo.status;

  if ((spo_value >= set.spo2_Ul) && (spo_value != 0))
  {
    if (alert.sph_alert == 0)
    {
      alert.sph_alert = 1;
      sph_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if (alert.sph_alert == 2)
    {
      sph_close();
      alert.sph_alert = 0;
    }
  }

  if ((spo_value <= set.spo2_Ll) && (spo_value != 0))
  {
    if (alert.spl_alert == 0)
    {
      alert.spl_alert = 1;
      spl_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if (alert.spl_alert == 2)
    {
      spl_close();
      alert.spl_alert = 0;
    }
  }

  if ((prt_value >= set.pRate_Ul) && (prt_value != 0))
  {
    if (alert.prh_alert == 0)
    {
      alert.prh_alert = 1;
      prh_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if (alert.prh_alert == 2)
    {
      prh_close();
      alert.prh_alert = 0;
    }
  }

  if ((prt_value <= set.pRate_Ll) && (prt_value != 0))
  {
    if (alert.prl_alert == 0)
    {
      alert.prl_alert = 1;
      prl_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if (alert.prl_alert == 2)
    {
      prl_close();
      alert.prl_alert = 0;
    }
  }
}	//spo_sensor_data_process

void bmd_sensor_data_process(void)
{
  float t1 = bmd.T1_skin;
  float t2 = bmd.T2_peri;
  float t3 = bmd.T3_air;
 	//bmd.weight;
  uint8_t htr = bmd.htr;
  float deltaT = bmd.deltaT;
  float Ll = set.Ll;
  float Ul = set.Ul;

 	// is_alert = false;

  if (mode == air_mode)
  {
    if ((t3 - set.T3) >= ahv)
    {
      if (alert.ath_alert == 0)
      {
        alert.ath_alert = 1;
        ath_ticks = app_timer_cnt_get();
      }
    }
    else
    {
      if ((alert.ath_alert == 2) && (ble_connected == true))
      {
        ath_close();
        alert.ath_alert = 0;
      }
      else if (alert.ath_alert == 1)
        alert.ath_alert = 0;
    }

   	//////////////////////////////////////
    if ((set.T3 - t3) >= alv)
    {
      if (alert.atl_alert == 0)
      {
        alert.atl_alert = 1;
        atl_ticks = app_timer_cnt_get();
      }
    }
    else
    {
      if ((alert.atl_alert == 2) && (ble_connected == true))
      {
        atl_close();
        alert.atl_alert = 0;
      }
      else if (alert.atl_alert == 1)
        alert.atl_alert = 0;
    }

   	//////////////////////////////////////////////////
    if (t3 > 39.0)
    {
      if (alert.bto_t3_alert == 0)
      {
        alert.bto_t3_alert = 1;
        bto_t3_ticks = app_timer_cnt_get();
      }
    }
    else
    {
      if ((alert.bto_t3_alert == 2) && (ble_connected == true))
      {
        bto_t3_close();
        alert.bto_t3_alert = 0;
      }
      else if (alert.bto_t3_alert == 1)
        alert.bto_t3_alert = 0;
    }
  }	//air_MODE

  else if (mode == skin_mode)
  {
    if ((t1 - set.T1) >= shv)
    {
      if (alert.bth_alert == 0)
      {
        alert.bth_alert = 1;
        bth_ticks = app_timer_cnt_get();
      }
    }
    else
    {
      if ((alert.bth_alert == 2) && (ble_connected == true))
      {
        bth_close();
        alert.bth_alert = 0;
      }
      else if (alert.bth_alert == 1)
        alert.bth_alert = 0;
    }

   	////////////////////////////////////////////////////
    if ((set.T1 - t1) >= slv)
    {
      if (alert.btl_alert == 0)
      {
        alert.btl_alert = 1;
        btl_ticks = app_timer_cnt_get();
      }
    }
    else
    {
      if ((alert.btl_alert == 2) && (ble_connected == true))
      {
        btl_close();
        alert.btl_alert = 0;
      }
      else if (alert.btl_alert == 1)
        alert.btl_alert = 0;
    }

   	///////////////////////////////////////////////////////////
    if (t1 > 38.0)
    {
      if (alert.bto_t1_alert == 0)
      {
        alert.bto_t1_alert = 1;
        bto_t1_ticks = app_timer_cnt_get();
      }
    }
    else
    {
      if ((alert.bto_t1_alert == 2) && (ble_connected == true))
      {
        bto_t1_close();
        alert.bto_t1_alert = 0;
      }
      else if (alert.bto_t1_alert == 1)
        alert.bto_t1_alert = 0;
    }

   	/////////////////////////////////////////////////////
    if (t1 < 34.0)
    {
      if (alert.btc_alert == 0)
      {
        alert.btc_alert = 1;
        btc_ticks = app_timer_cnt_get();
      }
    }
    else
    {
      if ((alert.btc_alert == 2) && (ble_connected == true))
      {
        btc_close();
        alert.btc_alert = 0;
      }
      else if (alert.btc_alert == 1)
        alert.btc_alert = 0;
    }
  }	//skin_mode
 	//************************************************************
  if (deltaT < 0.0)
  {
    if (alert.cs_alert == 0)
    {
      alert.cs_alert = 1;
      cs_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if ((alert.cs_alert == 2) && (ble_connected == true))
    {
      cs_close();
      alert.cs_alert = 0;
    }
    else if (alert.cs_alert == 1)
      alert.cs_alert = 0;
  }

 	///////////////////////////////////////////////////////
  if ((Ll<t1 < Ul) && (deltaT >= 1.5))
  {
    if (alert.cso_alert == 0)
    {
      alert.cso_alert = 1;
      cso_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if ((alert.cso_alert == 2) && (ble_connected == true))
    {
      cso_close();
      alert.cso_alert = 0;
    }
    else if (alert.cso_alert == 1)
      alert.cso_alert = 0;
  }

 	///////////////////////////////////////////////////////////
  if ((Ll<t1 < Ul) && (deltaT <= 0.7))
  {
    if (alert.hso_alert == 0)
    {
      alert.hso_alert = 1;
      hso_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if ((alert.hso_alert == 2) && (ble_connected == true))
    {
      hso_close();
      alert.hso_alert = 0;
    }
    else if (alert.hso_alert == 1)
      alert.hso_alert = 0;
  }

 	/////////////////////////////////////////////////////////////
  if (((t1 - Ul) > 0) && (deltaT > 1.5))
  {
    if (alert.fo_alert == 0)
    {
      alert.fo_alert = 1;
      fo_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if ((alert.fo_alert == 2) && (ble_connected == true))
    {
      fo_close();
      alert.fo_alert = 0;
    }
    else if (alert.fo_alert == 1)
      alert.fo_alert = 0;
  }

 	//////////////////////////////////////////////////////////////////
  if (((t1 - Ul > 0) && (0.7 <= deltaT <= 1.3)) ||
    ((t1 - Ll < 0) && (0.7 <= deltaT <= 1.3)))
  {
    if (alert.bi_alert == 0)
    {
      alert.bi_alert = 1;
      bi_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if ((alert.bi_alert == 2) && (ble_connected == true))
    {
      bi_close();
      alert.bi_alert = 0;
    }
    else if (alert.bi_alert == 1)
      alert.bi_alert = 0;
  }

 	//////////////////////////////////////////////////////////////
  if ((set.T1 - t1 >= 0.5) && (deltaT <= 0.7))
  {
    if (alert.csp_alert == 0)
    {
      alert.csp_alert = 1;
      csp_ticks = app_timer_cnt_get();
    }
  }
  else
  {
    if ((alert.csp_alert == 2) && (ble_connected == true))
    {
      csp_close();
      alert.csp_alert = 0;
    }
    else if (alert.csp_alert == 1)
      alert.csp_alert = 0;
  }
}	//bmd_sensor_data_process

void check_alerts_with_dt(void)
{
  if (alert.ath_alert == 1)	//check chat box
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - ath_ticks)))
    {
      ath_open();
      alert.ath_alert = 2;
    }
  }
  else if (alert.atl_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - atl_ticks)))
    {
      atl_open();
      alert.atl_alert = 2;
    }
  }
  else if (alert.btl_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - btl_ticks)))
    {
      btl_open();
      alert.btl_alert = 2;
    }
  }
  else if (alert.bto_t1_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - bto_t1_ticks)))
    {
      bto_t1_open();
      alert.bto_t1_alert = 2;
    }
  }
  else if (alert.bto_t3_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - bto_t3_ticks)))
    {
      bto_t3_open();
      alert.bto_t3_alert = 2;
    }
  }
  else if (alert.btc_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - btc_ticks)))
    {
      btc_open();
      alert.btc_alert = 2;
    }
  }
  else if (alert.bth_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - bth_ticks)))
    {
      bth_open();
      alert.bth_alert = 2;
    }
  }
  else if (alert.cso_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - cso_ticks)))
    {
      cso_open();
      alert.cso_alert = 2;
    }
  }
  else if (alert.hso_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - hso_ticks)))
    {
      hso_open();
      alert.hso_alert = 2;
    }
  }
  else if (alert.fo_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - fo_ticks)))
    {
      fo_open();
      alert.fo_alert = 2;
    }
  }
  else if (alert.bi_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - bi_ticks)))
    {
      bi_open();
      alert.bi_alert = 2;
    }
  }
  else if (alert.csp_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - csp_ticks)))
    {
      csp_open();
      alert.csp_alert = 2;
    }
  }
  else if (alert.cs_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - cs_ticks)))
    {
      cs_open();
      alert.cs_alert = 2;
    }
  }
  else if (alert.sph_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - sph_ticks)))
    {
      sph_open();
      alert.sph_alert = 2;
    }
  }
  else if (alert.spl_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - spl_ticks)))
    {
      spl_open();
      alert.spl_alert = 2;
    }
  }

 	////////////////////////
  else if (((15 *1000) <= (app_timer_ms((app_timer_cnt_get()) - spo_ticks))) && (alert.spo_alert == 0))
  {
    uint8_t buff[128];
    memset(&buff, 0, 128);
    sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"204\",\"SPO\":\"0\",\"PRT\":\"0\"}", DID);
    uint16_t length = (uint16_t) strlen(buff);
    ret_code_t ret;
    do {
      ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
    } while (ret != NRF_SUCCESS && ble_live != 0);
    alert.spo_alert = 1;
    spo_ticks = app_timer_cnt_get();
    spo.spo = 0;
    spo.prt = 0;
    close_spo_alarms();
  }

 	/////////////////////////////
  else if (alert.prh_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - prh_ticks)))
    {
      prh_open();
      alert.prh_alert = 2;
    }
  }
  else if (alert.prl_alert == 1)
  {
    if ((adt *1000) <= (app_timer_ms((app_timer_cnt_get()) - prl_ticks)))
    {
      prl_open();
      alert.prl_alert = 2;
    }
  }

 	//*****************************************************************
  else if ((alert.spf_alert != 0) && (alert.bsf_alert == 0))
  {
    if (alert.spf_alert == 1)
    {
      spf_open();
      alert.spf_alert = 2;
      spf_ticks = app_timer_cnt_get();
    }
    else if (((Hadt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - spf_ticks))) && (alert.spf_alert == 2))
    {
      spf_open();
      alert.spf_alert = 2;
      spf_ticks = app_timer_cnt_get();
    }
  }

 	//*****************************************************************
  else if ((alert.t2f_alert != 0) && (alert.bsf_alert == 0))
  {
    if (alert.t2f_alert == 1)
    {
      t2f_open();
      alert.t2f_alert = 2;
      t2f_ticks = app_timer_cnt_get();
    }
    else if (((Hadt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - t2f_ticks))) && (alert.t2f_alert == 2))
    {
      t2f_open();
      alert.t2f_alert = 2;
      t2f_ticks = app_timer_cnt_get();
    }
  }

 	//*****************************************************************
  else if ((alert.apf_alert != 0) && (alert.bsf_alert == 0))
  {
    if (alert.apf_alert == 1)
    {
      apf_open();
      alert.apf_alert = 2;
      apf_ticks = app_timer_cnt_get();
    }
    else if (((Hadt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - apf_ticks))) && (alert.apf_alert == 2))
    {
      apf_open();
      alert.apf_alert = 2;
      apf_ticks = app_timer_cnt_get();
    }
  }

 	//*****************************************************************
  else if ((alert.hf_alert != 0) && (alert.bsf_alert == 0))
  {
    if (alert.hf_alert == 1)
    {
      hf_open();
      alert.hf_alert = 2;
      hf_ticks = app_timer_cnt_get();
    }
    else if (((Hadt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - hf_ticks))) && (alert.hf_alert == 2))
    {
      hf_open();
      alert.hf_alert = 2;
      hf_ticks = app_timer_cnt_get();
    }
  }

 	//**********************************************************************************
  else if (alert.sd_alert != 0)
  {
    if (alert.sd_alert == 1)
    {
      sd_open();
      alert.sd_alert = 2;
      sd_ticks = app_timer_cnt_get();
    }
    else if (((Hadt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - sd_ticks))) && (alert.sd_alert == 2))
    {
      sd_open();
      alert.sd_alert = 2;
      sd_ticks = app_timer_cnt_get();
    }
  }

 	//**********************************************************************************
  else if (alert.lp_alert != 0)
  {
    if (alert.lp_alert == 1)
    {
      lp_open();
      alert.lp_alert = 2;
      lp_ticks = app_timer_cnt_get();
    }
    else if (((Hadt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - lp_ticks))) && (alert.lp_alert == 2))
    {
      lp_open();
      alert.lp_alert = 2;
      lp_ticks = app_timer_cnt_get();
    }
  }

 	//**********************************************************************************
  else if (alert.lm_alert != 0)
  {
    if (alert.lm_alert == 1)
    {
      lm_open();
      alert.lm_alert = 2;
      lm_ticks = app_timer_cnt_get();
    }
    else if (((Hadt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - lm_ticks))) && (alert.lm_alert == 2))
    {
      lm_open();
      alert.lm_alert = 2;
      lm_ticks = app_timer_cnt_get();
    }
  }

 	//*******************************************************************************
  else if (alert.bsf_alert != 0)	//BMD sys fail
  {
    if (alert.bsf_alert == 1)
    {
      bsf_open();
      alert.bsf_alert = 2;
      bsf_ticks = app_timer_cnt_get();
    }

   	//else if (((Hadt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - bsf_ticks)))&&(alert.bsf_alert == 2))
   	//{
   	//  bsf_open();
   	//  alert.bsf_alert = 2;
   	//  bsf_ticks = app_timer_cnt_get();
   	//}
  }

 	//*******************************************************************************

}	//check_alerts_with_dt

void sendSerial(char *sdata, uint8_t len)
{
  for (int i = 0; i < len; i++)
  {
    ret_code_t ret;
    ret = nrf_serial_write(&serial1_uarte, &sdata[i],
      sizeof(sdata[i]),
      NULL,
      NRF_SERIAL_MAX_TIMEOUT);
    APP_ERROR_CHECK(ret);
    nrf_delay_ms(2);
  }
}

void readVitals()	// to read all the vital parameters from birdmeditech
{
  char read_vitals[8] = { DEVICE_ADDR, FC_READ, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09
  };
  nrf_gpio_pin_write(rs485_RE, 0);
  nrf_gpio_pin_write(rs485_DE, 1);	//for sending (1/0-HIGH/LOW)
  nrf_delay_ms(10);
  sendSerial(read_vitals, 8);
 	//nrf_delay_ms(15);
  nrf_gpio_pin_write(rs485_DE, 0);	//for receiving
  nrf_gpio_pin_write(rs485_RE, 0);	//for receiving
  nrf_delay_ms(10);
}

void readWeight()	// to read weight from birdmeditech
{
  char read_weight[8] = { DEVICE_ADDR, FC_READ, 0x00, 0x04, 0x00, 0x01, 0xC5, 0xCB
  };
  nrf_gpio_pin_write(rs485_RE, 0);
  nrf_gpio_pin_write(rs485_DE, 1);	//for sending (1/0-HIGH/LOW)
  nrf_delay_ms(10);
  sendSerial(read_weight, 8);
  nrf_gpio_pin_write(rs485_DE, 0);	//for receiving
  nrf_gpio_pin_write(rs485_RE, 0);
  nrf_delay_ms(10);
}

void setTWeight()	// to read weight from birdmeditech
{
  char Tweight[8] = { DEVICE_ADDR, FC_WRITE, 0x00, 0x03, 0x00, 0x00, 0x79, 0xCA
  };
  nrf_gpio_pin_write(rs485_RE, 0);
  nrf_gpio_pin_write(rs485_DE, 1);	//for sending (1/0-HIGH/LOW)
  nrf_delay_ms(10);
  sendSerial(Tweight, 8);
  nrf_gpio_pin_write(rs485_DE, 0);	//for receiving
  nrf_gpio_pin_write(rs485_RE, 0);
  nrf_delay_ms(10);
  set_com.Tweight_setflag = 2;
}

void removeStandby()	// to remove standby mode
{
  char write_removeStandby[8] = { DEVICE_ADDR, FC_WRITE, 0x00, 0x04, 0x00, 0x00, 0xC8, 0x0B
  };
  nrf_gpio_pin_write(rs485_RE, 0);
  nrf_gpio_pin_write(rs485_DE, 1);	//for sending (1/0-HIGH/LOW)
  nrf_delay_ms(10);
  sendSerial(write_removeStandby, 8);
  nrf_gpio_pin_write(rs485_DE, 0);	//for receiving
  nrf_gpio_pin_write(rs485_RE, 0);
  nrf_delay_ms(10);
}

void setManual_heat()	// to set manual %
{
  char write_manual_heat[8] = { DEVICE_ADDR, FC_WRITE, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00
  };
 	//char write_manual_heat[8] = {DEVICE_ADDR, FC_WRITE, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
  int hex = set.htr;
  write_manual_heat[4] = hex >> 8;
  write_manual_heat[5] = hex &0x00FF;
  int tcrc = 0;
  tcrc = getCRC(write_manual_heat, 6);
  write_manual_heat[7] = tcrc >> 8;
  write_manual_heat[6] = tcrc &0xFF;
  nrf_gpio_pin_write(rs485_DE, 1);	//for sending (1/0-HIGH/LOW)
  nrf_delay_ms(10);
  sendSerial(write_manual_heat, 8);
  nrf_gpio_pin_write(rs485_DE, 0);	//for receiving
  set_com.htr_setflag = 2;
  nrf_delay_ms(10);
}

void setAir()
{
  char set_Air[8] = { DEVICE_ADDR, FC_WRITE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  int hex = set.T3 * 100;
  set_Air[4] = hex >> 8;
  set_Air[5] = hex &0x00FF;
  int tcrc = 0;
  tcrc = getCRC(set_Air, 6);
  set_Air[7] = tcrc >> 8;
  set_Air[6] = tcrc &0xFF;
  nrf_gpio_pin_write(rs485_DE, 1);	//for sending (1/0-HIGH/LOW)
  nrf_delay_ms(10);
  sendSerial(set_Air, 8);
  nrf_gpio_pin_write(rs485_DE, 0);	//for receiving
  set_com.T3air_setflag = 2;
  nrf_delay_ms(10);
}

void setSkin()
{
  char set_Skin[8] = { DEVICE_ADDR, FC_WRITE, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00
  };

  int hex = set.T1 * 100;
  set_Skin[4] = hex >> 8;
  set_Skin[5] = hex &0x00FF;

  int tcrc = 0;
  tcrc = getCRC(set_Skin, 6);

  set_Skin[7] = tcrc >> 8;
  set_Skin[6] = tcrc &0xFF;

 	//nrf_gpio_pin_write(rs485_RE, 0);
  nrf_gpio_pin_write(rs485_DE, 1);	//for sending (1/0-HIGH/LOW)
  nrf_delay_ms(10);
  sendSerial(set_Skin, 8);

  nrf_gpio_pin_write(rs485_DE, 0);	//for receiving
  set_com.T1skin_setflag = 2;
 	//nrf_gpio_pin_write(rs485_RE, 0);
  nrf_delay_ms(10);
}

int getCRC(unsigned char message[], int length)
{
  unsigned char ct, cnt1;
  unsigned char recd, rdwr[16];
  unsigned int crc;
  crc = 0xffff;
  for (ct = 0; ct < length; ct++)
  {
    recd = message[ct];
    crc ^= recd;
    for (cnt1 = 0; cnt1 < 8; cnt1++)
    {
      if (crc & 1)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
        crc >>= 1;
    }
  }

  return crc;
}

static void repeated_timer_handler_bmb(void *p_context)
{
  UNUSED_PARAMETER(p_context);
  if (bsf_flag == 1)
  {
    bsf_flag = 0;

    if (alert.bsf_alert != 0)
    {
      bsf_close();
      alert.bsf_alert = 0;
    }
  }
  else
  {
    alert.bsf_alert = 1;
  }
}

static void bmd_process()
{
  if (set_com.T3air_setflag == 1 || set_com.T3air_setflag == 2)
  {
    setAir();
  }
  else if (set_com.T1skin_setflag == 1 || set_com.T1skin_setflag == 2)
  {
    setSkin();
  }
  else if (set_com.standby_setflag == 1 || set_com.standby_setflag == 2)
  {
    removeStandby();
  }
  else if (set_com.htr_setflag == 1 || set_com.htr_setflag == 2)
  {
    setManual_heat();
  }
  else if (set_com.Tweight_setflag == 1 || set_com.Tweight_setflag == 2)
  {
    setTWeight();
  }
  else if ((weight_dt *1000) <= (app_timer_ms(app_timer_cnt_get() - weight_ticks)) && (1000 <= app_timer_ms(app_timer_cnt_get() - weight_send_ticks)))
  {
    readWeight();
    weight_send_ticks = app_timer_cnt_get();
  }
  else
  {
    readVitals();
  }

  bmd_ticks = app_timer_cnt_get();
}

static void create_timers()
{
  ret_code_t err_code;

 	// Create timers
  err_code = app_timer_create(&bmb_repeated_timer_id,
    APP_TIMER_MODE_REPEATED,
    repeated_timer_handler_bmb);

  APP_ERROR_CHECK(err_code);

}

void led_blink(void)	//if ((adt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - csp_ticks)))
{
 	//nowtime = app_timer_cnt_get();
  if (blink_led == true)
  {
    if (((blink_dt *1000) <= (app_timer_ms(app_timer_cnt_get() - led_ontime))) && (blink_state == true))
    {
      nrf_gpio_pin_write(LED, 0);
      led_offtime = app_timer_cnt_get();
      blink_state = false;
    }
    else if (((blink_dt *1000) <= (app_timer_ms((app_timer_cnt_get()) - led_offtime))) && (blink_state == false))
    {
      nrf_gpio_pin_write(LED, 1);
      led_ontime = app_timer_cnt_get();
      blink_state = true;
    }
  }
}

void buz_beep(void)	//if ((adt *60 *1000) <= (app_timer_ms((app_timer_cnt_get()) - csp_ticks)))
{
 	//nowtime = app_timer_cnt_get();
  if (buz_on == true)
  {
    if (((beep_dt *1000) <= (app_timer_ms(app_timer_cnt_get() - buz_ontime))) && (buz_state == true))
    {
      while (app_pwm_channel_duty_set(&PWM1, 0, 0) == NRF_ERROR_BUSY);
      buz_offtime = app_timer_cnt_get();
      buz_state = false;
      buz_cnt++;
    }
    else if (((beep_dt *1000) <= (app_timer_ms((app_timer_cnt_get()) - buz_offtime))) && (buz_state == false))
    {
      while (app_pwm_channel_duty_set(&PWM1, 0, 260) == NRF_ERROR_BUSY);
      buz_ontime = app_timer_cnt_get();
      buz_state = true;
    }
  }
}

void sendVitalParams_ble(void)
{
  uint8_t buff[128];
  memset(&buff, NULL, 128);
 	//{"DID":"SBWXXX","MID":"201","AT":"23.4","BT":"25.6","PT":"35.6","WT":"+30.535", "HT:"100"}

  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"201\",\"AT\":\"%2.1f\",\"BT\":\"%2.1f\",\"PT\":\"%2.1f\",\"HT\":\"%d\",\"M\":\"%d\"}", DID, bmd.T3_air, bmd.T1_skin, bmd.T2_peri, bmd.htr, mode);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
  send_ble_ticks = app_timer_cnt_get();

}

void send_spo_ble(void)
{
  uint8_t buff[128];
  memset(&buff, NULL, 128);
  sprintf(buff, "{\"DID\":\"%s\",\"MID\":\"204\",\"SPO\":\"%d\",\"PRT\":\"%d\"}", DID, spo.spo, spo.prt);
  uint16_t length = (uint16_t) strlen(buff);
  ret_code_t ret;
  do {
    ret = ble_nus_data_send(&m_nus, buff, &length, m_conn_handle);
  } while (ret != NRF_SUCCESS && ble_live != 0);
}

//***********************************************************************************************
uint8_t ir_flag = 0;
uint32_t ir_ticks = 0;
uint32_t ir_prev_ticks = 0;
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
 	//if (adt * 1000 <= (app_timer_ms((app_timer_cnt_get()) - ir_prev_ticks)))
 	//{
  ir_ticks = app_timer_cnt_get();
  while (!nrf_gpio_pin_read(IR_PIN))
  {
    if ((600) <= (app_timer_ms((app_timer_cnt_get()) - ir_ticks)))
      break;
  }

 	//********************************
  if ((500) <= (app_timer_ms((app_timer_cnt_get()) - ir_ticks)))
  {
    ir_open();
    ir_ticks = app_timer_cnt_get();
    ir_prev_ticks = app_timer_cnt_get();
    ir_flag++;
  }

 	//}// Wait for alarm delay time to next trigger

}

static void gpio_init(void)
{
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_PULLDOWN;

  err_code = nrf_drv_gpiote_in_init(IR_PIN, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(IR_PIN, true);
}

//********************************************************************************************************
uint8_t isBMDalert(void)
{
  if (
    alert.t2f_alert > 0 ||
    alert.apf_alert > 0 ||
    alert.spf_alert > 0 ||
    alert.hf_alert > 0 ||	//Heater Fail

    alert.ath_alert > 1 ||	//1
    alert.atl_alert > 1 ||	//2
    alert.bi_alert > 1 ||	//3
    alert.btc_alert > 1 ||	//4
    alert.bth_alert > 1 ||	//5
    alert.btl_alert > 1 ||	//6
    alert.bto_t1_alert > 1 ||	//7
    alert.bto_t3_alert > 1 ||	//7
    alert.cso_alert > 1 ||	//8
    alert.csp_alert > 1 ||	//9
    alert.cs_alert > 1 ||	//10
    alert.fo_alert > 1 ||	//11
    alert.hso_alert > 1	//12
 )
  {
    return 1;
  }
  else
    return 0;
}

uint8_t isSPOalert(void)
{
  if (alert.lm_alert > 0 ||
    alert.lp_alert > 0 ||
    alert.sd_alert > 0 ||

    alert.sph_alert > 1 ||
    alert.spl_alert > 1 ||
    alert.prh_alert > 1 ||
    alert.prl_alert > 1
 )
  {
    return 1;
  }
  else
    return 0;
}

uint8_t isHighPriorityAlert(void)
{
  if (
    alert.t2f_alert > 0 ||
    alert.apf_alert > 0 ||
    alert.spf_alert > 0 ||
    alert.hf_alert > 0 ||	//Heater Fail
    alert.bsf_alert > 0 ||

    alert.lm_alert > 0 ||
    alert.lp_alert > 0 ||
    alert.sd_alert > 0
 )
  {
    return 1;
  }
  else
    return 0;
}

//********************************************************************************************************
void wdt_event_handler(void)
{
  nrf_gpio_pin_write(FLED, 0);

 	//NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

//********************************************************************************************************
//uint8_t ir_cnt=55;

/**@brief Application main function.
 */

int main(void)
{
  bool erase_bonds;
  ret_code_t ret, err_code;

 	// Initialize.
 	// uart_init();
  log_init();
  timers_init();
  create_timers();
 	// buttons_leds_init(&erase_bonds);
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();

  gpio_init();

 	//Configure WDT.
  nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
  err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
  APP_ERROR_CHECK(err_code);
  nrf_drv_wdt_enable();

  memset(&bmd, 0, sizeof(bmd));
  memset(&set, 0, sizeof(bmd));
  memset(&set_com, 0, sizeof(set_com));
  memset(&alert, 0, sizeof(alert));
  memset(&cal, 0, sizeof(cal));
  memset(&F_sta, 0, sizeof(F_sta));

 	//mode = skin_mode;
 	//set.T1 = 30;

 	//cal.T2_peri = 1.3;

  advertising_start();

 	//|*********************************************************************************************************

 	//ret = nrf_drv_clock_init();
 	//APP_ERROR_CHECK(ret);
 	//ret = nrf_drv_power_init(NULL);
 	//APP_ERROR_CHECK(ret);

  nrf_drv_clock_lfclk_request(NULL);
  ret = app_timer_init();
  APP_ERROR_CHECK(ret);

  fds_test_init();

  flash_R_t1_t3_htr(fds_read(1));
  nrf_delay_ms(200);
  set.Ul = set.T1 + 0.3;
  set.Ll = set.T1 - 0.3;

  flash_R_sp_pr(fds_read(2));
  nrf_delay_ms(200);
  flash_R_3(fds_read(3));
  nrf_delay_ms(200);
  flash_R_4(fds_read(4));
  nrf_delay_ms(200);
  flash_R_5(fds_read(5));
  nrf_delay_ms(200);
  flash_R_6(fds_read(6));
  nrf_delay_ms(200);

  real_mode = mode;

  ret = nrf_serial_init(&serial0_uarte, &m_uarte0_drv_config, &serial0_config);
  APP_ERROR_CHECK(ret);

  ret = nrf_serial_init(&serial1_uarte, &m_uarte1_drv_config, &serial1_config);
  APP_ERROR_CHECK(ret);

  app_timer_start(bmb_repeated_timer_id, APP_TIMER_TICKS(60000UL), NULL);
  app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, buzz);
  ret = app_pwm_init(&PWM1, &pwm1_cfg, NULL);
  APP_ERROR_CHECK(ret);
  app_pwm_enable(&PWM1);
 	//|*************************************************************************************************************
  nrf_gpio_cfg_output(rs485_DE);
  nrf_gpio_cfg_output(rs485_RE);
  nrf_gpio_cfg_output(LED);
  nrf_gpio_cfg_output(FLED);
 	//nrf_gpio_cfg_input(IR_PIN,NRF_GPIO_PIN_PULLDOWN);

  nrf_gpio_pin_write(rs485_RE, 0);
 	//nrf_gpio_pin_write(FLED,1);
 	//baby_status = discharged;

 	// Enter main loop.
  for (;;)
  {
    if ((350) <= (app_timer_ms((app_timer_cnt_get()) - bmd_ticks)))
    {
      bmd_process();
    }

    if (bmd_data_check)
    {
      check_bmd_data();
      bmd_data_check = false;
    }

   	/////////////////////////////////////////////////////////////////////////////
    if (ble_data_flag)
    {
      ble_data_process(ble_data_array, strlen(ble_data_array));
      ble_data_flag = 0;
    }
    else if (ble_data_flag1)
    {
      ble_data_process(ble_data_array1, strlen(ble_data_array1));
      ble_data_flag1 = 0;
    }
    else if (ble_data_flag2)
    {
      ble_data_process(ble_data_array2, strlen(ble_data_array2));
      ble_data_flag3 = 0;
    }
    else if (ble_data_flag3)
    {
      ble_data_process(ble_data_array3, strlen(ble_data_array3));
      ble_data_flag3 = 0;
    }

   	/////////////////////////////////////////////////////////////////////////////////
    if ((1000) <= (app_timer_ms((app_timer_cnt_get()) - send_ble_ticks)))
    {
      sendVitalParams_ble();
    }

    if ((spo_send_ready == 1))
    {
      send_spo_ble();
      spo_send_ready = 0;
    }
    if (alert.blef_alert == 1)
      {
        buz_on = true;
        buz_beep();
        if ((buz_cnt > 15) && (ble_fail_process))
        {
          real_mode = mode;
          mode = manual_mode;
          set.htr = 0;
          set_com.htr_setflag = 1;
          set_com.set_flag++;
          ble_fail_process = false;
        }
      }
      else
      {
        while (app_pwm_channel_duty_set(&PWM1, 0, 0) == NRF_ERROR_BUSY);
        buz_offtime = app_timer_cnt_get();
        buz_state = false;
        buz_cnt = 0;
        ble_fail_process = true;
      }
      if ((!isHighPriorityAlert()) && (mode != real_mode))
      {
        mode = real_mode;
        if (mode == skin_mode)
        {
          set_com.T1skin_setflag = 1;
        }
        else if (mode == air_mode)
        {
          set_com.T3air_setflag = 1;
        }
        else if(mode == manual_mode)
        {
          set.htr = real_htr;
          set_com.htr_setflag = 1;
        }
      }
////////////////////////////////////////////////////////////////////////////////////////
if ((F_sta.sMODE_flag == 1) || (F_sta.sBaby_status_flag == 1))
      {
        if (2000 <= (app_timer_ms((app_timer_cnt_get()) - flash_ticks)))
        {
          if (F_sta.sMODE_flag == 1)
          {
            uint8_t cT1 = 0, cT2 = 0, cT3 = 0, pol = 0;

            cT3 = abs(cal.T3_air *10);
            cT2 = abs(cal.T2_peri *10);
            cT1 = abs(cal.T1_skin *10);

            if (cal.T1_skin < 0)
              pol = pol | 0x01;
            if (cal.T2_peri < 0)
              pol = pol | 0x02;
            if (cal.T3_air < 0)
              pol = pol | 0x04;

            fds_test_write(3, flash_W_3(cT1, cT2, cT3, pol, mode));
            F_sta.sMODE_flag = 2;
          }
          else
          {
            uint8_t cSP = 0, cPR = 0, pol = 0;
            cSP = abs(cal.spo *10);
            cPR = abs(cal.prt *10);

            if (cal.spo < 0)
              pol = pol | 0x01;
            if (cal.prt < 0)
              pol = pol | 0x02;

            F_sta.sBaby_status_flag = 1;

            fds_test_write(4, flash_W_4(cSP, cPR, pol, adt, baby_status));
            F_sta.sBaby_status_flag = 2;
          }
        }	//if morethan 2000 ms
      }	//if sMode=1 || sBaby_status=1
////////////////////////////////////////////////////////////////////////////////////////
      if (mode == manual_mode)
      {
        close_vital_alarms();
        close_general_alarms();
      }
     nrf_drv_wdt_channel_feed(m_channel_id);
//***********************************************************************************************
    if (baby_status != discharged)
    {
     	// nrf_delay_ms(300);
      if (mode == air_mode || mode == skin_mode)
      {
        bmd_sensor_data_process();
        spo_sensor_data_process();
        if (ble_connected == true)
          check_alerts_with_dt();
      }
      else
      {
        spo_sensor_data_process();
        if (ble_connected == true)
          check_alerts_with_dt();
      }     

      if (isBMDalert() || isSPOalert())
      {
        blink_led = true;
        led_blink();
      }
      else
      {
        blink_led = false;
        nrf_gpio_pin_write(LED, 0);
        led_offtime = app_timer_cnt_get();
      }
     // nrf_drv_wdt_channel_feed(m_channel_id);
    }	//if baby admitted
    else
    {
      if (mode != manual_mode)
      {
        mode = manual_mode;
        set.htr = 0;
        set_com.set_flag++;
        setManual_heat();

        blink_led = false;
        nrf_gpio_pin_write(LED, 0);
        led_offtime = app_timer_cnt_get();
       	//nrf_delay_ms(500);
      }
    }	//if baby discharged
   	//    */

  }	//for loop
}	//main

/**
   @}

*/
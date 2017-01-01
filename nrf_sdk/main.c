/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 * Copyright (c) 2015 Nemik Consulting Inc. All Wrongs Reversed.
 *
 */

/** @file
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "boards.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "ble_radio_notification.h"
#include "app_gpiote.h"
#include "hb_app_button.h"

#define APP_GPIOTE_MAX_USERS            1  // Maximum number of users of the GPIOTE handler.
#define BUTTON_DEBOUNCE_DELAY			10 // Delay from a GPIOTE event until a button is reported as pushed.

#define PRESENCE_ADV_INTERVAL 2500
#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define BUTTON1                       18   // minew button
#define BLUE_LED	15
#define RED_LED	16


#define APP_TIMER_PRESCALER               0                                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS              6                                                 /**< Maximum number of simultaneously created timers. 1 for Battery measurement, 1 for sensor updates  */
#define APP_TIMER_OP_QUEUE_SIZE           6                                                 /**< Size of timer operation queues. */

#define APP_CFG_NON_CONN_ADV_TIMEOUT     0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL     MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (5 s). This value can vary between 100ms to 10.24s). */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     1200                                              /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      3                                                 /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/


#define BATTERY_LEVEL_MEAS_INTERVAL       APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)      /**< Battery level measurement interval (ticks). This value corresponds to 60 seconds. */

#define COMPANY_IDENTIFIER               0x5604
const char PRODUCT_IDENTIFIER[] = {0x00, 0x01};

#define UICR_ADDR_0x80         (*((uint32_t *) 0x10001080))
#define UICR_ADDR_0x84         (*((uint32_t *) 0x10001084))

#define DEAD_BEEF                        0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//#define TX_POWER_LEVEL                    (-8)                                              /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */
//accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm
#define TX_POWER_LEVEL                    (0)                                              /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     1200                                              /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      3                                                 /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */

static app_timer_id_t                     m_battery_timer_id;                               /**< Battery measurement timer. */

typedef struct sensor_payload
{
    uint8_t product_id[2];
    uint8_t device_id[4];
    uint8_t b1;
    uint8_t b2;
    uint8_t b3;
    // even though this should be a uint16_t, ARM insists on 4-byte boundaries 
    // so split into two uint8_t's. otherwise we lose a byte to alignmentpadding
    uint8_t battery_level[2];
    uint8_t random;
    uint8_t digest[2]; 
} sensor_payload;

static sensor_payload sensor_data;

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void gpio_set_sense_low(bool low)
{
	/*
	int sense;
	if(low)
	{
		sense = NRF_GPIO_PIN_SENSE_LOW;
	}
	else
	{
		sense = NRF_GPIO_PIN_SENSE_HIGH;
	}
	*/
	//nrf_gpio_cfg_sense_input(BUTTON1, NRF_GPIO_PIN_PULLUP, sense);
	//NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t        err_code;
    ble_advdata_t   advdata;
    uint8_t         flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    int8_t          tx_power_level = TX_POWER_LEVEL;

    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = COMPANY_IDENTIFIER;

    uint8_t spp[sizeof(sensor_data)];
    memcpy(spp, &sensor_data, sizeof(sensor_data));        

    manuf_specific_data.data.p_data        = spp;
    manuf_specific_data.data.size          = sizeof(spp);

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.p_tx_power_level        = &tx_power_level;
    advdata.p_manuf_specific_data   = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
    
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

		//TODO: depedning on mode
    //advertising_init();
    //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}

static void update_advertising()
{
    advertising_init();
    //advertising_start();
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION, false);
		NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
		NRF_CLOCK->TASKS_LFCLKSTART    = 1;
		while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0); // Wait for clock to start

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

uint16_t battery_reading = 0;

void ADC_IRQHandler(void)
{
    if (NRF_ADC->EVENTS_END != 0)
    {
        uint16_t    adc_result;
        //uint8_t adc_result;

        NRF_ADC->EVENTS_END     = 0;
        adc_result              = NRF_ADC->RESULT;
        adc_result = NRF_ADC->RESULT;
        NRF_ADC->TASKS_STOP     = 1;

        uint16_t mv = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
        //update struct with battery value
        uint8_t hi_lo[] = { (uint8_t)(mv >> 8), (uint8_t)mv }; 
        memcpy(sensor_data.battery_level, hi_lo, 2);
        //memcpy(sensor_data.battery_level, adc_result, 1);
        update_advertising();
    }
}

static void adc_start(void)
{
    uint32_t err_code;

    // Configure ADC
    NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;
    //NRF_ADC->CONFIG     = (ADC_CONFIG_RES_10bit                        << ADC_CONFIG_RES_Pos)     |
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_Disabled                   << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

    // Enable ADC interrupt
    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;
}

static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    adc_start();
}

static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
    // start battery timer
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void do_button_adv()
{
	advertising_init();

	uint8_t len = 1;
	uint8_t rand;
	sd_rand_application_bytes_available_get(&len);
	nrf_delay_ms(5);
	sd_rand_application_vector_get(&rand, 1);

	//sd_rand_application_bytes_available_get();
	//sd_rand_application_pool_capacity_get()

	/*
		 int b1_state = button_states & (1 << 0);
		 int b2_state = button_states & (1 << 1);
		 int b3_state = button_states & (1 << 2);
		 */

	//do the checks
	//if(b1_state != b1)

	/*
		 button_states = 0;
		 button_states |= (b1 << 0);
		 button_states |= (b2 << 0);
		 button_states |= (b3 << 0);
		 */

	/*
		 if(b1 == 0 || b2 == 0 || b3 == 0) 
		 {
		 gpio_set_sense_low(true);
		 prev_state = 1;
		 }
		 if(b1 != 0 || b2 != 0 || b3 != 0) 
		 {
		 gpio_set_sense_low(false);
		 nrf_delay_ms(1);
		 prev_state = 0;
	//sd_power_system_off();
	}
	*/

	sensor_data.b1 = 1;
	sensor_data.b2 = 0;
	sensor_data.b3 = 0;
	sensor_data.random = rand;
	update_advertising();

	nrf_gpio_pin_toggle(BLUE_LED);
	nrf_gpio_pin_toggle(RED_LED);
	advertising_start();
	nrf_delay_ms(150);
	sd_ble_gap_adv_stop();
	nrf_gpio_pin_toggle(BLUE_LED);
	nrf_gpio_pin_toggle(RED_LED);
}

static void button_handler(uint8_t pin_no, uint8_t button_action)
{
    if(button_action == APP_BUTTON_PUSH)
    {
        switch(pin_no)
        {
            case BUTTON1:
							do_button_adv();
							break;
            default:
                break;
        }
    }
}

static void gpio_init()
{
	uint32_t        err_code;
	static app_button_cfg_t p_button[] = {{BUTTON1, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_handler}};
	// Initializing the buttons.
	err_code = hb_app_button_init(p_button, 1, BUTTON_DEBOUNCE_DELAY, 0);
	//app_button_init(p_button, 0, BUTTON_DEBOUNCE_DELAY, 0);
	APP_ERROR_CHECK(err_code);

	// Enabling the buttons.										
	err_code = hb_app_button_enable();
	APP_ERROR_CHECK(err_code);

	nrf_gpio_cfg_output(BLUE_LED);
	nrf_gpio_cfg_output(RED_LED);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
		// Initialize.
		//gpio_set_sense_low(true);
		// Workaround for PAN_028 rev1.1 anomaly 22 - System: Issues with disable system OFF mechanism
		nrf_delay_ms(1);

		/*
    NRF_SPI0->ENABLE = 0;
    NRF_TWI0->ENABLE=0;
    NRF_TWI1->ENABLE=0;
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE = 0;
		*/
    
    char device_id[]                 = { (UICR_ADDR_0x80 & 0x000000ff),
                                         (UICR_ADDR_0x80 & 0x0000ff00) >> 8, 
                                         (UICR_ADDR_0x80 & 0x00ff0000) >> 16, 
                                         (UICR_ADDR_0x80 & 0xff000000) >> 24,
    };

    memcpy(sensor_data.product_id, PRODUCT_IDENTIFIER, 2);
    memset(sensor_data.device_id, 0, 4);
    memcpy(sensor_data.device_id, device_id, 4);
    //sensor_data.sensor_state = 0;
    memset(sensor_data.battery_level, 0, 2);
    //sensor_data.power_level = 0xbf; //signed int -65 rssi at 1 meter
    memset(sensor_data.digest, 0, 4);

		timers_init();
    ble_stack_init();

		APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
		gpio_init();
    adc_start();

		do_button_adv();
		// Enter main loop.
		//sd_power_system_off();
    for (;;)
    {
        power_manage();
    }
}

/**
 * @}
 */

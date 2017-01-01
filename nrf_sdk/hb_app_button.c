/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "hb_app_button.h"
#include <string.h>
#include "nordic_common.h"
#include "app_util.h"
#include "app_gpiote.h"
#include "app_timer.h"
#include "app_error.h"

#define APP_TIMER_PRESCALER               0
#define APP_TIMER_MAX_TIMERS              6
#define APP_TIMER_OP_QUEUE_SIZE           6
#define BUTTON_DELAY							        APP_TIMER_TICKS(800, APP_TIMER_PRESCALER)

static app_button_cfg_t *             mp_buttons = NULL;           /**< Button configuration. */
static uint8_t                        m_button_count;              /**< Number of configured buttons. */
static uint32_t                       m_detection_delay;           /**< Delay before a button is reported as pushed. */
static app_button_evt_schedule_func_t m_evt_schedule_func;         /**< Pointer to function for propagating button events to the scheduler. */
static app_gpiote_user_id_t           m_gpiote_user_id;            /**< GPIOTE user id for buttons module. */
static app_timer_id_t                 m_detection_delay_timer_id;  /**< Polling timer id. */
static pin_transition_t               m_pin_transition;            /**< pin transaction direction. */

static uint8_t 												push_counter = 0;
static bool														timer_running = false;

/**@brief Function for executing the application button handler for specified button.
 *
 * @param[in]  p_btn   Button that has been pushed.
 */
static void button_handler_execute(app_button_cfg_t * p_btn, uint32_t transition, uint8_t counter)
{
    if (m_evt_schedule_func != NULL)
    {
        uint32_t err_code = m_evt_schedule_func(p_btn->button_handler, p_btn->pin_no,transition, counter);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
				if(transition == APP_BUTTON_PUSH)
				{
            p_btn->button_handler(p_btn->pin_no, APP_BUTTON_PUSH, counter);
				}
        else if(transition == APP_BUTTON_RELEASE)
        {
            p_btn->button_handler(p_btn->pin_no, APP_BUTTON_RELEASE, counter);
        }
    }
}


/**@brief Function for handling the timeout that delays reporting buttons as pushed.
 *
 * @details    The detection_delay_timeout_handler(...) is a call-back issued from the app_timer
 *             module. It is called with the p_context parameter. The p_context parameter is
 *             provided to the app_timer module when a timer is started, using the call
 *             @ref app_timer_start. On @ref app_timer_start the p_context will be holding the
 *             currently pressed buttons.
 *
 * @param[in]  p_context   Pointer used for passing information app_start_timer() was called.
 *                         In the app_button module the p_context holds information on pressed
 *                         buttons.
 */
static void detection_delay_timeout_handler(void * p_context)
{
    uint32_t err_code;
    uint32_t current_state_pins;

		timer_running = false;
    // Get current state of pins.
    err_code = app_gpiote_pins_state_get(m_gpiote_user_id, &current_state_pins);

    if (err_code != NRF_SUCCESS)
    {
        return;
    }

    uint8_t i;
    
    // Pushed button(s) detected, execute button handler(s).
    for (i = 0; i < m_button_count; i++)
    {
        app_button_cfg_t * p_btn = &mp_buttons[i];

				//TODO: check if button is still held after being initially held. if it is, then consider it a long-push
				bool is_long = !nrf_gpio_pin_read(p_btn->pin_no);
				//bool is_long = false;

        //if (((m_pin_transition.high_to_low & (1 << p_btn->pin_no)) != 0) && (p_btn->button_handler != NULL))
        if (((1 << p_btn->pin_no) != 0) && (p_btn->button_handler != NULL) && push_counter > 0)
        {
            //If it's active high then going from high to low was a release of the button.
            if(p_btn->active_state == APP_BUTTON_ACTIVE_HIGH)
            {
                button_handler_execute(p_btn, APP_BUTTON_RELEASE, push_counter);
            }
            //If it's active low then going from high to low was a push of the button.
            else
            {
								if(is_long)
								{
                	button_handler_execute(p_btn, APP_BUTTON_PUSH, APP_BUTTON_PUSH_LONG);
								}
								else
								{
                	button_handler_execute(p_btn, APP_BUTTON_PUSH, push_counter);
								}
            }
        }
				// reset counter to 0
				push_counter = 0;
    }
}


/**@brief Function for handling the GPIOTE event.
 *
 * @details Saves the current status of the button pins, and starts a timer. If the timer is already
 *          running, it will be restarted.
 *
 * @param[in]  event_pins_low_to_high   Mask telling which pin(s) had a low to high transition.
 * @param[in]  event_pins_high_to_low   Mask telling which pin(s) had a high to low transition.
 * HB FUNCTION
 */

static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
    uint32_t err_code;

    // Start detection timer. If timer is already running, the detection period is restarted.
    // NOTE: Using the p_context parameter of app_timer_start() to transfer the pin states to the
    //       timeout handler (by casting event_pins_mask into the equally sized void * p_context
    //       parameter).
    STATIC_ASSERT(sizeof(void *) == sizeof(uint32_t));

		m_pin_transition.low_to_high = event_pins_low_to_high;
		m_pin_transition.high_to_low = event_pins_high_to_low;
		
		//TODO: only do this if timer is NOT running
		if(!timer_running)
		{
    
    	err_code = app_timer_start(m_detection_delay_timer_id,
															 BUTTON_DELAY,
                               //m_detection_delay,
                               (void *)(event_pins_low_to_high | event_pins_high_to_low));
  	  if (err_code != NRF_SUCCESS)
	    {
        // The impact in app_button of the app_timer queue running full is losing a button press.
        // The current implementation ensures that the system will continue working as normal. 
    	}
			else
			{
	    	timer_running = true;
			}
		}
		//TODO: check this, might have to be other way arround
		//if(event_pins_low_to_high)
		if(event_pins_high_to_low)
		{
			push_counter++;
		}
}


/*
static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
    uint32_t err_code;

    // Start detection timer. If timer is already running, the detection period is restarted.
    // NOTE: Using the p_context parameter of app_timer_start() to transfer the pin states to the
    //       timeout handler (by casting event_pins_mask into the equally sized void * p_context
    //       parameter).
    STATIC_ASSERT(sizeof(void *) == sizeof(uint32_t));

    err_code = app_timer_stop(m_detection_delay_timer_id);
    if (err_code != NRF_SUCCESS)
    {
        // The impact in app_button of the app_timer queue running full is losing a button press.
        // The current implementation ensures that the system will continue working as normal.
        return;
    }

    m_pin_transition.low_to_high = event_pins_low_to_high;
    m_pin_transition.high_to_low = event_pins_high_to_low;

    err_code = app_timer_start(m_detection_delay_timer_id,
                               m_detection_delay,
															 //BUTTON_DELAY,
                               (void *)(event_pins_low_to_high | event_pins_high_to_low));
    if (err_code != NRF_SUCCESS)
    {
        // The impact in app_button of the app_timer queue running full is losing a button press.
        // The current implementation ensures that the system will continue working as normal.
    }
}
*/

uint32_t hb_app_button_init(app_button_cfg_t *             p_buttons,
                         uint8_t                        button_count,
                         uint32_t                       detection_delay,
                         app_button_evt_schedule_func_t evt_schedule_func)
{
    uint32_t err_code;
    
    if (detection_delay < APP_TIMER_MIN_TIMEOUT_TICKS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Save configuration.
    mp_buttons          = p_buttons;
    m_button_count      = button_count;
    m_detection_delay   = detection_delay;
    m_evt_schedule_func = evt_schedule_func;
  
    // Configure pins.
    uint32_t pins_transition_mask = 0;
    
    while (button_count--)
    {
        app_button_cfg_t * p_btn = &p_buttons[button_count];

        // Configure pin.
        nrf_gpio_cfg_input(p_btn->pin_no, p_btn->pull_cfg);
        
        // Build GPIOTE user registration masks.
        pins_transition_mask |= (1 << p_btn->pin_no);
    }
    
    // Register button module as a GPIOTE user.
    err_code = app_gpiote_user_register(&m_gpiote_user_id,
                                        pins_transition_mask,
                                        pins_transition_mask,
                                        gpiote_event_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Create polling timer.
    return app_timer_create(&m_detection_delay_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            detection_delay_timeout_handler);
}


uint32_t hb_app_button_enable(void)
{
    if (mp_buttons == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return app_gpiote_user_enable(m_gpiote_user_id);
}


uint32_t hb_app_button_disable(void)
{
    uint32_t err_code;
    
    if (mp_buttons == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    err_code = app_gpiote_user_disable(m_gpiote_user_id);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Make sure polling timer is not running.
    return app_timer_stop(m_detection_delay_timer_id);
}


uint32_t hb_app_button_is_pushed(uint8_t button_id, bool * p_is_pushed)
{
    uint32_t err_code;
    uint32_t active_pins;
    
    if (button_id > m_button_count)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    app_button_cfg_t * p_btn = &mp_buttons[button_id];
    
    if (mp_buttons == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    err_code = app_gpiote_pins_state_get(m_gpiote_user_id, &active_pins);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if(p_btn->active_state == APP_BUTTON_ACTIVE_LOW)
    {
        // If the pin is active low, then the pin being high means it is not pushed.
        if(((active_pins >> p_btn->pin_no) & 0x01))
        {
            *p_is_pushed = false;
        }
        else
        {
            *p_is_pushed = true;  
        }            
    }
    else if(p_btn->active_state == APP_BUTTON_ACTIVE_HIGH)
    {
        // If the pin is active high, then the pin being high means it is pushed.
        if(((active_pins >> p_btn->pin_no) & 0x01))
        {
            *p_is_pushed = true;
        }
        else
        {
            *p_is_pushed = false;   
        }
    }
    
    return NRF_SUCCESS;
}

/*******************************************************************************
* File Name: scan_task.c
*
* Description: This file contains functions that perform network related tasks
* like connecting to an AP, scanning for APs, connection event callbacks, and
* utility function for printing scan results.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include <inttypes.h>

/* FreeRTOS header file.*/
#include "FreeRTOS.h"

/*Wi-Fi Connection Manager header file.*/
#include "cy_wcm.h"

/*Task header file*/
#include "scan_task.h"

#if WNM_LP_FILTER_ENABLE
#include "whd_utils.h"
#endif

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
TaskHandle_t scan_task_handle;

/* This variable holds the value of the total number of the scan results that is
 * available in the scan callback function in the current scan.
 */
uint32_t num_scan_result;

/* This is used to decide the type of scan filter to be used. The valid values
 * are provided in the enumeration scan_filter_mode.
 */
enum scan_filter_mode scan_filter_mode_select = SCAN_FILTER_NONE;

const char* band_string[] =
{
    [CY_WCM_WIFI_BAND_ANY] = "2.4 and 5 GHz",
    [CY_WCM_WIFI_BAND_2_4GHZ] = "2.4 GHz",
    [CY_WCM_WIFI_BAND_5GHZ] = "5 GHz"
};

bool is_retarget_io_initialized = false;
bool is_led_initialized = false;


/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
static void scan_callback( cy_wcm_scan_result_t *result_ptr, void *user_data, cy_wcm_scan_status_t status );
static void print_scan_result(cy_wcm_scan_result_t *result);
#if WNM_LP_FILTER_ENABLE
static void print_filtered_scan_result(cy_wcm_scan_result_t *result);
#endif /* WNM_LP_FILTER_ENABLE */

void print_heap_usage(char *msg);


/*******************************************************************************
 * Function Definitions
 ******************************************************************************/


/*******************************************************************************
 * Function Name: scan_task
 *******************************************************************************
 * Summary: This task initializes the Wi-Fi device, Wi-Fi transport, lwIP
 * network stack, and issues a scan for the available networks. A scan filter
 * is applied depending on the value of scan_filter_mode_select. After starting
 * the scan, it waits for the notification from the scan callback for completion
 * of the scan. It then waits for a delay specified by SCAN_DELAY_PERIOD_MS
 * before repeating the process.
 *
 *
 * Parameters:
 *  void* arg: Task parameter defined during task creation (unused).
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void scan_task(void *arg)
{
    cy_wcm_scan_filter_t scan_filter;
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_wcm_config_t wcm_config = { .interface = CY_WCM_INTERFACE_TYPE_STA };
    cy_wcm_mac_t scan_for_mac_value = {SCAN_FOR_MAC_ADDRESS};

    memset(&scan_filter, 0, sizeof(cy_wcm_scan_filter_t));
    result = cy_wcm_init(&wcm_config);
    error_handler(result, "Failed to initialize Wi-Fi Connection Manager.\n");

    while (true)
    {
        /* Select the type of filter to use.*/
        switch (scan_filter_mode_select)
        {
            case SCAN_FILTER_NONE:
                APP_INFO(("Scanning without any filter\n"));
                break;

            case SCAN_FILTER_SSID:
                APP_INFO(("Scanning for %s.\n", SCAN_FOR_SSID_VALUE));

                /* Configure the scan filter for SSID specified by
                 * SCAN_FOR_SSID_VALUE.
                 */
                scan_filter.mode = CY_WCM_SCAN_FILTER_TYPE_SSID;
                memcpy(scan_filter.param.SSID, SCAN_FOR_SSID_VALUE, sizeof(SCAN_FOR_SSID_VALUE));
                break;

            case SCAN_FILTER_RSSI:
                APP_INFO(("Scanning for RSSI > %d dBm.\n", SCAN_FOR_RSSI_VALUE));

                /* Configure the scan filter for RSSI range specified by
                 * SCAN_FOR_RSSI_VALUE.
                 */
                scan_filter.mode = CY_WCM_SCAN_FILTER_TYPE_RSSI;
                scan_filter.param.rssi_range = SCAN_FOR_RSSI_VALUE;
                break;

            case SCAN_FILTER_MAC:
                APP_INFO(("Scanning for %02X:%02X:%02X:%02X:%02X:%02X.\n", scan_for_mac_value[0], scan_for_mac_value[1], scan_for_mac_value[2], scan_for_mac_value[3], scan_for_mac_value[4], scan_for_mac_value[5]));

                /* Configure the scan filter for MAC specified by scan_for_mac_value
                 */
                scan_filter.mode = CY_WCM_SCAN_FILTER_TYPE_MAC;
                memcpy(scan_filter.param.BSSID, &scan_for_mac_value, sizeof(scan_for_mac_value));
                break;

            case SCAN_FILTER_BAND:
                APP_INFO(("Scanning in %s band.\n", band_string[SCAN_FOR_BAND_VALUE]));

                /* Configure the scan filter for band specified by
                 * SCAN_FOR_BAND_VALUE.
                 */
                scan_filter.mode = CY_WCM_SCAN_FILTER_TYPE_BAND;
                scan_filter.param.band = SCAN_FOR_BAND_VALUE;
                break;

            default:
                break;
        }

        PRINT_SCAN_TEMPLATE();

        if(SCAN_FILTER_NONE == scan_filter_mode_select)
        {
            result = cy_wcm_start_scan(scan_callback, NULL, NULL);
        }
        else
        {
            result = cy_wcm_start_scan(scan_callback, NULL, &scan_filter);
        }

        /* Wait for scan completion if scan was started successfully. The API
         * cy_wcm_start_scan doesn't wait for scan completion. If it is called
         * again when the scan hasn't completed, the API returns
         * CY_WCM_RESULT_SCAN_IN_PROGRESS.
         */
        if (CY_RSLT_SUCCESS == result)
        {
            xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        }

        /* Define PRINT_HEAP_USAGE using DEFINES variable in the Makefile. */
        print_heap_usage("After scan results are printed to UART");

        vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
    }
}


/*******************************************************************************
 * Function Name: scan_callback
 *******************************************************************************
 * Summary: The callback function which accumulates the scan results. After
 * completing the scan, it sends a task notification to scan_task.
 *
 * Parameters:
 *  cy_wcm_scan_result_t *result_ptr: Pointer to the scan result
 *  void *user_data: User data.
 *  cy_wcm_scan_status_t status: Status of scan completion.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void scan_callback(cy_wcm_scan_result_t *result_ptr, void *user_data, cy_wcm_scan_status_t status)
{
    if ((strlen((const char *)result_ptr->SSID) != 0) && (status == CY_WCM_SCAN_INCOMPLETE))
    {
        num_scan_result++;
#if WNM_LP_FILTER_ENABLE
		print_filtered_scan_result(result_ptr);
#else
        print_scan_result(result_ptr);
#endif /* WNM_LP_FILTER_ENABLE */
    }

    if ( (CY_WCM_SCAN_COMPLETE == status) )
    {
        /* Reset the number of scan results to 0 for the next scan.*/
        num_scan_result = 0;

        /* Notify that scan has completed.*/
        xTaskNotify(scan_task_handle, 0, eNoAction);
    }
}

#if WNM_LP_FILTER_ENABLE
#define WNM_EXTENDED_FEATURES_MIN_LENGTH	4

typedef struct __attribute__((packed))
{
    whd_tlv8_header_t tlv_header; /* id, length */
	uint8_t bss_coex_mgmt:1;
	uint8_t reserved1:1;
	uint8_t ext_chan_switch:1;
	uint8_t reserved2:1;
	uint8_t psmp:1;
	uint8_t reserved3:1;
	uint8_t spsmp:1;
	uint8_t event:1;
	uint8_t diagnostics:1;
	uint8_t mc_diagnostics:1;
	uint8_t loc_tracking:1;
	uint8_t fms:1;
	uint8_t proxy_arp:1;
	uint8_t collocated_interference:1;
	uint8_t civic_location:1;
	uint8_t geospatial_location:1;
	uint8_t tfs:1;
	uint8_t wnm_sleep_mode:1;
	uint8_t tim_broadcast:1;
	uint8_t bss_transition:1;
	uint8_t qos_traffic_capability:1;
	uint8_t ac_station_count:1;
	uint8_t multi_bssid:1;
	uint8_t timing_measurement:1;
	uint8_t channel_usage:1;
	uint8_t ssid_list:1;
	uint8_t dms:1;
	uint8_t utc_tsf_offset:1;
	uint8_t tpu_buffer_sta:1;
	uint8_t tdls_peer_psm:1;
	uint8_t tdls_chan_switch:1;
	uint8_t interworking:1;
	uint8_t qos_map:1;
	uint8_t ebr:1;
	uint8_t sspn_if:1;
	uint8_t reserved4:1;
	uint8_t msgcf_cap:1;
	uint8_t tdls_support:1;
	uint8_t tdls_prohibited:1;
	uint8_t tdls_chan_switch_prohibited:1;
	uint8_t reject_unadmitted_frame:1;
	uint8_t service_interval_granularity:3;
	uint8_t identifier_location:1;
	uint8_t uapsd_coexistence:1;
	uint8_t wnm_notification:1;
	uint8_t qab_capability:1;
	uint8_t utf8_ssid:1;
	uint8_t qmf_activated:1;
	uint8_t qmf_reconfig_active:1;
	uint8_t robust_av_streaming:1;
	uint8_t advanced_gcr:1;
	uint8_t mesh_gcr:1;
	uint8_t scs:1;
	uint8_t qload_report:1;
	uint8_t alt_edca:1;
	uint8_t unprotected_txop_negotiation:1;
	uint8_t protected_txop_negotiation:1;
	uint8_t reserved5:1;
	uint8_t protected_qload_report:1;
	uint8_t tdls_wider_bw:1;
	uint8_t operating_mode_notification:1;
	uint8_t max_msdu_in_amsdu:2;
	uint8_t chan_schedule_mgmt:1;
	uint8_t geo_database_ib_enab_signal:1;
	uint8_t network_chan_ctrl:1;
	uint8_t white_space_map:1;
	uint8_t chan_avail_query:1;
	uint8_t fine_timing_measure_responder:1;
	uint8_t fine_timing_measure_initiator:1;
	uint8_t fils_capable:1;
	uint8_t ext_spectrum_mgmt_capable:1;
	uint8_t future_chan_capable:1;
	uint8_t reserved6:2;
	uint8_t twt_requester:1;
	uint8_t twt_responder:1;
	uint8_t obss_nb_ru_in_ulofdma_tolerance:1;

}extended_capabilities_ie_t;

static void print_filtered_scan_result(cy_wcm_scan_result_t *result)
{
	extended_capabilities_ie_t *extended_capabilities_ie;

	extended_capabilities_ie = (extended_capabilities_ie_t *)whd_parse_tlvs(result->ie_ptr, result->ie_ptr, DOT11_IE_ID_EXTENDED_CAPABILITIES);

	if(extended_capabilities_ie != NULL &&
		(extended_capabilities_ie->tlv_header.length >= WNM_EXTENDED_FEATURES_MIN_LENGTH)) /* Check if all the octals for 11v features are present */
	{
		/* 11V feature is part of the beacon/probe packet */
		uint8_t *rp = (uint8_t *)extended_capabilities_ie;
		    printf("%-24s   FMS=%d   ARP=%d   WNM_Sleep=%d   DMS=%d   TIM_Broadcast=%d R- %02x %02x %02x\n",
           result->SSID, extended_capabilities_ie->fms, extended_capabilities_ie->proxy_arp, extended_capabilities_ie->wnm_sleep_mode,
           extended_capabilities_ie->dms, extended_capabilities_ie->tim_broadcast, rp[3], rp[4], rp[5]);
	}
	else
	{
		 printf("%-32s	 doesn't support 11v features\n",
				result->SSID);
	}
}
#endif /* WNM_LP_FILTER_ENABLE */

/*******************************************************************************
 * Function Name: print_scan_result
 *******************************************************************************
 * Summary: This function prints the scan result accumulated by the scan
 * handler.
 *
 *
 * Parameters:
 *  cy_wcm_scan_result_t *result: Pointer to the scan result.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void print_scan_result(cy_wcm_scan_result_t *result)
{
    char* security_type_string;

    /* Convert the security type of the scan result to the corresponding
     * security string
     */
    switch (result->security)
    {
    case CY_WCM_SECURITY_OPEN:
        security_type_string = SECURITY_OPEN;
        break;
    case CY_WCM_SECURITY_WEP_PSK:
        security_type_string = SECURITY_WEP_PSK;
        break;
    case CY_WCM_SECURITY_WEP_SHARED:
        security_type_string = SECURITY_WEP_SHARED;
        break;
    case CY_WCM_SECURITY_WPA_TKIP_PSK:
        security_type_string = SECURITY_WEP_TKIP_PSK;
        break;
    case CY_WCM_SECURITY_WPA_AES_PSK:
        security_type_string = SECURITY_WPA_AES_PSK;
        break;
    case CY_WCM_SECURITY_WPA_MIXED_PSK:
        security_type_string = SECURITY_WPA_MIXED_PSK;
        break;
    case CY_WCM_SECURITY_WPA2_AES_PSK:
        security_type_string = SECURITY_WPA2_AES_PSK;
        break;
    case CY_WCM_SECURITY_WPA2_TKIP_PSK:
        security_type_string = SECURITY_WPA2_TKIP_PSK;
        break;
    case CY_WCM_SECURITY_WPA2_MIXED_PSK:
        security_type_string = SECURITY_WPA2_MIXED_PSK;
        break;
    case CY_WCM_SECURITY_WPA2_FBT_PSK:
        security_type_string = SECURITY_WPA2_FBT_PSK;
        break;
    case CY_WCM_SECURITY_WPA3_SAE:
        security_type_string = SECURITY_WPA3_SAE;
        break;
    case CY_WCM_SECURITY_WPA3_WPA2_PSK:
        security_type_string = SECURITY_WPA3_WPA2_PSK;
        break;
    case CY_WCM_SECURITY_IBSS_OPEN:
        security_type_string = SECURITY_IBSS_OPEN;
        break;
    case CY_WCM_SECURITY_WPS_SECURE:
        security_type_string = SECURITY_WPS_SECURE;
        break;
    case CY_WCM_SECURITY_UNKNOWN:
        security_type_string = SECURITY_UNKNOWN;
        break;
    case CY_WCM_SECURITY_WPA2_WPA_AES_PSK:
        security_type_string = SECURITY_WPA2_WPA_AES_PSK;
        break;
    case CY_WCM_SECURITY_WPA2_WPA_MIXED_PSK:
        security_type_string = SECURITY_WPA2_WPA_MIXED_PSK;
        break;
    case CY_WCM_SECURITY_WPA_TKIP_ENT:
        security_type_string = SECURITY_WPA_TKIP_ENT;
        break;
    case CY_WCM_SECURITY_WPA_AES_ENT:
        security_type_string = SECURITY_WPA_AES_ENT;
        break;
    case CY_WCM_SECURITY_WPA_MIXED_ENT:
        security_type_string = SECURITY_WPA_MIXED_ENT;
        break;
    case CY_WCM_SECURITY_WPA2_TKIP_ENT:
        security_type_string = SECURITY_WPA2_TKIP_ENT;
        break;
    case CY_WCM_SECURITY_WPA2_AES_ENT:
        security_type_string = SECURITY_WPA2_AES_ENT;
        break;
    case CY_WCM_SECURITY_WPA2_MIXED_ENT:
        security_type_string = SECURITY_WPA2_MIXED_ENT;
        break;
    case CY_WCM_SECURITY_WPA2_FBT_ENT:
        security_type_string = SECURITY_WPA2_FBT_ENT;
        break;
    default:
        security_type_string = SECURITY_UNKNOWN;
        break;
    }

    printf(" %2"PRIu32"   %-32s     %4d     %2d      %02X:%02X:%02X:%02X:%02X:%02X         %-15s\n",
           num_scan_result, result->SSID,
           result->signal_strength, result->channel, result->BSSID[0], result->BSSID[1],
           result->BSSID[2], result->BSSID[3], result->BSSID[4], result->BSSID[5],
           security_type_string);
}


/*******************************************************************************
 * Function Name: gpio_interrupt_handler
 *******************************************************************************
 * Summary:
 *  GPIO interrupt service routine. This function detects button presses and
 *  changes the type of the applied scan filter.
 *
 * Parameters:
 *  void *callback_arg : pointer to the variable passed to the ISR
 *  cyhal_gpio_event_t event : GPIO event type
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    /* Increment and check if the scan filter selected is invalid. If invalid,
     * reset to no scan filter.
     */
    if(++scan_filter_mode_select >= SCAN_FILTER_INVALID)
    {
        scan_filter_mode_select = SCAN_FILTER_NONE;
    }
}


/*******************************************************************************
* Function Name: error_handler
********************************************************************************
*
* Summary:
* This function processes unrecoverable errors such as any component
* initialization errors etc. In case of such error, the system will halt the
* CPU.
*
* Parameters:
* cy_rslt_t result: contains the result of an operation.
* char* message: contains the error message that is printed to the serial
* terminal.
*
* Note: If error occurs, interrupts are disabled.
*
*******************************************************************************/
void error_handler(cy_rslt_t result, char* message)
{
    if(CY_RSLT_SUCCESS != result)
    {
        if(is_led_initialized)
        {
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        }

        if( is_retarget_io_initialized && (NULL != message) )
        {
            ERR_INFO(("%s", message));
        }

        __disable_irq();
        CY_ASSERT(0);
    }
}


/* [] END OF FILE */

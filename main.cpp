/*
 * Copyright (c) 2019 Riot Micro. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*****************************************************************************************************************************************************
 *
 * I N C L U D E S
 *
 ****************************************************************************************************************************************************/
#include "mbed.h"
#include "CellularContext.h"
#include "AT_CellularDevice.h"
#include "CellularLog.h"
#include "ThisThread.h"

#if (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_MANHOLE)
#include "LIS3DH.h"             /*Accelerometer sensor*/
#include "BME280.h"             /*Atmospheric sensor*/
#include "OPT3001.h"            /*Light sensor*/
#include "VL53L1X.h"            /*Distance sensor*/
#include "LIS2MDLSensor.h"      /*Magnetic sensor*/
#endif

#include "SEGGER_RTT.h"

/*****************************************************************************************************************************************************
 *
 * E X T E R N S
 *
 ****************************************************************************************************************************************************/


/*****************************************************************************************************************************************************
 *
 * M A C R O S
 *
 ****************************************************************************************************************************************************/

#define DEMO_NONE               0
#define DEMO_DWEET_SIGNAL       1
#define DEMO_DWEET_MANHOLE      2

#define LIVE_NETWORK

#define LED_ON      (0)
#define LED_OFF     (1)

#define SGNL_ACTV   (0)
#define SGNL_INACTV (1)

#if (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_MANHOLE)
  #define MANHOLE_CHN_HEARTBEAT_OUT         (1)
  #define MANHOLE_CHN_TEMPERATURE_OUT       (2)
  #define MANHOLE_CHN_HUMIDITY_OUT          (3)
  #define MANHOLE_CHN_PRESSURE_OUT          (4)
  #define MANHOLE_CHN_ORIENTATION_X_OUT     (5)
  #define MANHOLE_CHN_ORIENTATION_Y_OUT     (6)
  #define MANHOLE_CHN_ORIENTATION_Z_OUT     (7)
  #define MANHOLE_CHN_LIGHT_OUT             (8)
  #define MANHOLE_CHN_DIST_OUT              (9)
  #define MANHOLE_CHN_RSSI_OUT              (10)
  #define MANHOLE_CHN_MAG_X_OUT             (11)
  #define MANHOLE_CHN_MAG_Y_OUT             (12)
  #define MANHOLE_CHN_MAG_Z_OUT             (13)

  #define ENV_IDX_TEMPERATURE               (0)
  #define ENV_IDX_PRESSURE                  (1)
  #define ENV_IDX_HUMIDITY                  (2)

  #define TILT_IDX_X                        (0)
  #define TILT_IDX_Y                        (1)
  #define TILT_IDX_Z                        (2)

  #define I2C_TILT_SENSOR_ADDR              ((uint8_t) (0x30))
  #define I2C_ENV_SENSOR_ADDR               ((uint8_t) (0xEC))
  #define I2C_LIGHT_SENSOR_ADDR             ((uint8_t) (0x88))
  #define I2C_DIST_SENSOR_ADDR              ((uint8_t) (0x52))
  #define I2C_MAGN_SENSOR_ADDR              ((uint8_t) (0x3C))

  #define DIST_SENSOR_WAIT_STEP             (100) // ms
  #define DIST_SENSOR_WAIT_MAX              (3000)

  #define DISCARD_SENSOR_FLUCT              (1)

  #define DWEET_UPDATE_MS                   (1000)
  #define SENSOR_TIME_RESOLUTION            (100)
#endif

#if (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_SIGNAL) || (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_MANHOLE)
  #define MSG_LEN                           (500)
  #define SERVER_NAME                       "www.dweet.io"
#endif

#define CALC_ABSOLUTE_DIFF(a, b)            ((a) > (b) ? (a) - (b) : (b) - (a))

#define UPDATE_SENSOR_READS(a, b) \
{ \
    a[0]    = b[0]; \
    a[1]    = b[1]; \
    a[2]    = b[2]; \
}

#define SYSTEM_RECOVERY() \
{ \
    LOG_ERROR("SYSTEM RESET..."); \
    ThisThread::sleep_for(2000); \
    NVIC_SystemReset(); \
}

#define BAND3_EARFCN    1440
#define BAND5_EARFCN    2525
#define BAND28_EARFCN   9300
#define BAND2_EARFCN    744
#define BAND8_EARFCN    3606
#define BAND20_EARFCN   6300
#define BAND86_EARFCN   70546

#define RADIO_EARFCN    BAND28_EARFCN

#if (BAND3_EARFCN == RADIO_EARFCN)
  #define RADIO_BAND    3
#elif (BAND5_EARFCN == RADIO_EARFCN)
  #define RADIO_BAND    5
#elif (BAND28_EARFCN == RADIO_EARFCN)
  #define RADIO_BAND    28
#elif (BAND2_EARFCN == RADIO_EARFCN)
  #define RADIO_BAND    2
#elif (BAND8_EARFCN == RADIO_EARFCN)
  #define RADIO_BAND    8
#elif (BAND20_EARFCN == RADIO_EARFCN)
  #define RADIO_BAND    20
#elif (BAND86_EARFCN == RADIO_EARFCN)
  #define RADIO_BAND    86
#endif

#define XSTR(x) STR(x)
#define STR(x)  #x
/*****************************************************************************************************************************************************
 *
 * T Y P E   D E F I N I T I O N S
 *
 ****************************************************************************************************************************************************/

/** Sensors list.
 */
typedef enum
{
    SENSOR_TILT_LIS3DH,     /*Accelerometer sensor*/
    SENSOR_ENVIRO_BME280,   /*Atmospheric sensor*/
    SENSOR_LIGHT_OPT3001,   /*Light sensor*/
    SENSOR_DIST_VL53L1X,    /*Distance sensor*/
    SENSOR_MAGNT_LIS2MDL    /*Magnetic sensor*/
} SensorSelect_e;


/*****************************************************************************************************************************************************
 *
 * G L O B A L   V A R I A B L E   D E F I N I T I O N S
 *
 ****************************************************************************************************************************************************/

/*****************************************************************************************************************************************************
 *
 * L O C A L   V A R I A B L E   D E F I N I T I O N S
 *
 ****************************************************************************************************************************************************/

/* General. */
DigitalOut  ledsPtr[]  = {  DigitalOut(P0_10, SGNL_INACTV),
                            DigitalOut(P0_22, SGNL_INACTV)  };

DigitalOut modemPowerEn[] = { DigitalOut(P0_7, SGNL_INACTV),
                            DigitalOut(P0_8, SGNL_INACTV),
                            DigitalOut(P0_9, SGNL_INACTV)  };

DigitalOut modem_chen = DigitalOut(MDMCHEN, SGNL_INACTV);
DigitalOut modem_remap = DigitalOut(MDMREMAP, SGNL_INACTV);
DigitalOut modem_reset = DigitalOut(MDMRST, SGNL_INACTV);

DigitalOut xshut = DigitalOut(P0_6, SGNL_INACTV);

AnalogIn   batteryMon(A0);
DigitalOut batteryMonEn = DigitalOut(P0_3, SGNL_INACTV);

AnalogIn   flexAnalog(A2);

NetworkInterface*   interface   = NULL;

/*****************************************************************************************************************************************************
 *
 * L O C A L   F U N C T I O N   D E F I N I T I O N S
 *
 ****************************************************************************************************************************************************/
#if MBED_CONF_MBED_TRACE_ENABLE
static rtos::Mutex trace_mutex;

static void trace_wait()
{
    trace_mutex.lock();
}

static void trace_release()
{
    trace_mutex.unlock();
}

static char time_st[50];

static char* trace_time(size_t ss)
{
    snprintf(time_st, 49, "[%08llums]", Kernel::get_ms_count());
    return time_st;
}

static void trace_print_function(const char *format)
{
    SEGGER_RTT_printf(0, format);
    SEGGER_RTT_printf(0, "\n");
}

static void trace_open()
{
    mbed_trace_init();
    mbed_trace_prefix_function_set( &trace_time );

    mbed_trace_mutex_wait_function_set(trace_wait);
    mbed_trace_mutex_release_function_set(trace_release);

    mbed_trace_cmdprint_function_set(trace_print_function);
    mbed_trace_print_function_set(trace_print_function);

    mbed_cellular_trace::mutex_wait_function_set(trace_wait);
    mbed_cellular_trace::mutex_release_function_set(trace_release);
}

static void trace_close()
{
    mbed_cellular_trace::mutex_wait_function_set(NULL);
    mbed_cellular_trace::mutex_release_function_set(NULL);

    mbed_trace_free();
}
#endif // #if MBED_CONF_MBED_TRACE_ENABLE

#if defined(LIVE_NETWORK)
/**
 * Connects to the Cellular Network
 */
static nsapi_error_t do_connect()
{
    nsapi_error_t retcode = NSAPI_ERROR_OK;
    uint8_t retry_counter = 0;

    while (interface->get_connection_status() != NSAPI_STATUS_GLOBAL_UP) {
        retcode = interface->connect();
        if (retcode == NSAPI_ERROR_AUTH_FAILURE) {
            LOG_ERROR("Authentication Failure. Exiting application\n");
        } else if (retcode == NSAPI_ERROR_OK) {
            LOG_HI("Connection Established.\n");
        } else if (retry_counter > 3) {
            LOG_ERROR("Fatal connection failure: %d\n", retcode);
        } else {
            LOG_WARN("\n\nCouldn't connect: %d, will retry\n", retcode);
            retry_counter++;
            continue;
        }
        break;
    }
    return retcode;
}

static void status_callback(nsapi_event_t status, intptr_t param)
{
    CellularContext * context = (CellularContext *)interface;
    AT_CellularDevice * device = (AT_CellularDevice *) context->get_device();

    if (status == NSAPI_EVENT_CONNECTION_STATUS_CHANGE) {
        switch(param) {
            case NSAPI_STATUS_LOCAL_UP:
                LOG_LO("Local IP address set (NSAPI_STATUS_LOCAL_UP)!");
                break;
            case NSAPI_STATUS_GLOBAL_UP:
                LOG_LO("Global IP address set (NSAPI_STATUS_GLOBAL_UP)!");
                break;
            case NSAPI_STATUS_CONNECTING:
                LOG_LO("Connecting to network (NSAPI_STATUS_CONNECTING)!");
                break;
            case NSAPI_STATUS_DISCONNECTED:
                LOG_LO("No connection to network (NSAPI_STATUS_DISCONNECTED)!");
                SYSTEM_RECOVERY();
                break;
            default:
                LOG_ERROR("Not supported (%x%X)", param);
                SYSTEM_RECOVERY();
                break;
        }
    }
    else {
        cell_callback_data_t* cb_status = (cell_callback_data_t*)param;
        switch((cellular_connection_status_t)status) {
            case CellularDeviceReady:
            {
                LOG_LO("CellularDeviceReady (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                /* Set the frequency band and earfcn */
                device->_at->at_cmd_discard("+CFUN", "=4");
                device->_at->at_cmd_discard("+BAND", "=" XSTR(RADIO_BAND));
                device->_at->at_cmd_discard("+CFUN", "=1");
                device->_at->at_cmd_discard("+EARFCN", "=" XSTR(RADIO_EARFCN));
                break;
            }
            case CellularSIMStatusChanged:
                LOG_LO("CellularSIMStatusChanged (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularRegistrationStatusChanged:
                LOG_LO("CellularRegistrationStatusChanged (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularRegistrationTypeChanged:
                LOG_LO("CellularRegistrationTypeChanged (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularCellIDChanged:
                LOG_LO("CellularCellIDChanged (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularRadioAccessTechnologyChanged:
                LOG_LO("CellularRadioAccessTechnologyChanged (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularAttachNetwork:
                LOG_LO("CellularAttachNetwork (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularActivatePDPContext:
                LOG_LO("CellularActivatePDPContext (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularSignalQuality:
                LOG_LO("CellularSignalQuality (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularStateRetryEvent:
                LOG_HI("CellularStateRetryEvent (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            case CellularDeviceTimeout:
                //LOG_LO("CellularDeviceTimeout (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                break;
            default:
                LOG_ERROR("Not supported status (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
                SYSTEM_RECOVERY();
                break;
        }
        if (NSAPI_ERROR_OK != cb_status->error)
        {
            LOG_ERROR("Unrecoverable Error: (error=%d) (status=%d) (is_final_try=%d)", cb_status->error, cb_status->status_data, cb_status->final_try);
            SYSTEM_RECOVERY();
        }
    }
}
#endif //#if defined(LIVE_NETWORK)

/* --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- */

void blink_led(
    int aCount)
{
    for (int i = 0; i < aCount; i++)
    {
        ledsPtr[0] = ledsPtr[1] = LED_ON;
        ThisThread::sleep_for(200);
        ledsPtr[0] = ledsPtr[1] = LED_OFF;
        ThisThread::sleep_for(400);
    }
}

/* --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- */

#if (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_SIGNAL)
int send_dweet_signal(const char *key, int val)
{
    TCPSocket   socket;

    int result;
    int bytes;
    int retValue    = 0;

    char*   message     = (char*) malloc(MSG_LEN);
    char*   response    = (char*) malloc(MSG_LEN);
    if (NULL == message ||
        NULL == response)
    {
        LOG_HI("ERROR: Failed to allocate buffer(s)");
        retValue    = -1;
        goto FuncExit;
    }
    memset(message, 0, MSG_LEN);
    memset(response, 0, MSG_LEN);

    socket.set_timeout(30000);

    /* create the socket */
    LOG_HI("socket.open...");
    result  = socket.open(interface);
    if (result < 0)
    {
        LOG_WARN("Failed to open TCP Socket ... error = %d", result);
        retValue    = -1;
        goto FuncExit;
    }

    /* connect the socket */
    LOG_HI("socket.connect...");
    result  = socket.connect(SERVER_NAME, 80);
    if (result < 0)
    {
        LOG_WARN("Failed to connect with %s ... error = %d", SERVER_NAME, result);
        retValue    = -1;
        goto FuncExit;
    }

    // compose GET message buffer
    bytes = snprintf(message, MSG_LEN, "GET /dweet/for/" MBED_APP_CONF_DWEET_PAGE "?%s=%d HTTP/1.1\nHost: dweet.io\r\nConnection: close\r\n\r\n", key, val);
    message[bytes] = 0;

    LOG_HI("socket.send...");
    result  = socket.send(message, strlen(message));
    if (result < 0)
    {
        LOG_WARN("Failed to send HTTP request ... error = %d", result);
        retValue    = -1;
        goto FuncExit;
    }

    /* receive the response */
    memset(response, 0, MSG_LEN);

    LOG_HI("socket.recv...");
    result  = socket.recv(response, MSG_LEN - 1);
    if (result < 0)
    {
        LOG_WARN("Failed to receive HTTP response, error = %d", result);
        retValue    = -1;
        goto FuncExit;
    }
    // LOG_HI("Socket received: %s", response);

FuncExit:
    LOG_HI("socket.close...");
    socket.close();
    if (NULL != message)
    {
        free((void*) message);
    }
    if (NULL != response)
    {
        free((void*) response);
    }
    return retValue;
}
#endif /*#if (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_SIGNAL)*/

#if (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_MANHOLE)
char* manhole_channel_enum(int aChannel)
{
    char*   enum_string = NULL;

    switch(aChannel)
    {
        case MANHOLE_CHN_HEARTBEAT_OUT:
            enum_string = (char*) "HEARTBEAT";
            break;
        case MANHOLE_CHN_TEMPERATURE_OUT:
            enum_string = (char*) "TEMPERATURE";
            break;
        case MANHOLE_CHN_HUMIDITY_OUT:
            enum_string = (char*) "HUMIDITY";
            break;
        case MANHOLE_CHN_PRESSURE_OUT:
            enum_string = (char*) "PRESSURE";
            break;
        case MANHOLE_CHN_ORIENTATION_X_OUT:
            enum_string = (char*) "ORIENTATION_X";
            break;
        case MANHOLE_CHN_ORIENTATION_Y_OUT:
            enum_string = (char*) "ORIENTATION_Y";
            break;
        case MANHOLE_CHN_ORIENTATION_Z_OUT:
            enum_string = (char*) "ORIENTATION_Z";
            break;
        case MANHOLE_CHN_LIGHT_OUT:
            enum_string = (char*) "LIGHT";
            break;
        case MANHOLE_CHN_DIST_OUT:
            enum_string = (char*) "DISTANCE";
            break;
        case MANHOLE_CHN_RSSI_OUT:
            enum_string = (char*) "RSSI";
            break;
        case MANHOLE_CHN_MAG_X_OUT:
            enum_string = (char*) "MAG_X";
            break;
        case MANHOLE_CHN_MAG_Y_OUT:
            enum_string = (char*) "MAG_Y";
            break;
        case MANHOLE_CHN_MAG_Z_OUT:
            enum_string = (char*) "MAG_Z";
            break;
        default:
            LOG_ERROR("%s: Invalid channel (%d) selection", __func__, aChannel);
            break;
    }

    return enum_string;
}

int sendSensorReadings(char* readings)
{
    TCPSocket   socket;

    int result;
    int bytes;
    int retValue    = 0;
    char*   message     = (char*) malloc(MSG_LEN);
    char*   response    = (char*) malloc(MSG_LEN);

    static bool resolved = 0;
    static SocketAddress addr_resolved;

    if (NULL == message ||
        NULL == response)
    {
        LOG_HI("ERROR: Failed to allocate buffer(s)");
        retValue    = -1;
        goto FuncExit;
    }
    memset(message, 0, MSG_LEN);
    memset(response, 0, MSG_LEN);

    socket.set_timeout(30000);

    if (!resolved)
    {
        LOG_HI("Resolve addresee...");
        result = interface->gethostbyname(SERVER_NAME, &addr_resolved);
        if (result < 0)
        {
            LOG_WARN("Failed to resolve address with error = %d", result);
            retValue    = -1;
            goto FuncExit2;
        }
        addr_resolved.set_port(80);
        resolved = 1;
    }

    /* create the socket */
    LOG_HI("socket.open...");
    result  = socket.open(interface);
    if (result < 0)
    {
        LOG_WARN("Failed to open TCP Socket ... error = %d", result);
        retValue    = -1;
        goto FuncExit;
    }

    /* connect the socket */
    LOG_HI("socket.connect...");
    result  = socket.connect(addr_resolved);
    if (result < 0)
    {
        LOG_WARN("Failed to connect with %s ... error = %d", SERVER_NAME, result);
        retValue    = -1;
        goto FuncExit;
    }

    // TODO: may be retry GET if HTTP request or response is in error

    // compose GET message buffer
    bytes = snprintf(message, MSG_LEN, "GET /dweet/for/" MBED_APP_CONF_DWEET_PAGE "?%s HTTP/1.1\nHost: dweet.io\r\nConnection: close\r\n\r\n", readings);
    message[bytes] = 0;

    LOG_HI("socket.send...");
    result  = socket.send(message, strlen(message));
    if (result < 0)
    {
        LOG_WARN("Failed to send HTTP request ... error = %d", result);
        retValue    = -1;
        goto FuncExit;
    }

    /* receive the response */
    memset(response, 0, MSG_LEN);

    LOG_HI("socket.recv...");
    result  = socket.recv(response, MSG_LEN - 1);
    if (result < 0)
    {
        LOG_WARN("Failed to receive HTTP response, error = %d", result);
        retValue    = -1;
        goto FuncExit;
    }
    // LOG_HI("Socket received: %s", response);

FuncExit:
    LOG_HI("socket.close...");
    socket.close();
    if (NULL != message)
    {
        free((void*) message);
    }
    if (NULL != response)
    {
        free((void*) response);
    }
    return retValue;

FuncExit2:
    return retValue;
}
#endif
/* --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- */

#if (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_MANHOLE)

/**
 * I2C
 * SCL (Green wire) -> pin 1 on UARTs connector; J3 / ANA2 / P0_2
 * SDA (Blue wire)  -> pin 3 on UARTs connector; J3 / ANA3 / P0_3
 */

void rm_i2c_scan(
    uint8_t    aFoundAddrs[])
{
    int addrCount   = 0;

    I2C i2c((PinName) I2C_SDA0, (PinName) I2C_SCL0);
    ThisThread::sleep_for(1000);
    LOG_HI(">>>>>>>>>  START  I2C  ADDR  SCAN  <<<<<<<<<");
    for(int i = 0; i < 128 ; i++)
    {
        int         addr8bit    = (i << 1);
        // LOG_HI("I2C scan 8 bit address 0x%02X", addr8bit);

        const char  data        = 1;

        ThisThread::sleep_for(50);
        i2c.start();
        if(0 == i2c.write(addr8bit, &data, 1))
        {
            LOG_HI("0x%02X ACK", addr8bit);
            aFoundAddrs[addrCount++]    = addr8bit;
        }
        i2c.stop();
    }
    LOG_HI(">>>>>>>>>  FINISHED  I2C  ADDR  SCAN  <<<<<<<<<");
}

/* --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- */

void update_sensor_params(
    SensorSelect_e  aSensor,
    int             aValNew[],
    int             aValOld[],
    int             aValSend[],
    bool*           aIsSendUpdate)
{
    switch (aSensor)
    {
        case SENSOR_LIGHT_OPT3001:
        case SENSOR_DIST_VL53L1X:
        {
            if ( CALC_ABSOLUTE_DIFF(aValNew[0], aValOld[0]) > 10 )
            {
                if(false == *aIsSendUpdate)
                {
                    aValSend[0] = aValNew[0];
                }
                else if (CALC_ABSOLUTE_DIFF(aValNew[0], aValOld[0]) > CALC_ABSOLUTE_DIFF(aValOld[0], aValSend[0]))
                {
                    aValSend[0] = aValNew[0];
                }
                *aIsSendUpdate    = true;
            }
            break;
        }

        case SENSOR_TILT_LIS3DH:
        {
            if ( CALC_ABSOLUTE_DIFF(aValNew[0], aValOld[0]) > 8 ||
                 CALC_ABSOLUTE_DIFF(aValNew[1], aValOld[1]) > 8 ||
                 CALC_ABSOLUTE_DIFF(aValNew[2], aValOld[2]) > 8 )
            {
                LOG_WARN("Tilt REMOVED X, Y, Z = %d, %d, %d vs %d, %d, %d",
                         aValNew[TILT_IDX_X], aValNew[TILT_IDX_Y], aValNew[TILT_IDX_Z],
                         aValOld[TILT_IDX_X], aValOld[TILT_IDX_Y], aValOld[TILT_IDX_Z]);
                if(false == *aIsSendUpdate)
                {
                    UPDATE_SENSOR_READS(aValSend, aValNew);
                }
                else if (CALC_ABSOLUTE_DIFF(aValNew[0], aValOld[0]) > CALC_ABSOLUTE_DIFF(aValOld[0], aValSend[0]) ||
                         CALC_ABSOLUTE_DIFF(aValNew[1], aValOld[1]) > CALC_ABSOLUTE_DIFF(aValOld[1], aValSend[1]) ||
                         CALC_ABSOLUTE_DIFF(aValNew[2], aValOld[2]) > CALC_ABSOLUTE_DIFF(aValOld[2], aValSend[2]))
                {
                    UPDATE_SENSOR_READS(aValSend, aValNew);
                }
                *aIsSendUpdate    = true;
            }
            break;
        }

        case SENSOR_ENVIRO_BME280:
        {
            if ( CALC_ABSOLUTE_DIFF(aValNew[0], aValOld[0]) > DISCARD_SENSOR_FLUCT ||
                 CALC_ABSOLUTE_DIFF(aValNew[1], aValOld[1]) > DISCARD_SENSOR_FLUCT ||
                 CALC_ABSOLUTE_DIFF(aValNew[2], aValOld[2]) > DISCARD_SENSOR_FLUCT )
            {
                if(false == *aIsSendUpdate)
                {
                    UPDATE_SENSOR_READS(aValSend, aValNew);
                }
                else if (CALC_ABSOLUTE_DIFF(aValNew[0], aValOld[0]) > CALC_ABSOLUTE_DIFF(aValOld[0], aValSend[0]) ||
                         CALC_ABSOLUTE_DIFF(aValNew[1], aValOld[1]) > CALC_ABSOLUTE_DIFF(aValOld[1], aValSend[1]) ||
                         CALC_ABSOLUTE_DIFF(aValNew[2], aValOld[2]) > CALC_ABSOLUTE_DIFF(aValOld[2], aValSend[2]))
                {
                    UPDATE_SENSOR_READS(aValSend, aValNew);
                }
                *aIsSendUpdate    = true;
            }
            break;
        }

        case SENSOR_MAGNT_LIS2MDL:
        {
            if ( CALC_ABSOLUTE_DIFF(aValNew[0], aValOld[0]) > 10 ||
                 CALC_ABSOLUTE_DIFF(aValNew[1], aValOld[1]) > 10 ||
                 CALC_ABSOLUTE_DIFF(aValNew[2], aValOld[2]) > 10 )
            {
                if(false == *aIsSendUpdate)
                {
                    UPDATE_SENSOR_READS(aValSend, aValNew);
                }
                else if (CALC_ABSOLUTE_DIFF(aValNew[0], aValOld[0]) > CALC_ABSOLUTE_DIFF(aValOld[0], aValSend[0]) ||
                         CALC_ABSOLUTE_DIFF(aValNew[1], aValOld[1]) > CALC_ABSOLUTE_DIFF(aValOld[1], aValSend[1]) ||
                         CALC_ABSOLUTE_DIFF(aValNew[2], aValOld[2]) > CALC_ABSOLUTE_DIFF(aValOld[2], aValSend[2]))
                {
                    UPDATE_SENSOR_READS(aValSend, aValNew);
                }
                *aIsSendUpdate    = true;
            }
            break;
        }
        default:
            LOG_ERROR("%s: Invalid sensor selection %d", __func__, aSensor);
            break;
    }
}

void demo_loop(void)
{
    uint8_t tiltId;
    float   totalWaitTime   = 0;

    I2C     i2c((PinName) I2C_SDA0,
                (PinName) I2C_SCL0);
    i2c.frequency(100000);

    LIS3DH  sensorTilt(i2c, I2C_TILT_SENSOR_ADDR);
    BME280  sensorEnv(i2c, I2C_ENV_SENSOR_ADDR);
    OPT3001 sensorLight(I2C_SDA0, I2C_SCL0);
    VL53L1X sensorDist(I2C_SDA0, I2C_SCL0);
//    LIS2MDL sensorMagn(i2c, I2C_MAGN_SENSOR_ADDR);
    static DevI2C devI2c(I2C_SDA0,I2C_SCL0);
    LIS2MDLSensor sensorMagnentic(&devI2c, I2C_MAGN_SENSOR_ADDR);

    float   tiltRead[3]     = {0, 0, 0};
    int     tiltValOld[3]   = {0, 0, 0};
    int     tiltValNew[3]   = {0, 0, 0};
    int     tiltValSend[3]  = {0, 0, 0};

    int     envValOld[3]    = {0, 0, 0};
    int     envValNew[3]    = {0, 0, 0};
    int     envValSend[3]   = {0, 0, 0};

    int     lightValOld[1]  = {0};
    int     lightValNew[1]  = {0};
    int     lightValSend[1] = {0};

    int     distValOld[1]   = {0};
    int     distValNew[1]   = {0};
    int     distValSend[1]  = {0};
    int     distWaitTotal   = 0;

    int     magValOld[3]    = {0, 0, 0};
    int16_t magVal[3]       = {0, 0, 0};
    int     magValNew[3]    = {0, 0, 0};
    int     magValSend[3]   = {0, 0, 0};

    bool    isSendUpdateTilt    = false;
    bool    isSendUpdateEnv     = true;
    bool    isSendUpdateLight   = true;
    bool    isSendUpdateDist    = true;
    bool    isSendUpdateMag     = true;

    do {
        ThisThread::sleep_for(2000);
        blink_led(3);

        volatile unsigned int reg;
        volatile unsigned int *regPtr = &reg;


        unsigned battery = batteryMon.read_u16();
        unsigned flex = flexAnalog.read_u16();

        LOG_HI("Battery normalized: 0x%04X \n", battery);
        *regPtr = battery;

        LOG_HI("Flex normalized: 0x%04X \n", flex);
        *regPtr = flex;

    } while(0);

    do
    {

        tiltId = sensorTilt.read_id();
        if (I_AM_LIS3DH != tiltId)
        {
            LOG_ERROR("LIS3DH ID mismatch!... Expected = 0x%02X, Actual = 0x%02X", I_AM_LIS3DH, tiltId);
        }
        else
        {
            LOG_HI("LIS3DH ID = %x", tiltId);
        }

        blink_led(2);
    } while(0);

    // Tilt Sensor LIS3DH
    if (0 != sensorTilt.data_ready())
    {
        sensorTilt.read_data(tiltRead);
        tiltValOld[TILT_IDX_X]  = (int) tiltRead[TILT_IDX_X];
        tiltValOld[TILT_IDX_Y]  = (int) tiltRead[TILT_IDX_Y];
        tiltValOld[TILT_IDX_Z]  = (int) tiltRead[TILT_IDX_Z];
        LOG_WARN("Tilt REFERENCE X, Y, Z = %d, %d, %d", tiltValOld[TILT_IDX_X], tiltValOld[TILT_IDX_Y], tiltValOld[TILT_IDX_Z]);
    }

    // Distance sensor init
    xshut = 1;
    ThisThread::sleep_for(2);    // 1.2 ms sensor boot (Fig 7 in data sheet)

    sensorDist.setDistanceMode(0);
    ThisThread::sleep_for(100);

    // Magnetometer

    // Initialize the CHIP
    sensorMagnentic.init(NULL);

    //Test the Chip ID. Should return 64 (0x40)
    unsigned int ret;
    uint8_t id;
    ret = sensorMagnentic.read_id(&id);
    if (0 != ret)
    {
        LOG_WARN("Magnetometer failed to read");
    }

    if (0x40 != id)
    {
        LOG_WARN("Magnetometer BAD ID is read, should be 0x40 but read 0x%x", id);
    }
    else
    {
        LOG_HI("Magn ID = 0x%x (VALID)", id);
    }

    // enable it
    sensorMagnentic.enable();

    while (true)
    {
        /* No wait time needed here as reading the sensors implies a total 100 ms delay! */
        totalWaitTime  += SENSOR_TIME_RESOLUTION;

        blink_led(2);

        // Tilt Sensor LIS3DH
        if (0 != sensorTilt.data_ready())
        {
            sensorTilt.read_data(tiltRead);
            tiltValNew[TILT_IDX_X]  = (int) tiltRead[TILT_IDX_X];
            tiltValNew[TILT_IDX_Y]  = (int) tiltRead[TILT_IDX_Y];
            tiltValNew[TILT_IDX_Z]  = (int) tiltRead[TILT_IDX_Z];
            LOG_HI("Tilt NEW X, Y, Z = %d, %d, %d", tiltValNew[TILT_IDX_X], tiltValNew[TILT_IDX_Y], tiltValNew[TILT_IDX_Z]);

            update_sensor_params(SENSOR_TILT_LIS3DH,
                                 tiltValNew,
                                 tiltValOld,
                                 tiltValSend,
                                 &isSendUpdateTilt);
        }
        else
        {
            LOG_WARN("LIS3DH not ready");
        }

        // Environment Sensor BME280
        envValNew[ENV_IDX_TEMPERATURE]  = (int) sensorEnv.getTemperature();
        envValNew[ENV_IDX_PRESSURE]     = (int) sensorEnv.getPressure();
        envValNew[ENV_IDX_HUMIDITY]     = (int) sensorEnv.getHumidity();
        LOG_HI("Temperature = %d, Pressure = %d, Humidity = %d", envValNew[ENV_IDX_TEMPERATURE], envValNew[ENV_IDX_PRESSURE], envValNew[ENV_IDX_HUMIDITY]);

        update_sensor_params(SENSOR_ENVIRO_BME280,
                             envValNew,
                             envValOld,
                             envValSend,
                             &isSendUpdateEnv);

        lightValNew[0]  = sensorLight.readSensor();
        LOG_HI("Light = %d", lightValNew[0]);

        update_sensor_params(SENSOR_LIGHT_OPT3001,
                             lightValNew,
                             lightValOld,
                             lightValSend,
                             &isSendUpdateLight);

        // Distance sensor
        sensorDist.startMeasurement();
        ThisThread::sleep_for(DIST_SENSOR_WAIT_STEP);

        distWaitTotal   = 0;
        while (distWaitTotal <= DIST_SENSOR_WAIT_MAX &&
               false == sensorDist.newDataReady())
        {
            LOG_HI("Waiting Distance sensor, total wait = %d, max wait = %d", distWaitTotal, DIST_SENSOR_WAIT_MAX);
            ThisThread::sleep_for(DIST_SENSOR_WAIT_STEP);
            distWaitTotal += 100;
        }

        if (distWaitTotal > DIST_SENSOR_WAIT_MAX)
        {
            LOG_WARN("Waiting Distance sensor reading timed out");
            isSendUpdateDist = false;
        }
        else
        {
            distValNew[0] = (int) (sensorDist.getDistance() / 10);
//            distValNew[0] = sensorDist.getDistance();

            LOG_HI("Distance = %d", distValNew[0]);

            update_sensor_params(SENSOR_DIST_VL53L1X,
                                 distValNew,
                                 distValOld,
                                 distValSend,
                                 &isSendUpdateDist);
        }

        // Magnetometer
        sensorMagnentic.get_m_axes_raw(magVal);
        magValNew[0] = magVal[0];
        magValNew[1] = magVal[1];
        magValNew[2] = magVal[2];
        LOG_HI("magX = %d, magY = %d, magZ = %d", magValNew[0], magValNew[1], magValNew[2]);
        update_sensor_params(SENSOR_MAGNT_LIS2MDL,
                             magValNew,
                             magValOld,
                             magValSend,
                             &isSendUpdateMag);

#if defined(LIVE_NETWORK)
        if (totalWaitTime > DWEET_UPDATE_MS)
        {
            static char sensors_key_values[MSG_LEN - 100];
            int bytes_written = 0;

            /* **** ATMOSPHERIC SENSOR **** */
            if (isSendUpdateEnv)
            {
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_TEMPERATURE_OUT), envValSend[ENV_IDX_TEMPERATURE]);
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_PRESSURE_OUT), envValSend[ENV_IDX_PRESSURE]);
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_HUMIDITY_OUT), envValSend[ENV_IDX_HUMIDITY]);

                UPDATE_SENSOR_READS(envValOld, envValSend);
                isSendUpdateEnv = false;
            }

            /* **** ACCELERMETER SENSOR **** */
            if (isSendUpdateTilt)
            {
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_ORIENTATION_X_OUT), tiltValSend[TILT_IDX_X]);
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_ORIENTATION_Y_OUT), tiltValSend[TILT_IDX_Y]);
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_ORIENTATION_Z_OUT), tiltValSend[TILT_IDX_Z]);

                UPDATE_SENSOR_READS(tiltValOld, tiltValSend);
                isSendUpdateTilt    = false;
            }


            /* **** LIGHT SENSOR **** */
            if (isSendUpdateLight)
            {
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_LIGHT_OUT), lightValSend[0]);

                lightValOld[0]      = lightValSend[0];
                isSendUpdateLight   = false;
            }

            /* **** DISTANCE SENSOR **** */
            if (isSendUpdateDist)
            {
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_DIST_OUT), distValSend[0]);

                distValOld[0]       = distValSend[0];
                isSendUpdateDist    = false;
            }

            if (isSendUpdateMag)
            {
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_MAG_X_OUT), magValSend[0]);
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_MAG_Y_OUT), magValSend[1]);
                bytes_written += sprintf(sensors_key_values + bytes_written, "%s=%d&", manhole_channel_enum(MANHOLE_CHN_MAG_Z_OUT), magValSend[2]);

                UPDATE_SENSOR_READS(magValOld, magValSend);
                isSendUpdateMag     = false;
            }

            if (bytes_written)
            {
                sensors_key_values[bytes_written-1] = '\0';
                MBED_ASSERT(bytes_written <= (MSG_LEN - 100));

                if (0 == sendSensorReadings(sensors_key_values))
                {
                    LOG_HI("[ [[ [[[ [[[[  All sensors readings sent successfully (len=%d) ]]]] ]]] ]] ]", bytes_written);
                }
                else
                {
                    LOG_WARN("Sending sensors readings failed");
                }
            }
            totalWaitTime   = 0;
        }
#else
        ThisThread::sleep_for(1000);
#endif // #if defined(LIVE_NETWORK)
    }

    return;
}

/* --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- */

#elif (MBED_APP_CONF_TEST_TYPE == DEMO_DWEET_SIGNAL)
void demo_loop(void)
{
    int i       = 0;
    int signal;
    int success = 0;
    int fail    = 0;

    int consecutiveFail = 0;

    while(true)
    {
        ThisThread::sleep_for(1000);
        signal  = (i % 2) ? i : 0;
        if ( 0 != send_dweet_signal("Signal", signal) )
        {
            blink_led(4);
            LOG_WARN("DWEET signal failed");
            fail++;
            consecutiveFail++;

            if (3 <= consecutiveFail)
            {
                LOG_ERROR("A lot of consecutive errors");
                SYSTEM_RECOVERY();
            }
        }
        else
        {
            blink_led(1);
            success++;
            consecutiveFail = 0;
        }
        i++;
        LOG_HI("[[[[ [[[ [[ [ %d Success / %d Failure ] ]] ]]] ]]]]", success, fail);
    }
}

/* --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- */

#elif (MBED_APP_CONF_TEST_TYPE == DEMO_NONE)
void demo_loop(void)
{
    while (true)
    {
        ThisThread::sleep_for(2000);
        LOG_HI("Idle APP...");
        blink_led(2);
    }
}
#endif

/*****************************************************************************************************************************************************
 *
 * G L O B A L   F U N C T I O N   D E F I N I T I O N S
 *
 ****************************************************************************************************************************************************/

int main()
{
#if MBED_CONF_MBED_TRACE_ENABLE
    trace_open();
#endif // #if MBED_CONF_MBED_TRACE_ENABLE

    LOG_HI("RM7100 Demo\n");
    LOG_HI("Built: %s, %s\n", __DATE__, __TIME__);
	
    ThisThread::sleep_for(1000);
    blink_led(1);

    xshut = 0;      // needs to be low when we power on 2V9

    modemPowerEn[0] = 1;
    modemPowerEn[1] = 1;
    modemPowerEn[2] = 1;

    batteryMonEn = 1;

    /* Get Modem out of reset */
    modem_chen  = 0;
    modem_remap = 0;
    modem_reset = 0;
    ThisThread::sleep_for(100);
    modem_reset = 1;

    do {
        ThisThread::sleep_for(2000);
        blink_led(3);
    } while(0);

#if (MBED_APP_CONF_TEST_TYPE != DEMO_NONE)
#if defined(LIVE_NETWORK)
#ifdef MBED_CONF_NSAPI_DEFAULT_CELLULAR_PLMN
    LOG_HI("[MAIN], plmn: %s\n", (MBED_CONF_NSAPI_DEFAULT_CELLULAR_PLMN ? MBED_CONF_NSAPI_DEFAULT_CELLULAR_PLMN : "NULL"));
#endif
    LOG_HI("Establishing connection\n");

    interface = CellularContext::get_default_instance();

    MBED_ASSERT(interface);

    /* Attach a status change callback */
    interface->attach(&status_callback);

    // sim pin, apn, credentials and possible plmn are taken automatically from json when using NetworkInterface::set_default_parameters()
    interface->set_default_parameters();

    /* Attempt to connect to a cellular network */
    while (do_connect() != NSAPI_ERROR_OK) {
        LOG_WARN("Could not connect to cellular network .. try again\n");
    }
#endif /*#if defined(LIVE_NETWORK)*/
#endif

    demo_loop();

#if MBED_CONF_MBED_TRACE_ENABLE
    trace_close();
#endif // #if MBED_CONF_MBED_TRACE_ENABLE
    return 0;
}


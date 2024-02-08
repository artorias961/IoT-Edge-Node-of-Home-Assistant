/*
 * Link for X-Nucleo-iks01a3: https://www.st.com/resource/en/user_manual/um2559-getting-started-with-the-xnucleoiks01a3-motion-mems-and-environmental-sensor-expansion-board-for-stm32-nucleo-stmicroelectronics.pdf
 * Link for Nucleo Datasheet: https://www.st.com/resource/en/data_brief/x-nucleo-iks01a3.pdf 
 * Link for Dev Academy: https://academy.nordicsemi.com/courses/nrf-connect-sdk-fundamentals/
 * Link for Nucleo Datasheet: https://www.st.com/resource/en/user_manual/um2559-getting-started-with-the-xnucleoiks01a3-motion-mems-and-environmental-sensor-expansion-board-for-stm32-nucleo-stmicroelectronics.pdf 
 * Link for Zephry Logging: https://docs.zephyrproject.org/3.1.0/services/logging/index.html
 * 
 * HINT:
 * WHEN ADDING A NEW SENSOR TO THE OVERLAY FILE, I NEED TO REBUILD THE ENTIRE PROGRAM
*/
// Installing libraries for zephry
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

// Installing libraries for bluetooth low energy
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>

// Installing libraries for mathematical and arduino
#include <math.h>

// Defining the LED's (nRF52-dk)
#define LED_ONE 17 
#define LED_TWO 18
#define LED_THREE 19
#define LED_FOUR 20
#define NUMBER_LEDS 4

// Defining the buttons (nRF52-dk)
#define BUTTON_ONE 13
#define BUTTON_TWO 14
#define BUTTON_THREE 15
#define BUTTON_FOUR 16

// Global variables (calling the leds using an array) (BUTTON 1 is "ENTER")
led_array[4] = {LED_ONE, LED_TWO, LED_THREE, LED_FOUR}; 
button_array[4] = {BUTTON_ONE, BUTTON_TWO, BUTTON_THREE, BUTTON_FOUR};

// This will store all the physical attributes for the nRF52dk board
struct gpio_dt_spec* leds[NUMBER_LEDS];

// Creating the LOG Module 
LOG_MODULE_REGISTER(i2c_test, LOG_LEVEL_INF);

// Declaring global variables (can not be changed)
#define BT_UUID_SST_SVC_BYTES  BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x18282, 0x785fdeadbeef)
#define BT_UUID_SST_SVC  BT_UUID_DECLARE_128(BT_UUID_SST_SVC_BYTES)

// Declaring the functions
void sensor_stts751(void);
void sensor_hts221(void);
void sensor_lis2mdl(void);
void sensor_lis2dw12(void);
void sensor_lps22hh(void);
void sensor_lsm6dso(void);
void on_connected(struct bt_conn *conn, uint8_t error);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
int my_sst_svc_gatt_notify(uint32_t sensor_value);
int api_ble_gatt_notify(uint32_t sensor_value);
void ble_manager();

// Creating global struct variables to store my sensor values [typedefinition struct variable_name]
typedef struct Sensor{
    double celsius_temperature;
    double fahrenheit_temperature;
    double humdity;
    double pressure;
    double pressure_reference;
    double pressure_offset;
    double x_axis_accelerometer;
    double y_axis_accelerometer;
    double z_axis_accelerometer;
    double x_axis_magnometer;
    double y_axis_magnometer;
    double z_axis_magnometer;
    double x_axis_angular_rate;
    double y_axis_angular_rate;
    double z_axis_angular_rate;
    double x_axis_linear_acceleration; 
    double y_axis_linear_acceleration; 
    double z_axis_linear_acceleration;
    double x_axis_linear_acceleration_offset; 
    double y_axis_linear_acceleration_offset; 
    double z_axis_linear_acceleration_offset;
    double status;
}Sensor;
// Creating multiple classes for each sensor
Sensor sensor_1; // STTS751
Sensor sensor_2; // HTS221
Sensor sensor_3; // LIS2MDL
Sensor sensor_4; // LIS2DW12
Sensor sensor_5; // LPS22HH
Sensor sensor_6; // LSM6DSO

// ============================================================================================
// BLUETOOTH PROTOCOLS START
// ============================================================================================
// Creating the bluetooth parameter advertising interval
static struct bt_le_adv_param* adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONNECTABLE|BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
    800, /*Min Advertising Interval 500ms (800*0.625ms) */
    801, /*Max Advertising Interval 500.625ms (801*0.625ms)*/
    NULL); /* Set to NULL for undirected advertising*/


// Creating advertising protocols and name
static const struct bt_data adv_data[] = {
    /* STEP 3.1 - Set the flags and populate the device name in the advertising packet */
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};


// The UUID for the nRF52-dk board
static const struct bt_data scan_data[] = {
    /* STEP 3.2.2 - Include the 16-bytes (128-Bits) UUID of our service in the scan response packet */
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SST_SVC_BYTES), // was BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SST_SVC_BYTES)
};

// Begin defining BLE GATT SST (Sensor Streaming)
#define BT_UUID_SST_MYSENSOR  BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x00001526, 0x1212, 0xefde, 0x1523, 0x785fdeadbeef)) // 0x785fdeadbeef

// Declaring global variables that can be change
static bool notify_mysensor_enabled;

/// @brief Define the configuration change callback function for the MYSENSOR characteristic
static void sst_ccc_mysensor_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_mysensor_enabled = (value == BT_GATT_CCC_NOTIFY);
}


///* GETTING ERRORS OVER HERE <-------------------------------------------------------------------------------------
BT_GATT_SERVICE_DEFINE(
    sst_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_SST_SVC),
    BT_GATT_CHARACTERISTIC(
        BT_UUID_SST_MYSENSOR,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE, 
        NULL, 
        NULL,
        NULL),
    BT_GATT_CCC(
        sst_ccc_mysensor_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
        
);
//*/

/// @brief Notify clients on the sensor via GATT
/// @param sensor_value 

int my_sst_svc_gatt_notify(uint32_t sensor_value)
{
    // Creating an variable to store the error
    int error;

    // Creating a conditional statement to determine if there is an error
    if (!notify_mysensor_enabled) {
        // If so, then return the following
        return -EACCES;
    }
    // &sst_service.attrs is the Service Attribute table: 0 = Service, 1 = Primary service, 2 = Characteristic
    error = bt_gatt_notify(NULL, &sst_service.attrs[2], &sensor_value, sizeof(sensor_value));

    // An error for the Bluetooth GATT Notify service
    if (error < 0) {
        // Log the module
        LOG_ERR("Warning: bt_gatt_notify failed sending %d with error: %d", sensor_value, error);
    }
    else {
        // Else, log the sensor value
        LOG_DBG("Sent sensor value: %d", sensor_value);
    }
    return error;
}


// A global variable to declare the status of the board (Essentially our real BLE manager)
struct bt_conn* our_conn = NULL;

void on_connected(struct bt_conn *conn, uint8_t error){
    // Creating a conditional statement if an error occur with the bluetooth connection
    if (error) {
        // Logging the module
        LOG_ERR("Connection error %d", error);
        
        // Stop the program
        return;
    }
    // Logging the module
    LOG_WRN("Connected");
    
    // The connection of the bluetooth device
    our_conn = bt_conn_ref(conn);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason){
    // Logging the module
    LOG_WRN("Disconnected. Reason %d", reason);
    
    // Disconnection of the bluetooth device
    bt_conn_unref(our_conn);
}

struct bt_conn_cb bluetooth_connection_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

// ============================================================================================
// BLUETOOTH PROTOCOLS END
// ============================================================================================

void sensor_stts751(void){ 
    // Creating the size of the i2c address (buffer)
    uint8_t i2c_buffer_write[2], i2c_buffer_read[2];

    // Creating the size of the variable to store the information
    uint8_t temp_reading[2] = {0};

    // Creating a variable that will be able to detect an error
    int error;

    // Logging the module
    LOG_DBG("Enter TEMP I2C!!! Setting up I2C");

    // Getting the sensor information from the humidity/temp i2c
    const struct i2c_dt_spec stss751_dt_spec = I2C_DT_SPEC_GET(DT_NODELABEL(my_stts751));

    // Creating a conditional statement if an error occur
    if (!device_is_ready(stss751_dt_spec.bus)){
        // Log the error
        LOG_ERR("I2C initialization failed for some reason !!! (TEMP I2C)");

        // Stop the program
        return;
    }
    // Lets wait for a second
    k_sleep(K_SECONDS(1));

    // Logging the module for the STTS751
    LOG_DBG("Checking TEMPERATURE I2C write/read (STTS751) . . . ");
    {
        // Creating the buffer for writing (00, temperature value high byte) <----------------------
        i2c_buffer_write[0] = 0x00;

        // WRITING the data from the I2C (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&stss751_dt_spec, &i2c_buffer_write[0], 1, &temp_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("An error occur with the STTS751 Temperature Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: STTS751 Temperature Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing (02, temperature value low byte) <----------------------
        i2c_buffer_write[1] = 0x02;

        // READING the data from the I2C ()
        error = i2c_write_read_dt(&stss751_dt_spec, &i2c_buffer_read[1], 1, &temp_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("An error occur with STTS751 Temperature Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: STTS751 Temperature Sensor (READING I2C) !!!");
    }

    // Converting the data to temperature
    {
        // Logging the module
        LOG_DBG("Now going to convert STTS751 data to degrees (f and c)");

        // Converting the data to temperature STTS751
        int temp = ((int)temp_reading[1] * 256 + ((int)temp_reading[0] & 0xF0)) / 16;

        // Creating a condition to determine of the data is out of bounds (temperature)
        if (temp > 2047){
            // Lower the value
            temp -= 4096;
        }

        // Converting the data address to C and F
        double celsius_temp = temp * 0.0625;
        double fahrenheit_temp = celsius_temp * 1.8 + 32;

        // Logging the module
        LOG_WRN("===================================================================");
        LOG_INF("(STTS751) The celsius temperature is %.2f", celsius_temp);
        LOG_INF("(STTS751) The fahrenheit temperature is %.2f", fahrenheit_temp);
        LOG_WRN("===================================================================");

        // Storing the information to a global struct
        sensor_1.celsius_temperature = celsius_temp;
        sensor_1.fahrenheit_temperature = fahrenheit_temp;
    }
}

void sensor_hts221(void){
    // Creating the size of the i2c address (buffer)
    uint8_t i2c_buffer_write[4], i2c_buffer_read[4];

    // Creating the size of the variable to store the information
    uint8_t temp_reading_temperature[2] = {0};
    uint8_t temp_reading_humidity[2] = {0};

    // Creating a variable that will be able to detect an error
    int error;

    // Logging the information
    LOG_DBG("Board is up!!! Setting up I2C for HTS221");

    // Getting the sensor information from the OVERLAY file (nrf52dk_nrf52832.overlay)
    const struct i2c_dt_spec hts221_dt_spec = I2C_DT_SPEC_GET(DT_NODELABEL(my_hts221));

    // Creating a conditional statement if the I2C connections fail
    if (!device_is_ready(hts221_dt_spec.bus)){
        // Log an error
        LOG_ERR("I2C initialization failed for some reason!!!");

        // Stop the application
        return;
    }

    // ============================================================================================
    // TEMPERATURE PORTION OF THE CODE
    // ============================================================================================

    // Logging the module for the Temperature HTS221 
    LOG_DBG("Checking TEMPERATURE I2C write/read . . . ");
    {
        // Creating the buffer for writing (TEMP_OUT_L) [LSB]
        i2c_buffer_write[0] = 0x2a;

        // WRITING the data from the I2C
        error = i2c_write_read_dt(&hts221_dt_spec, &i2c_buffer_read[0], 1, &temp_reading_temperature[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("An error occur with the HTS221 Temperature Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_INF("SUCCESS: HTS221 Temperature Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing (TEMP_OUT_H) [MSB]
        i2c_buffer_write[1] = 0x2b;

        // READING the data from the I2C
        error = i2c_write_read_dt(&hts221_dt_spec, &i2c_buffer_read[1], 1, &temp_reading_temperature[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("An error occur with HTS221 Temperature Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: HTS221 Temperature Sensor (READING I2C) !!!");        
    }
    
    // ============================================================================================
    // HUMIDITY PORTION OF THE CODE
    // ============================================================================================

     // Logging the module for the Humidity HTS221 
    LOG_DBG("Checking HUMIDITY I2C write/read . . . ");
    {
        // Creating the buffer for writing (HUMIDITY_OUT_L) [LSB]
        i2c_buffer_write[2] = 0x28;

        // WRITING the data from the I2C
        error = i2c_write_read_dt(&hts221_dt_spec, &i2c_buffer_read[2], 1, &temp_reading_humidity[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("An error occur with the HTS221 Humidity Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: HTS221 Humidity Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing (HUMIDITY_OUT_H) [MSB]
        i2c_buffer_write[3] = 0x29;

        // READING the data from the I2C
        error = i2c_write_read_dt(&hts221_dt_spec, &i2c_buffer_read[3], 1, &temp_reading_humidity[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("An error occur with HTS221 Humidity Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_DBG("SUCCESS: HTS221 Humidity Sensor (READING I2C) !!!");        
    }
    
    // ============================================================================================
    // THE OUTPUT OF THE CODE
    // ============================================================================================

     // Converting the data to temperature and humidity
    {
        // Logging the module
        LOG_DBG("Now going to convert HTS221 data to degrees (f and c)");

        // Creating a variable for singed 16 bit range value (we need this since this is signed bit) [2^16 = 65536]
        int bit_range = 65536;

        // Converting the temperature data using HTS221 raw_data = (MSB[15:8] | (LSB[7:0] << 8) )
        // Since LSB is not in the correct register we need to shift the bits 8 times to the left to compare the data with MSB
        // And according to the datasheet, 2^16 is the total number of possible values for 16bit raw data,
        // so we are going to do next is:
        // temp = (raw_data / 2^16) * max_temp_for_sensor - lowest_temp_for_sensor
        double celsius_temp = ((int)temp_reading_temperature[1] | ((int)temp_reading_temperature[0] << 8)/ bit_range) * 120 - 40; 
        double fahrenheit_temp = celsius_temp * 1.8 + 32;

        // Converting the humidity data using (HTS221), essentially the same progress as the temperature
        double humidity = (int)temp_reading_humidity[1] | ((int)temp_reading_humidity[0] << 8) * bit_range;
        double humidity_percentage = humidity * 100;

        // Logging the module
        LOG_WRN("===================================================================");
        LOG_INF("(HTS221) The celsius temperature is %.2f", celsius_temp);
        LOG_INF("(HTS221) The fahrenheit temperature is %.2f", fahrenheit_temp);
        LOG_INF("(HTS221) The humidity percentage is %.2f", humidity_percentage);
        LOG_WRN("===================================================================");

        // Storing the information to a global struct
        sensor_2.celsius_temperature = celsius_temp;
        sensor_2.fahrenheit_temperature = fahrenheit_temp;
        sensor_2.humdity = humidity_percentage;
    } 
}

void sensor_lis2mdl(void){
    // https://www.st.com/resource/en/datasheet/lis2mdl.pdf
    // Creating empty variables for the each axis for the 3D MEM magnetometer (getting the raw data)
    double x_axis_magnetometer, y_axis_magnetometer, z_axis_magnetometer;

    // Creating a empty variable to store Configuration Register Mode
    double reg_a, reg_b, reg_c;

    // Other necessary information we are going to use for the raw data (READ THE DATASHEET)
    double x_offset, y_offset, z_offset, x_scale, y_scale, z_scale;

    // Other addresses from the 3D MEM magnetometer [for status YOU NEED THE DATASHEET TO INTERPRET]
    double status, temperature_magnetometer, nominal_sensitivity;

    // From the datasheet, here is the sensitivity for the raw data will be needed [LSB]
    double sensitivity = 1.50;

    // Creating the size of the i2c address (buffer)
    uint8_t i2c_buffer_write[2], i2c_buffer_read[2];

    // Creating the size of the variable to store the information
    uint8_t mems_magnetometer_reading[2] = {0};

    // Creating a variable that will be able to detect an error
    int error;

    // Logging the module
    LOG_DBG("Enter MEMS 3D magnetometer I2C!!! Setting up I2C");

    // Getting the sensor information from the MEMS 3D magnetometer i2c
    const struct i2c_dt_spec lis2mdl_dt_spec = I2C_DT_SPEC_GET(DT_NODELABEL(my_lis2mdl));

    // Creating a conditional statement if an error occur
    if (!device_is_ready(lis2mdl_dt_spec.bus)){
        // Log the error
        LOG_ERR("I2C initialization failed for some reason !!! (MEMS 3D magnetometer I2C)");

        // Stop the program
        return;
    }
    // Lets wait for a second
    k_sleep(K_SECONDS(1));

    // ============================================================================================
    // CFG_REG_A for the 3D MEM MAGNETOMETER (TURNS ON ALL MAGNOMETER AXIS)
    // ============================================================================================

    // Logging the module for the LIS2MDL (CFG_REG_A of the sensor)
    LOG_DBG("Checking CFG_REG_A [MSB and LSB] for the 3D MEMS MAGNETOMETER I2C write/read (LIS2MDL) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x67; //0x60;

        // WRITING the data from the I2C [CFG_REG_A MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_write[0], 1, &mems_magnetometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[CFG_REG_A] An error occur with the LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [CFG_REG_A] LIS2MDL MEMS 3D magnetometer Sensor (READING AND WRITING I2C) !!!");


        // Understanding the raw data from CFG_REG_A
        {
            // Turning on the board
            reg_a = mems_magnetometer_reading[0];
        }
    }

    // // ============================================================================================
    // // CFG_REG_B for the 3D MEM MAGNETOMETER (TURNS ON TEMPERATURE)
    // // ============================================================================================

    // // Logging the module for the LIS2MDL (CFG_REG_B of the sensor)
    // LOG_DBG("Checking CFG_REG_B [MSB and LSB] for the 3D MEMS MAGNETOMETER I2C write/read (LIS2MDL) . . . ");
    // {
    //     // Creating the buffer for writing [MSB] 2's complement address <----------------------
    //     i2c_buffer_write[0] = 0x61;

    //     // WRITING the data from the I2C [CFG_REG_B MSB] (sensor data, buffer address for write, read, 1)
    //     error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_write[0], 1, &mems_magnetometer_reading[0], 1);

    //     // Creating a condition to see if the data the wrong data
    //     if (error < 0){
    //         // Logging the module
    //         LOG_ERR("[CFG_REG_B] An error occur with the LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) . . . ");

    //         // Stop the application
    //         return;
    //     }
    //     // Creating a 1 second delay
    //     k_sleep(K_SECONDS(1));

    //     // Logging the module
    //     LOG_DBG("SUCCESS: [CFG_REG_B] LIS2MDL MEMS 3D magnetometer Sensor (READING AND WRITING I2C) !!!");


    //     // Understanding the raw data from CFG_REG_B
    //     {
    //         // Turning on the board
    //         reg_b = mems_magnetometer_reading[0];
    //     }
    // }

    // // ============================================================================================
    // // CFG_REG_C for the 3D MEM MAGNETOMETER (TURNS ON TEMPERATURE)
    // // ============================================================================================

    // // Logging the module for the LIS2MDL (CFG_REG_C of the sensor)
    // LOG_DBG("Checking CFG_REG_C [MSB and LSB] for the 3D MEMS MAGNETOMETER I2C write/read (LIS2MDL) . . . ");
    // {
    //     // Creating the buffer for writing [MSB] 2's complement address <----------------------
    //     i2c_buffer_write[0] = 0x61;

    //     // WRITING the data from the I2C [CFG_REG_C MSB] (sensor data, buffer address for write, read, 1)
    //     error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_write[0], 1, &mems_magnetometer_reading[0], 1);

    //     // Creating a condition to see if the data the wrong data
    //     if (error < 0){
    //         // Logging the module
    //         LOG_ERR("[CFG_REG_C] An error occur with the LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) . . . ");

    //         // Stop the application
    //         return;
    //     }
    //     // Creating a 1 second delay
    //     k_sleep(K_SECONDS(1));

    //     // Logging the module
    //     LOG_DBG("SUCCESS: [CFG_REG_C] LIS2MDL MEMS 3D magnetometer Sensor (READING AND WRITING I2C) !!!");


    //     // Understanding the raw data from CFG_REG_C
    //     {
    //         // Turning on the board
    //         reg_c = mems_magnetometer_reading[0];
    //     }
    // }

    // ============================================================================================
    // X AXIS for the 3D MEM MAGNETOMETER
    // ============================================================================================

    // Logging the module for the LIS2MDL (X AXIS MSB AND LSB) [Going to use the offset address that is 2's complement]
    LOG_DBG("Checking X AXIS [MSB and LSB] for the 3D MEMS MAGNETOMETER I2C write/read (LIS2MDL) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x69;

        // WRITING the data from the I2C [X AXIS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_write[0], 1, &mems_magnetometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[X AXIS] An error occur with the LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [X AXIS] LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x68;

        // READING the data from the I2C (X AXIS LSB)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_read[1], 1, &mems_magnetometer_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[X AXIS] An error occur with LIS2MDL MEMS 3D magnetometer Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [X AXIS] LIS2MDL MEMS 3D magnetometer Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_magnetometer_reading[0] | (mems_magnetometer_reading[1]) * 8;

            // The code may not be done but convert it to float
            x_axis_magnetometer = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Y AXIS for the 3D MEM MAGNETOMETER
    // ============================================================================================

    // Logging the module for the LIS2MDL (Y AXIS MSB AND LSB)
    LOG_DBG("Checking Y AXIS [MSB and LSB] for the 3D MEMS MAGNETOMETER I2C write/read (LIS2MDL) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x6b;

        // WRITING the data from the I2C [Y AXIS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_write[0], 1, &mems_magnetometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Y AXIS] An error occur with the LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Y AXIS] LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x6a;

        // READING the data from the I2C (Y AXIS LSB)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_read[1], 1, &mems_magnetometer_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[Y AXIS] An error occur with LIS2MDL MEMS 3D magnetometer Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [Y AXIS] LIS2MDL MEMS 3D magnetometer Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_magnetometer_reading[0] | (mems_magnetometer_reading[1] << 8);

            // The code may not be done but convert it to float
            y_axis_magnetometer = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Z AXIS for the 3D MEM MAGNETOMETER
    // ============================================================================================

    // Logging the module for the LIS2MDL (Z AXIS MSB AND LSB)
    LOG_DBG("Checking Y AXIS [MSB and LSB] for the 3D MEMS MAGNETOMETER I2C write/read (LIS2MDL) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x6d;

        // WRITING the data from the I2C [Z AXIS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_write[0], 1, &mems_magnetometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Z AXIS] An error occur with the LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Z AXIS] LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x6c;

        // READING the data from the I2C (Z AXIS LSB)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_read[1], 1, &mems_magnetometer_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[Z AXIS] An error occur with LIS2MDL MEMS 3D magnetometer Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [Z AXIS] LIS2MDL MEMS 3D magnetometer Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_magnetometer_reading[0] | (mems_magnetometer_reading[1] << 8);

            // The code may not be done but convert it to float
            z_axis_magnetometer = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // TEMPERATURE from the 3D MEM MAGNETOMETER sensor
    // ============================================================================================

    // Logging the module for the LIS2MDL (TEMPERATURE MSB AND LSB)
    LOG_DBG("Checking X AXIS [MSB and LSB] for the 3D MEMS MAGNETOMETER I2C write/read (LIS2MDL) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x6F;

        // WRITING the data from the I2C [Z AXIS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_write[0], 1, &mems_magnetometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[TEMPERATURE] An error occur with the LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [TEMPERATURE] LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x6E;

        // READING the data from the I2C (Z AXIS LSB)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_read[1], 1, &mems_magnetometer_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[TEMPERATURE] An error occur with LIS2MDL MEMS 3D magnetometer Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [TEMPERATURE] LIS2MDL MEMS 3D magnetometer Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using 
            // According to the datasheet it is (nominal sensitivity is 8 LSB/Celsius)
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            nominal_sensitivity = 8.0;
            int16_t converted_raw_data = (mems_magnetometer_reading[0] | (mems_magnetometer_reading[1] << 8)) / nominal_sensitivity;

            // The code may not be done but convert it to float (celsius)
            temperature_magnetometer = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Checking the STATUS of the 3D MEM MAGNETOMETER
    // ============================================================================================

    // Logging the module for the LIS2MDL (STATUS of the sensor)
    LOG_DBG("Checking X AXIS [MSB and LSB] for the 3D MEMS MAGNETOMETER I2C write/read (LIS2MDL) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x67;

        // WRITING the data from the I2C [Z AXIS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2mdl_dt_spec, &i2c_buffer_write[0], 1, &mems_magnetometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[STATUS] An error occur with the LIS2MDL MEMS 3D magnetometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [STATUS] LIS2MDL MEMS 3D magnetometer Sensor (READING AND WRITING I2C) !!!");


        // Understanding the raw data from STATUS
        {
            // It should be noted that the address will have to be raw data
            // The default values is set to zero if the value is one then a new set of data has been written
            // The status register is only one byte and each bit are the following:
            // Zyxor, zor, yor, xor, Zyxda, zda, yda, xda
            // The code may not be done but convert it to float
            status = mems_magnetometer_reading[0];
        }
    }

    // Creating the calibration for the MEM 3D Magnometer
    {
        //
        // cosine = tan(x_axis_magnetometer/y_axis_magnetometer);

        // 
    }

    // Printing all the information to the module
    {
        // Logging the module
        LOG_WRN("===================================================================");
        LOG_INF("(LIS2MDL) The X axis 3D MEM magnetometer is %.2f", x_axis_magnetometer);
        LOG_INF("(LIS2MDL) The Y axis 3D MEM magnetometer is %.2f", y_axis_magnetometer);
        LOG_INF("(LIS2MDL) The Z axis 3D MEM magnetometer is %.2f", z_axis_magnetometer);
        LOG_INF("(LIS2MDL) The STATUS of the 3D MEM magnetometer is %.8f", status);
        LOG_INF("(LIS2MDL) The TEMPERATURE sensor from the 3D MEM magnotometer is %.2f", temperature_magnetometer);
        LOG_WRN("===================================================================");

        // Storing the data to the struct
        sensor_3.x_axis_magnometer = x_axis_magnetometer;
        sensor_3.y_axis_magnometer = y_axis_magnetometer;
        sensor_3.y_axis_magnometer = z_axis_magnetometer;
        sensor_3.status = status;
        sensor_3.celsius_temperature = temperature_magnetometer;
    }
}

void sensor_lis2dw12(void){
    // https://www.st.com/resource/en/datasheet/lis2dw12.pdf
    // https://www.st.com/resource/en/application_note/an5038-lis2dw12-alwayson-3d-accelerometer-stmicroelectronics.pdf
    // Creating empty variables for the each axis for the 3D MEM accelerometer (getting the raw data)
    double x_axis_accelerometer, y_axis_accelerometer, z_axis_accelerometer;

    // Other necessary information we are going to use for the raw data (READ THE DATASHEET)
    // double x_offset, y_offset, z_offset, x_scale, y_scale, z_scale; 

    // Other addresses from the 3D MEM accelerometer [for status YOU NEED THE DATASHEET TO INTERPRET]
    double status, temperature_accelerometer, nominal_sensitivity;

    // From the datasheet, here is the sensitivity for the raw data will be needed [LSB]
    double sensitivity = 1.50;

    // Creating the size of the i2c address (buffer)
    uint8_t i2c_buffer_write[2], i2c_buffer_read[2];

    // Creating the size of the variable to store the information
    uint8_t mems_accelerometer_reading[2] = {0};

    // Creating a variable that will be able to detect an error
    int error;

    // Logging the module
    LOG_DBG("Enter MEMS 3D accelerometer I2C!!! Setting up I2C");

    // Getting the sensor information from the MEMS 3D accelerometer i2c
    const struct i2c_dt_spec lis2dw12_dt_spec = I2C_DT_SPEC_GET(DT_NODELABEL(my_lis2dw12));

    // Creating a conditional statement if an error occur
    if (!device_is_ready(lis2dw12_dt_spec.bus)){
        // Log the error
        LOG_ERR("I2C initialization failed for some reason !!! (MEMS 3D accelerometer I2C)");

        // Stop the program
        return;
    }
    // Lets wait for a second
    k_sleep(K_SECONDS(1));

    // ============================================================================================
    // X AXIS for the 3D MEM ACCELEROMETER 
    // ============================================================================================

    // Logging the module for the LIS2DW12 (X AXIS MSB AND LSB) [Going to use the offset address that is 2's complement]
    LOG_DBG("Checking X AXIS [MSB and LSB] for the 3D MEMS accelerometer I2C write/read (LIS2DW12) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x29;

        // WRITING the data from the I2C [X AXIS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_write[0], 1, &mems_accelerometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[X AXIS] An error occur with the LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [X AXIS] LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x28;

        // READING the data from the I2C (X AXIS LSB)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_read[1], 1, &mems_accelerometer_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[X AXIS] An error occur with LIS2DW12 MEMS 3D accelerometer Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [X AXIS] LIS2DW12 MEMS 3D accelerometer Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_accelerometer_reading[0] | (mems_accelerometer_reading[1] << 8);

            // The code may not be done but convert it to float
            x_axis_accelerometer = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Y AXIS for the 3D MEM ACCELEROMETER
    // ============================================================================================

    // Logging the module for the LIS2DW12 (Y AXIS MSB AND LSB)
    LOG_DBG("Checking Y AXIS [MSB and LSB] for the 3D MEMS accelerometer I2C write/read (LIS2DW12) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x2b;

        // WRITING the data from the I2C [Y AXIS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_write[0], 1, &mems_accelerometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Y AXIS] An error occur with the LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Y AXIS] LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x2a;

        // READING the data from the I2C (Y AXIS LSB)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_read[1], 1, &mems_accelerometer_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[Y AXIS] An error occur with LIS2DW12 MEMS 3D accelerometer Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [Y AXIS] LIS2DW12 MEMS 3D accelerometer Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_accelerometer_reading[0] | (mems_accelerometer_reading[1] << 8);

            // The code may not be done but convert it to float
            y_axis_accelerometer = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Z AXIS for the 3D MEM ACCELEROMETER
    // ============================================================================================

    // Logging the module for the LIS2DW12 (Z AXIS MSB AND LSB)
    LOG_DBG("Checking Z AXIS [MSB and LSB] for the 3D MEMS accelerometer I2C write/read (LIS2DW12) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x2d;

        // WRITING the data from the I2C [Z AXIS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_write[0], 1, &mems_accelerometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Z AXIS] An error occur with the LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Z AXIS] LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x2c;

        // READING the data from the I2C (Z AXIS LSB)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_read[1], 1, &mems_accelerometer_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[Z AXIS] An error occur with LIS2DW12 MEMS 3D accelerometer Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [Z AXIS] LIS2DW12 MEMS 3D accelerometer Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_accelerometer_reading[0] | (mems_accelerometer_reading[1] << 8);

            // The code may not be done but convert it to float
            z_axis_accelerometer = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // TEMPERATURE from the 3D MEM ACCELEROMETER sensor
    // ============================================================================================

    // Logging the module for the LIS2DW12 (TEMPERATURE MSB AND LSB)
    LOG_DBG("Checking TEMPERATURE [MSB and LSB] for the 3D MEMS accelerometer I2C write/read (LIS2DW12) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x0e;

        // WRITING the data from the I2C [TEMPERATURE MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_write[0], 1, &mems_accelerometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[TEMPERATURE] An error occur with the LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [TEMPERATURE] LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x0d;

        // READING the data from the I2C (TEMPERATURE LSB)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_read[1], 1, &mems_accelerometer_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[TEMPERATURE] An error occur with LIS2DW12 MEMS 3D accelerometer Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [TEMPERATURE] LIS2DW12 MEMS 3D accelerometer Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using 
            // According to the datasheet it is (nominal sensitivity is 8 LSB/Celsius)
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            nominal_sensitivity = 16.0;
            int16_t converted_raw_data = (mems_accelerometer_reading[0] | (mems_accelerometer_reading[1] << 8)) / nominal_sensitivity;

            // The code may not be done but convert it to float (celsius)
            temperature_accelerometer = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Checking the STATUS of the 3D MEM ACCELEROMETER
    // ============================================================================================

    // Logging the module for the LIS2DW12 (STATUS of the sensor)
    LOG_DBG("Checking STATUS [MSB and LSB] for the 3D MEMS accelerometer I2C write/read (LIS2DW12) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x27;

        // WRITING the data from the I2C [STATUS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lis2dw12_dt_spec, &i2c_buffer_write[0], 1, &mems_accelerometer_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[STATUS] An error occur with the LIS2DW12 MEMS 3D accelerometer Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [STATUS] LIS2DW12 MEMS 3D accelerometer Sensor (READING AND WRITING I2C) !!!");


        // Understanding the raw data from STATUS
        {
            // It should be noted that the address will have to be raw data
            // The default values is set to zero if the value is one then a new set of data has been written
            // The status register is only one byte and each bit are the following:
            // 
            // The code may not be done but convert it to float
            status = mems_accelerometer_reading[0];
        }
    }

    // Printing all the information to the module
    {
        // Logging the module
        LOG_WRN("===================================================================");
        LOG_INF("(LIS2DW12) The X axis 3D MEM accelerometer is %.2f", x_axis_accelerometer);
        LOG_INF("(LIS2DW12) The Y axis 3D MEM accelerometer is %.2f", y_axis_accelerometer);
        LOG_INF("(LIS2DW12) The Z axis 3D MEM accelerometer is %.2f", z_axis_accelerometer);
        LOG_INF("(LIS2DW12) The STATUS of the 3D MEM accelerometer is %f", status);
        LOG_INF("(LIS2DW12) The TEMPERATURE sensor from the 3D MEM accelerometer is %.2f", temperature_accelerometer);
        LOG_WRN("===================================================================");
    }

    // Storing the information to a global struct
    sensor_4.x_axis_accelerometer = x_axis_accelerometer;
    sensor_4.y_axis_accelerometer = y_axis_accelerometer;
    sensor_4.z_axis_accelerometer = z_axis_accelerometer;
    sensor_4.status = status;
    sensor_4.celsius_temperature = temperature_accelerometer;
}

void sensor_lps22hh(void){
    // Creating pressure variable for the MEMS nano pressure sensor
    double pressure, pressure_offset, pressure_reference;

    // Other addresses from the MEM nano pressure sensor [for status YOU NEED THE DATASHEET TO INTERPRET]
    double status, temperature_pressure_sensor, nominal_sensitivity;

    // Creating the size of the i2c address (buffer)
    uint8_t i2c_buffer_write[2], i2c_buffer_read[2];

    // Creating the size of the variable to store the information
    uint8_t mems_pressure_sensor_reading[2] = {0};

    // Creating a variable that will be able to detect an error
    int error;

    // Logging the module
    LOG_DBG("Enter MEMS nano pressure sensor I2C!!! Setting up I2C");

    // Getting the sensor information from the MEMS nano pressure sensor i2c
    const struct i2c_dt_spec lps22hh_dt_spec = I2C_DT_SPEC_GET(DT_NODELABEL(my_lps22hh));

    // Creating a conditional statement if an error occur
    if (!device_is_ready(lps22hh_dt_spec.bus)){
        // Log the error
        LOG_ERR("I2C initialization failed for some reason !!! (MEMS nano pressure sensor I2C)");

        // Stop the program
        return;
    }
    // Lets wait for a second
    k_sleep(K_SECONDS(1));

    // ============================================================================================
    // PRESSURE REFERENCE for the MEM NANO PRESSURE SENSOR
    // ============================================================================================

    // Logging the module for the LPS22HH (PRESSURE REFERENCE MSB AND LSB) [Going to use the offset address that is 2's complement]
    LOG_DBG("Checking PRESSURE REFERENCE [MSB and LSB] for the MEMS nano pressure sensor I2C write/read (LPS22HH) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x16;

        // WRITING the data from the I2C [PRESSURE REFERENCE MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[PRESSURE REFERENCE] An error occur with the LPS22HH MEMS nano pressure sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [PRESSURE REFERENCE] LPS22HH MEMS nano pressure sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x15;

        // READING the data from the I2C (PRESSURE REFERENCE LSB)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[PRESSURE REFERENCE] An error occur with LPS22HH MEMS nano pressure sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [PRESSURE REFERENCE] LPS22HH MEMS nano pressure sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            pressure_reference = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // PRESSURE OFFSET for the MEM NANO PRESSURE SENSOR
    // ============================================================================================

    // Logging the module for the LPS22HH (PRESSURE OFFSET MSB AND LSB)
    LOG_DBG("Checking PRESSURE OFFSET [MSB and LSB] for the MEMS nano pressure sensor I2C write/read (LPS22HH) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x19;

        // WRITING the data from the I2C [PRESSURE OFFSET MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[PRESSURE OFFSET] An error occur with the LPS22HH MEMS nano pressure sensor  (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [PRESSURE OFFSET] LPS22HH MEMS nano pressure sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x18;

        // READING the data from the I2C (PRESSURE OFFSET LSB)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[PRESSURE OFFSET] An error occur with LPS22HH MEMS nano pressure sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [PRESSURE OFFSET] LPS22HH MEMS nano pressure sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            pressure_offset = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // PRESSURE for the MEM NANO PRESSURE SENSOR
    // ============================================================================================

    // Logging the module for the LPS22HH (PRESSURE MSB AND LSB)
    LOG_DBG("Checking PRESSURE [MSB and LSB] for the MEMS nano pressure sensor I2C write/read (LPS22HH) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x2a;

        // WRITING the data from the I2C [PRESSURE MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[PRESSURE] An error occur with the LPS22HH MEMS nano pressure sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [PRESSURE] LPS22HH MEMS nano pressure sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x29;

        // READING the data from the I2C (PRESSURE LSB)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[PRESSURE] An error occur with LPS22HH MEMS nano pressure sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [PRESSURE] LPS22HH MEMS nano pressure sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            pressure = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // TEMPERATURE from the 3D MEM NANO PRESSURE SENSOR
    // ============================================================================================

    // Logging the module for the LPS22HH (TEMPERATURE MSB AND LSB)
    LOG_DBG("Checking TEMPERATURE [MSB and LSB] for the MEMS nano pressure sensor I2C write/read (LPS22HH) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x2c;

        // WRITING the data from the I2C [TEMPERATURE MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[TEMPERATURE] An error occur with the LPS22HH MEMS nano pressure sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [TEMPERATURE] LPS22HH MEMS nano pressure sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x2b;

        // READING the data from the I2C (TEMPERATURE LSB)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[TEMPERATURE] An error occur with LPS22HH MEMS nano pressure sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [TEMPERATURE] LPS22HH MEMS nano pressure sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using 
            // According to the datasheet it is (nominal sensitivity is 8 LSB/Celsius)
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            nominal_sensitivity = 16.0;
            int16_t converted_raw_data = (mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8)) / nominal_sensitivity;

            // The code may not be done but convert it to float (celsius)
            temperature_pressure_sensor = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Checking the STATUS of the 3D MEM NANO PRESSURE SENSOR
    // ============================================================================================

    // Logging the module for the LPS22HH (STATUS of the sensor)
    LOG_DBG("Checking STATUS [MSB and LSB] for the MEMS nano pressure sensor I2C write/read (LPS22HH) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x27;

        // WRITING the data from the I2C [STATUS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lps22hh_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[STATUS] An error occur with the LPS22HH MEMS nano pressure sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [STATUS] LPS22HH MEMS nano pressure sensor (READING AND WRITING I2C) !!!");


        // Understanding the raw data from STATUS
        {
            // It should be noted that the address will have to be raw data
            // The default values is set to zero if the value is one then a new set of data has been written
            // The status register is only one byte and each bit are the following:
            // 
            // The code may not be done but convert it to float
            status = mems_pressure_sensor_reading[0];
        }
    }

    // Printing all the information to the module
    {
        // Logging the module
        LOG_WRN("===================================================================");
        LOG_INF("(LPS22HH) The PRESSURE REFERENCE is %.2f", pressure_reference);
        LOG_INF("(LPS22HH) The PRESSURE OFFSET is %.2f", pressure_offset);
        LOG_INF("(LPS22HH) The PRESSURE is %.2f", pressure);
        LOG_INF("(LPS22HH) The STATUS of the 3D MEM accelerometer is %f", status);
        LOG_INF("(LPS22HH) The TEMPERATURE sensor from the 3D MEM accelerometer is %.2f", temperature_pressure_sensor);
        LOG_WRN("===================================================================");

        // Storing the information to a global struct
        sensor_5.pressure_reference = pressure_reference;
        sensor_5.pressure_offset = pressure_offset;
        sensor_5.pressure = pressure;
        sensor_5.status = status;
        sensor_5.celsius_temperature = temperature_pressure_sensor;
    }
}

void sensor_lsm6dso(void){
    // Creating empty variable for the MEMs 3D Accelerometer + 3D Gryoscrope [ANGULAR RATE for gryoscope]
    double x_axis_angular_rate, y_axis_angular_rate, z_axis_angular_rate;

    // Creating empty variables for the MEMs 3D Accelerometer + 3D Gryoscrope [LINEAR ACCELERATION for accelerometer]
    double x_axis_linear_acceleration, y_axis_linear_acceleration, z_axis_linear_acceleration;

    // Creating empty varaibles for the MEMs 3D Accelerometer + 3D Gryoscrope [LINEAR ACCELERATION for accelerometer offset]
    double x_axis_linear_offset, y_axis_linear_offset, z_axis_linear_offset;

    // Other addresses from the MEM 3D Accelerometer + 3D Gryoscrope [for status YOU NEED THE DATASHEET TO INTERPRET]
    double status, temperature_accelerometer_gryoscrope_sensor, nominal_sensitivity;

    // Creating the size of the i2c address (buffer)
    uint8_t i2c_buffer_write[2], i2c_buffer_read[2];

    // Creating the size of the variable to store the information
    uint8_t mems_pressure_sensor_reading[2] = {0};

    // Creating a variable that will be able to detect an error
    int error;

    // Logging the module
    LOG_DBG("Enter MEMS 3D Accelerometer + 3D Gryoscrope I2C!!! Setting up I2C");

    // Getting the sensor information from the MEMS 3D Accelerometer + 3D Gryoscrope i2c
    const struct i2c_dt_spec lsm6dso_dt_spec = I2C_DT_SPEC_GET(DT_NODELABEL(my_lsm6dso));

    // Creating a conditional statement if an error occur
    if (!device_is_ready(lsm6dso_dt_spec.bus)){
        // Log the error
        LOG_ERR("I2C initialization failed for some reason !!! (MEMS 3D Accelerometer + 3D Gryoscrope I2C)");

        // Stop the program
        return;
    }
    // Lets wait for a second
    k_sleep(K_SECONDS(1));

    // ============================================================================================
    // X_AXIS_ANGULAR_RATE REFERENCE for the MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (X_AXIS_ANGULAR_RATE MSB AND LSB) [Going to use the offset address that is 2's complement]
    LOG_DBG("Checking X_AXIS_ANGULAR_RATE [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x23;

        // WRITING the data from the I2C [X_AXIS_ANGULAR_RATE MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[X_AXIS_ANGULAR_RATE] An error occur with the LPS22HH MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [X_AXIS_ANGULAR_RATE] LPS22HH MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x22;

        // READING the data from the I2C (X_AXIS_ANGULAR_RATE LSB)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[X_AXIS_ANGULAR_RATE] An error occur with LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [X_AXIS_ANGULAR_RATE] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            x_axis_angular_rate = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Y_AXIS_ANGULAR_RATE for the MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (Y_AXIS_ANGULAR_RATE MSB AND LSB)
    LOG_DBG("Checking Y_AXIS_ANGULAR_RATE [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x25;

        // WRITING the data from the I2C [Y_AXIS_ANGULAR_RATE MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Y_AXIS_ANGULAR_RATE] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor  (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Y_AXIS_ANGULAR_RATE] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x24;

        // READING the data from the I2C (Y_AXIS_ANGULAR_RATE LSB)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[Y_AXIS_ANGULAR_RATE] An error occur with LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [Y_AXIS_ANGULAR_RATE] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            y_axis_angular_rate = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Z_AXIS_ANGULAR_RATE for the MEM NANO 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (Z_AXIS_ANGULAR_RATE MSB AND LSB)
    LOG_DBG("Checking Z_AXIS_ANGULAR_RATE [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x27;

        // WRITING the data from the I2C [Z_AXIS_ANGULAR_RATE MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Z_AXIS_ANGULAR_RATE] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Z_AXIS_ANGULAR_RATE] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x26;

        // READING the data from the I2C (Z_AXIS_ANGULAR_RATE LSB)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[Z_AXIS_ANGULAR_RATE] An error occur with LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [Z_AXIS_ANGULAR_RATE] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            z_axis_angular_rate = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // X_AXIS_LINEAR_ACCELERATION REFERENCE for the MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (X_AXIS_LINEAR_ACCELERATION MSB AND LSB) [Going to use the offset address that is 2's complement]
    LOG_DBG("Checking X_AXIS_LINEAR_ACCELERATION [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x29;

        // WRITING the data from the I2C [X_AXIS_LINEAR_ACCELERATION MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[X_AXIS_LINEAR_ACCELERATION] An error occur with the LPS22HH MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [X_AXIS_LINEAR_ACCELERATION] LPS22HH MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x28;

        // READING the data from the I2C (X_AXIS_LINEAR_ACCELERATION LSB)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[X_AXIS_LINEAR_ACCELERATION] An error occur with LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [X_AXIS_LINEAR_ACCELERATION] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            x_axis_linear_acceleration = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Y_AXIS_LINEAR_ACCELERATION for the MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (Y_AXIS_LINEAR_ACCELERATION MSB AND LSB)
    LOG_DBG("Checking Y_AXIS_LINEAR_ACCELERATION [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x2b;

        // WRITING the data from the I2C [Y_AXIS_LINEAR_ACCELERATION MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Y_AXIS_LINEAR_ACCELERATION] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor  (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Y_AXIS_LINEAR_ACCELERATION] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x2a;

        // READING the data from the I2C (Y_AXIS_LINEAR_ACCELERATION LSB)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[Y_AXIS_LINEAR_ACCELERATION] An error occur with LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [Y_AXIS_LINEAR_ACCELERATION] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            y_axis_linear_acceleration = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Z_AXIS_LINEAR_ACCELERATION for the MEM NANO 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (Z_AXIS_LINEAR_ACCELERATION MSB AND LSB)
    LOG_DBG("Checking Z_AXIS_LINEAR_ACCELERATION [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x2d;

        // WRITING the data from the I2C [Z_AXIS_LINEAR_ACCELERATION MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Z_AXIS_LINEAR_ACCELERATION] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Z_AXIS_LINEAR_ACCELERATION] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x2c;

        // READING the data from the I2C (Z_AXIS_LINEAR_ACCELERATION LSB)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[Z_AXIS_LINEAR_ACCELERATION] An error occur with LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [Z_AXIS_LINEAR_ACCELERATION] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using signed int16_t to undo it 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8);

            // The code may not be done but convert it to float
            z_axis_linear_acceleration = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // X_AXIS_LINEAR_OFFSET from the 3D MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (X_AXIS_LINEAR_OFFSET MSB AND LSB)
    LOG_DBG("Checking X_AXIS_LINEAR_OFFSET [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address (DATA MUST BE IN THE RANGE [-127 127]) <----------------------
        i2c_buffer_write[0] = 0x73;

        // WRITING the data from the I2C [X_AXIS_LINEAR_OFFSET MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[X_AXIS_LINEAR_OFFSET] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [X_AXIS_LINEAR_OFFSET] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using 
            // According to the datasheet it is 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = (mems_pressure_sensor_reading[0]);

            // The code may not be done but convert it to float (celsius)
            x_axis_linear_offset = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Y_AXIS_LINEAR_OFFSET from the 3D MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (Y_AXIS_LINEAR_OFFSET MSB AND LSB)
    LOG_DBG("Checking Y_AXIS_LINEAR_OFFSET [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address (DATA MUST BE IN THE RANGE [-127 127]) <----------------------
        i2c_buffer_write[0] = 0x74;

        // WRITING the data from the I2C [Y_AXIS_LINEAR_OFFSET MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Y_AXIS_LINEAR_OFFSET] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Y_AXIS_LINEAR_OFFSET] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using 
            // According to the datasheet it is 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = (mems_pressure_sensor_reading[0]);

            // The code may not be done but convert it to float (celsius)
            y_axis_linear_offset = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Z_AXIS_LINEAR_OFFSET from the 3D MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (Z_AXIS_LINEAR_OFFSET MSB AND LSB)
    LOG_DBG("Checking Z_AXIS_LINEAR_OFFSET [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address (DATA MUST BE IN THE RANGE [-127 127]) <----------------------
        i2c_buffer_write[0] = 0x75;

        // WRITING the data from the I2C [Z_AXIS_LINEAR_OFFSET MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[Z_AXIS_LINEAR_OFFSET] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [Z_AXIS_LINEAR_OFFSET] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using 
            // According to the datasheet it is 
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            int16_t converted_raw_data = (mems_pressure_sensor_reading[0]);

            // The code may not be done but convert it to float (celsius)
            z_axis_linear_offset = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // TEMPERATURE from the 3D MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (TEMPERATURE MSB AND LSB)
    LOG_DBG("Checking TEMPERATURE [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x21;

        // WRITING the data from the I2C [TEMPERATURE MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[TEMPERATURE] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [TEMPERATURE] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) !!!");

        // Creating the buffer for writing [LSB] 2's copmlement address  <----------------------
        i2c_buffer_write[1] = 0x20;

        // READING the data from the I2C (TEMPERATURE LSB)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_read[1], 1, &mems_pressure_sensor_reading[1], 1);

        // Creating a condition to see if the data is not reading the correct data
        if (error < 0){
            // Logging the module 
            LOG_ERR("[TEMPERATURE] An error occur with LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) . . .");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));
        
        // Logging the module
        LOG_INF("SUCCESS: [TEMPERATURE] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING I2C) !!!");

        // Converting the raw data to usuasable data
        {
            // Since the data is in 2's complement we can using 
            // According to the datasheet it is (nominal sensitivity is 8 LSB/Celsius)
            // Going to use the same formula like always (value = (MSB | LSB << 8) )
            nominal_sensitivity = 16.0;
            int16_t converted_raw_data = (mems_pressure_sensor_reading[0] | (mems_pressure_sensor_reading[1] << 8)) / nominal_sensitivity;

            // The code may not be done but convert it to float (celsius)
            temperature_accelerometer_gryoscrope_sensor = (float)converted_raw_data;
        }
    }

    // ============================================================================================
    // Checking the STATUS of the 3D MEM 3D Accelerometer + 3D Gryoscrope
    // ============================================================================================

    // Logging the module for the LSM6DSO (STATUS of the sensor)
    LOG_DBG("Checking STATUS [MSB and LSB] for the MEMS 3D Accelerometer + 3D Gryoscrope Sensor I2C write/read (LSM6DSO) . . . ");
    {
        // Creating the buffer for writing [MSB] 2's complement address <----------------------
        i2c_buffer_write[0] = 0x1e;

        // WRITING the data from the I2C [STATUS MSB] (sensor data, buffer address for write, read, 1)
        error = i2c_write_read_dt(&lsm6dso_dt_spec, &i2c_buffer_write[0], 1, &mems_pressure_sensor_reading[0], 1);

        // Creating a condition to see if the data the wrong data
        if (error < 0){
            // Logging the module
            LOG_ERR("[STATUS] An error occur with the LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (WRITING I2C) . . . ");

            // Stop the application
            return;
        }
        // Creating a 1 second delay
        k_sleep(K_SECONDS(1));

        // Logging the module
        LOG_DBG("SUCCESS: [STATUS] LSM6DSO MEMS 3D Accelerometer + 3D Gryoscrope Sensor (READING AND WRITING I2C) !!!");


        // Understanding the raw data from STATUS
        {
            // It should be noted that the address will have to be raw data
            // The default values is set to zero if the value is one then a new set of data has been written
            // The status register is only one byte and each bit are the following:
            // 0, 0, 0, 0, 0, TDA, GDA, XLDA
            // The code may not be done but convert it to float
            status = mems_pressure_sensor_reading[0];
        }
    }

    // Printing all the information to the module
    {
        // Logging the module
        LOG_WRN("===================================================================");
        LOG_INF("(LSM6DSO) The X AXIS ANGULAR RATE is %.2f", x_axis_angular_rate);
        LOG_INF("(LSM6DSO) The Y AXIS ANGULAR RATE is %.2f", y_axis_angular_rate);
        LOG_INF("(LSM6DSO) The Z AXIS ANGULAR RATE is %.2f", z_axis_angular_rate);
        LOG_INF("(LSM6DSO) The X AXIS LINEAR ACCELERATION is %.2f", x_axis_linear_acceleration);
        LOG_INF("(LSM6DSO) The Y AXIS LINEAR ACCELERATION is %.2f", y_axis_linear_acceleration);
        LOG_INF("(LSM6DSO) The Z AXIS LINEAR ACCELERATION is %.2f", z_axis_linear_acceleration);
        LOG_INF("(LSM6DSO) The X AXIS LINEAR OFFSET is %.2f", x_axis_linear_offset);
        LOG_INF("(LSM6DSO) The Y AXIS LINEAR OFFSET is %.2f", y_axis_linear_offset);
        LOG_INF("(LSM6DSO) The Z AXIS LINEAR OFFSET is %.2f", z_axis_linear_offset);
        LOG_INF("(LSM6DSO) The STATUS of the sensor is %f", status);
        LOG_INF("(LSM6DSO) The TEMPERATURE is %.2f", temperature_accelerometer_gryoscrope_sensor);
        LOG_WRN("===================================================================");

        // Storing the information to a global struct
        sensor_6.x_axis_angular_rate = x_axis_angular_rate;
        sensor_6.y_axis_angular_rate = y_axis_angular_rate;
        sensor_6.z_axis_angular_rate = z_axis_angular_rate;
        sensor_6.x_axis_linear_acceleration = x_axis_linear_acceleration;
        sensor_6.y_axis_linear_acceleration = y_axis_linear_acceleration;
        sensor_6.z_axis_linear_acceleration = z_axis_linear_acceleration;
        sensor_6.x_axis_linear_acceleration_offset = x_axis_linear_offset;
        sensor_6.y_axis_linear_acceleration_offset = y_axis_linear_offset;
        sensor_6.z_axis_linear_acceleration_offset = z_axis_linear_offset;
        sensor_6.celsius_temperature = temperature_accelerometer_gryoscrope_sensor;
    }
}

void ble_manager(){
    // Creating a variable that will be able to detect an error
    int error;

    // Creating the counter to determine if notifications is updating (more for the logging module)
    uint32_t counter = 0;

    // Logging the module
    LOG_DBG("Setting up bluetooth . . . ");

    // Getting the bluetooth low energy device address (MAC Address)
    bt_addr_le_t ble_device_address;

    // Getting a boolean value if an error occur from the BLE device (converts the string to binary values, the bt function) ["ADDRESS", "TYPE", &ADDRESS]
    error = bt_addr_le_from_str("FF:EE:DE:AD:BE:EF", "random", &ble_device_address);

    // Creating an conditional statement if an error occur
    if(error){
        // Logging the module
        LOG_ERR("(BLE MANAGER) Invalid Bluetooth Address (error %d)", error);

        // Stop the program
        return;
    }

    // Creating a new identity for the advertisement
    error = bt_id_create(&ble_device_address, NULL);

    // Creating a conditional statement if the connectioned failed
    if (error < 0){
        // Logging the module
        LOG_ERR("(BLE MANAGER) Creating new identity failed (error %d)", error);

        // Stop the program
        return;
    }

    // Register callbacks to monitor the state of the connections (calling the struct if the BLE device is connected or not)
    bt_conn_cb_register(&bluetooth_connection_callbacks);

    // Getting a value to determine if an error occur or not
    error = bt_enable(NULL);

    // Determining if initializing bluetooth is a success or not
    if (error < 0){
        // Logging the module
        LOG_ERR("Bluetooth init failed (err %d)", error);

        // Stop the application
        return;
    }

    // Logging the module
    LOG_DBG("Bluetooth enabled!!! Starting advertising. . . ");

    // Start advertising reponse and scan response
    error = bt_le_adv_start(adv_param, adv_data, ARRAY_SIZE(adv_data), scan_data, ARRAY_SIZE(scan_data));

    // Creating an conditional statement if the values are wrong/incorrect
    if (error < 0){
        // Logging the module
        LOG_ERR("(BLE MANAGER) Bluetooth advertisement failed (error %d)", error);

        // Stop the program
        return;
    }

    // Logging the module
    LOG_DBG("Advertising started!!!");

    // Then do these operations
    while(1){
        LOG_WRN("==========================================================");
        // Basic testing
        // api_ble_gatt_notify(counter);        
        // LOG_INF("Hello, I am here %d", counter);
        // counter++;
        
        // Calling the function
        // sensor_stts751();
        sensor_lis2mdl();

        // Send notification to the ble gatt notify function
        api_ble_gatt_notify(sensor_1.celsius_temperature);
        api_ble_gatt_notify(sensor_1.fahrenheit_temperature);
        // api_ble_gatt_notify(sensor_3.x_axis_magnometer);
        // api_ble_gatt_notify(sensor_3.y_axis_magnometer);
        // api_ble_gatt_notify(sensor_3.z_axis_magnometer);

        // Logging the module (for us to see the code working in the terminal)
        LOG_INF("The sensor data is: %lf", sensor_1.celsius_temperature);
        LOG_INF("The sensor data is: %lf", sensor_1.fahrenheit_temperature);
        // LOG_INF("The sensor data is: %lf", sensor_3.x_axis_magnometer);
        // LOG_INF("The sensor data is: %lf", sensor_3.y_axis_magnometer);
        // LOG_INF("The sensor data is: %lf", sensor_3.z_axis_magnometer);        
        LOG_WRN("==========================================================");
        k_sleep(K_SECONDS(5));
    }
}

int api_ble_gatt_notify(uint32_t sensor_value){
    // Creating a variable to to store any error values
    int error;

    // Creating a conditional statement to determine if there is an error
	if (!notify_mysensor_enabled) {
        // Return the status of the access/status
		return -EACCES;
	}

    // &sst_service.attrs is the Service Attribute table: 0 = Service, 1 = Primary service, 2 = Characteristic
	error = bt_gatt_notify(NULL, &sst_service.attrs[2], &sensor_value, sizeof(sensor_value));
    
    // Creating a conditional statement if an error occurs
    if (error < 0) {
        // Logging the module
        LOG_ERR("(BLE NOTIFICATION) Warning: bt_gatt_notify failed sending %d with error: %d", sensor_value, error);
    }
    else {
        // Logging the module
        LOG_DBG("(BLE NOTIFICATION) Sent sensor value: %d", sensor_value);
    }

    // Return the error
    return error;
}

void setup_pins(){
    // Logging the module
    LOG_DGB("Initializing hardware . . . ");

    // Adding the devices input/output to the struct
    const struct gpio_dt_spec spec_pin_sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
	const struct gpio_dt_spec spec_pin_sw1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
	const struct gpio_dt_spec spec_pin_sw2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
	const struct gpio_dt_spec spec_pin_sw3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);
	const struct gpio_dt_spec spec_pin_led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	const struct gpio_dt_spec spec_pin_led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
	const struct gpio_dt_spec spec_pin_led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
	const struct gpio_dt_spec spec_pin_led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

    // Defining the inputs of the pin
    gpio_pin_configure_dt(&spec_pin_sw0, GPIO_INPUT);
    gpio_pin_configure_dt(&spec_pin_sw1, GPIO_INPUT);
    gpio_pin_configure_dt(&spec_pin_sw2, GPIO_INPUT);
    gpio_pin_configure_dt(&spec_pin_sw3, GPIO_INPUT);

    //

}

// void led_on(size_t index_led){
//   // Turns ON the LED
//   digitalWrite(led_array[index_led], LOW);
// }

// void led_off(size_t index_led){
//   // Turns OFF the LED
//   digitalWrite(led_array[index_led], HIGH);
// }

void main(void){
    // Setting up the GPIO PINS
    setup_pins();

    // Calling the GATT Bluetooth manager
    ble_manager();
}
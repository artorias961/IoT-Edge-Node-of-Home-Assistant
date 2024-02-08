/*
 * Link for Nucleo Datasheet: https://www.st.com/resource/en/data_brief/x-nucleo-iks01a3.pdf 
 * Link for Dev Academy: https://academy.nordicsemi.com/courses/nrf-connect-sdk-fundamentals/
 * Link for Nucleo Datasheet: https://www.st.com/resource/en/user_manual/um2559-getting-started-with-the-xnucleoiks01a3-motion-mems-and-environmental-sensor-expansion-board-for-stm32-nucleo-stmicroelectronics.pdf 
 * 
 * HINT:
 * WHEN ADDING A NEW SENSOR TO THE OVERLAY FILE, I NEED TO REBUILD THE ENTIRE PROGRAM
*/

/***
 * File to test that I2C is working with the HTS221. 
 * NOTE: The sensor and the nRF board are a bit picky.
 *       Since there is no RESET button on the sensor
 *          board, you'll have to unplug your nRF board
 *          if you get an I2C error (just something got
 *          goofed) and replug in.
 * NOTE: Also, make sure you do a pristine build (circle arrow icon).
 * 
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>


LOG_MODULE_REGISTER(i2c_test, LOG_LEVEL_DBG);


void main(void) {
    uint8_t i2c_buf_write[2], i2c_buf_read[2];
    int err;
    LOG_INF("Board is up!  Setting up I2C...");
    const struct i2c_dt_spec hts221_dt_spec = I2C_DT_SPEC_GET(DT_NODELABEL(my_hts221));
    if (!device_is_ready(hts221_dt_spec.bus)) {
        LOG_ERR("I2C initialization failed for some reason!");
        return;
    }
    k_sleep(K_SECONDS(1));

    LOG_INF("Checking individual I2C write/read..."); 
    {
        i2c_buf_write[0] = 0x0f;  // write that we want the WHOAMI register
        err = i2c_write_dt(&hts221_dt_spec, &i2c_buf_write[0], 1);
        if (err < 0) {
            LOG_ERR("i2c_write_dt() failed with err: %d", err);
            return;
        }

        err = i2c_read_dt(&hts221_dt_spec, &i2c_buf_read[0], 1);
        if (err < 0) {
            LOG_ERR("i2c_read_dt() failed with err: %d", err);
            return;
        }
        LOG_INF("Received this from asking 0x0f: 0x%2x", i2c_buf_read[0]);
        LOG_INF("!Check that we have the value 0xbc!");
    }
    k_sleep(K_SECONDS(1));

    LOG_INF("Checking combined I2C write/read command..."); 
    {
        i2c_buf_write[0] = 0x0f;  // write that we want the WHOAMI register
        err = i2c_write_read_dt(&hts221_dt_spec, &i2c_buf_write[0], 1, &i2c_buf_read[0], 1);
        if (err < 0) {
            LOG_ERR("i2c_write_read_dt() failed with err: %d", err);
            return;
        }
        LOG_INF("Received this from asking 0x0f: 0x%2x", i2c_buf_read[0]);
        LOG_INF("!Check that we have the value 0xbc!");
    }
    k_sleep(K_SECONDS(1));

    LOG_INF("Checking I2C register read command..."); 
    {
        i2c_buf_write[0] = 0x0f;  // write that we want the WHOAMI register
        err = i2c_reg_read_byte_dt(&hts221_dt_spec, 0x0f, &i2c_buf_read[0]);
        if (err < 0) {
            LOG_ERR("i2c_write_read_dt() failed with err: %d", err);
            return;
        }
        LOG_INF("Received this from asking 0x0f: 0x%2x", i2c_buf_read[0]);
        LOG_INF("!Check that we have the value 0xbc!");
    }
}

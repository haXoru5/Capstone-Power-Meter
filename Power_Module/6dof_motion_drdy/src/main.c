/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/devicetree.h>
 #include <zephyr/drivers/sensor.h>
 #include <stdio.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/drivers/adc.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
 
 static struct sensor_trigger data_trigger;
 
 /* Flag set from IMU device irq handler */
 static volatile int irq_from_device;
 

//ADC init
/* ADC node from the devicetree. */
#define ADC_NODE DT_ALIAS(adc0)

/* Auxiliary macro to obtain channel vref, if available. */
#define CHANNEL_VREF(node_id) DT_PROP_OR(node_id, zephyr_vref_mv, 0)

/* Data of ADC device specified in devicetree. */
static const struct device *adc = DEVICE_DT_GET(ADC_NODE);

/* Data array of ADC channels for the specified ADC. */
static const struct adc_channel_cfg channel_cfgs[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,))};

/* Data array of ADC channel voltage references. */
static uint32_t vrefs_mv[] = {DT_FOREACH_CHILD_SEP(ADC_NODE, CHANNEL_VREF, (,))};

/* Get the number of channels defined on the DTS. */
#define CHANNEL_COUNT ARRAY_SIZE(channel_cfgs)
#define CONFIG_SEQUENCE_SAMPLES 32

/*Cal Data*/
#define CAL_SLOPE -0.35
#define CAL_OFFSET 827

#define RADS_TO_RPM 9.5493

 /*
  * Get a device structure from a devicetree node from alias
  * "6dof_motion_drdy0".
  */
 static const struct device *get_6dof_motion_device(void)
 {
         const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(6dof_motion_drdy0));
 
         if (!device_is_ready(dev)) {
                 printk("\nError: Device \"%s\" is not ready; "
                        "check the driver initialization logs for errors.\n",
                        dev->name);
                 return NULL;
         }
 
         printk("Found device \"%s\", getting sensor data\n", dev->name);
         return dev;
 }
 
 static const char *now_str(void)
 {
         static char buf[16]; /* ...HH:MM:SS.MMM */
         uint32_t now = k_uptime_get_32();
         unsigned int ms = now % MSEC_PER_SEC;
         unsigned int s;
         unsigned int min;
         unsigned int h;
 
         now /= MSEC_PER_SEC;
         s = now % 60U;
         now /= 60U;
         min = now % 60U;
         now /= 60U;
         h = now;
 
         snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
         return buf;
 }
 
 static void handle_6dof_motion_drdy(const struct device *dev, const struct sensor_trigger *trig)
 {
         if (trig->type == SENSOR_TRIG_DATA_READY) {
                 int rc = sensor_sample_fetch_chan(dev, trig->chan);
 
                 if (rc < 0) {
                         printf("sample fetch failed: %d\n", rc);
                         printf("cancelling trigger due to failure: %d\n", rc);
                         (void)sensor_trigger_set(dev, trig, NULL);
                         return;
                 } else if (rc == 0) {
                         irq_from_device = 1;
                 }
         }
 }

 
 int main(void)
 {
        //IMU init
         const struct device *dev = get_6dof_motion_device();
         struct sensor_value accel[3];
         struct sensor_value gyro[3];
         struct sensor_value temperature;
 
         if (dev == NULL) {
                 return 0;
         }
 
         data_trigger = (struct sensor_trigger){
                 .type = SENSOR_TRIG_DATA_READY,
                 .chan = SENSOR_CHAN_ALL,
         };
         if (sensor_trigger_set(dev, &data_trigger, handle_6dof_motion_drdy) < 0) {
                 printf("Cannot configure data trigger!!!\n");
                 return 0;
         }

        //ADC init
         int err;
         //uint32_t count = 0;
         uint16_t channel_reading[CONFIG_SEQUENCE_SAMPLES][CHANNEL_COUNT];
         uint16_t avg_val_chans[CHANNEL_COUNT];
         double gyro_y;
         double torque;
         double power;
         double rpm;
 
         /* Options for the sequence sampling. */
         const struct adc_sequence_options options = {
                 .extra_samplings = CONFIG_SEQUENCE_SAMPLES - 1,
                 .interval_us = 0,
         };
 
         /* Configure the sampling sequence to be made. */
         struct adc_sequence sequence = {
                 .buffer = channel_reading,
                 /* buffer size in bytes, not number of samples */
                 .buffer_size = sizeof(channel_reading),
                 .resolution = CONFIG_SEQUENCE_RESOLUTION,
                 .options = &options,
         };
 
         if (!device_is_ready(adc)) {
                 printf("ADC controller device %s not ready\n", adc->name);
                 return 0;
         }
 
         /* Configure channels individually prior to sampling. */
         for (size_t i = 0U; i < CHANNEL_COUNT; i++) {
                 sequence.channels |= BIT(channel_cfgs[i].channel_id);
                 err = adc_channel_setup(adc, &channel_cfgs[i]);
                 if (err < 0) {
                         printf("Could not setup channel #%d (%d)\n", i, err);
                         return 0;
                 }
                 if (channel_cfgs[i].reference == ADC_REF_INTERNAL) {
                         vrefs_mv[i] = adc_ref_internal(adc);
                 }
         }
 
         k_sleep(K_MSEC(1000));
 
         while (1) {
                uint32_t now = k_uptime_get_32();
                static uint32_t last_time = 0;
                 /*if (irq_from_device) {
                         //sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
                         sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
                         sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
 
                         /*LOG_INF("[%s]: temp %.2f Cel "
                                "  accel %f %f %f m/s/s "
                                "  gyro  %f %f %f rad/s\n",
                                now_str(), sensor_value_to_double(&temperature),
                                sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
                                sensor_value_to_double(&accel[2]), sensor_value_to_double(&gyro[0]),
                                sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
                           
                                irq_from_device = 0;
                 }
                */
                 uint32_t elapsed = now - last_time;
                if (elapsed >= 1000) {
                        last_time = now;
                        //printf("Time elapsed: %u ms\n", elapsed);
                        //printf("Time elapsed: %s\n", now_str());
                        if(irq_from_device) {
                                //sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
                                sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
                                gyro_y = sensor_value_to_double(&gyro[1]);
                                irq_from_device = 0;
                        }

                        err = adc_read(adc, &sequence);
                        if (err < 0) {
                                printf("Could not read (%d)\n", err);
                                continue;
                        }

                        for (size_t channel_index = 0U; channel_index < CHANNEL_COUNT; channel_index++) {
                                int32_t val_mv;
                                int32_t avg_val_mv = 0;
                                /*printf("- %s, channel %" PRId32 ", %" PRId32 " sequence samples:\n",
                                       adc->name, channel_cfgs[channel_index].channel_id,
                                       CONFIG_SEQUENCE_SAMPLES);
                                */
                                for (size_t sample_index = 0U; sample_index < CONFIG_SEQUENCE_SAMPLES;
                                     sample_index++) {
        
                                        val_mv = channel_reading[sample_index][channel_index];
                                        //printf("- - %" PRId32, val_mv);
                                        err = adc_raw_to_millivolts(vrefs_mv[channel_index],
                                                                    channel_cfgs[channel_index].gain,
                                                                    CONFIG_SEQUENCE_RESOLUTION, &val_mv);
                                        
                                        /* conversion to mV may not be supported, skip if not */
                                        if ((err < 0) || vrefs_mv[channel_index] == 0) {
                                                printf(" (value in mV not available)\n");
                                        } else {
                                                avg_val_mv += val_mv;
                                                
                                                //printf(" = %" PRId32 "mV\n", val_mv);
                                        }
                                }
                                avg_val_mv /= CONFIG_SEQUENCE_SAMPLES;
                                avg_val_chans[channel_index] = avg_val_mv;
                                //printf("Average value: %" PRId32 "mV\n", avg_val_mv);
                        }
                        torque = avg_val_chans[0] * CAL_SLOPE + CAL_OFFSET;
                        power = torque * gyro_y;
                        rpm = gyro_y * RADS_TO_RPM;
                        LOG_INF("Torque: %.1f mV\n Power: %.1f\n cadence %.1f\n", torque, power, rpm);
                        //printf("Time: %s\n", now_str());
                        //LOG_INF("Temp: %.2f Cel",sensor_value_to_double(&temperature));

                }
         }
         return 0;
 }


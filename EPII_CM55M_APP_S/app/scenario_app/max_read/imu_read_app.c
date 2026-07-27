#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>


#ifdef TRUSTZONE_SEC
#ifdef FREERTOS
/* Trustzone config. */
//
/* FreeRTOS includes. */
//#include "secure_port_macros.h"
#else
#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif
#include "arm_cmse.h"
//#include "veneer_table.h"
//
#endif
#endif

#include "xprintf.h"
#include "timer_interface.h"
#include "hx_drv_scu.h"
#include "hx_drv_iic.h"
#include "imu_read_app.h"
#include "icm42688.h"
#include "max30102.h"
#include "Preprocessing.h"

/*!
 * @brief Main function
 */
int app_main(void) {

    uint32_t out_red = 0;
    uint32_t sample1 = 0;
    uint8_t avail = 0;
    uint8_t sample_count = 0;
	IIC_ERR_CODE_E i2c_err;
	
	xprintf("Start Max30102 read trial\n");

	// The output pin of I2C Master 0 is defined by the user application.
	hx_drv_scu_set_PA2_pinmux(SCU_PA2_PINMUX_I2C_M_SCL, 1);
	hx_drv_scu_set_PA3_pinmux(SCU_PA3_PINMUX_I2C_M_SDA, 1);

	// initializes the I2C Master 0 with SCL speed of 400 KHz
	// hx_drv_i2cm_init(USE_DW_IIC_0, HX_I2C_HOST_MST_0_BASE, DW_IIC_SPEED_FAST);
	//hx_drv_i2cm_set_err_cb(USE_DW_IIC_0, i2cm_0_err_cb);
    
    

    if (max30102_init()==0){
        xprintf("MAX30102 init success.\n");
    }
    else {
        xprintf("init gagal\n");
    }
	while ( 1 )
	{
        if (max30102_available_samples(&avail) == 0 && avail > 0){
            if (max30102_read_hr_sample(&out_red) == 0){
                xprintf ("sample: %lu\n", out_red);
                hx_drv_timer_cm55x_delay_ms(100, TIMER_STATE_DC);
                if (sample_count == 0) {
                    sample1 = raw_ir_data;
                    sample_count++;
                } else {
                    // Masukkan 2 sampel ke library preprocessing
                    ppg_add_samples_64hz(sample1, raw_ir_data);
                    sample_count = 0;
                }
            }
            else{
                xprintf("gagal baca data FIFO\n");
            }
        }
        if (ppg_is_buffer_ready()) {
            
            // A. Siapkan parameter input TFLite
            TfLiteTensor* input_tensor = interpreter->input(0);
            int8_t* model_input = input_tensor->data.int8;
            float input_scale = input_tensor->params.scale;
            int32_t input_zero_point = input_tensor->params.zero_point;

            // B. Proses Z-Score & Kuantisasi langsung ke memori Tensor
            ppg_preprocess_and_feed_int8(model_input, input_scale, input_zero_point);

            // C. Jalankan Inferensi di NPU / CPU
            if (interpreter->Invoke() == kTfLiteOk) {
                
                // D. Ekstrak dan De-Kuantisasi Output
                TfLiteTensor* output_tensor = interpreter->output(0);
                int8_t out_int8 = output_tensor->data.int8[0];
                float out_scale = output_tensor->params.scale;
                int32_t out_zero_point = output_tensor->params.zero_point;

                // Kembalikan INT8 ke Float (BPM)
                float bpm_prediksi = (out_int8 - out_zero_point) * out_scale;

                xprintf("==== INFERENSI SUKSES ====\n");
                xprintf("Tebakan BPM : %.2f\n", bpm_prediksi);
            } else {
                xprintf("Inference gagal!\n");
            }

            // E. Geser buffer untuk menebak 2 detik berikutnya
            ppg_shift_buffer();
        }

        // Delay sangat kecil (atau hilangkan sama sekali) agar polling I2C tidak terhambat
        hx_drv_timer_cm55x_delay_ms(5, TIMER_STATE_DC);

	}
    
	return 0;
}


volatile uint32_t g_err_cb = 0;

void i2cm_0_err_cb(void *status)
{
    HX_DRV_DEV_IIC *iic_obj = status;
    HX_DRV_DEV_IIC_INFO *iic_info_ptr = &(iic_obj->iic_info);

	g_err_cb = 1;
	xprintf("[%s] err:%d \n", __FUNCTION__, iic_info_ptr->err_state);
}

void i2c_scan() {
    printf("=== I2C SCAN START ===\n");
    int found = 0;
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t test_byte = 0;
        uint8_t reg_addr = 0x00;  // Register dummy
        
        // Gunakan write_stop_read bukan write_restart_read
        IIC_ERR_CODE_E ret = hx_drv_i2cm_write_stop_read(
            USE_DW_IIC_0,
            addr << 1,      // 8-bit address
            &reg_addr,      // Register address (bukan NULL)
            1,              // Address length = 1
            &test_byte,     // Buffer
            1               // Read 1 byte
        );
        
        if (ret == IIC_ERR_OK) {
            printf("Device found at 0x%02X (7-bit)\n", addr);
            found++;
        }
    }
    printf("Total devices found: %d\n", found);
    printf("=== I2C SCAN END ===\n");
}

void check_part_id() {
    uint8_t part_id;
    uint8_t reg_addr = 0xFF;
    
    IIC_ERR_CODE_E ret = hx_drv_i2cm_write_restart_read(
        USE_DW_IIC_0,
        0x57 << 1,  // MAX30102 default address
        &reg_addr,
        1,
        &part_id,
        1
    );
    
    if (ret == IIC_ERR_OK) {
        printf("Part ID: 0x%02X (Expected: 0x15)\n", part_id);
    } else {
        printf("Failed to read Part ID, error: %d\n", ret);
    }
}
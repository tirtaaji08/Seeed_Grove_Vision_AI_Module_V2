/*
 * cvapp.cpp
 *
 *  Created on: 2018�~12��4��
 *      Author: 902452
 */

#include <cstdio>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "WE2_device.h"
#include "board.h"
#include "cvapp_mb_cls.h"
#include "cisdp_sensor.h"
#include "Golden_Data.h"

#include "WE2_core.h"

#include "ethosu_driver.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
#if TFLM2209_U55TAG2205
#include "tensorflow/lite/micro/micro_error_reporter.h"
#endif
#include "img_proc_helium.h"


#include "xprintf.h"
#include "spi_master_protocol.h"
#include "cisdp_cfg.h"
#include "memory_manage.h"
#include <send_result.h>
#include <forward_list>

#include "pmu_ethosu.h" // tambahanku
#include "max30102.h" // tambahanku
#include "Preprocessing.h"    // tambahanku
#include "hx_drv_iic.h"
#define INPUT_IMAGE_CHANNELS 3


#define MB_CLS_INPUT_TENSOR_WIDTH   224
#define MB_CLS_INPUT_TENSOR_HEIGHT  224
#define MB_CLS_INPUT_TENSOR_CHANNEL INPUT_IMAGE_CHANNELS


#define MB_CLS_DBG_APP_LOG 0


// #define EACH_STEP_TICK
#define TOTAL_STEP_TICK

uint32_t systick_1, systick_2;
uint32_t loop_cnt_1, loop_cnt_2;
uint64_t cycle_start, cycle_end, npu_cycle; // Tambahanku
#define CPU_CLK	0xffffff+1
static uint32_t capture_image_tick = 0;
#ifdef TRUSTZONE_SEC
#define U55_BASE	BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#ifndef TRUSTZONE
#define U55_BASE	BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#define U55_BASE	BASE_ADDR_APB_U55_CTRL
#endif
#endif


using namespace std;

namespace {

constexpr int tensor_arena_size = 450*1024;

static uint32_t tensor_arena=0;

struct ethosu_driver ethosu_drv; /* Default Ethos-U device driver */
tflite::MicroInterpreter *mb_cls_int_ptr=nullptr;
TfLiteTensor *mb_cls_input, *mb_cls_output;
};

static void _arm_npu_irq_handler(void)
{
    /* Call the default interrupt handler from the NPU driver */
    ethosu_irq_handler(&ethosu_drv);
}

/**
 * @brief  Initialises the NPU IRQ
 **/
static void _arm_npu_irq_init(void)
{
    const IRQn_Type ethosu_irqnum = (IRQn_Type)U55_IRQn;

    /* Register the EthosU IRQ handler in our vector table.
     * Note, this handler comes from the EthosU driver */
    EPII_NVIC_SetVector(ethosu_irqnum, (uint32_t)_arm_npu_irq_handler);

    /* Enable the IRQ */
    NVIC_EnableIRQ(ethosu_irqnum);

}

static int _arm_npu_init(bool security_enable, bool privilege_enable)
{
    int err = 0;

    /* Initialise the IRQ */
    _arm_npu_irq_init();

    /* Initialise Ethos-U55 device */
#if TFLM2209_U55TAG2205
	const void * ethosu_base_address = (void *)(U55_BASE);
#else 
	void * const ethosu_base_address = (void *)(U55_BASE);
#endif

    if (0 != (err = ethosu_init(
                            &ethosu_drv,             /* Ethos-U driver device pointer */
                            ethosu_base_address,     /* Ethos-U NPU's base address. */
                            NULL,       /* Pointer to fast mem area - NULL for U55. */
                            0, /* Fast mem region size. */
							security_enable,                       /* Security enable. */
							privilege_enable))) {                   /* Privilege enable. */
    	xprintf("failed to initalise Ethos-U device\n");
            return err;
        }

    xprintf("Ethos-U55 device initialised\n");

    return 0;
}


int cv_mb_cls_init(bool security_enable, bool privilege_enable, uint32_t model_addr) {
	int ercode = 0;

	//set memory allocation to tensor_arena
	tensor_arena = mm_reserve_align(tensor_arena_size,0x20); //1mb
	xprintf("TA[%x]\r\n",tensor_arena);


	if(_arm_npu_init(security_enable, privilege_enable)!=0)
		return -1;

	if(model_addr != 0) {
		static const tflite::Model*mb_cls_model = tflite::GetModel((const void *)model_addr);

		if (mb_cls_model->version() != TFLITE_SCHEMA_VERSION) {
			xprintf(
				"[ERROR] mb_cls_model's schema version %d is not equal "
				"to supported version %d\n",
				mb_cls_model->version(), TFLITE_SCHEMA_VERSION);
			return -1;
		}
		else {
			xprintf("mb_cls model's schema version %d\n", mb_cls_model->version());
		}
		static tflite::MicroMutableOpResolver<1> mb_cls_op_resolver;
		if (kTfLiteOk != mb_cls_op_resolver.AddEthosU()){
			xprintf("Failed to add Arm NPU support to op resolver.");
			return false;
		}

		static tflite::MicroInterpreter mb_cls_static_interpreter(mb_cls_model, mb_cls_op_resolver,
				(uint8_t*)tensor_arena, tensor_arena_size);  
 


		if(mb_cls_static_interpreter.AllocateTensors()!= kTfLiteOk) {
			return false;
		}
		mb_cls_int_ptr = &mb_cls_static_interpreter;
		mb_cls_input = mb_cls_static_interpreter.input(0);
		mb_cls_output = mb_cls_static_interpreter.output(0);

		/*
		if (mb_cls_input->dims->size != 2 ||
        mb_cls_input->dims->data[0] != 1 ||
        mb_cls_input->dims->data[1] != PPG_INPUT_SEQ_LEN) {
        xprintf("Warning: input tensor shape is [%d, %d], expected [1, %d]\n",
                mb_cls_input->dims->data[0], mb_cls_input->dims->data[1], PPG_INPUT_SEQ_LEN);
		*/
	}	

	//Tambahanku
	ETHOSU_PMU_Enable(&ethosu_drv);
    ETHOSU_PMU_CYCCNT_Reset(&ethosu_drv);
    ETHOSU_PMU_EVCNTR_ALL_Reset(&ethosu_drv);
	ETHOSU_PMU_Set_EVTYPER(&ethosu_drv, 0, ETHOSU_PMU_NPU_ACTIVE);
    ETHOSU_PMU_CNTR_Enable(&ethosu_drv, ETHOSU_PMU_CCNT_Msk | ETHOSU_PMU_CNT1_Msk);


	xprintf("initial done\n");
	return ercode;
}

int cv_mb_cls_run(int32_t *bpm) {
	// 1. Check if PPG buffer has accumulated enough samples
    if (!ppg_is_buffer_ready()) {
        return 1;   // Not ready, skip this iteration
    }

    // 2. Prepare input tensor
    int8_t* model_input = mb_cls_input->data.int8;
    float input_scale = mb_cls_input->params.scale;
    int32_t input_zero_point = mb_cls_input->params.zero_point;

    // 3. Preprocess (Z-score + quantize) and feed directly into NPU input buffer
    ppg_preprocess_and_feed_int8(model_input, input_scale, input_zero_point);

    // 4. Measure NPU cycles (optional profiling)
	ETHOSU_PMU_CYCCNT_Reset(&ethosu_drv);
	ETHOSU_PMU_Set_EVCNTR(&ethosu_drv, 0, 0);
    TfLiteStatus invoke_status = mb_cls_int_ptr->Invoke();
	uint64_t total_cycles = ETHOSU_PMU_Get_CCNTR(&ethosu_drv);
	uint32_t active_cycles = ETHOSU_PMU_Get_EVCNTR(&ethosu_drv, 0);
	uint64_t idle_cycles = total_cycles - (uint64_t)active_cycles;

    // 5. Check inference result
    if (invoke_status != kTfLiteOk) {
        xprintf("PPG inference failed (Invoke returned error)\n");
        return -1;
    }

    // 6. Dequantize output (assuming single scalar BPM)
    int8_t out_int8 = mb_cls_output->data.int8[0];
    float out_scale = mb_cls_output->params.scale;
    int32_t out_zero_point = mb_cls_output->params.zero_point;
    int32_t bpm_predicted = (out_int8 - out_zero_point) * out_scale;
	
    if (bpm != NULL) {
        *bpm = bpm_predicted;
    }
    // 7. Print results
	xprintf("B:%lu;T:%lu;A:%lu;I:%lu\n", 
		bpm_predicted,
        total_cycles, 
        active_cycles, 
        idle_cycles);
    // 8. Shift buffer for next window (remove oldest samples)
    //ppg_shift_buffer();	
    ppg_reset_buffer();
	return 0;
}

int cv_mb_cls_deinit()
{
	
	return 0;

}


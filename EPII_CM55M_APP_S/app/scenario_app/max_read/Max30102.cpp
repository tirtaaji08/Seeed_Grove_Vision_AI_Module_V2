#include "Max30102.h"
#include <string.h>
#include "hx_drv_iic.h"
#include "hx_drv_timer.h"
#include "timer_interface.h"

// Static function prototypes
static bool readReg(DFRobot_MAX30102* sensor, uint8_t reg, void* pBuf, uint8_t size);
static bool writeReg(DFRobot_MAX30102* sensor, uint8_t reg, const void* pBuf, uint8_t size);

static bool readReg(DFRobot_MAX30102* sensor, uint8_t reg, void* pBuf, uint8_t size) {
    if (sensor == NULL || pBuf == NULL) return false;
    
    uint8_t addr = reg;
    IIC_ERR_CODE_E ret = hx_drv_i2cm_write_restart_read(
        USE_DW_IIC_0,           // I2C master 0
        sensor->i2c.addr << 1,  // Slave address shift left (8-bit address)
        &addr,                  // Register address
        1,                      // Address length
        (uint8_t*)pBuf,         // Buffer untuk data
        size                    // Data length
    );

        // Tambahkan debug
    if (ret != IIC_ERR_OK) {
        printf("I2C read error: reg=0x%02X, error=%d\n", reg, ret);
    }
    
    return (ret == IIC_ERR_OK);
}

static bool writeReg(DFRobot_MAX30102* sensor, uint8_t reg, const void* pBuf, uint8_t size) {
    if (sensor == NULL || pBuf == NULL) return false;
    
    // buffer untuk write (register + data)
    uint8_t buffer[32];
    if (size + 1 > sizeof(buffer)) return false;
    
    buffer[0] = reg;
    memcpy(&buffer[1], pBuf, size);
    
    IIC_ERR_CODE_E ret = hx_drv_i2cm_write_data(
        USE_DW_IIC_0,           // I2C master 0
        sensor->i2c.addr << 1,  // Slave address shift left
        buffer,                 // Data buffer
        1,                      // Address (register) length
        buffer + 1,             // Data buffer
        size                    // Data length
    );
    if (ret != IIC_ERR_OK) {
        printf("I2C write error: reg=0x%02X, error=%d\n", reg, ret);
    }
    return (ret == IIC_ERR_OK);
}

// sensor instance
DFRobot_MAX30102* max30102_create(MAX30102_I2C_Handle* i2c_handle) {
    if (i2c_handle == NULL) return NULL;
    
    // gunakan I2C master 0 dengan speed 400kHz
    IIC_ERR_CODE_E ret = hx_drv_i2cm_init(USE_DW_IIC_0, HX_I2C_HOST_MST_0_BASE, DW_IIC_SPEED_FAST);
    if (ret != IIC_ERR_OK) {
        return NULL;
    }
    
    DFRobot_MAX30102* sensor = (DFRobot_MAX30102*)malloc(sizeof(DFRobot_MAX30102));
    if (sensor == NULL) return NULL;
    
    // Initialize sensor structure
    memset(sensor, 0, sizeof(DFRobot_MAX30102));
    sensor->i2c.addr = i2c_handle->addr;
    sensor->activeLEDs = 0;
    sensor->senseBuf.head = 0;
    sensor->senseBuf.tail = 0;
    
    return sensor;
}

void max30102_destroy(DFRobot_MAX30102* sensor) {
    if (sensor != NULL) {
        hx_drv_i2cm_deinit(USE_DW_IIC_0);
        free(sensor);
    }
}

// Begin function
bool max30102_begin(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return false;
    
    // Cek Part ID
    if (max30102_getPartID(sensor) != MAX30102_EXPECTED_PARTID) {
        return false;
    }
    
    // Reset sensor
    max30102_softReset(sensor);
    return true;
}

// Soft Reset
void max30102_softReset(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return;
    
    //uint8_t modeReg;
    sMode_t modeReg;
    readReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
    //modeReg |= 0x40;
    modeReg.reset = 1;
    writeReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
    
    uint32_t startTime = 0; // millis equivalent

    for (int i = 0; i < 100; i++) {
        readReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
        //if ((modeReg & 0x40) == 0) break;
        if(modeReg.reset ==0) break;
        // Delay 1ms
        hx_drv_timer_cm55m_sec_delay_ms(10, TIMER_STATE_DC);
    }
}

void max30102_shutDown(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return;
    //uint8_t modeReg;
    sMode_t modeReg;
    readReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
    //modeReg |= 0x80; // Set shutdown bit (bit 7)
    modeReg.shutDown =1;
    writeReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
}

void max30102_wakeUp(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return;
    //uint8_t modeReg;
    sMode_t modeReg;
    readReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
    //modeReg &= ~0x80; // Clear shutdown bit
    modeReg.shutDown = 0;
    writeReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
}

void max30102_setLEDMode(DFRobot_MAX30102* sensor, uint8_t ledMode) {
    if (sensor == NULL) return;
    uint8_t modeReg;
    readReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
    modeReg = (modeReg & 0xF8) | (ledMode & 0x07); // Mask ledMode bits
    writeReg(sensor, MAX30102_MODECONFIG, &modeReg, 1);
}

void max30102_setADCRange(DFRobot_MAX30102* sensor, uint8_t adcRange) {
    if (sensor == NULL) return;
    uint8_t particleReg;
    readReg(sensor, MAX30102_PARTICLECONFIG, &particleReg, 1);
    particleReg = (particleReg & 0x3F) | ((adcRange & 0x03) << 6);
    writeReg(sensor, MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void max30102_setSampleRate(DFRobot_MAX30102* sensor, uint8_t sampleRate) {
    if (sensor == NULL) return;
    uint8_t particleReg;
    readReg(sensor, MAX30102_PARTICLECONFIG, &particleReg, 1);
    particleReg = (particleReg & 0xF0) | ((sampleRate & 0x07) << 2);
    writeReg(sensor, MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void max30102_setPulseWidth(DFRobot_MAX30102* sensor, uint8_t pulseWidth) {
    if (sensor == NULL) return;
    uint8_t particleReg;
    readReg(sensor, MAX30102_PARTICLECONFIG, &particleReg, 1);
    particleReg = (particleReg & 0xFC) | (pulseWidth & 0x03);
    writeReg(sensor, MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void max30102_setPulseAmplitudeRed(DFRobot_MAX30102* sensor, uint8_t amplitude) {
    if (sensor == NULL) return;
    writeReg(sensor, MAX30102_LED1_PULSEAMP, &amplitude, 1);
}

void max30102_setPulseAmplitudeIR(DFRobot_MAX30102* sensor, uint8_t amplitude) {
    if (sensor == NULL) return;
    writeReg(sensor, MAX30102_LED2_PULSEAMP, &amplitude, 1);
}

void max30102_enableSlot(DFRobot_MAX30102* sensor, uint8_t slotNumber, uint8_t device) {
    if (sensor == NULL) return;
    uint8_t multiLEDReg;
    readReg(sensor, MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    
    if (slotNumber == 1) {
        multiLEDReg = (multiLEDReg & 0x0F) | ((device & 0x0F) << 4);
    } else if (slotNumber == 2) {
        multiLEDReg = (multiLEDReg & 0xF0) | (device & 0x0F);
    }
    writeReg(sensor, MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
}

void max30102_disableAllSlots(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return;
    uint8_t multiLEDReg = 0;
    writeReg(sensor, MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
}

void max30102_setFIFOAverage(DFRobot_MAX30102* sensor, uint8_t numberOfSamples) {
    if (sensor == NULL) return;
    uint8_t FIFOReg;
    readReg(sensor, MAX30102_FIFOCONFIG, &FIFOReg, 1);
    FIFOReg = (FIFOReg & 0x1F) | ((numberOfSamples & 0x07) << 5);
    writeReg(sensor, MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void max30102_enableFIFORollover(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return;
    uint8_t FIFOReg;
    readReg(sensor, MAX30102_FIFOCONFIG, &FIFOReg, 1);
    FIFOReg |= 0x10; // Set rollover bit (bit 4)
    writeReg(sensor, MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void max30102_disableFIFORollover(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return;
    uint8_t FIFOReg;
    readReg(sensor, MAX30102_FIFOCONFIG, &FIFOReg, 1);
    FIFOReg &= ~0x10; // Clear rollover bit
    writeReg(sensor, MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void max30102_setFIFOAlmostFull(DFRobot_MAX30102* sensor, uint8_t numberOfSamples) {
    if (sensor == NULL) return;
    uint8_t FIFOReg;
    readReg(sensor, MAX30102_FIFOCONFIG, &FIFOReg, 1);
    FIFOReg = (FIFOReg & 0xE0) | (numberOfSamples & 0x1F);
    writeReg(sensor, MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void max30102_resetFIFO(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return;
    uint8_t zero = 0;
    writeReg(sensor, MAX30102_FIFOWRITEPTR, &zero, 1);
    writeReg(sensor, MAX30102_FIFOOVERFLOW, &zero, 1);
    writeReg(sensor, MAX30102_FIFOREADPTR, &zero, 1);
}

uint8_t max30102_getWritePointer(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return 0;
    uint8_t byteTemp;
    readReg(sensor, MAX30102_FIFOWRITEPTR, &byteTemp, 1);
    return byteTemp;
}

uint8_t max30102_getReadPointer(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return 0;
    uint8_t byteTemp;
    readReg(sensor, MAX30102_FIFOREADPTR, &byteTemp, 1);
    return byteTemp;
}

float max30102_readTemperatureC(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return -999.0;
    
    // Mulai konversi temperature
    uint8_t tempConfig = 0x01;
    writeReg(sensor, MAX30102_DIETEMPCONFIG, &tempConfig, 1);
    
    // Tunggu konversi selesai
    for (int i = 0; i < 100; i++) {
        readReg(sensor, MAX30102_DIETEMPCONFIG, &tempConfig, 1);
        if ((tempConfig & 0x01) == 0) break;
       hx_drv_timer_cm55x_delay_ms(100, TIMER_STATE_DC);
    }
    
    uint8_t tempInt, tempFrac;
    readReg(sensor, MAX30102_DIETEMPINT, &tempInt, 1);
    readReg(sensor, MAX30102_DIETEMPFRAC, &tempFrac, 1);
    
    return (float)tempInt + ((float)tempFrac * 0.0625);
}

float max30102_readTemperatureF(DFRobot_MAX30102* sensor) {
    float temp = max30102_readTemperatureC(sensor);
    if (temp != -999.0) {
        temp = temp * 1.8 + 32.0;
    }
    return temp;
}

uint8_t max30102_getPartID(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return 0;
    uint8_t byteTemp;
    readReg(sensor, MAX30102_PARTID, &byteTemp, 1);
    return byteTemp;
}

void max30102_sensorConfiguration(DFRobot_MAX30102* sensor, 
                                  uint8_t ledBrightness, 
                                  uint8_t sampleAverage, 
                                  uint8_t ledMode, 
                                  uint8_t sampleRate, 
                                  uint8_t pulseWidth, 
                                  uint8_t adcRange) {
    if (sensor == NULL) return;
    
    max30102_setFIFOAverage(sensor, sampleAverage);
    max30102_setADCRange(sensor, adcRange);
    max30102_setSampleRate(sensor, sampleRate);
    max30102_setPulseWidth(sensor, pulseWidth);
    max30102_setPulseAmplitudeRed(sensor, ledBrightness);
    max30102_setPulseAmplitudeIR(sensor, ledBrightness);
    
    max30102_enableSlot(sensor, 1, SLOT_RED_LED);
    if (ledMode > MODE_REDONLY) {
        max30102_enableSlot(sensor, 2, SLOT_IR_LED);
    }
    
    max30102_setLEDMode(sensor, ledMode);
    
    if (ledMode == MODE_REDONLY) {
        sensor->activeLEDs = 1;
    } else {
        sensor->activeLEDs = 2;
    }
    
    max30102_enableFIFORollover(sensor);
    max30102_resetFIFO(sensor);
}

uint32_t max30102_getRed(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return 0;
    max30102_getNewData(sensor);
    return sensor->senseBuf.red[sensor->senseBuf.head];
}

uint32_t max30102_getIR(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return 0;
    max30102_getNewData(sensor);
    return sensor->senseBuf.IR[sensor->senseBuf.head];
}

void max30102_getNewData(DFRobot_MAX30102* sensor) {
    if (sensor == NULL) return;
    
    int32_t numberOfSamples = 0;
    uint8_t readPointer = 0;
    uint8_t writePointer = 0;
    
    while (1) {
        readPointer = max30102_getReadPointer(sensor);
        writePointer = max30102_getWritePointer(sensor);
        
        if (readPointer == writePointer) {
            // Tidak ada data
            hx_drv_timer_cm55m_sec_delay_ms(100, TIMER_STATE_DC);
            continue;
        }
        
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0) numberOfSamples += 32;
        
        int32_t bytesNeedToRead = numberOfSamples * sensor->activeLEDs * 3;
        
        while (bytesNeedToRead > 0) {
            sensor->senseBuf.head++;
            sensor->senseBuf.head %= MAX30102_SENSE_BUF_SIZE;
            
            uint32_t tempBuf = 0;
            
            if (sensor->activeLEDs > 1) {
                // Baca data Red dan IR (6 bytes)
                uint8_t temp[6];
                readReg(sensor, MAX30102_FIFODATA, temp, 6);
                
                // Byte swap untuk little endian
                for (int i = 0; i < 3; i++) {
                    uint8_t tempex = temp[i];
                    temp[i] = temp[5-i];
                    temp[5-i] = tempex;
                }
                
                // Parse IR (3 bytes pertama setelah swap)
                memcpy(&tempBuf, temp, 3);
                tempBuf &= 0x3FFFF;
                sensor->senseBuf.IR[sensor->senseBuf.head] = tempBuf;
                
                // Parse Red (3 bytes terakhir)
                memcpy(&tempBuf, temp + 3, 3);
                tempBuf &= 0x3FFFF;
                sensor->senseBuf.red[sensor->senseBuf.head] = tempBuf;
            } else {
                // Hanya Red (3 bytes)
                uint8_t temp[3];
                readReg(sensor, MAX30102_FIFODATA, temp, 3);
                
                // Byte swap
                uint8_t tempex = temp[0];
                temp[0] = temp[2];
                temp[2] = tempex;
                
                memcpy(&tempBuf, temp, 3);
                tempBuf &= 0x3FFFF;
                sensor->senseBuf.red[sensor->senseBuf.head] = tempBuf;
            }
            
            bytesNeedToRead -= sensor->activeLEDs * 3;
        }
        return;
    }
}

uint32_t get_raw_data(DFRobot_MAX30102* sensor){

    return 0;
}

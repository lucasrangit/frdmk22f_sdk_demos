/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_MAX9867_H_
#define _FSL_MAX9867_H_

#include "fsl_codec_common.h"

/*!
 * @addtogroup max5000
 * @{
 */


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Define the register address of max5000. */
#define MAX9867_STATUS       0x00
#define MAX9867_JACKSTATUS   0x01
#define MAX9867_AUXHIGH      0x02
#define MAX9867_AUXLOW       0x03
#define MAX9867_INTEN        0x04
#define MAX9867_SYSCLK       0x05
#define MAX9867_FREQ_MASK    0xF
#define MAX9867_PSCLK_SHIFT  0x4
#define MAX9867_PSCLK_WIDTH  0x2
#define MAX9867_PSCLK_MASK   (0x03 << MAX9867_PSCLK_SHIFT)
#define MAX9867_PSCLK_10_20  0x1
#define MAX9867_PSCLK_20_40  0x2
#define MAX9867_PSCLK_40_60  0x3
#define MAX9867_AUDIOCLKHIGH 0x06
#define MAX9867_NI_HIGH_WIDTH 0x7
#define MAX9867_NI_HIGH_MASK 0x7F
#define MAX9867_NI_LOW_MASK  0x7F
#define MAX9867_NI_LOW_SHIFT 0x1
#define MAX9867_PLL          (1<<7)
#define MAX9867_AUDIOCLKLOW  0x07
#define MAX9867_RAPID_LOCK   0x01
#define MAX9867_IFC1A        0x08
#define MAX9867_MASTER       (1<<7)
#define MAX9867_MASTER_CLR_MASK 0x7F
#define MAX9867_WCI_MODE     (1<<6)
#define MAX9867_BCI_MODE     (1<<5)
#define MAX9867_I2S_DLY      (1<<4)
#define MAX9867_I2S_DLY_CLR_MASK 0xEF
#define MAX9867_SDOUT_HIZ    (1<<3)
#define MAX9867_TDM_MODE     (1<<2)
#define MAX9867_IFC1B        0x09
#define MAX9867_IFC1B_BCLK_MASK 7
#define MAX9867_IFC1B_32BIT  0x01
#define MAX9867_IFC1B_24BIT  0x02
#define MAX9867_IFC1B_PCLK_2 4
#define MAX9867_IFC1B_PCLK_4 5
#define MAX9867_IFC1B_PCLK_8 6
#define MAX9867_IFC1B_PCLK_16 7
#define MAX9867_CODECFLTR    0x0a
#define MAX9867_DACGAIN      0x0b
#define MAX9867_DACLEVEL     0x0c
#define MAX9867_DAC_MUTE_SHIFT 0x6
#define MAX9867_DAC_MUTE_WIDTH 0x1
#define MAX9867_DAC_MUTE_MASK (0x1 << MAX9867_DAC_MUTE_SHIFT)
#define MAX9867_ADCLEVEL     0x0d
#define MAX9867_LEFTLINELVL  0x0e
#define MAX9867_RIGTHLINELVL 0x0f
#define MAX9867_LEFTVOL      0x10
#define MAX9867_RIGHTVOL     0x11
#define MAX9867_VOLUME_CLR_MASK (0xC0)
#define MAX9867_VOLUME_MUTE (1<<6)
#define MAX9867_VOLUME_MUTE_CLR_MASK (0xBF)
#define MAX9867_LEFTMICGAIN  0x12
#define MAX9867_RIGHTMICGAIN 0x13
#define MAX9867_INPUTCONFIG  0x14
#define MAX9867_INPUT_SHIFT  0x6
#define MAX9867_MICCONFIG    0x15
#define MAX9867_MODECONFIG   0x16
#define MAX9867_PWRMAN       0x17
#define MAX9867_SHTDOWN_MASK (1<<7)
#define MAX9867_REVISION     0xff

/*! @brief MAX9867 I2C address. */
#define MAX9867_I2C_ADDR 0x18 // 7-bit address (0x30/0x31 W/R 8-bit address)

/*! @brief Modules in Max5000 board. */
typedef enum _max5000_module
{
    kMAX_ModuleADC = 0x0, /*!< ADC module in MAX9867 */
    kMAX_ModuleDAC,       /*!< DAC module in MAX9867 */
    kMAX_ModuleDAP,       /*!< DAP module in MAX9867 */
    kMAX_ModuleHP,        /*!< Headphone module in MAX9867 */
    kMAX_ModuleI2SIN,     /*!< I2S-IN module in MAX9867 */
    kMAX_ModuleI2SOUT,    /*!< I2S-OUT module in MAX9867 */
    kMAX_ModuleLineIn,    /*!< Line-in moudle in MAX9867 */
    kMAX_ModuleLineOut,   /*!< Line-out module in MAX9867 */
    kMAX_ModuleMicin      /*!< Micphone module in MAX9867 */
} max_module_t;

/*!
* @brief Max5000 data route.
* @note Only provide some typical data route, not all route listed.
* Users cannot combine any routes, once a new route is set, the precios one would be replaced.
*/
typedef enum _max_route
{
    kMAX_RouteBypass = 0x0,             /*!< LINEIN->Headphone. */
    kMAX_RoutePlayback,                 /*!< I2SIN->DAC->Headphone. */
    kMAX_RoutePlaybackandRecord,        /*!< I2SIN->DAC->Headphone, LINEIN->ADC->I2SOUT. */
    kMAX_RoutePlaybackwithDAP,          /*!< I2SIN->DAP->DAC->Headphone. */
    kMAX_RoutePlaybackwithDAPandRecord, /*!< I2SIN->DAP->DAC->HP, LINEIN->ADC->I2SOUT. */
    kMAX_RouteRecord                    /*!< LINEIN->ADC->I2SOUT. */
} max_route_t;

/*!
* @brief The audio data transfer protocol choice.
* Max5000 only supports I2S format and PCM format.
*/
typedef enum _max_protocol
{
    kMAX_BusI2S = 0x0,      /*!< I2S Type */
    kMAX_BusLeftJustified,  /*!< Left justified */
    kMAX_BusRightJustified, /*!< Right Justified */
    kMAX_BusPCMA,           /*!< PCMA */
    kMAX_BusPCMB            /*!< PCMB */
} max_protocol_t;

/*! @brief Initailize structure of max5000 */
typedef struct _max_config
{
    max_route_t route;  /*!< Audio data route.*/
    max_protocol_t bus; /*!< Audio transfer protocol */
    bool master_slave;   /*!< Master or slave. True means master, false means slave. */
} max_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief max5000 initialize function.
 *
 * This function calls MAX_I2CInit(), and in this function, some configurations
 * are fixed. The second parameter can be NULL. If users want to change the MAX9867 settings,
 * a configure structure should be prepared.
 * @note If the codec_config is NULL, it would initialize max5000 using default settings.
 * The default setting:
 * @code
 * max_init_t codec_config
 * codec_config.route = kMAX_RoutePlaybackandRecord
 * codec_config.bus = kMAX_BusI2S
 * codec_config.master = slave
 * @endcode
 *
 * @param handle Max5000 handle structure.
 * @param codec_config max5000 configuration structure. If this pointer equals to NULL,
 * it means using the default configuration.
 * @return Initialization status
 */
status_t MAX_Init(codec_handle_t *handle, void *config);

/*!
 * @brief Set audio data route in max5000.
 *
 * This function would set the data route according to route. The route cannot be combined,
 * as all route would enable different modules.
 *
 * @note If a new route is set, the previous route would not work.
 * @param handle Max5000 handle structure.
 * @param route Audio data route in max5000.
 */
status_t MAX_SetDataRoute(codec_handle_t *handle, max_route_t route);

/*!
 * @brief Set the audio transfer protocol.
 *
 * Max5000 only supports I2S, I2S left, I2S right, PCM A, PCM B format.
 * @param handle Max5000 handle structure.
 * @param bus Audio data transfer protocol.
 */
status_t MAX_SetProtocol(codec_handle_t *handle, max_protocol_t protocol);

/*!
 * @brief Set max5000 as master or slave.
 *
 * @param handle Max5000 handle structure.
 * @param master 1 represent master, 0 represent slave.
 */
void MAX_SetMasterSlave(codec_handle_t *handle, bool master);

/*!
 * @brief Set the volume of different modules in max5000.
 *
 * This function would set the volume of max5000 modules. This interface set module volume.
 * The function assume that left channel and right channel has the same volume.
 * @param handle Max5000 handle structure.
 * @param module Max5000 module, such as DAC, ADC and etc.
 * @param volume Volume value need to be set. The value is the exact value in register.
 */
status_t MAX_SetVolume(codec_handle_t *handle, max_module_t module, uint32_t volume);

/*!
 * @brief Get the volume of different modules in max5000.
 *
 * This function gets the volume of max5000 modules. This interface get DAC module volume.
 * The function assume that left channel and right channel has the same volume.
 * @param handle Max5000 handle structure.
 * @param module Max5000 module, such as DAC, ADC and etc.
 * @return Module value, the value is exact value in register.
 */
uint32_t MAX_GetVolume(codec_handle_t *handle, max_module_t module);

/*!
 * @brief Mute/unmute modules in max5000.
 *
 * @param handle Max5000 handle structure.
 * @param module Max5000 module, such as DAC, ADC and etc.
 * @param mute True means mute, and false means unmute.
 */
status_t MAX_SetMute(codec_handle_t *handle, max_module_t module, bool mute);

/*!
 * @brief Enable expected devices.
 * @param handle Max5000 handle structure.
 * @param module Module expected to enable.
 */
status_t MAX_EnableModule(codec_handle_t *handle, max_module_t module);

/*!
 * @brief Disable expected devices.
 * @param handle Max5000 handle structure.
 * @param module Module expected to enable.
 */
status_t MAX_DisableModule(codec_handle_t *handle, max_module_t module);

/*!
 * @brief Deinit the max5000 codec. Shut down Max5000 modules.
 * @param handle Max5000 handle structure pointer.
 */
status_t MAX_Deinit(codec_handle_t *handle);

/*!
 * @brief Configure the data format of audio data.
 *
 * This function would configure the registers about the sample rate, bit depths.
 * @param handle Max5000 handle structure pointer.
 * @param mclk Master clock frequency of I2S.
 * @param sample_rate Sample rate of audio file running in max5000. Max5000 now
 * supports 8k, 11.025k, 12k, 16k, 22.05k, 24k, 32k, 44.1k, 48k and 96k sample rate.
 * @param bits Bit depth of audio file (Max5000 only supports 16bit, 20bit, 24bit
 * and 32 bit in HW).
 */
status_t MAX_ConfigDataFormat(codec_handle_t *handle, uint32_t mclk, uint32_t sample_rate, uint32_t bits);

/*!
 * @brief Write register to max using I2C.
 * @param handle Max5000 handle structure.
 * @param reg The register address in max.
 * @param val Value needs to write into the register.
 */
status_t MAX_WriteReg(codec_handle_t *handle, uint8_t reg, uint8_t val);

/*!
 * @brief Read register from max using I2C.
 * @param handle Max5000 handle structure.
 * @param reg The register address in max.
 * @param val Value written to.
 */
status_t MAX_ReadReg(codec_handle_t *handle, uint8_t reg, uint8_t *val);

/*!
 * @brief Modify some bits in the register using I2C.
 * @param handle Max5000 handle structure.
 * @param reg The register address in max.
 * @param mask The mask code for the bits want to write. The bit you want to write should be 0.
 * @param val Value needs to write into the register.
 */
status_t MAX_ModifyReg(codec_handle_t *handle, uint8_t reg, uint8_t clr_mask, uint8_t val);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _FSL_MAX9867_H_ */

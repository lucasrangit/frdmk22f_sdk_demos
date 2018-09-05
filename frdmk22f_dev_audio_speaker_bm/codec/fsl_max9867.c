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

#include "fsl_debug_console.h"
#include "fsl_max9867.h"

/*******************************************************************************
 * Definitations
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t MAX_Init(codec_handle_t *handle, void *codec_config)
{
    max_config_t *config = (max_config_t *)codec_config;
    uint8_t rev = 0;

    handle->slaveAddress = MAX9867_I2C_ADDR;

    // MAX9867 revision should be 0x42
    MAX_ReadReg(handle, MAX9867_REVISION, &rev);
    PRINTF("MAX9867 revision 0x%02x\n", rev);

    /* PSCLK 01 (MCLK is between 10 and 20 MHz) */
    MAX_WriteReg(handle, MAX9867_SYSCLK, 0x10);

    /* PLL Mode 0 (LRCLK 48 kHz */
    MAX_WriteReg(handle, MAX9867_AUDIOCLKHIGH, 0x60);
    MAX_WriteReg(handle, MAX9867_AUDIOCLKLOW, 0x00);

    /* Set max9867 to master or slave */
    //    MAX_WriteReg(handle, MAX9867_IFC1A, 0x00);
    MAX_ModifyReg(handle, MAX9867_IFC1A, MAX9867_MASTER_CLR_MASK, config->master_slave ? MAX9867_MASTER : 0);
    MAX_WriteReg(handle, MAX9867_IFC1B, 0x00);

    /* SDIN/SDOUT data is delayed one BCLK cycle */
    //MAX_ModifyReg(handle, MAX9867_IFC1A, MAX9867_I2S_DLY_CLR_MASK, MAX9867_I2S_DLY);

    /* FIR Audio Filter, DAC Filter ? */
    MAX_WriteReg(handle, MAX9867_CODECFLTR, 0x00); // 0x91

    MAX_WriteReg(handle, MAX9867_DACGAIN, 0x00);

    /* DAG Gain +12dB */
    MAX_WriteReg(handle, MAX9867_DACLEVEL, 0x00); // 0x30
    MAX_WriteReg(handle, MAX9867_ADCLEVEL, 0x33);
    MAX_WriteReg(handle, MAX9867_LEFTLINELVL, 0x0c);
    MAX_WriteReg(handle, MAX9867_RIGTHLINELVL, 0x0c);

    MAX_WriteReg(handle, MAX9867_LEFTVOL, 0x09); // 0x06
    MAX_WriteReg(handle, MAX9867_RIGHTVOL, 0x09); // 0x06

    /* Enable mics */
    MAX_ModifyReg(handle, MAX9867_LEFTMICGAIN, 0xFFU, 0x34);
    MAX_ModifyReg(handle, MAX9867_RIGHTMICGAIN, 0xFFU, 0x34);

    MAX_ModifyReg(handle, MAX9867_INPUTCONFIG, 0xf0, 0xf0);

    MAX_WriteReg(handle, MAX9867_MICCONFIG, 0x00); // 0x30

    MAX_WriteReg(handle, MAX9867_MODECONFIG, 0x00);

    /* Toggle /SHDN bit whenever a configuration change is made */
    MAX_WriteReg(handle, MAX9867_PWRMAN, 0x0F);

    /* Power up Inputs/Outputs/Digital Blocks
     * Power up LINEOUT, HP, ADC, DAC. */
    MAX_WriteReg(handle, MAX9867_PWRMAN, 0x8f);

#if 0    /* FIXME from fsl_sgtl5000.c */
    /* NULL pointer means default setting. */
    if (config == NULL)
    {
        /* Power up Inputs/Outputs/Digital Blocks
           Power up LINEOUT, HP, ADC, DAC. */
        MAX_WriteReg(handle, CHIP_ANA_POWER, 0x6AFFU);

        /* Power up desired digital blocks.
        I2S_IN (bit 0), I2S_OUT (bit 1), DAP (bit 4), DAC (bit 5), ADC (bit 6) are powered on */
        MAX_WriteReg(handle, CHIP_DIG_POWER, 0x0063U);

        /* Configure SYS_FS clock to 48kHz, MCLK_FREQ to 256*Fs. */
        MAX_ModifyReg(handle, CHIP_CLK_CTRL, 0xFFC8U, 0x0008U);

        /* Configure the I2S clocks in slave mode.
           I2S LRCLK is same as the system sample clock.
           Data length = 16 bits. */
        MAX_WriteReg(handle, CHIP_I2S_CTRL, 0x0170U);

        /* I2S_IN -> DAC -> HP_OUT, Route I2S_IN to DAC */
        MAX_ModifyReg(handle, CHIP_SSS_CTRL, 0xFFDFU, 0x0010U);

        /* Select DAC as the input to HP_OUT */
        MAX_ModifyReg(handle, CHIP_ANA_CTRL, 0xFFBFU, 0x0000U);

        /* LINE_IN -> ADC -> I2S_OUT. Set ADC input to LINE_IN. */
        MAX_ModifyReg(handle, CHIP_ANA_CTRL, 0xFFFFU, 0x0004U);

        /* Route ADC to I2S_OUT */
        MAX_ModifyReg(handle, CHIP_SSS_CTRL, 0xFFFCU, 0x0000U);

        /* Default using I2S left format. */
        MAX_SetProtocol(handle, kMAX_BusI2S);
    }
    else
    {
        MAX_WriteReg(handle, CHIP_ANA_POWER, 0x6AFF);

        /* Set the data route */
        MAX_SetDataRoute(handle, config->route);

        /* Set the audio format */
        MAX_SetProtocol(handle, config->bus);
    }

    /* Input Volume Control
    Configure ADC left and right analog volume to desired default.
    Example shows volume of 0dB. */
    MAX_WriteReg(handle, CHIP_ANA_ADC_CTRL, 0x0000U);

    /* Volume and Mute Control
       Configure HP_OUT left and right volume to minimum, unmute.
       HP_OUT and ramp the volume up to desired volume.*/
    MAX_WriteReg(handle, CHIP_ANA_HP_CTRL, 0x1818U);
    MAX_ModifyReg(handle, CHIP_ANA_CTRL, 0xFFEFU, 0x0000U);

    /* LINEOUT and DAC volume control */
    MAX_ModifyReg(handle, CHIP_ANA_CTRL, 0xFEFFU, 0x0000U);

    /* Configure DAC left and right digital volume */
    MAX_WriteReg(handle, CHIP_DAC_VOL, 0x5C5CU);

    /* Configure ADC volume, reduce 6db. */
    MAX_WriteReg(handle, CHIP_ANA_ADC_CTRL, 0x0100U);

    /* Unmute DAC */
    MAX_ModifyReg(handle, CHIP_ADCDAC_CTRL, 0xFFFBU, 0x0000U);
    MAX_ModifyReg(handle, CHIP_ADCDAC_CTRL, 0xFFF7U, 0x0000U);

    /* Unmute ADC */
    MAX_ModifyReg(handle, CHIP_ANA_CTRL, 0xFFFEU, 0x0000U);
#endif

    /* Delay for some seconds */
    for (int i = 0; i < 3000000; i++)
        __ASM("nop");

#if defined(DEBUG)
    PRINTF("Configuration:\n");
    for (int i = 0; i <= 0x17; i++) {
        uint8_t val;
        MAX_ReadReg(handle, i, &val);
        PRINTF("0x%02x: 0x%02x\n", i, val);
    }
#endif

    return kStatus_Success;
}

status_t MAX_Deinit(codec_handle_t *handle)
{
// TODO
//    MAX_DisableModule(handle, kMAX_ModuleADC);
//    MAX_DisableModule(handle, kMAX_ModuleDAC);
//    MAX_DisableModule(handle, kMAX_ModuleDAP);
//    MAX_DisableModule(handle, kMAX_ModuleI2SIN);
//    MAX_DisableModule(handle, kMAX_ModuleI2SOUT);
//    MAX_DisableModule(handle, kMAX_ModuleLineOut);

    return kStatus_Success;
}

#if 0 /* TODO */
status_t MAX_EnableModule(codec_handle_t *handle, max_module_t module)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kMAX_ModuleADC:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_ADC_ENABLE_CLR_MASK,
                           ((uint8_t)1U << MAX9867_ADC_ENABLE_SHIFT));
            MAX_ModifyReg(handle, CHIP_ANA_POWER, MAX9867_ADC_POWERUP_CLR_MASK,
                           ((uint8_t)1U << MAX9867_ADC_POWERUP_SHIFT));
            break;
        case kMAX_ModuleDAC:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_DAC_ENABLE_CLR_MASK,
                           ((uint8_t)1U << MAX9867_DAC_ENABLE_SHIFT));
            MAX_ModifyReg(handle, CHIP_ANA_POWER, MAX9867_DAC_POWERUP_CLR_MASK,
                           ((uint8_t)1U << MAX9867_DAC_POWERUP_SHIFT));
            break;
        case kMAX_ModuleDAP:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_DAP_ENABLE_CLR_MASK,
                           ((uint8_t)1U << MAX9867_DAP_ENABLE_SHIFT));
            MAX_ModifyReg(handle, MAX9867_DAP_CONTROL, MAX9867_DAP_CONTROL_DAP_EN_CLR_MASK,
                           ((uint8_t)1U << MAX9867_DAP_CONTROL_DAP_EN_SHIFT));
            break;
        case kMAX_ModuleI2SIN:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_I2S_IN_ENABLE_CLR_MASK,
                           ((uint8_t)1U << MAX9867_I2S_IN_ENABLE_SHIFT));
            break;
        case kMAX_ModuleI2SOUT:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_I2S_OUT_ENABLE_CLR_MASK,
                           ((uint8_t)1U << MAX9867_I2S_OUT_ENABLE_SHIFT));
            break;
        case kMAX_ModuleHP:
            MAX_ModifyReg(handle, CHIP_ANA_POWER, MAX9867_HEADPHONE_POWERUP_CLR_MASK,
                           ((uint8_t)1U << MAX9867_HEADPHONE_POWERUP_SHIFT));
            break;
        case kMAX_ModuleLineOut:
            MAX_ModifyReg(handle, CHIP_ANA_POWER, MAX9867_LINEOUT_POWERUP_CLR_MASK,
                           ((uint8_t)1U << MAX9867_LINEOUT_POWERUP_SHIFT));
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t MAX_DisableModule(codec_handle_t *handle, max_module_t module)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kMAX_ModuleADC:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_ADC_ENABLE_CLR_MASK,
                           ((uint8_t)0U << MAX9867_ADC_ENABLE_SHIFT));
            MAX_ModifyReg(handle, CHIP_ANA_POWER, MAX9867_ADC_POWERUP_CLR_MASK,
                           ((uint8_t)0U << MAX9867_ADC_POWERUP_SHIFT));
            break;
        case kMAX_ModuleDAC:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_DAC_ENABLE_CLR_MASK,
                           ((uint8_t)0U << MAX9867_DAC_ENABLE_SHIFT));
            MAX_ModifyReg(handle, CHIP_ANA_POWER, MAX9867_DAC_POWERUP_CLR_MASK,
                           ((uint8_t)0U << MAX9867_DAC_POWERUP_SHIFT));
            break;
        case kMAX_ModuleDAP:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_DAP_ENABLE_CLR_MASK,
                           ((uint8_t)0U << MAX9867_DAP_ENABLE_SHIFT));
            MAX_ModifyReg(handle, MAX9867_DAP_CONTROL, MAX9867_DAP_CONTROL_DAP_EN_CLR_MASK,
                           ((uint8_t)0U << MAX9867_DAP_CONTROL_DAP_EN_SHIFT));
            break;
        case kMAX_ModuleI2SIN:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_I2S_IN_ENABLE_CLR_MASK,
                           ((uint8_t)0U << MAX9867_I2S_IN_ENABLE_SHIFT));
            break;
        case kMAX_ModuleI2SOUT:
            MAX_ModifyReg(handle, CHIP_DIG_POWER, MAX9867_I2S_OUT_ENABLE_CLR_MASK,
                           ((uint8_t)0U << MAX9867_I2S_OUT_ENABLE_SHIFT));
            break;
        case kMAX_ModuleHP:
            MAX_ModifyReg(handle, CHIP_ANA_POWER, MAX9867_HEADPHONE_POWERUP_CLR_MASK,
                           ((uint8_t)0U << MAX9867_HEADPHONE_POWERUP_SHIFT));
            break;
        case kMAX_ModuleLineOut:
            MAX_ModifyReg(handle, CHIP_ANA_POWER, MAX9867_LINEOUT_POWERUP_CLR_MASK,
                           ((uint8_t)0U << MAX9867_LINEOUT_POWERUP_SHIFT));
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t MAX_SetDataRoute(codec_handle_t *handle, max_route_t route)
{
    status_t ret = kStatus_Success;
    switch (route)
    {
        case kMAX_RouteBypass:
            /* Bypass means from line-in to HP*/
            MAX_WriteReg(handle, CHIP_DIG_POWER, 0x0000);
            MAX_EnableModule(handle, kMAX_ModuleHP);
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_SEL_HP_CLR_MASK, MAX9867_SEL_HP_LINEIN);
            break;
        case kMAX_RoutePlayback:
            /* Data route I2S_IN-> DAC-> HP */
            MAX_EnableModule(handle, kMAX_ModuleHP);
            MAX_EnableModule(handle, kMAX_ModuleDAC);
            MAX_EnableModule(handle, kMAX_ModuleI2SIN);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_DAC_SEL_CLR_MASK, MAX9867_DAC_SEL_I2S_IN);
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_SEL_HP_CLR_MASK, MAX9867_SEL_HP_DAC);
            break;
        case kMAX_RoutePlaybackandRecord:
            /* I2S IN->DAC->HP  LINE_IN->ADC->I2S_OUT */
            MAX_EnableModule(handle, kMAX_ModuleHP);
            MAX_EnableModule(handle, kMAX_ModuleDAC);
            MAX_EnableModule(handle, kMAX_ModuleI2SIN);
            MAX_EnableModule(handle, kMAX_ModuleI2SOUT);
            MAX_EnableModule(handle, kMAX_ModuleADC);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_DAC_SEL_CLR_MASK, MAX9867_DAC_SEL_I2S_IN);
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_SEL_HP_CLR_MASK, MAX9867_SEL_HP_DAC);
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_SEL_ADC_CLR_MASK, MAX9867_SEL_ADC_LINEIN);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_I2S_OUT_SEL_CLR_MASK, MAX9867_I2S_OUT_SEL_ADC);
            break;
        case kMAX_RoutePlaybackwithDAP:
            /* I2S_IN->DAP->DAC->HP */
            MAX_EnableModule(handle, kMAX_ModuleHP);
            MAX_EnableModule(handle, kMAX_ModuleDAC);
            MAX_EnableModule(handle, kMAX_ModuleI2SIN);
            MAX_EnableModule(handle, kMAX_ModuleDAP);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_DAP_SEL_CLR_MASK, MAX9867_DAP_SEL_I2S_IN);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_DAC_SEL_CLR_MASK, MAX9867_DAC_SEL_DAP);
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_SEL_HP_CLR_MASK, MAX9867_SEL_HP_DAC);
            break;
        case kMAX_RoutePlaybackwithDAPandRecord:
            /* I2S_IN->DAP->DAC->HP,  LINE_IN->ADC->I2S_OUT */
            MAX_EnableModule(handle, kMAX_ModuleHP);
            MAX_EnableModule(handle, kMAX_ModuleDAC);
            MAX_EnableModule(handle, kMAX_ModuleI2SIN);
            MAX_EnableModule(handle, kMAX_ModuleI2SOUT);
            MAX_EnableModule(handle, kMAX_ModuleADC);
            MAX_EnableModule(handle, kMAX_ModuleDAP);
            MAX_ModifyReg(handle, MAX9867_DAP_CONTROL, MAX9867_DAP_CONTROL_DAP_EN_CLR_MASK, 0x0001);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_DAP_SEL_CLR_MASK, MAX9867_DAP_SEL_I2S_IN);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_DAC_SEL_CLR_MASK, MAX9867_DAC_SEL_DAP);
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_SEL_HP_CLR_MASK, MAX9867_SEL_HP_DAC);
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_SEL_ADC_CLR_MASK, MAX9867_SEL_ADC_LINEIN);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_I2S_OUT_SEL_CLR_MASK, MAX9867_I2S_OUT_SEL_ADC);
            break;
        case kMAX_RouteRecord:
            /* LINE_IN->ADC->I2S_OUT */
            MAX_EnableModule(handle, kMAX_ModuleI2SOUT);
            MAX_EnableModule(handle, kMAX_ModuleADC);
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_SEL_ADC_CLR_MASK, MAX9867_SEL_ADC_LINEIN);
            MAX_ModifyReg(handle, CHIP_SSS_CTRL, MAX9867_I2S_OUT_SEL_CLR_MASK, MAX9867_I2S_OUT_SEL_ADC);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}
#endif

status_t MAX_SetProtocol(codec_handle_t *handle, max_protocol_t protocol)
{
    status_t ret = kStatus_Success;
    switch (protocol)
    {
        case kMAX_BusI2S:
            MAX_ModifyReg(handle, MAX9867_IFC1A, MAX9867_I2S_DLY, 0x10);
//            MAX_ModifyReg(handle, MAX9867_IFC1B, 0x20, 0); // FIXME there's no register at bit 5
            break;
        case kMAX_BusLeftJustified:
            MAX_ModifyReg(handle, MAX9867_IFC1A, MAX9867_I2S_DLY, 0x00);
//            MAX_ModifyReg(handle, MAX9867_IFC1B, 0x20, 0); // FIXME there's no register at bit 5
            break;
        case kMAX_BusRightJustified:
        case kMAX_BusPCMA:
        case kMAX_BusPCMB:
#if 0 /* FIXME from sgtl5000.c */
        case kMAX_BusI2S:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_MODE_CLR_MASK, MAX9867_I2S_MODE_I2S_LJ);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_LRALIGN_CLR_MASK, MAX9867_I2S_ONE_BIT_DELAY);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_SCLK_INV_CLR_MASK, MAX9867_I2S_VAILD_RISING_EDGE);
            break;
        case kMAX_BusLeftJustified:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_MODE_CLR_MASK, MAX9867_I2S_MODE_I2S_LJ);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_LRALIGN_CLR_MASK, MAX9867_I2S_NO_DELAY);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_SCLK_INV_CLR_MASK, MAX9867_I2S_VAILD_RISING_EDGE);
            break;
        case kMAX_BusRightJustified:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_MODE_CLR_MASK, MAX9867_I2S_MODE_RJ);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_SCLK_INV_CLR_MASK, MAX9867_I2S_VAILD_RISING_EDGE);
            break;
        case kMAX_BusPCMA:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_MODE_CLR_MASK, MAX9867_I2S_MODE_PCM);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_LRALIGN_CLR_MASK, MAX9867_I2S_ONE_BIT_DELAY);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_SCLK_INV_CLR_MASK, MAX9867_I2S_VAILD_FALLING_EDGE);
            break;
        case kMAX_BusPCMB:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_MODE_CLR_MASK, MAX9867_I2S_MODE_PCM);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_LRALIGN_CLR_MASK, MAX9867_I2S_NO_DELAY);
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_SCLK_INV_CLR_MASK, MAX9867_I2S_VAILD_FALLING_EDGE);
            break;
#endif
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t MAX_SetVolume(codec_handle_t *handle, max_module_t module, uint32_t volume)
{
    uint8_t vol = 0;
    status_t ret = kStatus_Success;

    ret = MAX_ModifyReg(handle, MAX9867_LEFTVOL, MAX9867_VOLUME_CLR_MASK, vol);
    ret = MAX_ModifyReg(handle, MAX9867_RIGHTVOL, MAX9867_VOLUME_CLR_MASK, vol);

#if 0 /* FIXME from sgtl5000.c */
    switch (module)
    {
        case kMAX_ModuleADC:
            vol = volume | (volume << 4U);
            ret = MAX_ModifyReg(handle, CHIP_ANA_ADC_CTRL,
                                 MAX9867_ADC_VOL_LEFT_CLR_MASK & MAX9867_ADC_VOL_RIGHT_CLR_MASK, vol);
            break;
        case kMAX_ModuleDAC:
            vol = volume | (volume << 8U);
            ret = MAX_WriteReg(handle, CHIP_DAC_VOL, vol);
            break;
        case kMAX_ModuleHP:
            vol = volume | (volume << 8U);
            ret = MAX_WriteReg(handle, CHIP_ANA_HP_CTRL, vol);
            break;
        case kMAX_ModuleLineOut:
            vol = volume | (volume << 8U);
            ret = MAX_WriteReg(handle, CHIP_LINE_OUT_VOL, vol);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
#endif

    return ret;
}

#if 0 /* TODO */
uint32_t MAX_GetVolume(codec_handle_t *handle, max_module_t module)
{
    uint8_t vol = 0;
    switch (module)
    {
        case kMAX_ModuleADC:
            MAX_ReadReg(handle, CHIP_ANA_ADC_CTRL, &vol);
            vol = (vol & (uint8_t)MAX9867_ADC_VOL_LEFT_GET_MASK) >> MAX9867_ADC_VOL_LEFT_SHIFT;
            break;
        case kMAX_ModuleDAC:
            MAX_ReadReg(handle, CHIP_DAC_VOL, &vol);
            vol = (vol & (uint8_t)MAX9867_DAC_VOL_LEFT_GET_MASK) >> MAX9867_DAC_VOL_LEFT_SHIFT;
            break;
        case kMAX_ModuleHP:
            MAX_ReadReg(handle, CHIP_ANA_HP_CTRL, &vol);
            vol = (vol & (uint8_t)MAX9867_HP_VOL_LEFT_GET_MASK) >> MAX9867_HP_VOL_LEFT_SHIFT;
            break;
        case kMAX_ModuleLineOut:
            MAX_ReadReg(handle, CHIP_LINE_OUT_VOL, &vol);
            vol = (vol & (uint8_t)MAX9867_LINE_OUT_VOL_LEFT_GET_MASK) >> MAX9867_LINE_OUT_VOL_LEFT_SHIFT;
            break;
        default:
            vol = 0;
            break;
    }
    return vol;
}
#endif

status_t MAX_SetMute(codec_handle_t *handle, max_module_t module, bool mute)
{
    status_t ret = kStatus_Success;

    /* MAX9867 incorporates volume and mute control to allow level control for the playback audio path. */
    MAX_ModifyReg(handle, MAX9867_LEFTVOL, MAX9867_VOLUME_MUTE_CLR_MASK, mute ? MAX9867_VOLUME_MUTE : 0);
    MAX_ModifyReg(handle, MAX9867_RIGHTVOL, MAX9867_VOLUME_MUTE_CLR_MASK, mute ? MAX9867_VOLUME_MUTE : 0);

#if 0 /* FIXME from sgtl5000.c */
        case kMAX_ModuleADC:
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_MUTE_ADC_CLR_MASK, mute);
            break;
        case kMAX_ModuleDAC:
            if (mute)
            {
                MAX_ModifyReg(handle, CHIP_ADCDAC_CTRL,
                               MAX9867_DAC_MUTE_LEFT_CLR_MASK & MAX9867_DAC_MUTE_RIGHT_CLR_MASK, 0x000C);
            }
            else
            {
                MAX_ModifyReg(handle, CHIP_ADCDAC_CTRL,
                               MAX9867_DAC_MUTE_LEFT_CLR_MASK & MAX9867_DAC_MUTE_RIGHT_CLR_MASK, 0x0000);
            }
            break;
        case kMAX_ModuleHP:
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_MUTE_HP_CLR_MASK,
                           ((uint8_t)mute << MAX9867_MUTE_HP_SHIFT));
            break;
        case kMAX_ModuleLineOut:
            MAX_ModifyReg(handle, CHIP_ANA_CTRL, MAX9867_MUTE_LO_CLR_MASK,
                           ((uint8_t)mute << MAX9867_MUTE_LO_SHIFT));
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
#endif

    return ret;
}

status_t MAX_ConfigDataFormat(codec_handle_t *handle, uint32_t mclk, uint32_t sample_rate, uint32_t bits)
{
    status_t retval = kStatus_Success;
#if 0 // TODO
    uint8_t val = 0;
    uint8_t regVal = 0;
    uint8_t mul_clk = 0U;
    uint32_t sysFs = 0U;

    regVal = 0x1b; // MCLK between 10 and 20 MHz, PCLK=13 MHz, LRCLK = 16KHz
    MAX_WriteReg(handle, 0x05, regVal);

    MAX_WriteReg(handle, 0x06, 0);
    MAX_WriteReg(handle, 0x07, 0);
#endif

#if 0 /* FIXME from sgtl5000.c */
    /* Over sample rate can only up to 512, the least to 8k */
    if ((mclk / (MIN(sample_rate * 6U, 96000U)) > 512U) || (mclk / sample_rate < 256U))
    {
        return kStatus_InvalidArgument;
    }

    /* Configure the sample rate */
    switch (sample_rate)
    {
        case 8000:
            if (mclk > 32000U * 512U)
            {
                val = 0x0038;
                sysFs = 48000;
            }
            else
            {
                val = 0x0020;
                sysFs = 32000;
            }
            break;
        case 11025:
            val = 0x0024;
            sysFs = 44100;
            break;
        case 12000:
            val = 0x0028;
            sysFs = 48000;
            break;
        case 16000:
            if (mclk > 32000U * 512U)
            {
                val = 0x003C;
                sysFs = 96000;
            }
            else
            {
                val = 0x0010;
                sysFs = 32000;
            }
            break;
        case 22050:
            val = 0x0014;
            sysFs = 44100;
            break;
        case 24000:
            if (mclk > 48000U * 512U)
            {
                val = 0x002C;
                sysFs = 96000;
            }
            else
            {
                val = 0x0018;
                sysFs = 48000;
            }
            break;
        case 32000:
            val = 0x0000;
            sysFs = 32000;
            break;
        case 44100:
            val = 0x0004;
            sysFs = 44100;
            break;
        case 48000:
            if (mclk > 48000U * 512U)
            {
                val = 0x001C;
                sysFs = 96000;
            }
            else
            {
                val = 0x0008;
                sysFs = 48000;
            }
            break;
        case 96000:
            val = 0x000C;
            sysFs = 96000;
            break;
        default:
            retval = kStatus_InvalidArgument;
            break;
    }

    MAX_ReadReg(handle, CHIP_I2S_CTRL, &regVal);

    /* While as slave, Fs is input */
    if ((regVal & MAX9867_I2S_MS_GET_MASK) == 0U)
    {
        sysFs = sample_rate;
    }
    mul_clk = mclk / sysFs;
    /* Configure the mul_clk. Sgtl-5000 only support 256, 384 and 512 oversample rate */
    if ((mul_clk / 128U - 2U) > 2U)
    {
        return kStatus_InvalidArgument;
    }
    else
    {
        val |= (mul_clk / 128U - 2U);
    }
    MAX_WriteReg(handle, CHIP_CLK_CTRL, val);

    /* Data bits configure,sgtl supports 16bit, 20bit 24bit, 32bit */
    MAX_ModifyReg(handle, CHIP_I2S_CTRL, 0xFEFF, MAX9867_I2S_SCLKFREQ_64FS);
    switch (bits)
    {
        case 16:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_DLEN_CLR_MASK, MAX9867_I2S_DLEN_16);
            break;
        case 20:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_DLEN_CLR_MASK, MAX9867_I2S_DLEN_20);
            break;
        case 24:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_DLEN_CLR_MASK, MAX9867_I2S_DLEN_24);
            break;
        case 32:
            MAX_ModifyReg(handle, CHIP_I2S_CTRL, MAX9867_I2S_DLEN_CLR_MASK, MAX9867_I2S_DLEN_32);
            break;
        default:
            retval = kStatus_InvalidArgument;
            break;
    }
#endif

    return retval;
}

status_t MAX_WriteReg(codec_handle_t *handle, uint8_t reg, uint8_t val)
{
    status_t retval = 0;

    retval = CODEC_I2C_WriteReg(handle->slaveAddress, kCODEC_RegAddr8Bit, reg, kCODEC_RegWidth8Bit, val,
                                handle->I2C_SendFunc);

    return retval;
}

status_t MAX_ReadReg(codec_handle_t *handle, uint8_t reg, uint8_t *val)
{
    status_t retval = 0;

    retval = CODEC_I2C_ReadReg(handle->slaveAddress, kCODEC_RegAddr8Bit, reg, kCODEC_RegWidth8Bit, val,
                               handle->I2C_ReceiveFunc);

    return retval;
}

status_t MAX_ModifyReg(codec_handle_t *handle, uint8_t reg, uint8_t clr_mask, uint8_t val)
{
    status_t retval = 0;
    uint8_t reg_val;

    /* Read the register value out */
    retval = MAX_ReadReg(handle, reg, &reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* Modify the value */
    reg_val &= clr_mask;
    reg_val |= val;

    /* Write the data to register */
    retval = MAX_WriteReg(handle, reg, reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

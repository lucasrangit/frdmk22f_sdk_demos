/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
#include "fsl_wm8962.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitations
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*
 * wm8962 register cache
 * We can't read the WM8962 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const uint16_t wm8962_reg[WM8962_CACHEREGNUM] = {
    0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000a, 0x01c0, 0x0000, 0x00ff, 0x00ff, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x007b, 0x0100, 0x0032, 0x0000, 0x00c3, 0x00c3, 0x01c0, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000, 0x0002, 0x0037, 0x004d, 0x0080, 0x0008, 0x0031, 0x0026, 0x00e9,
};

static uint16_t reg_cache[WM8962_CACHEREGNUM];
/*******************************************************************************
 * Code
 ******************************************************************************/

status_t WM8962_Init(codec_handle_t *handle, void *wm8962_config)
{
    wm8962_config_t *config = (wm8962_config_t *)wm8962_config;

    memcpy(reg_cache, wm8962_reg, sizeof(wm8962_reg));

    /* Set WM8962 I2C address */
    handle->slaveAddress = WM8962_I2C_ADDR;

    /* Reset the codec */
    WM8962_WriteReg(handle, WM8962_RESET, 0x00);
    /* Set VMID */
    WM8962_WriteReg(handle, WM8962_POWER1, 0xC0);
    /* ADC and DAC uses same clock */
    WM8962_WriteReg(handle, WM8962_IFACE2, 0x40);
    /* NULL pointer means default setting. */
    if (config == NULL)
    {
        /*
        * VMID=50K, Enable VREF, AINL, AINR, ADCL and ADCR
        * I2S_IN (bit 0), I2S_OUT (bit 1), DAP (bit 4), DAC (bit 5), ADC (bit 6) are powered on
        */
        WM8962_WriteReg(handle, WM8962_POWER1, 0xFE);

        /*
         * Enable DACL, DACR, LOUT1, ROUT1, PLL down
         */
        WM8962_WriteReg(handle, WM8962_POWER2, 0x1E0);

        /*
         * Enable left and right channel input PGA, left and right output mixer
         */
        WM8962_WriteReg(handle, WM8962_POWER3, 0x3C);

        /* Configure SYS_FS clock to 44.1kHz, MCLK_FREQ to 256*Fs, SYSCLK derived from MCLK input */
        WM8962_WriteReg(handle, WM8962_CLOCK1, 0x00);

        /*
         * Audio data length = 16bit, I2S data format
         */
        WM8962_WriteReg(handle, WM8962_IFACE1, 0x02);

        /*
         * LMICBOOST = 0dB, Connect left and right PGA to left and right Input Boost Mixer
         */
        WM8962_WriteReg(handle, WM8962_LINPATH, 0x1B8);
        WM8962_WriteReg(handle, WM8962_RINPATH, 0x178);

        /*
         * Left and right input boost, LIN3BOOST and RIN3BOOST = 0dB
         */
        WM8962_WriteReg(handle, WM8962_INBMIX1, 0x00);
        WM8962_WriteReg(handle, WM8962_INBMIX2, 0x00);

        /*
         * Left DAC and LINPUT3 to left output mixer, LINPUT3 left output mixer volume = 0dB
         */
        WM8962_WriteReg(handle, WM8962_LOUTMIX, 0x100);

        /*
         * Right DAC and RINPUT3 to right output mixer, RINPUT3 right output mixer volume = 0dB
         */
        WM8962_WriteReg(handle, WM8962_ROUTMIX, 0x100);

        WM8962_WriteReg(handle, WM8962_MONOMIX1, 0x00);
        WM8962_WriteReg(handle, WM8962_MONOMIX2, 0x00);
    }
    else
    {
        WM8962_SetDataRoute(handle, config->route);
        WM8962_SetProtocol(handle, config->bus);
        WM8962_SetMasterSlave(handle, config->master_slave);
        WM8962_SetLeftInput(handle, config->leftInputSource);
        WM8962_SetRightInput(handle, config->rightInputSource);
        if (config->enableSpeaker)
        {
            WM8962_SetModule(handle, kWM8962_ModuleSpeaker, true);
        }
    }
    WM8962_WriteReg(handle, WM8962_ADDCTL1, 0x0C0);
    WM8962_WriteReg(handle, WM8962_ADDCTL4, 0x40);

    WM8962_WriteReg(handle, WM8962_BYPASS1, 0x0);
    WM8962_WriteReg(handle, WM8962_BYPASS2, 0x0);
    /*
     * ADC volume, 0dB
     */
    WM8962_WriteReg(handle, WM8962_LADC, 0x1C3);
    WM8962_WriteReg(handle, WM8962_RADC, 0x1C3);

    /*
     * Digital DAC volume, 0dB
     */
    WM8962_WriteReg(handle, WM8962_LDAC, 0x1E0);
    WM8962_WriteReg(handle, WM8962_RDAC, 0x1E0);

    /*
     * Headphone volume, LOUT1 and ROUT1, 0dB
     */
    WM8962_WriteReg(handle, WM8962_LOUT1, 0x16F);
    WM8962_WriteReg(handle, WM8962_ROUT1, 0x16F);

    /* Unmute DAC. */
    WM8962_WriteReg(handle, WM8962_DACCTL1, 0x0000);
    WM8962_WriteReg(handle, WM8962_LINVOL, 0x117);
    WM8962_WriteReg(handle, WM8962_RINVOL, 0x117);

    return kStatus_Success;
}

status_t WM8962_Deinit(codec_handle_t *handle)
{
    WM8962_SetModule(handle, kWM8962_ModuleADC, false);
    WM8962_SetModule(handle, kWM8962_ModuleDAC, false);
    WM8962_SetModule(handle, kWM8962_ModuleVREF, false);
    WM8962_SetModule(handle, kWM8962_ModuleLineIn, false);
    WM8962_SetModule(handle, kWM8962_ModuleLineOut, false);
    WM8962_SetModule(handle, kWM8962_ModuleSpeaker, false);

    return kStatus_Success;
}

void WM8962_SetMasterSlave(codec_handle_t *handle, bool master)
{
    if (master == 1)
    {
        WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_MS_MASK, WM8962_IFACE1_MS(WM8962_IFACE1_MASTER));
    }
    else
    {
        WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_MS_MASK, WM8962_IFACE1_MS(WM8962_IFACE1_SLAVE));
    }
}

status_t WM8962_SetModule(codec_handle_t *handle, wm8962_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kWM8962_ModuleADC:
            WM8962_ModifyReg(handle, WM8962_POWER1, WM8962_POWER1_ADCL_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER1_ADCL_SHIFT));
            WM8962_ModifyReg(handle, WM8962_POWER1, WM8962_POWER1_ADCR_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER1_ADCR_SHIFT));
            break;
        case kWM8962_ModuleDAC:
            WM8962_ModifyReg(handle, WM8962_POWER2, WM8962_POWER2_DACL_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER2_DACL_SHIFT));
            WM8962_ModifyReg(handle, WM8962_POWER2, WM8962_POWER2_DACR_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER2_DACR_SHIFT));
            break;
        case kWM8962_ModuleVREF:
            WM8962_ModifyReg(handle, WM8962_POWER1, WM8962_POWER1_VREF_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER1_VREF_SHIFT));
            break;
        case kWM8962_ModuleLineIn:
            WM8962_ModifyReg(handle, WM8962_POWER1, WM8962_POWER1_AINL_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER1_AINL_SHIFT));
            WM8962_ModifyReg(handle, WM8962_POWER1, WM8962_POWER1_AINR_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER1_AINR_SHIFT));
            break;
        case kWM8962_ModuleLineOut:
            WM8962_ModifyReg(handle, WM8962_POWER2, WM8962_POWER2_LOUT1_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER2_LOUT1_SHIFT));
            WM8962_ModifyReg(handle, WM8962_POWER2, WM8962_POWER2_ROUT1_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER2_ROUT1_SHIFT));
            break;
        case kWM8962_ModuleMICB:
            WM8962_ModifyReg(handle, WM8962_POWER1, WM8962_POWER1_MICB_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER1_MICB_SHIFT));
            break;
        case kWM8962_ModuleSpeaker:
            WM8962_ModifyReg(handle, WM8962_POWER2, WM8962_POWER2_SPKL_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER2_SPKL_SHIFT));
            WM8962_ModifyReg(handle, WM8962_POWER2, WM8962_POWER2_SPKR_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER2_SPKR_SHIFT));
            WM8962_WriteReg(handle, WM8962_CLASSD1, 0xF7);
            break;
        case kWM8962_ModuleMIC:
            WM8962_ModifyReg(handle, WM8962_POWER3, WM8962_POWER3_LMIC_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER3_LMIC_SHIFT));
            WM8962_ModifyReg(handle, WM8962_POWER3, WM8962_POWER3_RMIC_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER3_RMIC_SHIFT));
            break;
        case kWM8962_ModuleOMIX:
            WM8962_ModifyReg(handle, WM8962_POWER3, WM8962_POWER3_LOMIX_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER3_LOMIX_SHIFT));
            WM8962_ModifyReg(handle, WM8962_POWER3, WM8962_POWER3_ROMIX_MASK,
                             ((uint16_t)isEnabled << WM8962_POWER3_ROMIX_SHIFT));
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t WM8962_SetDataRoute(codec_handle_t *handle, wm8962_route_t route)
{
    status_t ret = kStatus_Success;
    switch (route)
    {
        case kWM8962_RouteBypass:
            /* Bypass means from line-in to HP*/
            /*
             * Left LINPUT3 to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            WM8962_WriteReg(handle, WM8962_LOUTMIX, 0x80);

            /*
             * Right RINPUT3 to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            WM8962_WriteReg(handle, WM8962_ROUTMIX, 0x80);
            break;
        case kWM8962_RoutePlayback:
            /* Data route I2S_IN-> DAC-> HP */
            /*
             * Left DAC to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            WM8962_WriteReg(handle, WM8962_LOUTMIX, 0x100);

            /*
             * Right DAC to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            WM8962_WriteReg(handle, WM8962_ROUTMIX, 0x100);
            WM8962_WriteReg(handle, WM8962_POWER3, 0x0C);
            /* Set power for DAC */
            WM8962_SetModule(handle, kWM8962_ModuleDAC, true);
            WM8962_SetModule(handle, kWM8962_ModuleOMIX, true);
            WM8962_SetModule(handle, kWM8962_ModuleLineOut, true);
            break;
        case kWM8962_RoutePlaybackandRecord:
            /*
             * Left DAC to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            WM8962_WriteReg(handle, WM8962_LOUTMIX, 0x100);

            /*
             * Right DAC to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            WM8962_WriteReg(handle, WM8962_ROUTMIX, 0x100);
            WM8962_WriteReg(handle, WM8962_POWER3, 0x3C);
            WM8962_SetModule(handle, kWM8962_ModuleDAC, true);
            WM8962_SetModule(handle, kWM8962_ModuleADC, true);
            WM8962_SetModule(handle, kWM8962_ModuleLineIn, true);
            WM8962_SetModule(handle, kWM8962_ModuleOMIX, true);
            WM8962_SetModule(handle, kWM8962_ModuleLineOut, true);
            break;
        case kWM8962_RouteRecord:
            /* LINE_IN->ADC->I2S_OUT */
            /*
             * Left and right input boost, LIN3BOOST and RIN3BOOST = 0dB
             */
            WM8962_WriteReg(handle, WM8962_POWER3, 0x30);
            /* Power up ADC and AIN */
            WM8962_SetModule(handle, kWM8962_ModuleLineIn, true);
            WM8962_SetModule(handle, kWM8962_ModuleADC, true);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t WM8962_SetLeftInput(codec_handle_t *handle, wm8962_input_t input)
{
    status_t ret = kStatus_Success;
    uint16_t val = 0;

    switch (input)
    {
        case kWM8962_InputSingleEndedMic:
            /* Only LMN1 enabled, LMICBOOST to 13db, LMIC2B enabled */
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINL_MASK | WM8962_POWER1_ADCL_MASK | WM8962_POWER1_MICB_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            ret = WM8962_WriteReg(handle, WM8962_LINPATH, 0x138);
            ret = WM8962_WriteReg(handle, WM8962_LINVOL, 0x117);
            break;
        case kWM8962_InputDifferentialMicInput2:
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINL_MASK | WM8962_POWER1_ADCL_MASK | WM8962_POWER1_MICB_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            ret = WM8962_WriteReg(handle, WM8962_LINPATH, 0x178);
            ret = WM8962_WriteReg(handle, WM8962_LINVOL, 0x117);
            break;
        case kWM8962_InputDifferentialMicInput3:
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINL_MASK | WM8962_POWER1_ADCL_MASK | WM8962_POWER1_MICB_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            ret = WM8962_WriteReg(handle, WM8962_LINPATH, 0x1B8);
            ret = WM8962_WriteReg(handle, WM8962_LINVOL, 0x117);
            break;
        case kWM8962_InputLineINPUT2:
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINL_MASK | WM8962_POWER1_ADCL_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            WM8962_ReadReg(WM8962_INBMIX1, &val);
            val |= 0xE;
            ret = WM8962_WriteReg(handle, WM8962_INBMIX1, val);
            break;
        case kWM8962_InputLineINPUT3:
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINL_MASK | WM8962_POWER1_ADCL_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            WM8962_ReadReg(WM8962_INBMIX1, &val);
            val |= 0x70;
            ret = WM8962_WriteReg(handle, WM8962_INBMIX1, val);
            break;
        default:
            break;
    }

    return ret;
}

status_t WM8962_SetRightInput(codec_handle_t *handle, wm8962_input_t input)
{
    status_t ret = kStatus_Success;
    uint16_t val = 0;

    switch (input)
    {
        case kWM8962_InputSingleEndedMic:
            /* Only LMN1 enabled, LMICBOOST to 13db, LMIC2B enabled */
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINR_MASK | WM8962_POWER1_ADCR_MASK | WM8962_POWER1_MICB_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            ret = WM8962_WriteReg(handle, WM8962_RINPATH, 0x138);
            ret = WM8962_WriteReg(handle, WM8962_RINVOL, 0x117);
            break;
        case kWM8962_InputDifferentialMicInput2:
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINR_MASK | WM8962_POWER1_ADCR_MASK | WM8962_POWER1_MICB_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            ret = WM8962_WriteReg(handle, WM8962_RINPATH, 0x178);
            ret = WM8962_WriteReg(handle, WM8962_RINVOL, 0x117);
            break;
        case kWM8962_InputDifferentialMicInput3:
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINR_MASK | WM8962_POWER1_ADCR_MASK | WM8962_POWER1_MICB_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            ret = WM8962_WriteReg(handle, WM8962_RINPATH, 0x1B8);
            ret = WM8962_WriteReg(handle, WM8962_RINVOL, 0x117);
            break;
        case kWM8962_InputLineINPUT2:
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINR_MASK | WM8962_POWER1_ADCR_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            WM8962_ReadReg(WM8962_INBMIX2, &val);
            val |= 0xE;
            ret = WM8962_WriteReg(handle, WM8962_INBMIX2, val);
            break;
        case kWM8962_InputLineINPUT3:
            WM8962_ReadReg(WM8962_POWER1, &val);
            val |= (WM8962_POWER1_AINR_MASK | WM8962_POWER1_ADCR_MASK);
            ret = WM8962_WriteReg(handle, WM8962_POWER1, val);
            WM8962_ReadReg(WM8962_INBMIX2, &val);
            val |= 0x70;
            ret = WM8962_WriteReg(handle, WM8962_INBMIX2, val);
            break;
        default:
            break;
    }

    return ret;
}

status_t WM8962_SetProtocol(codec_handle_t *handle, wm8962_protocol_t protocol)
{
    status_t ret = kStatus_Success;
    switch (protocol)
    {
        case kWM8962_BusI2S:
            WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_FORMAT_MASK,
                             WM8962_IFACE1_FORMAT(WM8962_IFACE1_FORMAT_I2S));
            break;
        case kWM8962_BusLeftJustified:
            WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_FORMAT_MASK,
                             WM8962_IFACE1_FORMAT(WM8962_IFACE1_FORMAT_LJ));
            break;
        case kWM8962_BusRightJustified:
            WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_FORMAT_MASK,
                             WM8962_IFACE1_FORMAT(WM8962_IFACE1_FORMAT_RJ));
            break;
        case kWM8962_BusPCMA:
            WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_FORMAT_MASK,
                             WM8962_IFACE1_FORMAT(WM8962_IFACE1_FORMAT_DSP));
            WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_LRP_MASK, WM8962_IFACE1_LRP(WM8962_IFACE1_DSP_MODEA));
            break;
        case kWM8962_BusPCMB:
            WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_FORMAT_MASK,
                             WM8962_IFACE1_FORMAT(WM8962_IFACE1_FORMAT_DSP));
            WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_LRP_MASK, WM8962_IFACE1_LRP(WM8962_IFACE1_DSP_MODEB));
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_WL_MASK, WM8962_IFACE1_WL(WM8962_IFACE1_WL_32BITS));
    return ret;
}

status_t WM8962_SetVolume(codec_handle_t *handle, wm8962_module_t module, uint32_t volume)
{
    uint16_t vol = 0;
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kWM8962_ModuleADC:
            vol = volume;
            ret = WM8962_WriteReg(handle, WM8962_LADC, vol);
            ret = WM8962_WriteReg(handle, WM8962_RADC, vol);
            /* Update volume */
            vol = 0x100 | volume;
            ret = WM8962_WriteReg(handle, WM8962_LADC, vol);
            ret = WM8962_WriteReg(handle, WM8962_RADC, vol);
            break;
        case kWM8962_ModuleDAC:
            vol = volume;
            ret = WM8962_WriteReg(handle, WM8962_LDAC, vol);
            ret = WM8962_WriteReg(handle, WM8962_RDAC, vol);
            vol = 0x100 | volume;
            ret = WM8962_WriteReg(handle, WM8962_LDAC, vol);
            ret = WM8962_WriteReg(handle, WM8962_RDAC, vol);
            break;
        case kWM8962_ModuleHP:
            vol = volume;
            ret = WM8962_WriteReg(handle, WM8962_LOUT1, vol);
            ret = WM8962_WriteReg(handle, WM8962_ROUT1, vol);
            vol = 0x100 | volume;
            ret = WM8962_WriteReg(handle, WM8962_LOUT1, vol);
            ret = WM8962_WriteReg(handle, WM8962_ROUT1, vol);
            break;
        case kWM8962_ModuleLineIn:
            vol = volume;
            ret = WM8962_WriteReg(handle, WM8962_LINVOL, vol);
            ret = WM8962_WriteReg(handle, WM8962_RINVOL, vol);
            vol = 0x100 | volume;
            ret = WM8962_WriteReg(handle, WM8962_LINVOL, vol);
            ret = WM8962_WriteReg(handle, WM8962_RINVOL, vol);
            break;
        case kWM8962_ModuleSpeaker:
            vol = volume;
            ret = WM8962_WriteReg(handle, WM8962_LOUT2, vol);
            ret = WM8962_WriteReg(handle, WM8962_ROUT2, vol);
            vol = 0x100 | volume;
            ret = WM8962_WriteReg(handle, WM8962_LOUT2, vol);
            ret = WM8962_WriteReg(handle, WM8962_ROUT2, vol);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

uint32_t WM8962_GetVolume(codec_handle_t *handle, wm8962_module_t module)
{
    uint16_t vol = 0;
    switch (module)
    {
        case kWM8962_ModuleADC:
            WM8962_ReadReg(WM8962_LADC, &vol);
            vol &= 0xFF;
            break;
        case kWM8962_ModuleDAC:
            WM8962_ReadReg(WM8962_LDAC, &vol);
            vol &= 0xFF;
            break;
        case kWM8962_ModuleHP:
            WM8962_ReadReg(WM8962_LOUT1, &vol);
            vol &= 0x7F;
            break;
        case kWM8962_ModuleLineOut:
            WM8962_ReadReg(WM8962_LINVOL, &vol);
            vol &= 0x3F;
            break;
        default:
            vol = 0;
            break;
    }
    return vol;
}

status_t WM8962_SetMute(codec_handle_t *handle, wm8962_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kWM8962_ModuleADC:
            /*
             * Digital Mute
             */
            if (isEnabled)
            {
                ret = WM8962_WriteReg(handle, WM8962_LADC, 0x100);
                ret = WM8962_WriteReg(handle, WM8962_RADC, 0x100);
            }
            else
            {
                ret = WM8962_WriteReg(handle, WM8962_LADC, 0x1C3);
                ret = WM8962_WriteReg(handle, WM8962_RADC, 0x1C3);
            }
            break;
        case kWM8962_ModuleDAC:
            /*
             * Digital mute
             */
            if (isEnabled)
            {
                ret = WM8962_WriteReg(handle, WM8962_LDAC, 0x100);
                ret = WM8962_WriteReg(handle, WM8962_RDAC, 0x100);
            }
            else
            {
                ret = WM8962_WriteReg(handle, WM8962_LDAC, 0x1FF);
                ret = WM8962_WriteReg(handle, WM8962_RDAC, 0x1FF);
            }
            break;
        case kWM8962_ModuleHP:
            /*
             * Analog mute
             */
            if (isEnabled)
            {
                ret = WM8962_WriteReg(handle, WM8962_LOUT1, 0x100);
                ret = WM8962_WriteReg(handle, WM8962_ROUT1, 0x100);
            }
            else
            {
                ret = WM8962_WriteReg(handle, WM8962_LOUT1, 0x179);
                ret = WM8962_WriteReg(handle, WM8962_ROUT1, 0x179);
            }
            break;
        case kWM8962_ModuleLineOut:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t WM8962_ConfigDataFormat(codec_handle_t *handle, uint32_t mclk, uint32_t sample_rate, uint32_t bits)
{
    status_t retval = kStatus_Success;
    uint32_t div = 0;
    uint16_t val = 0;

    /* Compute sample rate div, dac and adc are the same sample rate */
    div = mclk / sample_rate;
    if (div == 256)
    {
        val = 0;
    }
    else if (div > 256)
    {
        val = (((div / 256U) << 6U) | ((div / 256U) << 3U));
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    retval = WM8962_WriteReg(handle, WM8962_CLOCK1, val);

    /* Compute bclk div */
    div /= 64U;
    switch (div)
    {
        case 4:
        case 5:
        case 6:
            val = (0x1C0 | div);
            break;
        case 8:
            val = 0x1C7;
            break;
        case 11:
            val = 0x1C8;
            break;
        case 12:
            val = 0x1C9;
            break;
        case 16:
            val = 0x1CA;
            break;
        case 22:
            val = 0x1CB;
            break;
        case 24:
            val = 0x1CC;
            break;
        case 32:
            val = 0x1CF;
            break;
        default:
            val = 0;
            retval = kStatus_InvalidArgument;
            break;
    }

    retval = WM8962_WriteReg(handle, WM8962_CLOCK2, val);
    /*
     * Slave mode (MS = 0), LRP = 0, 32bit WL, left justified (FORMAT[1:0]=0b01)
     */
    switch (bits)
    {
        case 16:
            retval = WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_WL_MASK,
                                      WM8962_IFACE1_WL(WM8962_IFACE1_WL_16BITS));
            break;
        case 20:
            retval = WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_WL_MASK,
                                      WM8962_IFACE1_WL(WM8962_IFACE1_WL_20BITS));
            break;
        case 24:
            retval = WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_WL_MASK,
                                      WM8962_IFACE1_WL(WM8962_IFACE1_WL_24BITS));
            break;
        case 32:
            retval = WM8962_ModifyReg(handle, WM8962_IFACE1, WM8962_IFACE1_WL_MASK,
                                      WM8962_IFACE1_WL(WM8962_IFACE1_WL_32BITS));
            break;
        default:
            retval = kStatus_InvalidArgument;
            break;
    }

    return retval;
}

status_t WM8962_SetJackDetect(codec_handle_t *handle, bool isEnabled)
{
    uint8_t retval = 0;
    uint16_t val = 0;

    WM8962_ReadReg(WM8962_ADDCTL2, &val);

    if (isEnabled)
    {
        val |= 0x40U;
    }
    else
    {
        val &= 0xCF;
    }

    retval = WM8962_WriteReg(handle, WM8962_ADDCTL2, val);

    return retval;
}

status_t WM8962_WriteReg(codec_handle_t *handle, uint16_t reg, uint16_t val)
{
    uint8_t cmd, buff;
    uint8_t retval = 0;

    /* The register address */
    cmd = (reg << 1) | ((val >> 8U) & 0x0001U);
    /* Data */
    buff = val & 0xFF;

    retval = CODEC_I2C_WriteReg(handle->slaveAddress, kCODEC_RegAddr16Bit, cmd, kCODEC_RegWidth16Bit, buff,
                                handle->I2C_SendFunc);

    if (retval == kStatus_Success)
    {
        reg_cache[reg] = val;
    }

    return retval;
}

status_t WM8962_ReadReg(uint16_t reg, uint16_t *val)
{
    if (reg >= WM8962_CACHEREGNUM)
    {
        return kStatus_InvalidArgument;
    }

    *val = reg_cache[reg];

    return kStatus_Success;
}

status_t WM8962_ModifyReg(codec_handle_t *handle, uint16_t reg, uint16_t mask, uint16_t val)
{
    uint8_t retval = 0;
    uint16_t reg_val = 0;
    retval = WM8962_ReadReg(reg, &reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }
    reg_val &= (uint16_t)~mask;
    reg_val |= val;
    retval = WM8962_WriteReg(handle, reg, reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

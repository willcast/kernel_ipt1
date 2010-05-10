#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "iphone-audio.h"

#ifdef CONFIG_IPHONE_3G
#include "../codecs/wm8991.h"
#endif

static int iphone_soc_to_wm8758_init(struct snd_soc_codec *codec)
{
	pr_debug("ENTER iphone_soc_to_wm8758_init\n");
	return 0;
}

static int iphone_soc_to_bb_init(struct snd_soc_codec *codec)
{
	pr_debug("ENTER iphone_soc_to_bb_init\n");
	return 0;
}

static int iphone_wm8991_init(struct snd_soc_codec *codec)
{
	int ret = 0;

	printk("WM8991 initialising...\n");

	if(codec == NULL || codec->write == NULL)
	{
		ret = -1;
		return ret;
	}
	
	ret = codec->write(codec, WM8991_POWER_MANAGEMENT_1, 0x1003);
	if(ret < 0)
	{
		printk("iphone-wm8991: failed to set power mode.\n");
		return ret;
	}

	// speaker enabled, VREF bias enabled, Vmid to 2 * 50kOhm (normal mode) 
	ret = codec->write(codec, WM8991_POWER_MANAGEMENT_1, 0x1003);

	// PLL enable, thermal sensor enable, thermal shutdown enable
	ret = codec->write(codec, WM8991_POWER_MANAGEMENT_2, 0xe000);

	// Left DAC enable, right DAC enable
	ret = codec->write(codec, WM8991_POWER_MANAGEMENT_3, 0x0003);

	// Right ADC data is output on right channel, normal i2s format
	ret = codec->write(codec, WM8991_AUDIO_INTERFACE_1, 0x4010);

	// Right DAC outputs right channel.
	ret = codec->write(codec, WM8991_AUDIO_INTERFACE_2, 0x4000);

	// Timeout clock disabled
	// GPIO output clock = SYSCLK
	// Class D Clock Divider = SYSCLK / 16
	// BCLK frequency = SYSCLK / 8
	ret = codec->write(codec, WM8991_CLOCKING_1, 0x01ce);

	// SYSCLK is from PLL output
	// SYSCLK = PLL / 2
	ret = codec->write(codec, WM8991_CLOCKING_2, 0x5000);

	// Interface 1 = master, interface 2 = slave, interface 1 selected, ADCLRC rate = BCLK / 32
	ret = codec->write(codec, WM8991_AUDIO_INTERFACE_3, 0x8020);

	// ADCLRC/GPIO1 pin is GPIO1, DACLRC rate = BCLK / 32
	ret = codec->write(codec, WM8991_AUDIO_INTERFACE_4, 0x8020);

	// Full DAC volume
	ret = codec->write(codec, WM8991_LEFT_DAC_DIGITAL_VOLUME, 0xc0);
	ret = codec->write(codec, WM8991_RIGHT_DAC_DIGITAL_VOLUME, 0x1c0);

	// ADC digital high pass enable
	ret = codec->write(codec, WM8991_ADC_CTRL, 0x100);

	// Full ADC volume
	ret = codec->write(codec, WM8991_LEFT_ADC_DIGITAL_VOLUME, 0xc0);
	ret = codec->write(codec, WM8991_LEFT_ADC_DIGITAL_VOLUME, 0x1c0);

	// GPIO 2 pull down enable, GPIO 2 is IRQ output, GPIO 1 is an input pin
	ret = codec->write(codec, WM8991_GPIO1_GPIO2, 0x1700);

	// GPIO 4 pull down enable and is input, GPIO 3 is input
	ret = codec->write(codec, WM8991_GPIO3_GPIO4, 0x1000);

	// GPIO 6 pull down enable and is input, GPIO 5 is input and has IRQ enabled
	ret = codec->write(codec, WM8991_GPIO5_GPIO6, 0x1040);

	// 4-wire readback mode
	ret = codec->write(codec, WM8991_GPIOCTRL_2, 0x0);

	// Temperature sensor inverted, gpio 3 polarity inverted
	ret = codec->write(codec, WM8991_GPIO_POL, 0x0804);

	// All input PGAs to 0dB and muted
	ret = codec->write(codec, WM8991_LEFT_LINE_INPUT_1_2_VOLUME, 0x008b);
	ret = codec->write(codec, WM8991_LEFT_LINE_INPUT_3_4_VOLUME, 0x008b);
	ret = codec->write(codec, WM8991_RIGHT_LINE_INPUT_1_2_VOLUME, 0x008b);
	ret = codec->write(codec, WM8991_RIGHT_LINE_INPUT_3_4_VOLUME, 0x008b);

	// headphones at -73 dB, zero cross enabled
	ret = codec->write(codec, WM8991_LEFT_OUTPUT_VOLUME, 0x00b0);
	ret = codec->write(codec, WM8991_RIGHT_OUTPUT_VOLUME, 0x01b0);

	// Line out muted
	ret = codec->write(codec, WM8991_LINE_OUTPUTS_VOLUME, 0x0066);

	// out3 and out4 muted
	ret = codec->write(codec, WM8991_OUT3_4_VOLUME, 0x0022);

	// zero cross enabled, 0 dB 
	ret = codec->write(codec, WM8991_LEFT_OPGA_VOLUME, 0x00f9);
	ret = codec->write(codec, WM8991_RIGHT_OPGA_VOLUME, 0x01f9);

	// no speaker output attenuation
	ret = codec->write(codec, WM8991_SPEAKER_VOLUME, 0x0);

	// speaker amplifier in class AB mode
	ret = codec->write(codec, WM8991_CLASSD1, 0x103);

	// reserved. data sheet says should be 0x55, but 0x57 on iPhone
	ret = codec->write(codec, WM8991_CLASSD1 + 1, 0x57);

	// no speaker boost
	ret = codec->write(codec, WM8991_CLASSD3, 0x0100);

	// speaker volume zero cross enabled, -1 dB
	ret = codec->write(codec, WM8991_CLASSD3 + 1, 0x01f8);

	ret = codec->write(codec, WM8991_INPUT_MIXER1, 0x0);
	ret = codec->write(codec, WM8991_INPUT_MIXER2, 0x0);
	ret = codec->write(codec, WM8991_INPUT_MIXER3, 0x0);
	ret = codec->write(codec, WM8991_INPUT_MIXER4, 0x0);
	ret = codec->write(codec, WM8991_INPUT_MIXER5, 0x0);
	ret = codec->write(codec, WM8991_INPUT_MIXER6, 0x0);
	ret = codec->write(codec, WM8991_OUTPUT_MIXER1, 0x0);
	ret = codec->write(codec, WM8991_OUTPUT_MIXER2, 0x0);
	ret = codec->write(codec, WM8991_OUTPUT_MIXER3, 0x0);
	ret = codec->write(codec, WM8991_OUTPUT_MIXER4, 0x0);
	ret = codec->write(codec, WM8991_OUTPUT_MIXER5, 0x0);
	ret = codec->write(codec, WM8991_OUTPUT_MIXER6, 0x0);

	// analogue bias optimisation to lowest (default)
	ret = codec->write(codec, WM8991_OUT3_4_MIXER, 0x180);

	ret = codec->write(codec, WM8991_LINE_MIXER1, 0x0);
	ret = codec->write(codec, WM8991_LINE_MIXER2, 0x0);

	// unmute left and right DAC to SPKMIX
	ret = codec->write(codec, WM8991_SPEAKER_MIXER, 0x3);

	ret = codec->write(codec, WM8991_ADDITIONAL_CONTROL, 0x0);
	ret = codec->write(codec, WM8991_ANTIPOP1, 0x0);

	// Enables the VGS / R current generator and the analogue input and output bias
	ret = codec->write(codec, WM8991_ANTIPOP2, 0x8);

	ret = codec->write(codec, WM8991_MICBIAS, 0x0);

	// PLL fractional mode, N = 7
	ret = codec->write(codec, WM8991_PLL1, 0x0087);

	// K = 0x85FC = 34300
	ret = codec->write(codec, WM8991_PLL2, 0x0085);
	ret = codec->write(codec, WM8991_PLL3, 0x00fc);

	// R = 7.523376465
	
	return ret;
}

#ifdef CONFIG_IPHONE_3G
static int iphone_wm8991_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;

	ret = snd_soc_dai_set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM);
	if(ret < 0)
	{
		printk("iphone-wm8991: failed to set cpu dai format.\n");
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM);
	if(ret < 0)
	{
		printk("iphone-wm8991: failed to set cpu dai format.\n");
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8991_BCLK_DIV, 0x1ce);
	if(ret < 0)
	{
		printk("iphone-wm8991: failed too set BCLK.\n");
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, -1, 0x785fc);
	if(ret < 0)
	{
		printk("iphone-wm8991: failed to set PLL.\n");
		return ret;
	}

	return ret;
}

static struct snd_soc_ops iphone_wm8991_ops = {
	.hw_params = iphone_wm8991_params,
	//.hw_free = iphone_wm8991_free,
};

#endif


static struct snd_soc_dai_link iphone_dai_links[] = {
#ifdef CONFIG_IPHONE_2G
	{
		.name           = "WM8758",
		.stream_name    = "WM8758",
		.cpu_dai        = &iphone_i2s_wm8758_dai,
		.codec_dai      = &iphone_wm8758_dai,
		.init           = iphone_soc_to_wm8758_init,
	},
	{
		.name           = "Baseband",
		.stream_name    = "Baseband",
		.cpu_dai        = &iphone_i2s_bb_dai,
		.codec_dai      = &iphone_bb_dai,
		.init           = iphone_soc_to_bb_init,
	}
#endif
#ifdef CONFIG_IPHONE_3G
	{
		.name			= "WM8991",
		.stream_name	= "WM8991",
		.cpu_dai		= &iphone_i2s_wm8758_dai, // This is bad, jah?
		.codec_dai		= &wm8991_dai,
		.init			= iphone_wm8991_init,
		.ops			= &iphone_wm8991_ops,
	},
#endif
};

static struct snd_soc_card iphone_snd_soc_card = {
	.name           = "iPhoneSound",
	.platform       = &snd_iphone_soc_platform,
	.dai_link       = iphone_dai_links,
	.num_links      = ARRAY_SIZE(iphone_dai_links),
};

//#ifdef CONFIG_IPHONE_3G
#if 0
static struct wm8990_setup_data wm8991_i2c_setup = {
	.i2c_bus = 0,
	.i2c_address = 0x36,
};
#endif

static struct snd_soc_device iphone_snd_soc_device = {
	.card           = &iphone_snd_soc_card,

#ifdef CONFIG_IPHONE_2G
	.codec_dev      = &soc_codec_dev_wm8758,
#endif

#ifdef CONFIG_IPHONE_3G
	.codec_dev		= &soc_codec_dev_wm8991,
	//.codec_data		= &wm8991_i2c_setup,
#endif
};

static struct platform_device *snd_dev;

static int __init iphone_sound_init(void)
{
	int ret = 0;

	snd_dev = platform_device_alloc("soc-audio", -1);
	if (!snd_dev) {
		printk("failed to alloc soc-audio device\n");
		ret = -ENOMEM;
		return ret;
	}

	platform_set_drvdata(snd_dev, &iphone_snd_soc_device);
	iphone_snd_soc_device.dev = &snd_dev->dev;

	ret = platform_device_add(snd_dev);
	if (ret) {

		printk("failed to add soc-audio dev\n");
		platform_device_put(snd_dev);
	}

	return ret;
}

static void __exit iphone_sound_exit(void)
{
	platform_device_unregister(snd_dev);
}

module_init(iphone_sound_init);
module_exit(iphone_sound_exit);

MODULE_DESCRIPTION("iPhone SoC sound driver");
MODULE_AUTHOR("Yiduo Wang");
MODULE_LICENSE("GPL");

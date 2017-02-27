#include "hal.h"

/* Private macro -------------------------------------------------------------*/
#define WM8978_ADDRESS 0x1A

/* Private variables ---------------------------------------------------------*/
static uint16_t wm8978_regval[58]=
{
	0x0000,0x0000,0x0000,0x0000,0x0050,0x0000,0x0140,0x0000,
	0x0000,0x0000,0x0000,0x00FF,0x00FF,0x0000,0x0100,0x00FF,
	0x00FF,0x0000,0x012C,0x002C,0x002C,0x002C,0x002C,0x0000,
	0x0032,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0x0038,0x000B,0x0032,0x0000,0x0008,0x000C,0x0093,0x00E9,
	0x0000,0x0000,0x0000,0x0000,0x0003,0x0010,0x0010,0x0100,
	0x0100,0x0002,0x0001,0x0001,0x0039,0x0039,0x0039,0x0039,
	0x0001,0x0001
};

uint8_t  wm8978_i2c_write(uint8_t DevAddr,uint8_t RegAddr,uint8_t Data)
{
    uint8_t res;
    uint8_t wm8978_send_data[2];
    
    hal_i2c_config_t i2c_init;
    hal_i2c_port_t i2c_port = HAL_I2C_MASTER_0;
    hal_i2c_frequency_t input_frequency = HAL_I2C_FREQUENCY_50K;
    
    /*Step1: Init GPIO and set GPIO pinmux(if EPT tool hasn't been used to configure the related pinmux).*/
    hal_gpio_init(HAL_GPIO_27);
    hal_gpio_init(HAL_GPIO_28);
    /* Call hal_pinmux_set_function() to set GPIO pinmux */
    hal_pinmux_set_function(HAL_GPIO_27, HAL_GPIO_27_I2C1_CLK);
    hal_pinmux_set_function(HAL_GPIO_28, HAL_GPIO_28_I2C1_DATA);

    /*Step2: Write data to i2c slave.*/
    /* Initialize I2C */
    i2c_init.frequency = input_frequency;
    hal_i2c_master_init(i2c_port, &i2c_init);

    wm8978_send_data[0] = RegAddr;
    wm8978_send_data[1] = Data;
    
    //log_hal_info("wm8978_send_data[0]: %d\r\n", wm8978_send_data[0]);
    //og_hal_info("wm8978_send_data[1]:%d\r\n", wm8978_send_data[1]);
    
    res = hal_i2c_master_send_polling(i2c_port, DevAddr, wm8978_send_data, 2);
    /*Wait some time till the data is stable*/
    hal_gpt_delay_ms(10);
    /* Deinitialize I2C */
    hal_i2c_master_deinit(i2c_port); 
    
    return res;
}

uint8_t wm8978_write_reg(uint8_t reg,uint16_t val)
{
    uint8_t res;
    uint8_t RegAddr;
    uint8_t RegValue;
    
    uint32_t test_fail = 0;

    RegAddr=((reg<<1)|((uint8_t)((val>>8)&0x01)));
    RegValue=(uint8_t)val;
    res = wm8978_i2c_write(WM8978_ADDRESS,RegAddr,RegValue);
        
    if(res==0)
        wm8978_regval[reg]=val;
    return res;
}

uint8_t wm8978_read_reg(uint8_t reg)
{
    return wm8978_regval[reg];	
}

//WM8978 L2/R2(Line In)db config(L2/R2-->ADC input db)
//gain:0~7,0 means channel mute,1~7,对应-12dB~6dB,3dB/Step
void wm8978_linen_gain(uint8_t gain)
{
    uint16_t regval;
    gain &= 0x07;
    regval = wm8978_read_reg(47);
    regval &= ~(7<<4);
    wm8978_write_reg(47,regval|gain<<4);		
    regval = wm8978_read_reg(48);
    regval &= ~(7<<4); 
    wm8978_write_reg(48,regval|gain<<4);
}

//WM8978 AUXR,AUXL(PWM audio config)db config(AUXR/L-->ADC input db)
//gain:0~7,0 channel mute,1~7,对应-12dB~6dB,3dB/Step
void wm8978_aux_gain(uint8_t gain)
{
    uint16_t regval;
    gain&=0x07;
    regval=wm8978_read_reg(47);	
    regval&=~(7<<0); 
    wm8978_write_reg(47,regval|gain<<0);
    regval=wm8978_read_reg(48);	
    regval&=~(7<<0);
    wm8978_write_reg(48,regval|gain<<0);
}  

//WM8978 input config
//micen:MIC enable(1)/disable(0)
//lineinen:Line In enable(1)/disable(0)
//auxen:aux enable(1)/disable(0)
void WM8978_Input_Cfg(uint8_t micen,uint8_t lineinen,uint8_t auxen)
{
    uint16_t regval;  
    regval=wm8978_read_reg(2);
    if(micen)
        regval|=3<<2;//open INPPGAENR,INPPGAENL(MIC的PGA放大)
    else 
        regval&=~(3<<2);//close INPPGAENR,INPPGAENL.
    wm8978_write_reg(2,regval);
    regval=wm8978_read_reg(44);
    if(micen)
        regval|=3<<4|3<<0;//enable LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
    else 
        regval&=~(3<<4|3<<0);//disable LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
    wm8978_write_reg(44,regval);
    if(lineinen)
        wm8978_linen_gain(5);//LINE IN 0dB
    else 
        wm8978_linen_gain(0);//disable LINE IN
    if(auxen)
        wm8978_aux_gain(7);//AUX 6dB
    else 
        wm8978_aux_gain(0);//disable AUX input
}
//WM8978 MIC db config(not include BOOST's 20dB, MIC-->ADC input db)
//gain:0~63:-12dB~35.25dB,0.75dB/Step
void wm8978_mic_gain(uint8_t gain)
{
    gain&=0x3F;
    wm8978_write_reg(45,gain);//R45,left channel PGA config
    wm8978_write_reg(46,gain|1<<8);//R46,right channel PGA config
}

//WM8978 output setting
//dacen:DAC output enabled(1)/disabled(0)
//bpsen:Bypass output(record,include MIC,LINE IN,AUX等) enabled(1)/disabled(0)
void wm8978_output_cfg(uint8_t dacen,uint8_t bpsen)
{
    uint16_t regval=0;
    if(dacen)
        regval|=1<<0;//DAC output enabled
    if(bpsen)
    {
        regval|=1<<1;//BYPASS enabled
        regval|=5<<2;//0dB													//0dB??
    } 
    wm8978_write_reg(50,regval);//R50 config
    wm8978_write_reg(51,regval);//R51 config
}

//WM8978 DAC/ADC config
//adcen:adc enable(1)/disable(0)
//dacen:dac enable(1)/disable(0)
void wm8978_adda_cfg(uint8_t dacen,uint8_t adcen)
{
    uint16_t regval;
    regval=wm8978_read_reg(3);//read R3
    if(dacen)
        regval|=3<<0;//R3 set lowest 2 bits to 1,enable DACR&DACL
    else 
        regval&=~(3<<0);//R3 set lowest 2 bits to 0,disable DACR&DACL
    wm8978_write_reg(3,regval);//R3 config
    regval=wm8978_read_reg(2);//read R2
    if(adcen)
        regval|=3<<0;//R2 set lowest bits to 1, enable ADCR&ADCL
    else 
        regval&=~(3<<0);//R2 set lowest bits to 0, disable ADCR&ADCL.
    wm8978_write_reg(2,regval);//R2 config
} 

//speaker volume
void wm8978_hpvol_set(uint8_t voll,uint8_t volr)
{
    voll&=0x3F;
    volr&=0x3F;															//О??Χ
    if(voll==0)voll|=1<<6;//volume is 0, mute on
    if(volr==0)volr|=1<<6;//volume is 0, mute on
    wm8978_write_reg(52,voll);//R52, left channel volume
    wm8978_write_reg(53,volr|(1<<8));//R53, right channel volume
}

//speaker volume
//voll:left channel volume(0~63)
void wm8978_spkvol_set(uint8_t volx)
{
    volx&=0x3F;
    if(volx==0)volx|=1<<6;//volume is 0, mute on
    wm8978_write_reg(54,volx);//R54, left channel audio volume
    wm8978_write_reg(55,volx|(1<<8));//R55, right channel audio volume
}

//I2S working mode
//fmt:0,LSB;1,MSB;2,I2S;3,PCM/DSP;
//len:0,16bist;1,20bits;2,24bits;3,32bits;
void wm8978_i2s_cfg(uint8_t fmt,uint8_t len)
{
    fmt&=0x03;
    len&=0x03;
    wm8978_write_reg(4,(fmt<<3)|(len<<5));//R4,WM8978 working mode	
}

void wm8978_play_start(void)
{
    wm8978_write_reg(0,0);//soft reset WM8978
    wm8978_write_reg(1,0x1B);//R1,MICEN 1(MIC enabled),BIASEN 1(emu on),VMIDSEL[1:0]:11(5K)
    wm8978_write_reg(2,0x1B0);//R2, ROUT1,LOUT1 output enabled(headphone),BOOSTENR,BOOSTENL enabled
    wm8978_write_reg(5,0);
    wm8978_write_reg(3,0x6C);//R3, LOUT2,ROUT2 output enabled(speaker on),RMIX,LMIX enabled

    wm8978_write_reg(6,0x8D);//R6, BCLK and LRC Clock are outputs generated by the WM8978(Master)
	
    //wm8978_write_reg(7,0x02);

    wm8978_write_reg(43,1<<4);//R43,INVROUT2 inverted, drive speaker
    wm8978_write_reg(47,1<<8);//R47,PGABOOSTL,left MIC got 20 db	
    wm8978_write_reg(48,1<<8);//R48,PGABOOSTR, right MIC got 20 db

    wm8978_write_reg(49,1<<1);//R49,TSDEN, open hot protecting
    wm8978_write_reg(10,1<<3);//R10,SOFTMUTE closed,128x sample rate, best SNR
    wm8978_write_reg(14,1<<3);//R14,ADC 128x sample rate

    wm8978_i2s_cfg(2,0);//I2S work mode

    wm8978_adda_cfg(1,0);//open DAC
    WM8978_Input_Cfg(0,0,0);//close input channel

    wm8978_output_cfg(1,0);//open DAC output

    //wm8978_adda_cfg(0,1);//open ADC
    //WM8978_Input_Cfg(1,1,0);//open input channel(MIC&LINE IN)
    //wm8978_output_cfg(0,1);//open BYPASS output
    //wm8978_mic_gain(46);//MIC db setting
    wm8978_hpvol_set(25,25);
    wm8978_spkvol_set(50);
}

void wm8978_play_record_start(void)
{
    wm8978_write_reg(0,0);//soft reset WM8978
    wm8978_write_reg(1,0x1B);//R1,MICEN 1(MIC enabled),BIASEN 1(emu on),VMIDSEL[1:0]:11(5K)
    wm8978_write_reg(2,0x1B0);//R2, ROUT1,LOUT1 output enabled(headphone),BOOSTENR,BOOSTENL enabled
    wm8978_write_reg(5,0);
    wm8978_write_reg(3,0x6C);//R3, LOUT2,ROUT2 output enabled(speaker on),RMIX,LMIX enabled

    wm8978_write_reg(6,0x8D);//R6, BCLK and LRC Clock are outputs generated by the WM8978(Master)
	
    //wm8978_write_reg(7,0x02);

    wm8978_write_reg(43,1<<4);//R43,INVROUT2 inverted, drive speaker
    wm8978_write_reg(47,1<<8);//R47,PGABOOSTL,left MIC got 20 db	
    wm8978_write_reg(48,1<<8);//R48,PGABOOSTR, right MIC got 20 db

    wm8978_write_reg(49,1<<1);//R49,TSDEN, open hot protecting
    wm8978_write_reg(10,1<<3);//R10,SOFTMUTE closed,128x sample rate, best SNR
    wm8978_write_reg(14,1<<3);//R14,ADC 128x sample rate

    wm8978_i2s_cfg(2,0);//I2S work mode

    wm8978_adda_cfg(1,1);//open ADC&DAC
    WM8978_Input_Cfg(1,1,0);//open input channel(MIC&LINE IN)
    wm8978_output_cfg(1,0);//open DAC output
    wm8978_mic_gain(60);//MIC db setting
    wm8978_hpvol_set(25,25);
    wm8978_spkvol_set(60);
}
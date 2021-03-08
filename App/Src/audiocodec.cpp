#include "audiocodec.h"
#include "stm32h7xx_hal.h"


static uint16_t WM8978_REGVAL[58] =
    {
        0X0000, 0X0000, 0X0000, 0X0000, 0X0050, 0X0000, 0X0140, 0X0000,
        0X0000, 0X0000, 0X0000, 0X00FF, 0X00FF, 0X0000, 0X0100, 0X00FF,
        0X00FF, 0X0000, 0X012C, 0X002C, 0X002C, 0X002C, 0X002C, 0X0000,
        0X0032, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000,
        0X0038, 0X000B, 0X0032, 0X0000, 0X0008, 0X000C, 0X0093, 0X00E9,
        0X0000, 0X0000, 0X0000, 0X0000, 0X0003, 0X0010, 0X0010, 0X0100,
        0X0100, 0X0002, 0X0001, 0X0001, 0X0039, 0X0039, 0X0039, 0X0039,
        0X0001, 0X0001};

AudioCodec::AudioCodec(I2C_HandleTypeDef *hi2c, I2S_HandleTypeDef *hi2s)
{

    i2c = hi2c;
    i2s = hi2s;
    Init();
}

uint16_t AudioCodec::Read_Reg(uint8_t reg)
{
    return WM8978_REGVAL[reg];
}

uint8_t AudioCodec::Write_Reg(uint8_t reg, uint16_t val)
{
    uint8_t res;
    uint8_t RegAddr;
    uint8_t RegValue;
    RegAddr = (reg << 1) | ((uint8_t)((val >> 8) & 0x01)); //reg address + data highest bit

    RegValue = (uint8_t)(val & 0XFF);

    HAL_StatusTypeDef status = HAL_OK;
    uint8_t data[2];

    //     /* Assemble 2-byte data in WM8731 format */
    data[0] = RegAddr;  // 0x0C in your example
    data[1] = RegValue; // MSB byte of 16bit data

    status = HAL_I2C_Master_Transmit(i2c, CODEC_ADDRESS, data, 2, CODEC_LONG_TIMEOUT); // data is the start pointer of our array
                                                                                       //     /* Check the communication status */
    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        // Error handling, for example re-initialization of the I2C peripheral
        // Error_Handler();
    }

    WM8978_REGVAL[reg] = val;
    return res;
}

uint8_t AudioCodec::Init(void)
{
    uint8_t Res;
    Res = Write_Reg(0, 0); //soft reset WM8978
    if (Res)
    {
        return 1; //reset failed, WM8978 exception
    }

    Write_Reg(1, 0X1B);  //R1,MICEN 1(MIC enabled),BIASEN 1(emu on),VMIDSEL[1:0]:11(5K) R1,MICEN设置为1(MIC使能)，BIASEN设置为1(模拟工作，VMIDSEL[1:0]设置为：11（5）)
    Write_Reg(2, 0X1B0); //R2, ROUT1,LOUT1 output enabled(headphone),BOOSTENR,BOOSTENL enabled.
    Write_Reg(3, 0X6C);  //R3, LOUT2,ROUT2 output enabled(speaker on),RMIX,LMIX enabled

    Write_Reg(6, 0); //R6, MCLK from out
                     // CLK_Cfg();

    Write_Reg(43, 1 << 4); //R43,INVROUT2 inverted, drive speaker
    Write_Reg(47, 1 << 8); //R47,PGABOOSTL,left MIC got 20 db
    Write_Reg(48, 1 << 8); //R48,PGABOOSTR, right MIC got 20 db
    Write_Reg(49, 1 << 1); //R49,TSDEN, open hot protecting
    Write_Reg(10, 1 << 3); //R10,SOFTMUTE closed,128x sample rate, best SNR
    Write_Reg(14, 1 << 3); //R14,ADC 128x sample rate
        //I2S work mode

    //	//audio output setting
    ADDA_Cfg(1, 1);     //open DAC
    Input_Cfg(1, 0, 0); //close input channel
    Output_Cfg(1, 0);   //open DAC output
    I2S_Cfg(2, 0);
    //record setting
    // ADDA_Cfg(0, 1);     //open ADC
    // Input_Cfg(1, 1, 0); //open input channel(MIC&LINE IN)
    // Output_Cfg(0, 1);   //open BYPASS output
    // MIC_Gain(46);       //MIC db setting
    //
    HPvol_Set(HP_Volume, HP_Volume);
    SPKvol_Set(SPK_Volume);

    HAL_I2S_Transmit_DMA(i2s, (uint16_t *)&audiobuff[0], DMA_MAX((2 * BUFF_LEN) / AUDIODATA_SIZE)); // size must be in bytes
    return 0;
}

void AudioCodec::ADDA_Cfg(uint8_t dacen, uint8_t adcen)
{
    uint16_t regval;
    regval = Read_Reg(3); //read R3
    if (dacen)
        regval |= 3 << 0; //R3 set lowest 2 bits to 1,enable DACR&DACL
    else
        regval &= ~(3 << 0); //R3 set lowest 2 bits to 0,disable DACR&DACL.
    Write_Reg(3, regval);
    regval = Read_Reg(2); //read R2
    if (adcen)
        regval |= 3 << 0; //R2 set lowest bits to 1, enable ADCR&ADCL
    else
        regval &= ~(3 << 0); //R2 set lowest bits to 0, disable ADCR&ADCL.
    Write_Reg(2, regval);    //R2
}

void AudioCodec::Input_Cfg(uint8_t micen, uint8_t lineinen, uint8_t auxen)
{
    uint16_t regval;
    regval = Read_Reg(2);
    if (micen)
        regval |= 3 << 2; //open INPPGAENR,INPPGAENL(MIC��PGA�Ŵ�)
    else
        regval &= ~(3 << 2); //close INPPGAENR,INPPGAENL.
    Write_Reg(2, regval);
    regval = Read_Reg(44);
    if (micen)
        regval |= 3 << 4 | 3 << 0; //enable LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
    else
        regval &= ~(3 << 4 | 3 << 0); //disable LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
    Write_Reg(44, regval);
    if (lineinen)
        LINEIN_Gain(5); //LINE IN 0dB
    else
        LINEIN_Gain(0); //disable LINE IN
    if (auxen)
        AUX_Gain(7); //AUX 6dB
    else
        AUX_Gain(0); //disable AUX input
}

void AudioCodec::MIC_Gain(uint8_t gain)
{
    gain &= 0X3F;
    Write_Reg(45, gain);          //R45,left channel PGA config
    Write_Reg(46, gain | 1 << 8); //R46,right channel PGA config
}

//WM8978 L2/R2(Line In)db config(L2/R2-->ADC input db)
//gain:0~7,0 means channel mute,1~7,��Ӧ-12dB~6dB,3dB/Step
void AudioCodec::LINEIN_Gain(uint8_t gain)
{
    uint16_t regval;
    gain &= 0X07;
    regval = Read_Reg(47);
    regval &= ~(7 << 4); //reset old config
    Write_Reg(47, regval | gain << 4);
    regval = Read_Reg(48);
    regval &= ~(7 << 4); //reset old config
    Write_Reg(48, regval | gain << 4);
}
//WM8978 AUXR,AUXL(PWM audio config)db config(AUXR/L-->ADC input db)
//gain:0~7,0 channel mute,1~7,��Ӧ-12dB~6dB,3dB/Step
void AudioCodec::AUX_Gain(uint8_t gain)
{
    uint16_t regval;
    gain &= 0X07;
    regval = Read_Reg(47);
    regval &= ~(7 << 0); //reset old config
    Write_Reg(47, regval | gain << 0);
    regval = Read_Reg(48);
    regval &= ~(7 << 0); //reset old config
    Write_Reg(48, regval | gain << 0);
}

//WM8978 output setting
//dacen:DAC output enabled(1)/disabled(0)
//bpsen:Bypass output(record,include MIC,LINE IN,AUX��) enabled(1)/disabled(0)
void AudioCodec::Output_Cfg(uint8_t dacen, uint8_t bpsen)
{
    uint16_t regval = 0;
    if (dacen)
        regval |= 1 << 0; //DAC output enabled
    if (bpsen)
    {
        regval |= 1 << 1; //BYPASS enabled
        regval |= 5 << 2; //0dB
    }
    Write_Reg(50, regval);
    Write_Reg(51, regval);
}

//Headphone volume
void AudioCodec::HPvol_Set(uint8_t voll, uint8_t volr)
{
    HP_Volume = voll;

    voll &= 0X3F;
    volr &= 0X3F;
    if (voll == 0)
        voll |= 1 << 6; //volume is 0, mute on
    if (volr == 0)
        volr |= 1 << 6;             //volume is 0, mute on
    Write_Reg(52, voll);            //R52, left channel volume
    Write_Reg(53, volr | (1 << 8)); //R53, right channel volume
}

//speaker volume
//voll:left channel volume(0~63)
void AudioCodec::SPKvol_Set(uint8_t volx)
{
    SPK_Volume = volx;
    volx &= 0X3F;
    if (volx == 0)
        volx |= 1 << 6;             //volume is 0, mute on
    Write_Reg(54, volx);            //R54, left channel audio volume
    Write_Reg(55, volx | (1 << 8)); //R55, right channel audio volume
}

//I2S working mode
//fmt:0,LSB;1,MSB;2,I2S;3,PCM/DSP;
//len:0,16bist;1,20bits;2,24bits;3,32bits;
void AudioCodec::I2S_Cfg(uint8_t fmt, uint8_t len)
{
    fmt &= 0x02;
    len &= 0x03;
    Write_Reg(4, (fmt << 3) | (len << 5)); //R4,WM8978 working mode
}

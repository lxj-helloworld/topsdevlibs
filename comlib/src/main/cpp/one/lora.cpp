#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define LOGD printf
#define LOGE printf

#define SPI_SEND_BUF_SIZE 256 //(data 512) + (frame control 7)
#define SPI_RECV_BUF_SIZE 256  //(last frame 1543 + 4096)
#define SPI_SEND_FRM_SIZE 128
#define SPI_FRM_HEADER_SIZE 3
#define SPI_MAX_FRM_LEN 1536
#define SPI_FRM_SIZE_MAX (SPI_MAX_FRM_LEN+SPI_FRM_HEADER_SIZE)
#define HIGH 1
#define LOW  0

#define LoRaTxTimeOut 5
#define RECV_CONTINUE_MODE     1
#define RECV_SINGLE_MODE       0

#define TAG "myDemo-jni" // 这个是自定义的LOG的标识

#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG ,__VA_ARGS__) // 定义LOGD类型
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG ,__VA_ARGS__) // 定义LOGI类型
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,TAG ,__VA_ARGS__) // 定义LOGW类型
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG ,__VA_ARGS__) // 定义LOGE类型
#define LOGF(...) __android_log_print(ANDROID_LOG_FATAL,TAG ,__VA_ARGS__) // 定义LOGF类型



typedef  struct {
    int Power;
    int SignalBw;          /* LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,*/
    /* 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]*/
    int  SpreadingFactor;  /* LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]*/
    int  ErrorCoding;      /* LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]*/
    int  CrcOn;            /* [0: OFF, 1: ON]*/
    int  ImplicitHeaderOn; /* [0: OFF, 1: ON]*/
    int  RxSingleOn;       /* [0: Continuous, 1 Single]*/
    int  FreqHopOn;        /* [0: OFF, 1: ON]*/
    int  HopPeriod;        /* Hops every frequency hopping period symbols*/
    int  PayloadLength;
    int  CadTime;
    int  LowDatarateOptimize;
    int PreambleLength;
    int SysbTimeOut;
    int RFFrequency;
} tLoRaSettings;

int  LoRaReadBuffer(uint8_t addr,uint8_t size);
int  LoRaWriteBufffer(uint8_t addr, char *buffer, uint8_t size);
void  LoRaInit(void);
void  LoRaStartReceive(uint8_t recv_mode);
int  LoRaStartSend(char *data, uint8_t data_len);
void  LoRaStartCAD(void);
uint8_t LoRaGetRxBytesLength(void);
void  LoRaSetFifoAddrPointer(uint8_t value);

void LoRaSetOpMode(uint8_t op_mode);
void LoRaSetFrequency(int freq);
void LoRaSetPower(int8_t power);
void LoRaSetBandWidth(uint8_t bw);
void LoRaSetSpreadingFactor(uint8_t factor);
void LoRaSetErrorCoding(uint8_t value);
void LoRaSetPacketCrcOn(uint8_t value);
void LoRaSetPreambleLength(uint16_t length);
void LoRaSetImplicitHeaderOn(uint8_t value);
void LoRaSetPayloadLength(uint8_t value);
void LoRaSetPaRamp(uint8_t value);
void LoRaSetLowDatarateOptimize(uint8_t value);
void LoRaSetSymbTimeOut(uint16_t value);
/*SX1278 Internal Registers Address for LoRa Mode*/
#define REG_FIFO 0x00
#define REG_OpMode 0x01
#define REG_FreMsb 0x06
#define REG_FreMid 0x07
#define REG_FreLsb 0x08
#define REG_PaConfig 0x09
#define REG_PaRamp 0x0A
#define REG_OCP 0x0B
#define REG_LNA 0x0C
#define REG_FIFOAddrPtr 0x0D
#define REG_FIFOTxBaseAddr 0x0E
#define REG_FIFORxBaseAddr 0x0F
#define REG_FIFORxCurrentAddr 0x10
#define REG_IRQFlagsMask 0x11
#define REG_IRQFlags 0x12
#define REG_RxNumBytes 0x13
#define REG_RxHeaderCntValueMsb 0x14
#define REG_RxHeaderCntValueLsb 0x15
#define REG_RxPacketCntValueMsb 0x16
#define REG_RxPacketCntValueLsb 0x17
#define REG_ModeState 0x18
#define REG_PacketSnrValue 0x19
#define REG_PacketRssiValue 0x1A
#define REG_RssiValue 0x1B
#define REG_HopChannel 0x1C
#define REG_ModemConfig1 0x1D
#define REG_ModemConfig2 0x1E
#define REG_SymbTimeoutLsb 0x1F
#define REG_PreambleMsb 0x20
#define REG_PreambleLsb 0x21
#define REG_PayloadLength 0x22
#define REG_MaxPayloadLength 0x23
#define REG_HopPeriod 0x24
#define REG_FIFORxBytesAddr 0x25
#define REG_ModemConfig3 0x26
#define REG_FEIMSB 0x28
#define REG_FEIMIB 0x29
#define REG_FEILSB 0x2A
#define REG_DetectOptimize 0x31
#define REG_InvertIQ 0x33
#define REG_DetectionThreshold 0x37
#define REG_DioMapping1 0x40
#define REG_DioMapping2 0x41
#define REG_Version 0x42
#define REG_TCXO 0x4B
#define REG_PaDAC 0x4D
#define REG_FormerTemp 0x5B
#define REG_AgcRef 0x61
#define REG_AgcThresh1 0x62
#define REG_AgcThresh2 0x63
#define REG_AgcThresh3 0x64

/*SX1278 LoRa bit control definition*/
/*RegOpMode*/
#define RFLR_OPMODE_LONGRANGEMODE_MASK 0x7F
#define RFLR_OPMODE_LONGRANGEMODE_OFF 0x00 /*Default*/
#define RFLR_OPMODE_LONGRANGEMODE_ON 0x80

#define RFLR_OPMODE_ACCESSSHAREDREG_MASK 0xBF
#define RFLR_OPMODE_ACCESSSHAREDREG_ENABLE 0x40
#define RFLR_OPMODE_ACCESSSHAREDREG_DISABLE 0x00 /*Default*/

#define RFLR_OPMODE_FREQMODE_ACCESS_MASK 0xF7
#define RFLR_OPMODE_FREQMODE_ACCESS_LF 0x08 /*Default*/
#define RFLR_OPMODE_FREQMODE_ACCESS_HF 0x00

#define RFLR_OPMODE_MASK 0xF8
#define RFLR_OPMODE_SLEEP 0x00
#define RFLR_OPMODE_STANDBY 0x01 /*Default*/
#define RFLR_OPMODE_SYNTHESIZER_TX 0x02
#define RFLR_OPMODE_TRANSMITTER 0x03
#define RFLR_OPMODE_SYNTHESIZER_RX 0x04
#define RFLR_OPMODE_RECEIVER 0x05
#define RFLR_OPMODE_SHARE_FSK 0xC0
/* LoRa specific modes*/
#define RFLR_OPMODE_RECEIVER_SINGLE 0x06
#define RFLR_OPMODE_CAD 0x07

/** RegPaConfig*/
#define RFLR_PACONFIG_PASELECT_MASK 0x7F
#define RFLR_PACONFIG_PASELECT_PABOOST 0x80
#define RFLR_PACONFIG_PASELECT_RFO 0x00 /*Default*/

#define RFLR_PACONFIG_MAX_POWER_MASK 0x8F

#define RFLR_PACONFIG_OUTPUTPOWER_MASK 0xF0

/** RegPaRamp*/
#define RFLR_PARAMP_TXBANDFORCE_MASK 0xEF
#define RFLR_PARAMP_TXBANDFORCE_BAND_SEL 0x10
#define RFLR_PARAMP_TXBANDFORCE_AUTO 0x00 /*Default*/

#define RFLR_PARAMP_MASK 0xF0
#define RFLR_PARAMP_3400_US 0x00
#define RFLR_PARAMP_2000_US 0x01
#define RFLR_PARAMP_1000_US 0x02
#define RFLR_PARAMP_0500_US 0x03
#define RFLR_PARAMP_0250_US 0x04
#define RFLR_PARAMP_0125_US 0x05
#define RFLR_PARAMP_0100_US 0x06
#define RFLR_PARAMP_0062_US 0x07
#define RFLR_PARAMP_0050_US 0x08
#define RFLR_PARAMP_0040_US 0x09 /*Default*/
#define RFLR_PARAMP_0031_US 0x0A
#define RFLR_PARAMP_0025_US 0x0B
#define RFLR_PARAMP_0020_US 0x0C
#define RFLR_PARAMP_0015_US 0x0D
#define RFLR_PARAMP_0012_US 0x0E
#define RFLR_PARAMP_0010_US 0x0F

/** RegOcp**/
#define RFLR_OCP_MASK 0xDF
#define RFLR_OCP_ON 0x20 /*Default*/
#define RFLR_OCP_OFF 0x00

#define RFLR_OCP_TRIM_MASK 0xE0
#define RFLR_OCP_TRIM_045_MA 0x00
#define RFLR_OCP_TRIM_050_MA 0x01
#define RFLR_OCP_TRIM_055_MA 0x02
#define RFLR_OCP_TRIM_060_MA 0x03
#define RFLR_OCP_TRIM_065_MA 0x04
#define RFLR_OCP_TRIM_070_MA 0x05
#define RFLR_OCP_TRIM_075_MA 0x06
#define RFLR_OCP_TRIM_080_MA 0x07
#define RFLR_OCP_TRIM_085_MA 0x08
#define RFLR_OCP_TRIM_090_MA 0x09
#define RFLR_OCP_TRIM_095_MA 0x0A
#define RFLR_OCP_TRIM_100_MA 0x0B /*Default*/
#define RFLR_OCP_TRIM_105_MA 0x0C
#define RFLR_OCP_TRIM_110_MA 0x0D
#define RFLR_OCP_TRIM_115_MA 0x0E
#define RFLR_OCP_TRIM_120_MA 0x0F
#define RFLR_OCP_TRIM_130_MA 0x10
#define RFLR_OCP_TRIM_140_MA 0x11
#define RFLR_OCP_TRIM_150_MA 0x12
#define RFLR_OCP_TRIM_160_MA 0x13
#define RFLR_OCP_TRIM_170_MA 0x14
#define RFLR_OCP_TRIM_180_MA 0x15
#define RFLR_OCP_TRIM_190_MA 0x16
#define RFLR_OCP_TRIM_200_MA 0x17
#define RFLR_OCP_TRIM_210_MA 0x18
#define RFLR_OCP_TRIM_220_MA 0x19
#define RFLR_OCP_TRIM_230_MA 0x1A
#define RFLR_OCP_TRIM_240_MA 0x1B

/** RegLna*/
#define RFLR_LNA_GAIN_MASK 0x1F
#define RFLR_LNA_GAIN_G1 0x20 /*Default*/
#define RFLR_LNA_GAIN_G2 0x40
#define RFLR_LNA_GAIN_G3 0x60
#define RFLR_LNA_GAIN_G4 0x80
#define RFLR_LNA_GAIN_G5 0xA0
#define RFLR_LNA_GAIN_G6 0xC0

#define RFLR_LNA_BOOST_LF_MASK 0xE7
#define RFLR_LNA_BOOST_LF_DEFAULT 0x00 /*Default*/
#define RFLR_LNA_BOOST_LF_GAIN 0x08
#define RFLR_LNA_BOOST_LF_IP3 0x10
#define RFLR_LNA_BOOST_LF_BOOST 0x18

#define RFLR_LNA_RXBANDFORCE_MASK 0xFB
#define RFLR_LNA_RXBANDFORCE_BAND_SEL 0x04
#define RFLR_LNA_RXBANDFORCE_AUTO 0x00 /*Default*/

#define RFLR_LNA_BOOST_HF_MASK 0xFC
#define RFLR_LNA_BOOST_HF_OFF 0x00 /*Default*/
#define RFLR_LNA_BOOST_HF_ON 0x03

/** RegFifoAddrPtr*/
#define RFLR_FIFOADDRPTR 0x00 /*Default*/

/** RegFifoTxBaseAddr*/
#define RFLR_FIFOTXBASEADDR 0x80 /*Default*/

/** RegFifoTxBaseAddr*/
#define RFLR_FIFORXBASEADDR 0x00	// Default

/** RegIrqFlagsMask*/
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK 0x80
#define RFLR_IRQFLAGS_RXDONE_MASK 0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK 0x20
#define RFLR_IRQFLAGS_VALIDHEADER_MASK 0x10
#define RFLR_IRQFLAGS_TXDONE_MASK 0x08
#define RFLR_IRQFLAGS_CADDONE_MASK 0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK 0x02
#define RFLR_IRQFLAGS_CADDETECTED_MASK 0x01

/** RegIrqFlags*/
#define RFLR_IRQFLAGS_RXTIMEOUT 0x80
#define RFLR_IRQFLAGS_RXDONE 0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR 0x20
#define RFLR_IRQFLAGS_VALIDHEADER 0x10
#define RFLR_IRQFLAGS_TXDONE 0x08
#define RFLR_IRQFLAGS_CADDONE 0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL 0x02
#define RFLR_IRQFLAGS_CADDETECTED 0x01

#define RXTIMEOUT_IRQ_ENABLE 0x7F
#define RXDONE_IRQ_ENABLE 0xBF
#define PAYLOADCRCERROR_IRQ_ENABLE 0xDF
#define VALIDHEADER_IRQ_ENABLE 0xEF
#define TXDONE_IRQ_ENABLE 0xF7
#define CADDONE_IRQ_ENABLE 0xFB
#define PHSSCHANGEDCHANNEL_IRQ_ENABLE 0xFD
#define CADDETECTED_IRQ_ENABLE 0xFE

#define RFLR_MODEMCONFIG1_BW_MASK 0x0F
#define RFLR_MODEMCONFIG1_BW_7_81_KHZ 0x00
#define RFLR_MODEMCONFIG1_BW_10_41_KHZ 0x10
#define RFLR_MODEMCONFIG1_BW_15_62_KHZ 0x20
#define RFLR_MODEMCONFIG1_BW_20_83_KHZ 0x30
#define RFLR_MODEMCONFIG1_BW_31_25_KHZ 0x40
#define RFLR_MODEMCONFIG1_BW_41_66_KHZ 0x50
#define RFLR_MODEMCONFIG1_BW_62_50_KHZ 0x60
#define RFLR_MODEMCONFIG1_BW_125_KHZ 0x70	// Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ 0x80
#define RFLR_MODEMCONFIG1_BW_500_KHZ 0x90

#define RFLR_MODEMCONFIG1_CODINGRATE_MASK 0xF1
#define RFLR_MODEMCONFIG1_CODINGRATE_4_5 0x02
#define RFLR_MODEMCONFIG1_CODINGRATE_4_6 0x04	// Default
#define RFLR_MODEMCONFIG1_CODINGRATE_4_7 0x06
#define RFLR_MODEMCONFIG1_CODINGRATE_4_8 0x08

#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK 0xFE
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_ON 0x01
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF 0x00	// Default

/** RegModemConfig2*/
#define RFLR_MODEMCONFIG2_SF_MASK 0x0F
#define RFLR_MODEMCONFIG2_SF_6 0x60
#define RFLR_MODEMCONFIG2_SF_7 0x70			// Default
#define RFLR_MODEMCONFIG2_SF_8 0x80
#define RFLR_MODEMCONFIG2_SF_9 0x90
#define RFLR_MODEMCONFIG2_SF_10 0xA0
#define RFLR_MODEMCONFIG2_SF_11 0xB0
#define RFLR_MODEMCONFIG2_SF_12 0xC0

#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK 0xF7
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON 0x08
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF 0x00

#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK 0xFB
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON 0x04
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF 0x00		// Default

#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK 0xFC
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB 0x00		// Default

/** RegHopChannel (Read Only)*/
#define RFLR_HOPCHANNEL_PLL_LOCK_TIMEOUT_MASK 0x7F
#define RFLR_HOPCHANNEL_PLL_LOCK_FAIL 0x80
#define RFLR_HOPCHANNEL_PLL_LOCK_SUCCEED 0x00		// Default

#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_MASK 0xBF
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_ON 0x40
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_OFF 0x00		// Default

#define RFLR_HOPCHANNEL_CHANNEL_MASK 0x3F

/** RegSymbTimeoutLsb*/
#define RFLR_SYMBTIMEOUTLSB_SYMBTIMEOUT 0x64		// Default

/** RegPreambleLengthMsb*/
#define RFLR_PREAMBLELENGTHMSB 0x00			// Default

/** RegPreambleLengthLsb*/
#define RFLR_PREAMBLELENGTHLSB 0x08			// Default

/** RegPayloadLength*/
#define RFLR_PAYLOADLENGTH 0x0E				// Default

/** RegPayloadMaxLength*/
#define RFLR_PAYLOADMAXLENGTH 0xFF			// Default

/** RegHopPeriod*/
#define RFLR_HOPPERIOD_FREQFOPPINGPERIOD 0x00		// Default

/** RegDioMapping1*/
#define RFLR_DIOMAPP1_DIO0_MASK 0x3F
#define RFLR_DIOMAPP1_DIO0_RXDONE 0x00			// Default
#define RFLR_DIOMAPP1_DIO0_TXDONE 0x40
#define RFLR_DIOMAPP1_DIO0_CADDONE 0x80
#define RFLR_DIOMAPP1_DIO0_11 0xC0

#define RFLR_DIOMAP1_DIO1_MASK 0xCF
#define RFLR_DIOMAP1_DIO1_RXTIMEOUT 0x00		// Default
#define RFLR_DIOMAP1_DIO1_FHSSCHANGECHANNEL 0x10
#define RFLR_DIOMAP1_DIO1_CADDETECTED 0x20
#define RFLR_DIOMAP1_DIO1_11 0x30

#define RFLR_DIOMAPP1_DIO2_MASK 0xF3
#define RFLR_DIOMAPP1_DIO2_00 0x00			// Default
#define RFLR_DIOMAPP1_DIO2_01 0x04
#define RFLR_DIOMAPP1_DIO2_10 0x08
#define RFLR_DIOMAPP1_DIO2_11 0x0C

#define RFLR_DIOMAPP1_DIO3_MASK 0xFC
#define RFLR_DIOMAPP1_DIO3_CADDONE 0x00			// Default
#define RFLR_DIOMAPP1_DIO3_VALIDHEADER 0x01
#define RFLR_DIOMAPP1_DIO3_PAYLOADCRCERROR 0x02
#define RFLR_DIOMAPP1_DIO3_11 0x03

/** RegDioMapping2*/
#define RFLR_DIOMAP2_DIO4_MASK 0x3F
#define RFLR_DIOMAP2_DIO4_CADDETECTED 0x00		// Default
#define RFLR_DIOMAP2_DIO4_PIILOCK 0x40
#define RFLR_DIOMAP2_DIO4_PIILCOK 0x80
#define RFLR_DIOMAP2_DIO4_11 0xC0

#define RFLR_DIOMAP2_DIO5_MASK 0xCF
#define RFLR_DIOMAP2_DIO5_MODEREADY 0x00		// Default
#define RFLR_DIOMAP2_DIO5_CLKOUT 0x10
#define RFLR_DIOMAP2_DIO5_11 0x30

#define RFLR_DIOMAPPING2_MAP_MASK 0xFE
#define RFLR_DIOMAPPING2_MAP_PREAMBLEDETECT 0x01
#define RFLR_DIOMAPPING2_MAP_RSSI 0x00			// Default

/** RegPllHop*/
#define RFLR_PLLHOP_FASTHOP_MASK 0x7F
#define RFLR_PLLHOP_FASTHOP_ON 0x80
#define RFLR_PLLHOP_FASTHOP_OFF 0x00			// Default

/** RegTcxo*/
#define RFLR_TCXO_TCXOINPUT_MASK 0xEF
#define RFLR_TCXO_TCXOINPUT_ON 0x10
#define RFLR_TCXO_TCXOINPUT_OFF 0x00			// Default

/** RegPaDac*/
#define RFLR_PADAC_20DBM_MASK 0xF8
#define RFLR_PADAC_20DBM_ON 0x07
#define RFLR_PADAC_20DBM_OFF 0x04			// Default

/** RegPll*/
#define RFLR_PLL_BANDWIDTH_MASK 0x3F
#define RFLR_PLL_BANDWIDTH_75 0x00
#define RFLR_PLL_BANDWIDTH_150 0x40
#define RFLR_PLL_BANDWIDTH_225 0x80
#define RFLR_PLL_BANDWIDTH_300 0xC0			// Default

/** RegPllLowPn*/
#define RFLR_PLLLOWPN_BANDWIDTH_MASK 0x3F
#define RFLR_PLLLOWPN_BANDWIDTH_75 0x00
#define RFLR_PLLLOWPN_BANDWIDTH_150 0x40
#define RFLR_PLLLOWPN_BANDWIDTH_225 0x80
#define RFLR_PLLLOWPN_BANDWIDTH_300 0xC0		// Default

/** RegModemConfig3*/
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK 0xF7
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON 0x08
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF 0x00	// Default

#define RFLR_MODEMCONFIG3_AGCAUTO_MASK 0xFB
#define RFLR_MODEMCONFIG3_AGCAUTO_ON 0x04		// Default
#define RFLR_MODEMCONFIG3_AGCAUTO_OFF 0x00

static const char *lora_spi_dev =  "/dev/spidev1.0";
static uint8_t lora_spi_mode = 0;
static uint8_t lora_bits = 8;
static uint32_t lora_speed = 40000000;
static uint16_t delay;

static int lora_spi_fd = -1;

//static pthread_t tidFrm;
//static pthread_attr_t attrFrm;
static char lora_send_buf[SPI_SEND_BUF_SIZE];
static char lora_recv_buf[SPI_RECV_BUF_SIZE];
static uint16_t saved_size = 0;
static struct spi_ioc_transfer lora_tr;

#define RSSI_OFFSET_LF -164
#define RSSI_OFFSET_HF -157
#define XTAL_FREQ 32000000
#define FREQ_STEP 61.03515625

tLoRaSettings kLoRaParaDefault = {
        20,        //LoRaSetPower               /*Power    默认最大20 不需要设置  */
        7,         //LoRaSetBandWidth           /*SignalBandWith  带宽;     [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,*/
        /* 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]*/
        8,        //LoRaSetSpreadingFactor      /*SpreadingFactor; 扩频因子 决定传输速率    [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]*/
        1,        //LoRaSetErrorCoding          /*ErrorCoding;  错误编码       [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]*/
        0,         //LoRaSetPacketCrcOn         /*CrcOn;    crc开启标识          [0: OFF, 1: ON]*/
        0,        //LoRaSetImplicitHeaderOn     /*ImplicitHeaderOn; 隐式报头  [0: OFF, 1: ON]*/
        1,                                      /*RxSingleOn;        连续接收或者单帧接收 [0: Continuous, 1 Single]*/
        0,                                      /*FreqHopOn;       跳频开启   [0: OFF, 1: ON]*/
        0,                                      /*HopPeriod;      跳频周期    [0:disabled]*/
        255,      //LoRaSetPayloadLength        /*PayloadLength;  负载长度  */
        5,                                      /*CadTime; 用不着  */
        1,        //LoRaSetLowDatarateOptimize  /*LowDatarateOptimize; 低速率优化 */
        0x0A28,  //LoRaSetPreambleLength        /*PreambleLength; 前导码长度 0A28 */
        0x03FF,    //LoRaSetSymbTimeOut         /*SysbolTimeOut    */
        0x1D5AF420,  //LoRaSetFrequency         /* RFFrequency;  中心频率  手动输入0x1D5AF420*/
};

enum eChannelState {
    CS_CADING,
    CS_RECVING,
    CS_SLEEP,
    CS_SENDING,
    CS_RECVEND,
};

tLoRaSettings             lora_setting;
static enum eChannelState lora_state;

int  LoRaReadRegister(uint8_t addr);
void LoRaWriteRegister(uint8_t addr, uint8_t data);

void gpio_set_cs(int value)
{
    if (value) {
        system("echo 1 > /sys/class/gpio/gpio2/value");
        usleep(10);
    } else {
        system("echo 0 > /sys/class/gpio/gpio2/value");
        usleep(10);
    }
}

void gpio_cs_init(void)
{
    system("echo 1 > /sys/class/gpio/gpio2/value");
}

int loraOpen()
{
    int ret;

    system("echo 1 > /sys/class/gpio/gpio56/value");
    system("echo 1 > /sys/class/gpio/gpio138/value");

    lora_spi_fd = open(lora_spi_dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (lora_spi_fd < 0)
    {
        LOGE("open lora failed\n");
        return lora_spi_fd;
    }

    lora_spi_mode |= SPI_MODE_0;
    lora_spi_mode &= ~SPI_CS_HIGH;

    ret = ioctl(lora_spi_fd, SPI_IOC_WR_MODE, &lora_spi_mode);
    if (ret == -1)
    {
        LOGE("set spi write mode failed\n");
        close(lora_spi_fd);
        return ret;
    }
    ioctl(lora_spi_fd, SPI_IOC_RD_MODE, &lora_spi_mode);
    if (ret == -1)
    {
        LOGE("set spi read mode failed\n");
        close(lora_spi_fd);
        return ret;
    }
    ioctl(lora_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &lora_bits);
    if (ret == -1)
    {
        LOGE("set spi write bit failed\n");
        close(lora_spi_fd);
        return ret;
    }
    ioctl(lora_spi_fd, SPI_IOC_RD_BITS_PER_WORD, &lora_bits);
    if (ret == -1)
    {
        LOGE("set spi read bit failed\n");
        close(lora_spi_fd);
        return ret;
    }
    ioctl(lora_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &lora_speed);
    if (ret == -1)
    {
        LOGE("set spi write lora_speed failed\n");
        close(lora_spi_fd);
        return ret;
    }
    ioctl(lora_spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &lora_speed);
    if (ret == -1)
    {
        LOGE("set spi read lora_speed failed\n");
        close(lora_spi_fd);
        return ret;
    }

    lora_tr.tx_buf = (unsigned long)lora_send_buf;
    lora_tr.rx_buf = (unsigned long)lora_recv_buf;
    lora_tr.len = sizeof(lora_send_buf);
    lora_tr.delay_usecs = 0;
    lora_tr.speed_hz = lora_speed;
    lora_tr.bits_per_word = lora_bits;
    gpio_cs_init();
    LoRaInit();
    return lora_spi_fd;
}

void loraClose(void)
{
    if (lora_spi_fd >= 0) {
        close(lora_spi_fd);
        lora_spi_fd = -1;
    }
    system("echo 0 > /sys/class/gpio/gpio56/value");
    system("echo 0 > /sys/class/gpio/gpio138/value");
}


int LoraWrite(char *buf, uint8_t len)
{
    int ret,data;
    int count = 1000;
    LOGE("0");
    ret = LoRaStartSend(buf, len);
    //data = LoRaReadRegister(REG_IRQFlags);
    //LOGE("WRITE IRQ status = %x\n",data);

    while((LoRaReadRegister(REG_IRQFlags) & 0x08) == 0){
        if(count == 0)
            return -1;
        count--;
        usleep(1000);
    };
    LoRaWriteRegister(REG_IRQFlags, 0xff);
    LoRaStartReceive(RECV_CONTINUE_MODE);
    return ret;
}

int LoraRead(char *buf, uint8_t len)
{
    int ret = 0;
    int data,count;
    //LoRaStartReceive(RECV_CONTINUE_MODE);
    data = LoRaReadRegister(REG_IRQFlags);
    LOGE("READ IRQ status = %x\n",data);

    if (data & 0x40 ) {
        count = LoRaReadRegister(REG_RxNumBytes);
        ret = LoRaReadBuffer(REG_FIFO, count);
    } else
        return -1;

    if(count < len) {
        memcpy(buf,lora_recv_buf+1,count);
        ret = count;
    }
    else {
        memcpy(buf,lora_recv_buf+1,len);
        ret = len;
    }
    LoRaWriteRegister(REG_IRQFlags,0xff);
    return ret;
}

int SpiWrite(char *buf, int buf_len)
{
    int ret;

    if (buf == NULL)
    {
        LOGE("input buffer null\n");
        return -1;
    }
    lora_tr.len = buf_len;

    ret = ioctl(lora_spi_fd, SPI_IOC_MESSAGE(1), &lora_tr);

    return ret;
}

//need input buffer size > 1543
int SpiRead(char *buf, int buf_size)
{
    int ret;
    int cur_read_len;

    if (buf == NULL)
    {
        LOGE("input buffer null\n");
        return -1;
    }

    if (buf_size < SPI_FRM_SIZE_MAX)
    {
        LOGE("too little buffer size\n");
        return -2;
    }

    if ((buf_size - saved_size) > SPI_RECV_BUF_SIZE)
    {
        cur_read_len = SPI_RECV_BUF_SIZE;
    }
    else
    {
        cur_read_len = (buf_size - saved_size);
    }
    lora_tr.len = cur_read_len;

    memcpy(lora_send_buf, buf, buf_size);
    ret = ioctl(lora_spi_fd, SPI_IOC_MESSAGE(1), &lora_tr);

    return ret;
}

int LoRaReadBuffer(uint8_t addr, uint8_t size)
{
    int ret;

    memset(lora_recv_buf, 0, size+1);
    memset(lora_send_buf, 0, size+1);
    lora_send_buf[0] = (addr & 0x7F);
    ret = SpiWrite(lora_send_buf,size+1);

    return ret;
}

int LoRaWriteBufffer(uint8_t addr, char *buffer, uint8_t size)
{
    int i, ret;

    memset(lora_send_buf, 0 , size+1);
    lora_send_buf[0] = (addr|0x80);
    memcpy(lora_send_buf+1, buffer, size);

    ret = SpiWrite(buffer,size+1);
    LOGE("send to addr=%x data len = %d :\n",addr,ret);
    for(i=0; i<size+1; i++)
        LOGE("0x%x ",lora_send_buf[i]);
    LOGE("\n");
    return ret;
}

int LoRaReadRegister(uint8_t addr)
{
    int i,ret;
    lora_send_buf[0] = addr & 0x7F;
    lora_send_buf[1] = 0;
    ret = SpiWrite(lora_send_buf,2);
    //LOGE("recv reg: 0x%x data = 0x%x 0x%x 0x%x\n",addr,lora_recv_buf[0],lora_recv_buf[1],lora_recv_buf[2]);
    return lora_recv_buf[1];
}

void LoRaWriteRegister(uint8_t addr, uint8_t data)
{
    int i,ret;
    lora_send_buf[0] = (addr | 0x80);
    lora_send_buf[1] = data;
    ret = SpiWrite(lora_send_buf,2);
    //LOGE("send register addr len = %d addr=0x%x data=0x%x\n",ret,addr,data);

    //ret = LoRaReadRegister(addr);
}

static void LoRaSetPa20dBm(uint8_t enable)
{
    uint8_t pa_dac;
    if (enable == 1) {
        pa_dac = 0x87;
    } else {
        pa_dac = 0x84;
    }

    LoRaWriteRegister(REG_PaDAC, pa_dac);
}
static void LoRaReset(void)
{
    system("echo 0 > /sys/class/gpio/gpio138/value");
    usleep(100);
    system("echo 1 > /sys/class/gpio/gpio138/value");
}
void LoRaInit(void)
{
    uint8_t nirq = 0x00;
    nirq       = RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK | RFLR_IRQFLAGS_VALIDHEADER_MASK;
    LoRaReset();
    LoRaSetOpMode(RFLR_OPMODE_SLEEP);   /*sx1278 only can be set lora mode in sleep mode*/
    LoRaSetOpMode(RFLR_OPMODE_STANDBY); /*to set sx1278 in lora mode*/
    LoRaSetFrequency(kLoRaParaDefault.RFFrequency);
    LoRaSetSpreadingFactor(kLoRaParaDefault.SpreadingFactor);
    LoRaSetErrorCoding(kLoRaParaDefault.ErrorCoding);
    LoRaSetPacketCrcOn(kLoRaParaDefault.CrcOn);
    LoRaSetBandWidth(kLoRaParaDefault.SignalBw);
    LoRaSetImplicitHeaderOn(kLoRaParaDefault.ImplicitHeaderOn);
    LoRaSetPayloadLength(kLoRaParaDefault.PayloadLength);
    LoRaSetLowDatarateOptimize(kLoRaParaDefault.LowDatarateOptimize);
    LoRaSetPower(kLoRaParaDefault.Power);
    LoRaSetSymbTimeOut(kLoRaParaDefault.SysbTimeOut);
    LoRaWriteRegister(REG_IRQFlagsMask, nirq);
    LoRaSetOpMode(RFLR_OPMODE_SLEEP);
    LOGE("lora init over!\n");
}

void LoRaStartReceive(uint8_t recv_mode)
{
    uint8_t dio  = 0;
    lora_state = CS_RECVING;
    kLoRaParaDefault.PreambleLength = 2600;
    dio        = RFLR_DIOMAPP1_DIO0_RXDONE | RFLR_DIOMAP1_DIO1_RXTIMEOUT | RFLR_DIOMAPP1_DIO2_00;
    LoRaSetOpMode(RFLR_OPMODE_STANDBY);
    LoRaWriteRegister(REG_DioMapping1, dio);
    LoRaSetPreambleLength(kLoRaParaDefault.PreambleLength);
    LoRaWriteRegister(REG_FIFOAddrPtr, 0x00);
    if (recv_mode == 0) {
        LoRaSetOpMode(RFLR_OPMODE_RECEIVER_SINGLE);
    } else {
        LoRaSetOpMode(RFLR_OPMODE_RECEIVER);
    }
}

int LoRaStartSend(char *data, uint8_t data_len)
{
    uint8_t dio = 0;
    int ret = 0;

    lora_state = CS_SENDING;
    kLoRaParaDefault.PreambleLength = 30;
    dio = RFLR_DIOMAPP1_DIO0_TXDONE;

    LoRaSetOpMode(RFLR_OPMODE_STANDBY);
    LoRaWriteRegister(REG_DioMapping1, dio);
    LoRaSetPreambleLength(kLoRaParaDefault.PreambleLength);
    LoRaWriteRegister(REG_PayloadLength, data_len);
    LoRaWriteRegister(REG_FIFOTxBaseAddr, 0x00);
    LoRaWriteRegister(REG_FIFOAddrPtr, 0x00);
    ret = LoRaWriteBufffer(REG_FIFO, data, data_len);
    LoRaSetOpMode(RFLR_OPMODE_TRANSMITTER);

    return ret;
}

void LoRaStartCAD(void)
{
    uint8_t dio = 0;
    if (lora_state != CS_RECVING && lora_state != CS_SENDING) {
        lora_state = CS_CADING;
        LoRaSetOpMode(RFLR_OPMODE_STANDBY);
        dio = RFLR_DIOMAPP1_DIO0_CADDONE | RFLR_DIOMAP1_DIO1_CADDETECTED | RFLR_DIOMAPP1_DIO2_00 | RFLR_DIOMAPP1_DIO3_VALIDHEADER;
        LoRaWriteRegister(REG_DioMapping1, dio);
        LoRaSetOpMode(RFLR_OPMODE_CAD);
    }
}

uint8_t LoRaGetRxBytesLength(void)
{
    uint8_t num;
    num = LoRaReadRegister(REG_RxNumBytes);
    return num;
}

void LoRaSetFifoAddrPointer(uint8_t value)
{
    LoRaWriteRegister(REG_FIFOAddrPtr, value);
}


void LoRaSetOpMode(uint8_t op_mode)
{
    switch (op_mode)
    {
        case RFLR_OPMODE_CAD:
            lora_state = CS_CADING;
            break;
        case RFLR_OPMODE_RECEIVER:
        case RFLR_OPMODE_RECEIVER_SINGLE:
            lora_state = CS_RECVING;
            break;
        case RFLR_OPMODE_SLEEP:
        case RFLR_OPMODE_STANDBY:
            lora_state = CS_SLEEP;
            break;
        case RFLR_OPMODE_TRANSMITTER:
            lora_state = CS_SENDING;
            break;
        default:
            lora_state = CS_SLEEP;
            break;
    }
    op_mode |= 0x88;
    LoRaWriteRegister(REG_OpMode, op_mode);
}

void LoRaSetFrequency(int freq)
{
    char freq_buf[3];
    freq        = (int)(freq / FREQ_STEP);
    freq_buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    freq_buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    freq_buf[2] = (uint8_t)(freq & 0xFF);
    LoRaWriteBufffer(REG_FreMsb, freq_buf, 3);
}


void LoRaSetPower(int8_t power)
{
    uint8_t pa_config;
    {
        if (power >= 17) {
            LoRaSetPa20dBm(1);
            if (power > 20)
                power = 20;
            pa_config = power - 5;
        } else {
            LoRaSetPa20dBm(0);
            if (power < 2)
                power = 2;
            pa_config = power - 2;
        }
        pa_config |= 0xF0;
    }
    LoRaWriteRegister(REG_PaConfig, pa_config);
}

void LoRaSetBandWidth(uint8_t bw)
{

    uint8_t para;

    para = LoRaReadRegister(REG_ModemConfig1);
    para = (para & RFLR_MODEMCONFIG1_BW_MASK) | (bw << 4);
    LoRaWriteRegister(REG_ModemConfig1, para);
}
void LoRaSetSpreadingFactor(uint8_t factor)
{

    uint8_t para;

    if (factor > 12)
        factor = 12;
    else if (factor < 6)
        factor = 6;
    if (6 == factor)
        LoRaWriteRegister(REG_DetectOptimize, 0xC5);
    else
        LoRaWriteRegister(REG_DetectOptimize, 0xC3);

    para = LoRaReadRegister(REG_ModemConfig2);
    para = (para & RFLR_MODEMCONFIG2_SF_MASK) | (factor << 4);
    LoRaWriteRegister(REG_ModemConfig2, para);
}

int LoRaGetFrequency(void)
{
    int freq;
    uint8_t data1,data2,data3;
    data1 = LoRaReadRegister(REG_FreMsb);
    data2 = LoRaReadRegister(REG_FreMid);
    data3 = LoRaReadRegister(REG_FreLsb);
    printf("data1 = %x  data2 = %x  data3 = %x\n",data1,data2,data3);
    freq = (data1<<16|data2<<8|data3) * FREQ_STEP;
    printf("freq = %d\n",freq);
    return freq;
}

uint8_t LoRaGetBandWidth(void)
{
    uint8_t data;
    data = LoRaReadRegister(REG_ModemConfig1);
    printf("data = %x\n",data);
    data = (data >> 4)& 0xF;
    return data;
}

uint8_t LoRaGetSpreadingFactor(void)
{
    uint8_t data;
    data = LoRaReadRegister(REG_ModemConfig2);
    printf("data = %x\n",data);
    data = (data >> 4)& 0xF;
    return data;
}

void LoRaSetErrorCoding(uint8_t value)
{
    uint8_t para;

    para = LoRaReadRegister(REG_ModemConfig1);
    para = (para & RFLR_MODEMCONFIG1_CODINGRATE_MASK) | (value << 1);
    LoRaWriteRegister(REG_ModemConfig1, para);
}
void LoRaSetPacketCrcOn(uint8_t value)
{
    uint8_t para;

    para = LoRaReadRegister(REG_ModemConfig2);
    para = (para & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) | (value << 2);
    LoRaWriteRegister(REG_ModemConfig2, para);
}
void LoRaSetPreambleLength(uint16_t length)
{
    LoRaWriteRegister(REG_PreambleMsb, (uint8_t)(length >> 8));
    LoRaWriteRegister(REG_PreambleLsb, (uint8_t)length);
}
void LoRaSetImplicitHeaderOn(uint8_t value)
{
    uint8_t para;

    para = LoRaReadRegister(REG_ModemConfig1);
    para = (para & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) | (value);
    LoRaWriteRegister(REG_ModemConfig1, para);
}
void LoRaSetPayloadLength(uint8_t value)
{
    LoRaWriteRegister(REG_PayloadLength, value);
}
void LoRaSetPaRamp(uint8_t value)
{
    uint8_t para;

    para = LoRaReadRegister(REG_PaRamp);
    para = (para & RFLR_PARAMP_MASK) | (value & 0x0F);
    LoRaWriteRegister(REG_PaRamp, para);
}
void LoRaSetLowDatarateOptimize(uint8_t value)
{
    uint8_t para;

    para = LoRaReadRegister(REG_ModemConfig3);
    para = (para & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) | (value << 3);
    LoRaWriteRegister(REG_ModemConfig3, para);
}
void LoRaSetSymbTimeOut(uint16_t value)
{
    uint8_t m_config2;

    value &= 0x03FF;
    m_config2 = LoRaReadRegister(REG_ModemConfig2);
    m_config2 = (m_config2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) | (uint8_t)(value >> 8);
    LoRaWriteRegister(REG_ModemConfig2, m_config2);
    LoRaWriteRegister(REG_SymbTimeoutLsb, (uint8_t)value);
}


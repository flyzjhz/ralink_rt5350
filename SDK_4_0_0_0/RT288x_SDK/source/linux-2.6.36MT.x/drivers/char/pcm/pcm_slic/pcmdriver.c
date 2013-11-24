#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <asm/addrspace.h>
#include <linux/slab.h> 
#include <asm/tc3162/cmdparse.h>
#include <asm/tc3162/tc3162.h>
#include "pcmdriver.h"
#include "error.h"
#include "pcmverify.h"
#include "queue.h"

#include <asm/rt2880/surfboardint.h>
#include <linux/pci.h>
#include "_pcm.h"

extern int CustomerVoice_HWInit(void);
extern int ProSLIC_HWInit(void);

#define NOT_TO_INIT 0
#define CONFIG_INIT 1

#define DEFAULT_RX_QUEUE_SIZE 300
#define DEFAULT_RX_SAMPLE_SIZE 160
#define DEFAULT_CH_NUM 2
#define MAX_DESC_COUNT 50
#define TX_BUF_TYPE 0
#define RX_BUF_TYPE 1
#define TX_DESC_TYPE 2
#define RX_DESC_TYPE 3

typedef union {
	struct {
		uint32 pcmMode :1;
		uint32 bitClock :3;
		uint32 sampleClock :1;
		uint32 reserved5 :3;
		uint32 fsLen :2;
		uint32 fsEdge :1;
		uint32 dataEdge :1;
		uint32 delay :1;
		uint32 reserved4 :3;
		uint32 bitOrder :1;
		uint32 byteOrder :1;
		uint32 frameCount :5;
		uint32 reserved3 :1;
		uint32 softReset :1;
		uint32 loopBack :1;
		uint32 cfgValid :1;
		uint32 reserved2 :1;
		uint32 probing :3;
		uint32 lbGarbageEnable :1;
	} bits;

	uint32 value;
} pcmCtrl_t;

typedef struct dmaBufferHeader_s {
	dma_addr_t dmaHandle;
	uint32 bufSize;
	char reserved[24];
} dmaBufferHeader_t;

#define TC3262_SLIC
#define regRead(reg) 			VPint(reg)
#define regWrite(reg,regValue) 	VPint(reg)=regValue
#define pause(x)		mdelay(x)
#define PCM_IDENTIFIER	"TC3262"

char* pcmIdentifier(void);
int pcmConfig(configNode_t* tmpconfigNode, int* flag);
void defaultConfigNodeSet(configNode_t* configNode);
int pcmConfigSetup(configNode_t* tmpConfigNode, int* flag);
void pcmRestart(int configSet);
void pcmDMAStop(int direction);
int pcmSend(uint8** txBuf, int sampleSize, int* flag);
void pcmRecv(void);
int pcmRecvFuncRegister(void* funcP);
uint8* pcmRecvChBufDequeue(void);
void rxBufEnq(uint8* chRxBuf, uint32 tmpSampleSize);
void rxBufNextSet(uint8* rxBuf, uint8* nextBuf);
uint8* rxBufNextGet(uint8* rxBuf);
void queueClean(void);
int pcmRecvSampleSizeGet(uint8* rxBuf);
int rxBufMagicNumGet(unsigned char* rxBuf);

void* pcmKmalloc(int size, int gfpFlag, int type);
void pcmKfree(void* addr, int type);
void* pcmSendBufAlloc(int size);
void pcmTxBufKfree(void* addr);
char* pcmRecvBufAlloc(int flag);
void pcmRecvChBufFree(uint8* rxBuf);
void pcmDescRecvBufFree(uint32 descBufAddr);

void descGet(int dbgCat, int dbgLevel, int direct, int descNum);
void descGetAll(int direct, int dbgCat, int dbgLevel);
char* regGetAll(char* dbg);

void descInit(void);
void softReset(void);
void dmaEnable(int direct, int enable);
int rxDescSet(int sampleSize, int index);
void rxDescBufSet(int index, uint8* rxChBuf);
void rxDescSetVaild(int sampleSize, uint8* rxChBuf);
int txDescSetVaild(int sampleSize);
void pollingEnable(int driect);
void descFree(void);
void txDescBufFree(void);
void prepcmCfgVaild(void);
void postpcmCfgVaild(void);
void intMaskSet(uint32 mask);
void timeSlotCfgInit(uint8* bitWidthArray);

static irqreturn_t pcmIsr(int irq, void *devId);
void rxBufNumSet(uint8* rxBuf);
void rxBufNumGet(uint8* rxBuf);


static desc_t* txDesc = NULL;
static desc_t* rxDesc = NULL;
static int txProcIdx = 0;
static int txIdx = 0;
static int txRound = 0;
static int rxProcIdx = 0;
static int rxIdx = 0;
static int rxRound = 0;
static int txDMADisFlag = 0;
static int rxDMADisFlag = 0;
int rxRecvCounter = 0;

configNode_t* configNode;
configNode_t curConfigNode;

struct tasklet_struct tasklet;
int (*recvFunc)(void);

static regAttr_t regMap[MAX_REG_NUM]= { 
{"pcmCtrl" , READ_ONLY|WRITE_ONLY, 0x1f7f1f1f, (OPERATIONAL_BASE|0x0), 0x0500040a},
{"txTimeSlotCfg0", READ_ONLY|WRITE_ONLY, 0x13ff13ff, (OPERATIONAL_BASE|0x4), 0x00080000},
{"txTimeSlotCfg1", READ_ONLY|WRITE_ONLY, 0x13ff13ff, (OPERATIONAL_BASE|0x8), 0x00180010},
{"txTimeSlotCfg2", READ_ONLY|WRITE_ONLY, 0x13ff13ff, (OPERATIONAL_BASE|0xC), 0x00280020},
{"txTimeSlotCfg3", READ_ONLY|WRITE_ONLY, 0x13ff13ff, (OPERATIONAL_BASE|0x10), 0x00380030},
{"rxTimeSlotCfg0", READ_ONLY|WRITE_ONLY, 0x13ff13ff, (OPERATIONAL_BASE|0x14), 0x00080000},
{"rxTimeSlotCfg1", READ_ONLY|WRITE_ONLY, 0x13ff13ff, (OPERATIONAL_BASE|0x18), 0x00180010},
{"rxTimeSlotCfg2", READ_ONLY|WRITE_ONLY, 0x13ff13ff, (OPERATIONAL_BASE|0x1C), 0x00280020},
{"rxTimeSlotCfg3", READ_ONLY|WRITE_ONLY, 0x13ff13ff, (OPERATIONAL_BASE|0x20), 0x00380030},
{"ISR", READ_ONLY|WRITE_ONLY, 0x000007ff, (OPERATIONAL_BASE|0x24), 0x00000000},
{"INTMask", READ_ONLY|WRITE_ONLY, 0x000007ff, (OPERATIONAL_BASE|0x28), 0x00000000},
{"txPolling", READ_ONLY|WRITE_ONLY, 0xffffffff, (OPERATIONAL_BASE|0x2C), 0x00000000},
{"rxPolling", READ_ONLY|WRITE_ONLY, 0xffffffff, (OPERATIONAL_BASE|0x30), 0x00000000},
{"txRingBaseAddr", READ_ONLY|WRITE_ONLY, 0xffffffff, (OPERATIONAL_BASE|0x34), 0x00000000},
{"rxRingBaseAddr", READ_ONLY|WRITE_ONLY, 0xffffffff, (OPERATIONAL_BASE|0x38), 0x00000000},
{"txrxRingSizeAndOff", READ_ONLY|WRITE_ONLY, 0x000000ff, (OPERATIONAL_BASE|0x3C), 0x000000C0},
{"txRxDMA", READ_ONLY|WRITE_ONLY, 0x0000000f, (OPERATIONAL_BASE|0x40), 0x0f000000},
};

void* pcmKmalloc(int size, int gfpFlag, int type) {
	void* ret;
	dmaBufferHeader_t bufHeader;

	bufHeader.bufSize = size + sizeof(dmaBufferHeader_t);
	ret = (void *) dma_alloc_coherent(NULL,bufHeader.bufSize,
			&bufHeader.dmaHandle, gfpFlag);
	if( ret == NULL)
		return NULL;
	
	memset(ret,0,bufHeader.bufSize);
	memcpy(ret, &bufHeader, sizeof(dmaBufferHeader_t));

	return (ret + sizeof(dmaBufferHeader_t));
}

void pcmKfree(void* addr, int type) {
	dmaBufferHeader_t bufHeader;

	memcpy(&bufHeader, addr - sizeof(dmaBufferHeader_t),
			sizeof(dmaBufferHeader_t));
	if (addr != NULL) {
		dma_free_coherent(NULL,bufHeader.bufSize, addr
				- sizeof(dmaBufferHeader_t), bufHeader.dmaHandle);
	}
}

void* pcmSendBufAlloc(int size) {
	return pcmKmalloc(size, GFP_ATOMIC, TX_BUF_TYPE);
}

void pcmTxBufKfree(void* addr) {
	uint32 tmpMagicNum = 0;
	if (addr != NULL) {
		tmpMagicNum = rxBufMagicNumGet(addr);
		if (tmpMagicNum == 0x78563412) {
			pcmKfree(addr - sizeof(callBackParam_t), RX_BUF_TYPE);
		} else {
			pcmKfree(addr, TX_BUF_TYPE);
		}
	}
}

void queueClean(void) {
	uint8* rxChBuf;
	while ((rxChBuf = dequeue()) != NULL) {
		pcmRecvChBufFree(rxChBuf);
	}
}

int pcmRecvSampleSizeGet(unsigned char* rxBuf) {
	callBackParam_t* callBackParam;
	if (rxBuf != NULL) {
		callBackParam = (callBackParam_t*) (rxBuf - sizeof(callBackParam_t));
		return callBackParam->rxSampleSize;
	}
	return 0;

}

int rxBufMagicNumGet(uint8* rxBuf) {
	callBackParam_t* callBackParam;
	if (rxBuf != NULL) {
		callBackParam = (callBackParam_t *) (rxBuf - sizeof(callBackParam_t));
		return callBackParam->magicNum;
	}
	return 0;
}

void rxBufNextSet(uint8* rxBuf, uint8* nextBuf) {
	callBackParam_t* callBackParam;
	if (rxBuf != NULL) {
		callBackParam = (callBackParam_t *) (rxBuf - sizeof(callBackParam_t));
		callBackParam->next = nextBuf;
	}
}

uint8* rxBufNextGet(uint8* rxBuf) {
	callBackParam_t* callBackParam;
	if (rxBuf != NULL) {
		callBackParam = (callBackParam_t *) (rxBuf - sizeof(callBackParam_t));
		return callBackParam->next;
	}
	return NULL;
}

void descGet(int dbgCat, int dbgLevel, int direct, int descNum) {
	int i;
	descStatus_t* tmpDescStat;
	if ((dbgCat & curConfigNode.debugCategory) && (dbgLevel
			& curConfigNode.debugLevel)) {
		if (direct == TX) {
			tmpDescStat = (descStatus_t*) &(txDesc[descNum].descStatus);
			printk("-------Tx %d th desc addr:0x%08lx--------\n", descNum,
					(uint32) (&txDesc[descNum]));
			printk(KERN_INFO "desc status:0x%08lx(ownership:%d,chvaild:0x%08x,sample size:%u)\n",tmpDescStat->value,tmpDescStat->bits.ownership,(uint8)tmpDescStat->bits.chValid,(uint16)(tmpDescStat->bits.sampleSize));
			for (i = 0; i < MAX_BUF_NUM; i++) {
				printk("		buf%d addr:0x%08lx \n", i, txDesc[descNum].bufAddr[i]);
			}
		} else {
			tmpDescStat = (descStatus_t*) &(rxDesc[descNum].descStatus);
			printk("-------Rx %dth desc addr:0x%08lx--------\n", descNum,
					(uint32) (&rxDesc[descNum]));
			printk(KERN_INFO "desc status:0x%08lx(ownership:%d,chvaild:0x%08x,sample size:%u)\n",tmpDescStat->value,tmpDescStat->bits.ownership,(uint8)tmpDescStat->bits.chValid,(uint16)(tmpDescStat->bits.sampleSize));
			for (i = 0; i < MAX_BUF_NUM; i++) {
				printk("		buf%d addr:0x%08lx \n", i, rxDesc[descNum].bufAddr[i]);
			}
		}
	}
}

void descGetAll(int direct, int dbgCat, int dbgLevel) {
	int i = 0;
	if ((dbgCat & curConfigNode.debugCategory) && (dbgLevel
			& curConfigNode.debugLevel)) {
		if (direct == TX) {
			for (i = 0; i < MAX_TX_DESC_NUM; i++) {
				descGet(dbgCat, dbgLevel, direct, i);
			}
		} else {
			for (i = 0; i < MAX_RX_DESC_NUM; i++) {
				descGet(dbgCat, dbgLevel, direct, i);
			}
		}
	}
}

char* regGetAll(char* dbg) {
	int i;
	sprintf(dbg, "--Reg Map--\n");
	for (i = 0; i < MAX_REG_NUM; i++) {
		if (regMap[i].addr != (OPERATIONAL_BASE|0x24)) {
			sprintf(dbg + strlen(dbg), "name:%s", regMap[i].name);
			sprintf(dbg + strlen(dbg), " Addr:0x%08lx", regMap[i].addr);
			sprintf(dbg + strlen(dbg), " value:0x%08lx", regRead(regMap[i].addr));
			sprintf(dbg + strlen(dbg), "\n----\n");
		}
	}
	return dbg;
}

void descInit(void) {
	txDesc = pcmKmalloc(sizeof(desc_t) * MAX_TX_DESC_NUM, GFP_KERNEL,
			TX_DESC_TYPE);
	rxDesc = pcmKmalloc(sizeof(desc_t) * MAX_RX_DESC_NUM, GFP_KERNEL,
			RX_DESC_TYPE);
	memset(txDesc, 0, (sizeof(desc_t)) * (MAX_TX_DESC_NUM));
	memset(rxDesc, 0, (sizeof(desc_t)) * (MAX_RX_DESC_NUM));
	regWrite(regMap[PCM_TX_DESC_RING_BASE].addr,K1_TO_PHYSICAL(txDesc));
	regWrite(regMap[PCM_RX_DESC_RING_BASE].addr,K1_TO_PHYSICAL(rxDesc));
	regWrite(regMap[PCM_TX_RX_DESC_RING_SIZE_OFFSET].addr,((MAX_DESC_OFF<<4)|(MAX_TX_DESC_NUM)));

}

void softReset(void) {
	uint32 regAddr = regRead(regMap[PCM_INTFACE_Ctrl].addr);
	uint32 tmpValue = 0xfeffffff;
	regWrite(regMap[PCM_INTFACE_Ctrl].addr,tmpValue&regAddr );
	pause(5);
	regAddr = regRead(regMap[PCM_INTFACE_Ctrl].addr);
	regWrite(regMap[PCM_INTFACE_Ctrl].addr,(~tmpValue)|regAddr);
}

void dmaEnable(int direct, int enable) {
	uint32 txDmaEnable = 0x00000001;
	uint32 rxDmaEnable = 0x00000002;
	uint32 regValue = 0;
	uint32 dmaCtrlReg = OPERATIONAL_BASE | TX_RX_DMA_CTRL;
	uint32 DMARegRead;

	DMARegRead = regRead(dmaCtrlReg);
	if (direct == TX) {
		if (enable == ENABLE) {
			regValue = DMARegRead | txDmaEnable;
			txDMADisFlag = 0;
		} else {
			regValue = DMARegRead & (~txDmaEnable);
			txDMADisFlag = 1;
		}
	} else {
		if (enable == ENABLE) {

			regValue = DMARegRead | rxDmaEnable;
			rxDMADisFlag = 0;
		} else {
			regValue = DMARegRead & (~rxDmaEnable);
			rxDMADisFlag = 1;
		}
	}
	regWrite(dmaCtrlReg,regValue);
}

int rxDescSet(int sampleSize, int index) {
	descStatus_t* tmpDescStat = (descStatus_t *) &(rxDesc[index].descStatus);
	uint32 dmaCtrlReg = OPERATIONAL_BASE | TX_RX_DMA_CTRL;

	if (tmpDescStat->bits.ownership == 1) {
		printk("full- 0x%08lx\n", regRead(dmaCtrlReg));
		return DESC_FULL;
	}

	tmpDescStat->bits.chValid = (uint8) ((1 << curConfigNode.chNum) - 1);
	tmpDescStat->bits.sampleSize = sampleSize;
	tmpDescStat->bits.ownership = 0x1;

	return SUCCESS;
}

void rxDescBufSet(int index, uint8* rxChBuf) {
	int i;
	int bufSize = 0;
	int tmpBitWidth = 0;
	for (i = 0; i < curConfigNode.chNum; i++) {
		rxDesc[index].bufAddr[i] = K1_TO_PHYSICAL(rxChBuf+bufSize);
		if (curConfigNode.bitWidth[i] == BITWIDTH_8) {
			tmpBitWidth = 8;
		} else {
			tmpBitWidth = 16;
		}
		bufSize += curConfigNode.rxSampleSize * (tmpBitWidth >> 3);
	}
}

void rxDescSetVaild(int sampleSize, uint8* rxChBuf) {
	rxDescBufSet(rxIdx, rxChBuf);
	if (!rxDescSet(sampleSize, rxIdx)) {
		printk("rx desc full \n");
	}

	rxIdx = (rxIdx + 1) % (MAX_RX_DESC_NUM);
	if (rxIdx == 0) {
		rxRound++;
	}
}

void pollingEnable(int direct) {
	uint32 tmpValue = 0x1;
	if (direct == TX) {
		regWrite((OPERATIONAL_BASE|TX_POLLING_OFF),tmpValue);
	} else {
		regWrite((OPERATIONAL_BASE|RX_POLLING_OFF),tmpValue);
	}
}

void descFree(void) {
	int i;
	uint32 tmpBufAddr;
	if (txDesc != NULL) {
		for (i = 0; i < MAX_TX_DESC_NUM; i++) {
			if ((char*) txDesc[i].bufAddr[0] != NULL) {
				tmpBufAddr = PHYSICAL_TO_K1(txDesc[i].bufAddr[0]);
				pcmKfree((char *) tmpBufAddr, TX_BUF_TYPE);
				txDesc[i].bufAddr[0] = (uint32) NULL;
			}
		}
		pcmKfree(txDesc, TX_DESC_TYPE);
	}
	if (rxDesc != NULL) {
		for (i = 0; i < MAX_RX_DESC_NUM; i++) {
			if ((char*) rxDesc[i].bufAddr[0] != NULL) {
				tmpBufAddr = PHYSICAL_TO_K1(rxDesc[i].bufAddr[0]);
				pcmRecvChBufFree((uint8 *) tmpBufAddr);
				rxDesc[i].bufAddr[0] = (uint32) NULL;
			}
		}
		pcmKfree(rxDesc, RX_DESC_TYPE);
	}
}

void txDescBufFree(void) {
	descStatus_t* tmpDescStat;
	uint32 tmpBufAddr = 0;
	while (1) {
		tmpDescStat = (descStatus_t *) &(txDesc[txProcIdx].descStatus);
		if ((tmpDescStat->bits.ownership == 1)) {
			break;
		}
		if ((txProcIdx == txIdx) && (txRound <= 0)) {
			break;
		}
		if ((uint8*) txDesc[txProcIdx].bufAddr[0] != NULL) {
			tmpBufAddr = PHYSICAL_TO_K1(txDesc[txProcIdx].bufAddr[0]);
			tmpBufAddr = NONCACHE_TO_CACHE(tmpBufAddr);
			pcmKfree((char *) tmpBufAddr, TX_BUF_TYPE);
			txDesc[txProcIdx].bufAddr[0] = (uint32) NULL;
		}
		txProcIdx = (txProcIdx + 1) % (MAX_TX_DESC_NUM);
		if (txProcIdx == 0) {
			txRound--;
		}
	}
}

void prepcmCfgVaild(void) {
	uint32 tmpValue = 0xfbffffff;
	regWrite(regMap[PCM_INTFACE_Ctrl].addr,tmpValue&regRead(regMap[PCM_INTFACE_Ctrl].addr));
	pause(5);
}
void postpcmCfgVaild(void) {
	uint32 tmpValue = 0xfbffffff;
	regWrite(regMap[PCM_INTFACE_Ctrl].addr,(~tmpValue)|regRead(regMap[PCM_INTFACE_Ctrl].addr));
}

void rxBufEnq(unsigned char* chRxBuf, uint32 tmpSampleSize) {
	callBackParam_t* callBackParam;
	uint32 tmpBufAddr;
	if (chRxBuf != NULL) {
		tmpBufAddr = PHYSICAL_TO_K1(chRxBuf);
		callBackParam = (callBackParam_t *) (tmpBufAddr
				- sizeof(callBackParam_t));
		callBackParam->rxSampleSize = tmpSampleSize;
		callBackParam->magicNum = 0x78563412;
		enqueue((uint8 *)tmpBufAddr);
	}
}

static irqreturn_t pcmIsr(int irq, void *devId) {
	int i = 0,j=0;
	int rxUpdateCounter = 0;
	char* rxChBuf;
	descStatus_t* tmpDescStat;

#if 1
	pcmIsrTH();
	return IRQ_HANDLED;
#endif

	uint32 isrValue = regRead(regMap[PCM_ISR].addr);

	if (isrValue & END_OF_RX_DESC_INT) {
		for(i=0;i<MAX_RX_DESC_NUM;i++) {
			tmpDescStat = (descStatus_t *)&(rxDesc[i].descStatus);
			if(tmpDescStat->bits.ownership ==0) {
				if((char *)(rxDesc[i].bufAddr[0]) !=NULL) {
					if(queueSizeGet() <= curConfigNode.maxRxQueueSize) {
						if(recvFunc != NULL) {
							rxBufEnq((unsigned char *)rxDesc[i].bufAddr[0],curConfigNode.rxSampleSize);
							for(j = 0; i<curConfigNode.chNum;j++) {
								rxDesc[i].bufAddr[j] = (uint32)NULL;
							}
						}
						else {
							pcmDescRecvBufFree(rxDesc[i].bufAddr[0]);
						}
					}
				}
				else {
					rxChBuf = pcmRecvBufAlloc(GFP_KERNEL);
					rxDescSetVaild(curConfigNode.rxSampleSize,rxChBuf);
				}
			}
			else
				printk("rx own = 1\n");
		}
		pollingEnable(RX);
	}
	else {
		if(isrValue & RX_DESC_UPDATE_COMPLETED_INT) {
			while(1) {

				tmpDescStat = (descStatus_t *)&(rxDesc[rxProcIdx].descStatus);
				if((tmpDescStat->bits.ownership ==1) || (rxUpdateCounter >=7)) {
					break;
				}
				rxUpdateCounter++;
				if(queueSizeGet() <= curConfigNode.maxRxQueueSize) {
					if(recvFunc != NULL) {
						rxBufEnq((unsigned char *)rxDesc[rxProcIdx].bufAddr[0],curConfigNode.rxSampleSize);
						for(i = 0; i<curConfigNode.chNum;i++) {
							rxDesc[rxProcIdx].bufAddr[i] = (uint32)NULL;
						}
					}
					else {
						pcmDescRecvBufFree(rxDesc[rxProcIdx].bufAddr[0]);
					}
				}
				else {
					printk("rxov q=%d\n",queueSizeGet());
					pcmDescRecvBufFree(rxDesc[rxProcIdx].bufAddr[0]);
				}
				rxChBuf = pcmRecvBufAlloc(GFP_ATOMIC);
				if(rxChBuf == NULL) {
					printk("buf alloc failed \n");
				}
				else {
					rxDescBufSet(rxProcIdx,rxChBuf);
					rxDescSet(curConfigNode.rxSampleSize,rxIdx);
					rxIdx=(rxIdx+1)%(MAX_RX_DESC_NUM);
				}
				rxProcIdx=(rxProcIdx+1)%(MAX_RX_DESC_NUM);
			}
			if(queueSizeGet() != QUEUE_EMPTY) {
				tasklet_schedule(&tasklet);
			}

		}
	}
	if(isrValue &(HUNT_ERR_AFTER_FINISH_INT|HUNT_OVERTIME_INT|AHB_BUS_ERR_INT|RX_BUF_UNDER_RUN_INT|TX_BUF_UNDER_RUN_INT)) {
		dmaEnable(TX,DISABLE);
		dmaEnable(RX,DISABLE);
	}
	return IRQ_HANDLED;
}

int txDescSetVaild(int sampleSize) {
	descStatus_t* tmpDescStat;
	tmpDescStat = (descStatus_t *) &(txDesc[txIdx].descStatus);
	if (tmpDescStat->bits.ownership == 1) {
		return FAIL;
	}

	tmpDescStat->bits.sampleSize = sampleSize;
	tmpDescStat->bits.chValid = (uint8) ((1 << curConfigNode.chNum) - 1);
	return SUCCESS;
}

void intMaskSet(uint32 mask) {
	regWrite(regMap[PCM_IMR].addr,mask);
}

void timeSlotCfgInit(uint8* bitWidthArray) {
	int i;

	int nextTimeSlot = 0;
	int tmpBitCounter = 0;

	int preBitCounter = 0;

	slotCfgReg_t slotCfgReg;
	for (i = 0; i < MAX_CH_NUM; i++) {
		if (i % 2 == 0) {
			slotCfgReg.bits.bitWidth = bitWidthArray[i];
			slotCfgReg.bits.bitCounter = tmpBitCounter + preBitCounter;
			preBitCounter = slotCfgReg.bits.bitCounter;
		} else {
			slotCfgReg.bits.bitWidthNext = bitWidthArray[i];
			slotCfgReg.bits.bitCounterNext = tmpBitCounter + preBitCounter;
			preBitCounter = slotCfgReg.bits.bitCounterNext;

			regWrite(regMap[PCM_TX_TIME_SLOT_CFG0+nextTimeSlot].addr,slotCfgReg.value);
			regWrite(regMap[PCM_RX_TIME_SLOT_CFG0+nextTimeSlot].addr,slotCfgReg.value);
			nextTimeSlot++;
		}
		if (bitWidthArray[i] == BITWIDTH_8) {
			tmpBitCounter = 8;
		} else {
			tmpBitCounter = 16;
		}
	}
}

void defaultConfigNodeSet(configNode_t* configNode) {
	configNode->loopbackMode = LOOPBACK_OFF;
	configNode->frameCount = 1;
	configNode->byteOrder = LITTLE_ENDIAN;
	configNode->bitOrder = MSB;
	configNode->fsEdge = FRAME_SYNC_EDGE_RISING;
	configNode->fsLen = FRAME_SYNC_LEN_1;
	configNode->sampleClock = PCM_SAMPLE_CLOCK_8;
	configNode->bitClock = PCM_BIT_CLOCK_2048;
	configNode->pcmMode = MASTER_MODE;
	configNode->chNum = DEFAULT_CH_NUM;
	configNode->maxRxQueueSize = DEFAULT_RX_QUEUE_SIZE;
	memset(configNode->bitWidth, BITWIDTH_16, MAX_CH_NUM*sizeof(uint8));
	configNode->rxSampleSize = DEFAULT_RX_SAMPLE_SIZE;
	configNode->debugLevel = 2;//NO_DBG;//2;//NO_DBG;
	configNode->debugCategory = CATEGORY_CONFIG;//NO_DBG;//CATEGORY_RX;//CATEGORY_CONFIG|CATEGORY_TX|CATEGORY_RX;//NO_DBG;
}

int pcmConfigSetup(configNode_t* tmpConfigNode, int* flag) {
	pcmCtrl_t pcmCtrlReg;
	pcmCtrlReg.value = regRead(regMap[PCM_INTFACE_Ctrl].addr);
	pcmCtrlReg.bits.lbGarbageEnable = LB_GARBAGE_ON;
	pcmCtrlReg.bits.delay = ONE_BIT_DELAY;
	pcmCtrlReg.bits.dataEdge = FALLING_DATA_EDGE;//RISING_DATA_EDGE;

	if (tmpConfigNode->loopbackMode <= LOOPBACK_ON) {
		pcmCtrlReg.bits.loopBack = tmpConfigNode->loopbackMode;
	}
	if (tmpConfigNode->frameCount <= MAX_FRAME_COUNT) {
		pcmCtrlReg.bits.frameCount = tmpConfigNode->frameCount;
	} else {
		(*flag) = CONFIG_FRAMECOUNT_ERR;
		return FAIL;
	}
	if (tmpConfigNode->byteOrder <= BIG_ENDIAN) {
		pcmCtrlReg.bits.byteOrder = tmpConfigNode->byteOrder;
	} else {
		(*flag) = CONFIG_BYTEORDER_ERR;
		return FAIL;
	}
	if (tmpConfigNode->bitOrder <= MSB) {
		pcmCtrlReg.bits.bitOrder = tmpConfigNode->bitOrder;
	} else {
		(*flag) = CONFIG_BITORDER_ERR;
		return FAIL;
	}
	if (tmpConfigNode->fsEdge <= FRAME_SYNC_EDGE_FALLING) {
		pcmCtrlReg.bits.fsEdge = tmpConfigNode->fsEdge;
	} else {
		(*flag) = CONFIG_FSEDGE_ERR;
		return FAIL;
	}
	if (tmpConfigNode->sampleClock <= PCM_SAMPLE_CLOCK_16) {
		pcmCtrlReg.bits.sampleClock = tmpConfigNode->sampleClock;
	} else {
		(*flag) = CONFIG_SAMPLECLOCK_ERR;
		return FAIL;
	}
	if (tmpConfigNode->bitClock <= PCM_BIT_CLOCK_8192) {
		pcmCtrlReg.bits.bitClock = tmpConfigNode->bitClock;
	} else {
		(*flag) = CONFIG_BITCLOCK_ERR;
		return FAIL;
	}
	if (tmpConfigNode->pcmMode <= SLAVE_MODE) {
		pcmCtrlReg.bits.pcmMode = tmpConfigNode->pcmMode;
	} else {
		(*flag) = CONFIG_PCMMODE_ERR;
		return FAIL;
	}
	switch (tmpConfigNode->fsLen) {
	case FRAME_SYNC_LEN_1:
		pcmCtrlReg.bits.fsLen = FRAME_SYNC_LEN_1;
		break;
	case FRAME_SYNC_LEN_8:
		pcmCtrlReg.bits.fsLen = FRAME_SYNC_LEN_8;
		break;
	case FRAME_SYNC_LEN_16:
		pcmCtrlReg.bits.fsLen = FRAME_SYNC_LEN_16;
		break;
	default:
		(*flag) = CONFIG_FSLEN_ERR;
		return FAIL;
		break;
	}
	if (tmpConfigNode->rxSampleSize > MAX_SAMPLE_COUNT) {
		(*flag) = CONFIG_RX_SAMPLESIZE_ERR;
		return FAIL;
	}
	if (tmpConfigNode->chNum <= MAX_CH_NUM) {
		regWrite(regMap[PCM_TX_RX_DMA_Ctrl].addr,regRead(regMap[PCM_TX_RX_DMA_Ctrl].addr)&0x00ffffff);
		regWrite(regMap[PCM_TX_RX_DMA_Ctrl].addr,regRead(regMap[PCM_TX_RX_DMA_Ctrl].addr)|(((1<<tmpConfigNode->chNum)-1)<<24));
	} else {
		(*flag) = CONFIG_CHANNEL_ERR;
		return FAIL;
	}
	switch (tmpConfigNode->debugLevel) {
	case 1:
		tmpConfigNode->debugLevel = DBG_LEVEL_1;
		break;
	case 2:
		tmpConfigNode->debugLevel = DBG_LEVEL_1 + DBG_LEVEL_2;
		break;
	case 3:
		tmpConfigNode->debugLevel = DBG_LEVEL_1 + DBG_LEVEL_2 + DBG_LEVEL_3;
		break;
	default:
		tmpConfigNode->debugLevel = NO_DBG;
		break;
	}
	if (tmpConfigNode->debugCategory > CATEGORY_CONFIG + CATEGORY_TX
			+ CATEGORY_RX) {
		tmpConfigNode->debugCategory = NO_DBG;
	}
	timeSlotCfgInit(tmpConfigNode->bitWidth);
	VPint(0xbfbd0044) = 0x0;
	regWrite(regMap[PCM_INTFACE_Ctrl].addr,pcmCtrlReg.value);
	printk("[pcm ctrl:0x%08lx] \n",regRead(regMap[PCM_INTFACE_Ctrl].addr));
	prepcmCfgVaild();
	postpcmCfgVaild();
	intMaskSet(RX_DESC_UPDATE_COMPLETED_INT|END_OF_RX_DESC_INT);
	return SUCCESS;
}

void pcmtmpRestart(int configSet) {
	int flag;
	dmaEnable(RX,DISABLE);
	dmaEnable(TX,DISABLE);
	
	tasklet_disable(&tasklet);

	pause(10);
	queueClean();
	softReset();
	if ((configSet == CONFIG_INIT) && (configNode != NULL)) {
		pcmConfigSetup(configNode, &flag);
	}
	descFree();
	descInit();
	intMaskSet(RX_DESC_UPDATE_COMPLETED_INT|END_OF_RX_DESC_INT);

	tasklet_enable(&tasklet);
	dmaEnable(TX,ENABLE);
	txIdx = 0;
	txProcIdx = 0;
	txRound = 0;
	rxProcIdx = 0;
	rxIdx = 0;
	rxRound = 0;
}

int pcmConfig(configNode_t* tmpConfigNode, int* flag) {
#if 1
	pcmStart();
#else
	int ret = 0;
	intMaskSet(0);

	pcmtmpRestart(NOT_TO_INIT);
	memcpy(&curConfigNode, tmpConfigNode, sizeof(configNode_t));
	ret = pcmConfigSetup(&curConfigNode, &flag);

	if(ret == SUCCESS) {
		return SUCCESS;
	}
	else {
		return FAIL;
	}
#endif	
}

void pcmRestart(int configSet) {
	int flag;
	dmaEnable(RX,DISABLE);
	dmaEnable(TX,DISABLE);
	tasklet_disable(&tasklet);
	pause(10);
	queueClean();
	softReset();
	if ((configSet == CONFIG_INIT) && (configNode != NULL)) {
		pcmConfigSetup(configNode, &flag);
	}
	descFree();
	descInit();
	intMaskSet(RX_DESC_UPDATE_COMPLETED_INT|END_OF_RX_DESC_INT);
	tasklet_enable(&tasklet);
	dmaEnable(TX,ENABLE);
	txIdx = 0;
	txProcIdx = 0;
	txRound = 0;
	rxProcIdx = 0;
	rxIdx = 0;
	rxRound = 0;
}

void pcmDMAStop(int direction) {
	if (direction == TX) {
		dmaEnable(TX,DISABLE);
	} else {
		dmaEnable(RX,DISABLE);
	}
}

int pcmSend(uint8** txBuf, int sampleSize, int* flag) {
	int tmpBufIdx = 0;
	int i = 0;
	int tmpMagicNum = 0;
	descStatus_t* tmpDescStat = NULL;
	uint32 tmpBufAddr = 0;

	if(sampleSize> MAX_SAMPLE_COUNT) {
		(*flag) = TX_SAMPLESIZE_RANGE_EXCCEDED;
		return FAIL;
	}
	
	if(!txDescSetVaild(sampleSize)) {
		(*flag) = DESC_FULL;
		return FAIL;
	}

	if(txDesc[txIdx].bufAddr[0] != (uint32)NULL) {
		for(i = 0 ;i<curConfigNode.chNum;i++){
			tmpBufAddr = PHYSICAL_TO_K1(txDesc[txIdx].bufAddr[i]);
			if(txDesc[txIdx].bufAddr[i] != (uint32)NULL){
				tmpMagicNum = rxBufMagicNumGet((uint8 *)tmpBufAddr);
				if (tmpMagicNum == 0x78563412){
					printk("buf %d: 0x%08lx----0x%08X\n",i,tmpBufAddr,tmpMagicNum);
					break;
				}
			}
		}

		if(i == curConfigNode.chNum){
			tmpBufAddr = PHYSICAL_TO_K1(txDesc[txIdx].bufAddr[0]);
		}
		pcmTxBufKfree((char *)tmpBufAddr);
	}
	for(i =0;i<MAX_CH_NUM;i++) {
		if(txBuf[i] != NULL) {
			int tmpBitWidth=0;
			int size= 0;
		
			tmpBitWidth = 16;
			size = curConfigNode.rxSampleSize*(tmpBitWidth>>3);
			txDesc[txIdx].bufAddr[i]=K1_TO_PHYSICAL(CACHE_TO_NONCACHE(txBuf[i]));
		}
	}
	tmpDescStat = (descStatus_t *)&(txDesc[txIdx].descStatus);
	tmpDescStat->bits.ownership=0x1;

	txIdx = (txIdx+1)%MAX_TX_DESC_NUM;
	if(txIdx == 0) {
		txRound++;
	}
	pollingEnable(TX);
	if(tmpBufIdx> curConfigNode.chNum) {
		(*flag) = BUFFER_MORE_THAN_CHANNEL;
		printk("tmpBufIdx=%d, chNum=%d\n",tmpBufIdx,curConfigNode.chNum);
		return FAIL;
	}
	return SUCCESS;

}

char* pcmRecvBufAlloc(int flag) {
	int i;
	int bufSize = 0;
	int tmpBitWidth;
	char* tmpRxBuf = NULL;
	for (i = 0; i < curConfigNode.chNum; i++) {
		if (curConfigNode.bitWidth[i] == BITWIDTH_8) {
			tmpBitWidth = 8;
		} else {
			tmpBitWidth = 16;
		}
		bufSize += curConfigNode.rxSampleSize * (tmpBitWidth >> 3);
	}
	tmpRxBuf = pcmKmalloc(bufSize + sizeof(callBackParam_t), flag, RX_BUF_TYPE);
	if (tmpRxBuf == NULL) {
		return NULL;
	}
	memset(tmpRxBuf, 0x0, bufSize);
	tmpRxBuf = tmpRxBuf + sizeof(callBackParam_t);
	return tmpRxBuf;
}

void pcmRecvChBufFree(uint8* rxBuf) {
	pcmKfree(rxBuf - sizeof(callBackParam_t), RX_BUF_TYPE);
}

uint8* pcmRecvChBufDequeue(void) {
	uint8* tmpBuf = dequeue();
	uint32 tmpMagicNum = 0;
	if (tmpBuf != NULL) {
		tmpMagicNum = rxBufMagicNumGet(tmpBuf);
		if (tmpMagicNum != 0x78563412) {
			printk("Buf was crumbled \n");
			return NULL;
		}
	}
	return tmpBuf;
}

void pcmDescRecvBufFree(uint32 descBufAddr) {
	int i;
	uint8* rxBuf = (uint8 *) PHYSICAL_TO_K1(descBufAddr);
	pcmRecvChBufFree(rxBuf);
	for (i = 0; i < curConfigNode.chNum; i++) {
		rxDesc[rxProcIdx].bufAddr[i] = (uint32) NULL;
	}
}

void pcmRecv(void) {
	int i;
	char* rxChBuf;
	uint32 tmpBufAddr;
	dmaEnable(RX,DISABLE);
	pause(10);
	for (i = 0; i < MAX_RX_DESC_NUM; i++) {
		if ((char *) (rxDesc[i].bufAddr[0]) != NULL) {
			tmpBufAddr = PHYSICAL_TO_K1(rxDesc[i].bufAddr[0]);
			pcmRecvChBufFree((uint8 *) tmpBufAddr);
			rxDesc[i].bufAddr[0] = (uint32) NULL;
		}
		rxChBuf = pcmRecvBufAlloc(GFP_KERNEL);
		rxDescSetVaild(curConfigNode.rxSampleSize, rxChBuf);
	}

	dmaEnable(RX,ENABLE);
	pollingEnable(RX);
}

int pcmRecvFuncRegister(void* funcP) {
	if ((funcP != NULL) && (recvFunc == NULL)) {
		recvFunc = funcP;
		tasklet_init(&tasklet, (void *) recvFunc, 0);
		return SUCCESS;
	}
	return FAIL;
}

extern int SPI_cfg(int id);

static int __init pcmDriverInit(void) {
	int err = 0;
	int flag = 0;
	int data = 0;

	printk("RSTCTRL2 : SPI and PCM 0x40800\n");
	
	pcmInit();
	
	configNode = kmalloc(sizeof(configNode_t), GFP_KERNEL);	
	defaultConfigNodeSet(configNode);
	//err = pcmConfig(configNode, &flag);
	
	err = request_irq(SURFBOARDINT_PCM,pcmIsr, 0, "PCM", NULL);
	
	if (err) {
		printk(KERN_INFO "request irq fail \n");
		return err;
	}

	return 0;
}

static void __exit pcmDriverExit(void) {
	kfree(configNode);
	tasklet_disable_nosync(&tasklet);
	tasklet_kill(&tasklet);
	queueClean();
	descFree();
	free_irq(PCM_INT,NULL);
}

char* pcmIdentifier(void) {
	return PCM_IDENTIFIER;
}

EXPORT_SYMBOL(pcmConfig);
EXPORT_SYMBOL(pcmRestart);
EXPORT_SYMBOL(pcmDMAStop);
EXPORT_SYMBOL(pcmSendBufAlloc);
EXPORT_SYMBOL(pcmSend);
EXPORT_SYMBOL(pcmRecvFuncRegister);
EXPORT_SYMBOL(pcmRecv);
EXPORT_SYMBOL(pcmRecvChBufDequeue);
EXPORT_SYMBOL(pcmRecvSampleSizeGet);
EXPORT_SYMBOL(pcmRecvChBufFree);
EXPORT_SYMBOL(queueSizeGet);
EXPORT_SYMBOL(pcmTxBufKfree);
EXPORT_SYMBOL(pcmIdentifier);
EXPORT_SYMBOL(CustomerVoice_HWInit);
EXPORT_SYMBOL(ProSLIC_HWInit);
MODULE_LICENSE("GPL");
module_init(pcmDriverInit);
module_exit(pcmDriverExit);

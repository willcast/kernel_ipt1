#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <mach/iphone-spi.h>
#include <mach/gpio.h>

#ifdef CONFIG_IPHONE
#define MT_GPIO_POWER 0x804
#define MT_ATN_INTERRUPT 0xa3
#else
#define MT_GPIO_POWER 0x701
#define MT_ATN_INTERRUPT 0x9b
#endif

#ifdef CONFIG_3G
#define MT_SPI 1
#define MT_SPI_CS GPIO_SPI1_CS0
#else
#define MT_SPI 2
#define MT_SPI_CS GPIO_SPI2_CS0
#endif

#define MT_INFO_FAMILYID 0xD1
#define MT_INFO_SENSORINFO 0xD3
#define MT_INFO_SENSORREGIONDESC 0xD0
#define MT_INFO_SENSORREGIONPARAM 0xA1
#define MT_INFO_SENSORDIM 0xD9

typedef struct MTFrameHeader
{
	u8 type;
	u8 frameNum;
	u8 headerLen;
	u8 unk_3;
	u32 timestamp;
	u8 unk_8;
	u8 unk_9;
	u8 unk_A;
	u8 unk_B;
	u16 unk_C;
	u16 isImage;

	u8 numFingers;
	u8 fingerDataLen;
	u16 unk_12;
	u16 unk_14;
	u16 unk_16;
} MTFrameHeader;

typedef struct FingerData
{
	u8 id;
	u8 event;
	u8 unk_2;
	u8 unk_3;
	s16 x;
	s16 y;
	s16 rel_x;
	s16 rel_y;
	u16 size_major;
	u16 size_minor;
	u16 orientation;
	u16 force_major;
	u16 force_minor;
	u16 unk_16;
	u16 unk_18;
	u16 unk_1A;
} FingerData;

#define MAX_FINGER_ORIENTATION  16384

static irqreturn_t multitouch_atn(int irq, void* pToken);

volatile int GotATN;
spinlock_t GotATNLock;

static u8* OutputPacket;
static u8* InputPacket;
static u8* GetInfoPacket;
static u8* GetResultPacket;

static int MultitouchIRQ;

static int InterfaceVersion;
static int MaxPacketSize;
static int FamilyID;
static int FlipNOP;
static int SensorWidth;
static int SensorHeight;
static int SensorColumns;
static int SensorRows;
static int BCDVersion;
static int Endianness;
static u8* SensorRegionDescriptor;
static int SensorRegionDescriptorLen;
static u8* SensorRegionParam;
static int SensorRegionParamLen;

static int CurNOP;

typedef struct MTSPISetting
{
	int speed;
	int txDelay;
	int rxDelay;
} MTSPISetting;

const MTSPISetting MTNormalSpeed = {83000, 5, 10};
const MTSPISetting MTFastSpeed = {4500000, 0, 10};

#define NORMAL_SPEED (&MTNormalSpeed)
#define FAST_SPEED (&MTFastSpeed)

static int mt_spi_txrx(const MTSPISetting* setting, const u8* outBuffer, int outLen, u8* inBuffer, int inLen);
static int mt_spi_tx(const MTSPISetting* setting, const u8* outBuffer, int outLen);

static int makeBootloaderDataPacket(u8* output, u32 destAddress, const u8* data, int dataLen, int* cksumOut);
static void sendExecutePacket(void);

static bool loadConstructedFirmware(const u8* firmware, int len);
static int loadProxCal(const u8* firmware, int len);
static int loadCal(const u8* firmware, int len);
static bool determineInterfaceVersion(void);

static bool getReportInfo(int id, u8* err, u16* len);
static bool getReport(int id, u8* buffer, int* outLen);

static bool readFrameLength(int* len);
static int readFrame(void);
static bool readResultData(int len);

static int calibrate(void);

static void newPacket(const u8* data, int len);


static void multitouch_atn_handler(struct work_struct* work);

bool MultitouchOn = false;
bool FirmwareLoaded = false;

static struct device* multitouch_dev = NULL;
struct input_dev* input_dev;

u8* constructed_fw;
size_t constructed_fw_size;
u8* proxcal_fw;
size_t proxcal_fw_size;
u8* cal_fw;
size_t cal_fw_size;

DECLARE_WORK(multitouch_workqueue, &multitouch_atn_handler);

void multitouch_on(void)
{
	if(!MultitouchOn)
	{
		printk("multitouch: powering on\n");
		iphone_gpio_pin_output(MT_GPIO_POWER, 0);
		msleep(200);
		iphone_gpio_pin_output(MT_GPIO_POWER, 1);

		msleep(15);
		MultitouchOn = true;
	}
}

static u16 performHBPPATN_ACK(void)
{
        u8 tx[2];
        u8 rx[2];

//        while(GotATN == 0);
//        --GotATN;

        tx[0] = 0x1A;
        tx[1] = 0xA1;

        mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

        return (rx[0] << 8) | rx[1];
}

static u32 performHBPPLongATN_ACK(void)
{
        u8 tx[8];
        u8 rx[8];

//        while(GotATN == 0);
//      	--GotATN;

        tx[0] = 0x1A;
        tx[1] = 0xA1;
        tx[2] = 0x18;
        tx[3] = 0xE1;
        tx[4] = 0x18;
        tx[5] = 0xE1;
        tx[6] = 0x18;
        tx[7] = 0xE1;

        mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

        return (rx[2] << 8) | rx[3] | (rx[4] << 24) | (rx[5] << 16);
}


int multitouch_setup(const u8* constructedFirmware, int constructedFirmwareLen, const u8* prox_cal, int prox_cal_size, const u8* cal, int cal_size)
{
	int i;
	int ret;
	int err;
        u8* reportBuffer;
        int reportLen;

//	prox_cal = syscfg_get_entry(SCFG_PxCl, &prox_cal_size);
        if(!prox_cal)
        {
                printk("multitouch: could not find proximity calibration data\n");
                return -1;
        }

//	cal = syscfg_get_entry(SCFG_MtCl, &cal_size);
        if(!cal)
        {
                printk("multitouch: could not find calibration data\n");
                return -1;
        }



	printk("multitouch: Firmware at 0x%08x - 0x%08x\n",
			(u32) constructedFirmware, (u32)(constructedFirmware + constructedFirmwareLen));

	OutputPacket = (u8*) kmalloc(0x400, GFP_KERNEL);
	InputPacket = (u8*) kmalloc(0x400, GFP_KERNEL);
	GetInfoPacket = (u8*) kmalloc(0x400, GFP_KERNEL);
	GetResultPacket = (u8*) kmalloc(0x400, GFP_KERNEL);

	request_irq(MT_ATN_INTERRUPT + IPHONE_GPIO_IRQS, multitouch_atn, IRQF_TRIGGER_FALLING, "iphone-multitouch", (void*) 0);

	multitouch_on();

	iphone_gpio_pin_output(0x606, 0);
	msleep(200);
	iphone_gpio_pin_output(0x606, 1);
	msleep(15);


	printk("multitouch: Sending Firmware...\n");
	if(!loadConstructedFirmware(constructedFirmware, constructedFirmwareLen))
	{
                printk("multitouch: could not load preconstructed firmware\n");
                err = -1;
        	kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
	}

        printk("multitouch: loaded %d byte preconstructed firmware\n", constructedFirmwareLen);

        if(!loadProxCal(prox_cal, prox_cal_size))
        {
                printk("multitouch: could not load proximity calibration data\n");
                err = -1;
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
        }

        printk("multitouch: loaded %d byte proximity calibration data\n", prox_cal_size);

        if(!loadCal(cal, cal_size))
        {
                printk("multitouch: could not load calibration data\n");
                err = -1;
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
        }


        printk("multitouch: loaded %d byte calibration data\n", cal_size);

        if(!calibrate())
        {
                err = -1;
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
        	return err;
        }

        sendExecutePacket();

        msleep(1);

	printk("multitouch: Determining interface version...\n");
	if(!determineInterfaceVersion())
	{
                err = -1;
        	kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
	}

	reportBuffer = (u8*) kmalloc(MaxPacketSize, GFP_KERNEL);

        if(!getReport(MT_INFO_FAMILYID, reportBuffer, &reportLen))
        {
                printk("multitouch: failed getting family id!\n");
                err = -1;
	        kfree(reportBuffer);
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
        	kfree(GetResultPacket);
	        return err;
        }

	FamilyID = reportBuffer[0];

        if(!getReport(MT_INFO_SENSORINFO, reportBuffer, &reportLen))
        {
                printk("multitouch: failed getting sensor info!\r\n");
                err = -1;
	        kfree(reportBuffer);
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
        }

	SensorColumns = reportBuffer[2];
	SensorRows = reportBuffer[1];
	BCDVersion = ((reportBuffer[3] & 0xFF) << 8) | (reportBuffer[4] & 0xFF);
	Endianness = reportBuffer[0];


        if(!getReport(MT_INFO_SENSORREGIONDESC, reportBuffer, &reportLen))
        {
                printk("multitouch: failed getting sensor region descriptor!\r\n");
                err = -1;
	        kfree(reportBuffer);
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
        }


	SensorRegionDescriptorLen = reportLen;
	SensorRegionDescriptor = (u8*) kmalloc(reportLen, GFP_KERNEL);
	memcpy(SensorRegionDescriptor, reportBuffer, reportLen);

        if(!getReport(MT_INFO_SENSORREGIONPARAM, reportBuffer, &reportLen))
        {
                printk("multitouch: failed getting sensor region param!\r\n");
                err = -1;
	        kfree(SensorRegionDescriptor);
	        kfree(reportBuffer);
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
        }


	SensorRegionParamLen = reportLen;
	SensorRegionParam = (u8*) kmalloc(reportLen, GFP_KERNEL);
	memcpy(SensorRegionParam, reportBuffer, reportLen);

        if(!getReport(MT_INFO_SENSORDIM, reportBuffer, &reportLen))
        {
                printk("multitouch: failed getting sensor surface dimensions!\r\n");
                err = -1;
	        kfree(SensorRegionParam);
	        kfree(SensorRegionDescriptor);
	        kfree(reportBuffer);
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
        }


	SensorWidth = *((u32*)&reportBuffer[0]);
	SensorHeight = *((u32*)&reportBuffer[4]);

	printk("Family ID                : 0x%x\n", FamilyID);
	printk("Sensor rows              : 0x%x\n", SensorRows);
	printk("Sensor columns           : 0x%x\n", SensorColumns);
	printk("Sensor width             : 0x%x\n", SensorWidth);
	printk("Sensor height            : 0x%x\n", SensorHeight);
	printk("BCD Version              : 0x%x\n", BCDVersion);
	printk("Endianness               : 0x%x\n", Endianness);
	printk("Sensor region descriptor :");
	for(i = 0; i < SensorRegionDescriptorLen; ++i)
		printk(" %02x", SensorRegionDescriptor[i]);
	printk("\n");

	printk("Sensor region param      :");
	for(i = 0; i < SensorRegionParamLen; ++i)
		printk(" %02x", SensorRegionParam[i]);
	printk("\n");

        if(BCDVersion > 0x23)
                FlipNOP = true;
        else
                FlipNOP = false;

        kfree(reportBuffer);


	input_dev = input_allocate_device();
	if(!input_dev)
	{
                err = -1;
	        kfree(SensorRegionParam);
	        kfree(SensorRegionDescriptor);
	        kfree(reportBuffer);
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
	}


	input_dev->name = "iPhone Zephyr Multitouch Screen";
	input_dev->phys = "multitouch0";
	input_dev->id.vendor = 0x05AC;
	input_dev->id.product = 0;
	input_dev->id.version = 0x0000;
	input_dev->dev.parent = multitouch_dev;
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, 0, SensorWidth, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SensorHeight, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, max(SensorHeight, SensorWidth), 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, max(SensorHeight, SensorWidth), 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, max(SensorHeight, SensorWidth), 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, max(SensorHeight, SensorWidth), 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SensorWidth, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SensorHeight, 0, 0);

	/* not sure what the actual max is */
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 32, 0, 0);

	ret = input_register_device(input_dev);
	if(ret != 0)
	{
                err = -1;
        	kfree(SensorRegionParam);
	        kfree(SensorRegionDescriptor);
	        kfree(reportBuffer);
	        kfree(InputPacket);
	        kfree(OutputPacket);
	        kfree(GetInfoPacket);
	        kfree(GetResultPacket);
	        return err;
	}

        GotATN = 0;
        CurNOP = 1;

	spin_lock_init(&GotATNLock);

	FirmwareLoaded = true;

	return 0;
}

static void multitouch_atn_handler(struct work_struct* work)
{
//	unsigned long flags;
	printk("multitouch: Interrupt goes handling\n");

	if(!FirmwareLoaded)
		return;

//	spin_unlock_wait(&GotATNLock);
//	spin_lock_irqsave(&GotATNLock, flags);
       	disable_irq_nosync(MultitouchIRQ);
	while(GotATN)
	{
		--GotATN;
//		spin_unlock_irqrestore(&GotATNLock, flags);
	        enable_irq(MultitouchIRQ);
		while(readFrame() == 1);
//		spin_unlock_wait(&GotATNLock);
//		spin_lock_irqsave(&GotATNLock, flags);
        	disable_irq_nosync(MultitouchIRQ);
		printk("multitouch: Interrupt handled\n");
	}
	printk("multitouch: Interrupts handled\n");
//	spin_unlock_irqrestore(&GotATNLock, flags);
        enable_irq(MultitouchIRQ);
}

static void newPacket(const u8* data, int len)
{
	int i;
	FingerData* finger;
	MTFrameHeader* header = (MTFrameHeader*) data;
	if(header->type != 0x44 && header->type != 0x43)
		printk("multitouch: unknown frame type 0x%x\n", header->type);

	finger = (FingerData*)(data + (header->headerLen));

	if(header->headerLen < 12)
		printk("multitouch: no finger data in frame\n");

//	printk("------START------\n");

	for(i = 0; i < header->numFingers; ++i)
	{
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger->force_major);
		input_report_abs(input_dev, ABS_MT_TOUCH_MINOR, finger->force_minor);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, finger->size_major);
		input_report_abs(input_dev, ABS_MT_WIDTH_MINOR, finger->size_minor);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, MAX_FINGER_ORIENTATION - finger->orientation);
		input_report_abs(input_dev, ABS_MT_POSITION_X, finger->x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, SensorHeight - finger->y);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, finger->id);
		input_mt_sync(input_dev);
/*		printk("multitouch: finger %d -- id=%d, event=%d, X(%d/%d, vel: %d), Y(%d/%d, vel: %d), radii(%d, %d, %d, orientation: %d), force_minor: %d\n",
				i, finger->id, finger->event,
				finger->x, SensorWidth, finger->rel_x,
				finger->y, SensorHeight, finger->rel_y,
				finger->force_major, finger->size_major, finger->size_minor, finger->orientation,
				finger->force_minor);

		//framebuffer_draw_rect(0xFF0000, (finger->x * framebuffer_width()) / SensorWidth - 2 , ((SensorHeight - finger->y) * framebuffer_height()) / SensorHeight - 2, 4, 4);
		//hexdump((u32) finger, sizeof(FingerData));*/
		finger = (FingerData*) (((u8*) finger) + header->fingerDataLen);
	}

	if(header->numFingers > 0)
	{
		finger = (FingerData*)(data + (header->headerLen));

		input_report_abs(input_dev, ABS_X, finger->x);
		input_report_abs(input_dev, ABS_Y, SensorHeight - finger->y);
		input_report_key(input_dev, BTN_TOUCH, finger->size_minor > 0);
	}

	input_sync(input_dev);

//	printk("-------END-------\n");
}

static int readFrame(void)
{
        int ret = 0;
        int len = 0;

        if(!readFrameLength(&len))
        {
                printk("multitouch: error getting frame length\r\n");
                len = 0;
                ret = -1;
        }

        if(len)
        {
                if(!readResultData(len + 5))
                {
                        printk("multitouch: error getting frame data\r\n");
                        msleep(1);
                        ret = -1;
                }

                ret = 1;
        }

        if(FlipNOP)
        {
                if(CurNOP == 1)
                        CurNOP = 2;
                else
                        CurNOP = 1;
        }

        return ret;
}

static bool readResultData(int len)
{
        u32 checksum;
        int i;
        int packetLen;

        if(len > 0x200)
                len = 0x200;

        memset(GetResultPacket, 0, 0x200);

        if(FlipNOP)
                GetResultPacket[0] = 0xEB;
        else
                GetResultPacket[0] = 0xEA;

        GetResultPacket[1] = CurNOP;
        GetResultPacket[2] = 1;

        checksum = 0;
        for(i = 0; i < 14; ++i)
                checksum += GetResultPacket[i];

        GetResultPacket[len - 2] = checksum & 0xFF;
        GetResultPacket[len - 1] = (checksum >> 8) & 0xFF;

        mt_spi_txrx(NORMAL_SPEED, GetResultPacket, len, InputPacket, len);

        if(InputPacket[0] != 0xEA && !(FlipNOP && InputPacket[0] == 0xEB))
        {
                printk("multitouch: frame header wrong: got 0x%02X\n", InputPacket[0]);
                msleep(1);
                return false;
        }

        checksum = 0;
        for(i = 0; i < 5; ++i)
                checksum += InputPacket[i];

        if((checksum & 0xFF) != 0)
        {
                printk("multitouch: LSB of first five bytes of frame not zero: got 0x%02X\n", checksum);
                msleep(1);
                return false;
        }

        packetLen = (InputPacket[2] & 0xFF) | ((InputPacket[3] & 0xFF) << 8);

        if(packetLen <= 2)
                return true;

        checksum = 0;
        for(i = 5; i < (5 + packetLen - 2); ++i)
                checksum += InputPacket[i];
        if((InputPacket[len - 2] | (InputPacket[len - 1] << 8)) != checksum)
        {
                printk("multitouch: packet checksum wrong 0x%02X instead of 0x%02X\n", checksum, (InputPacket[len - 2] | (InputPacket[len - 1] << 8)));
                msleep(1);
                return false;
        }

        newPacket(InputPacket + 5, packetLen - 2);
        return true;
}

static bool readFrameLength(int* len)
{
        u8 tx[16];
        u8 rx[16];
        u32 checksum;
        int i;

        memset(tx, 0, sizeof(tx));

        if(FlipNOP)
                tx[0] = 0xEB;
        else
                tx[0] = 0xEA;

        tx[1] = CurNOP;
        tx[2] = 0;

        checksum = 0;
        for(i = 0; i < 14; ++i)
                checksum += tx[i];

        tx[14] = checksum & 0xFF;
        tx[15] = (checksum >> 8) & 0xFF;

        mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

        checksum = 0;
        for(i = 0; i < 14; ++i)
                checksum += rx[i];

        if((rx[14] | (rx[15] << 8)) != checksum)
        {
                udelay(1000);
                return false;
        }

        *len = (rx[1] & 0xFF) | ((rx[2] & 0xFF) << 8);

        return true;
}

static int shortControlRead(int id, u8* buffer, int size)
{
        u32 checksum;
        int i;

        memset(GetInfoPacket, 0, 0x200);
        GetInfoPacket[0] = 0xE6;
        GetInfoPacket[1] = id;
        GetInfoPacket[2] = 0;
        GetInfoPacket[3] = size & 0xFF;
        GetInfoPacket[4] = (size >> 8) & 0xFF;

        checksum = 0;
        for(i = 0; i < 5; ++i)
                checksum += GetInfoPacket[i];

        GetInfoPacket[14] = checksum & 0xFF;
        GetInfoPacket[15] = (checksum >> 8) & 0xFF;

        mt_spi_txrx(NORMAL_SPEED, GetInfoPacket, 16, InputPacket, 16);

        udelay(25);

        GetInfoPacket[2] = 1;

        mt_spi_txrx(NORMAL_SPEED, GetInfoPacket, 16, InputPacket, 16);

        checksum = 0;
        for(i = 0; i < 14; ++i)
                checksum += InputPacket[i];

        if((InputPacket[14] | (InputPacket[15] << 8)) != checksum)
                return false;

        memcpy(buffer, &InputPacket[3], size);

        return true;
}

static int longControlRead(int id, u8* buffer, int size)
{
        u32 checksum;
        int i;

        memset(GetInfoPacket, 0, 0x200);
        GetInfoPacket[0] = 0xE7;
        GetInfoPacket[1] = id;
        GetInfoPacket[2] = 0;
        GetInfoPacket[3] = size & 0xFF;
        GetInfoPacket[4] = (size >> 8) & 0xFF;

        checksum = 0;
        for(i = 0; i < 5; ++i)
                checksum += GetInfoPacket[i];

        GetInfoPacket[14] = checksum & 0xFF;
        GetInfoPacket[15] = (checksum >> 8) & 0xFF;

        mt_spi_txrx(NORMAL_SPEED, GetInfoPacket, 16, InputPacket, 16);

        udelay(25);

        GetInfoPacket[2] = 1;
        GetInfoPacket[14] = 0;
        GetInfoPacket[15] = 0;
        GetInfoPacket[3 + size] = checksum & 0xFF;
        GetInfoPacket[3 + size + 1] = (checksum >> 8) & 0xFF;

        mt_spi_txrx(NORMAL_SPEED, GetInfoPacket, size + 5, InputPacket, size + 5);

        checksum = 0;
        for(i = 0; i < (size + 3); ++i)
                checksum += InputPacket[i];

        if((InputPacket[3 + size] | (InputPacket[3 + size + 1] << 8)) != checksum)
                return false;

        memcpy(buffer, &InputPacket[3], size);

        return true;
}

static bool getReportInfo(int id, u8* err, u16* len)
{
        u8 tx[16];
        u8 rx[16];
        u32 checksum;
        int i;
        int try;

        for(try = 0; try < 4; ++try)
        {
                memset(tx, 0, sizeof(tx));

                tx[0] = 0xE3;
                tx[1] = id;

                checksum = 0;
                for(i = 0; i < 14; ++i)
                        checksum += tx[i];

                tx[14] = checksum & 0xFF;
                tx[15] = (checksum >> 8) & 0xFF;

                mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

                udelay(25);

                mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

                if(rx[0] != 0xE3)
                        continue;

                checksum = 0;
                for(i = 0; i < 14; ++i)
                        checksum += rx[i];

                if((rx[14] | (rx[15] << 8)) != checksum)
                        continue;

                *err = rx[2];
                *len = (rx[4] << 8) | rx[3];

                return true;
        }

        return false;
}

static bool getReport(int id, u8* buffer, int* outLen)
{
        u8 err;
        u16 len;
        int try;

        if(!getReportInfo(id, &err, &len))
                return false;

        if(err)
                return false;

        *outLen = len;

        for(try = 0; try < 4; ++try)
        {
                if(len < 12)
                {
                        if(shortControlRead(id, buffer, len))
                                return true;
                } else
                {
                        if(longControlRead(id, buffer, len))
                                return true;
                }
        }

        return false;
}

static bool determineInterfaceVersion(void)
{
        u8 tx[16];
        u8 rx[16];
        u32 checksum;
        int i;
        int try;

        memset(tx, 0, sizeof(tx));

        tx[0] = 0xE2;

        checksum = 0;
        for(i = 0; i < 14; ++i)
                checksum += tx[i];

        // Note that the byte order changes to little-endian after main firmware load

        tx[14] = checksum & 0xFF;
        tx[15] = (checksum >> 8) & 0xFF;

        mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

        for(try = 0; try < 4; ++try)
        {
                mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

                if(rx[0] == 0xE2)
                {
                        checksum = 0;
                        for(i = 0; i < 14; ++i)
                                checksum += rx[i];

                        if((rx[14] | (rx[15] << 8)) == checksum)
                        {
                                InterfaceVersion = rx[2];
                                MaxPacketSize = (rx[4] << 8) | rx[3];
                                printk("multitouch: interface version %d, max packet size: %d\n", InterfaceVersion, MaxPacketSize);

                                return true;
                        }
                }

                InterfaceVersion = 0;
                MaxPacketSize = 1000;
                msleep(3);
        }

        printk("multitouch: failed getting interface version!\n");

        return false;
}

static bool loadConstructedFirmware(const u8* firmware, int len)
{
	int try;

	for(try = 0; try < 5; ++try)
	{

		printk("multitouch: uploading firmware\n");

//                GotATN = 0;
                mt_spi_tx(FAST_SPEED, firmware, len);

		udelay(300);

		if(performHBPPATN_ACK() == 0x4BC1)
                        return true;

	}

	return false;
}

static int loadProxCal(const u8* firmware, int len)
{
        u32 address = 0x400180;
        const u8* data = firmware;
        int left = (len + 3) & ~0x3;
        int try;

        while(left > 0)
        {
                int toUpload = left;
                if(toUpload > 0x3F0)
                        toUpload = 0x3F0;

                OutputPacket[0] = 0x18;
                OutputPacket[1] = 0xE1;

                makeBootloaderDataPacket(OutputPacket + 2, address, data, toUpload, NULL);

                for(try = 0; try < 5; ++try)
                {
                        printk("multitouch: uploading prox calibration data packet\r\n");

//                        GotATN = 0;
                        mt_spi_tx(FAST_SPEED, OutputPacket, toUpload + 0x10);
			udelay(300);

                        if(performHBPPATN_ACK() == 0x4BC1)
                                break;
                }

                if(try == 5)
                        return false;

                address += toUpload;
                data += toUpload;
                left -= toUpload;
        }

        return true;
}

static int loadCal(const u8* firmware, int len)
{
        u32 address = 0x400200;
        const u8* data = firmware;
        int left = (len + 3) & ~0x3;
        int try;

        while(left > 0)
        {
                int toUpload = left;
                if(toUpload > 0x3F0)
                        toUpload = 0x3F0;

                OutputPacket[0] = 0x18;
                OutputPacket[1] = 0xE1;

                makeBootloaderDataPacket(OutputPacket + 2, address, data, toUpload, NULL);

                for(try = 0; try < 5; ++try)
                {
                        printk("multitouch: uploading calibration data packet\r\n");

//                        GotATN = 0;
                        mt_spi_tx(FAST_SPEED, OutputPacket, toUpload + 0x10);
			udelay(300);

                        if(performHBPPATN_ACK() == 0x4BC1)
                                break;
                }

                if(try == 5)
                        return false;

                address += toUpload;
                data += toUpload;
                left -= toUpload;
        }

        return true;
}

static u32 readRegister(u32 address)
{
        u8 tx[8];
        u8 rx[8];
        u32 checksum;
        int i;

        tx[0] = 0x1C;
        tx[1] = 0x73;
        tx[2] = (address >> 8) & 0xFF;
        tx[3] = address & 0xFF;
        tx[4] = (address >> 24) & 0xFF;
        tx[5] = (address >> 16) & 0xFF;

        checksum = 0;
        for(i = 2; i < 6; ++i)
                checksum += tx[i];

        tx[6] = (checksum >> 8) & 0xFF;
        tx[7] = checksum & 0xFF;

//        GotATN = 0;
        mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));
	udelay(300);

        return performHBPPLongATN_ACK();
}

static u32 writeRegister(u32 address, u32 value, u32 mask)
{
        u8 tx[16];
        u8 rx[16];
        u32 checksum;
        int i;

        tx[0] = 0x1E;
        tx[1] = 0x33;
        tx[2] = (address >> 8) & 0xFF;
        tx[3] = address & 0xFF;
        tx[4] = (address >> 24) & 0xFF;
        tx[5] = (address >> 16) & 0xFF;
        tx[6] = (mask >> 8) & 0xFF;
        tx[7] = mask & 0xFF;
        tx[8] = (mask >> 24) & 0xFF;
        tx[9] = (mask >> 16) & 0xFF;
        tx[10] = (value >> 8) & 0xFF;
        tx[11] = value & 0xFF;
        tx[12] = (value >> 24) & 0xFF;
        tx[13] = (value >> 16) & 0xFF;

        checksum = 0;
        for(i = 2; i < 14; ++i)
                checksum += tx[i];
        tx[14] = (checksum >> 8) & 0xFF;
        tx[15] = checksum & 0xFF;

//        GotATN = 0;
        mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));
	udelay(300);

        if(performHBPPATN_ACK() == 0x4AD1)
                return true;
        else
                return false;
}

static int calibrate(void)
{
        u32 z2_version;
        u8 tx[2];
        u8 rx[2];

        z2_version = readRegister(0x10008FFC);

        printk("multitouch: detected Zephyr2 version 0x%0X\n", z2_version);

        if(z2_version != 0x5A020028)
        {
                // Apparently we have to write registers here

                if(!writeRegister(0x10001C04, 0x16E4, 0x1FFF))
                {
                        printk("multitouch: error writing to register 0x10001C04\n");
                        return false;
                }

                if(!writeRegister(0x10001C08, 0x840000, 0xFF0000))
                {
                        printk("multitouch: error writing to register 0x10001C08\n");
                        return false;
                }

                if(!writeRegister(0x1000300C, 0x05, 0x85))
                {
                        printk("multitouch: error writing to register 0x1000300C\n");
                        return false;
                }

                if(!writeRegister(0x1000304C, 0x20, 0xFFFFFFFF))
                {
                        printk("multitouch: error writing to register 0x1000304C\n");
                        return false;
                }

                printk("multitouch: initialized registers\n");

        }

        tx[0] = 0x1F;
        tx[1] = 0x01;

        printk("multitouch: requesting calibration...\n");

//	GotATN = 0;
        mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

        msleep(65);

        printk("multitouch: calibration complete with 0x%x\n", performHBPPATN_ACK());

        return true;
}

static void sendExecutePacket(void)
{
        u8 tx[12];
        u8 rx[12];
        u32 checksum;
        int i;

        tx[0] = 0x1D;
        tx[1] = 0x53;
        tx[2] = 0x18;
        tx[3] = 0x00;
        tx[4] = 0x10;
        tx[5] = 0x00;
        tx[6] = 0x00;
        tx[7] = 0x01;
        tx[8] = 0x00;
        tx[9] = 0x00;

        checksum = 0;
        for(i = 2; i < 10; ++i)
                checksum += tx[i];

        tx[10] = (checksum >> 8) & 0xFF;
        tx[11] = checksum & 0xFF;

        mt_spi_txrx(NORMAL_SPEED, tx, sizeof(tx), rx, sizeof(rx));

        printk("multitouch: execute packet sent\r\n");
}

static int makeBootloaderDataPacket(u8* output, u32 destAddress, const u8* data, int dataLen, int* cksumOut)
{
        u32 checksum;
        int i;

        // This seems to be middle-endian! I've never seen this before.

        output[0] = 0x30;
        output[1] = 0x01;
        output[2] = ((dataLen >> 2) >> 10) & 0xFF;
        output[3] = (dataLen >> 2) & 0xFF;
        output[4] = (destAddress >> 8) & 0xFF;
        output[5] = destAddress & 0xFF;
        output[6] = (destAddress >> 24) & 0xFF;
        output[7] = (destAddress >> 16) & 0xFF;

        checksum = 0;

        for(i = 2; i < 8; ++i)
                checksum += output[i];

        output[8] = (checksum >> 8) & 0xFF;
        output[9] = checksum & 0xFF;

        for(i = 0; i < dataLen; i += 4)
        {
                output[10 + i + 0] = data[i + 1];
                output[10 + i + 1] = data[i + 0];
                output[10 + i + 2] = data[i + 3];
                output[10 + i + 3] = data[i + 2];
        }

        checksum = 0;
        for(i = 10; i < (dataLen + 10); ++i)
                checksum += output[i];

        output[dataLen + 10] = (checksum >> 8) & 0xFF;
        output[dataLen + 11] = checksum & 0xFF;
        output[dataLen + 12] = (checksum >> 24) & 0xFF;
        output[dataLen + 13] = (checksum >> 16) & 0xFF;

        if(cksumOut)
                *cksumOut = checksum;

        return dataLen;
}

static irqreturn_t multitouch_atn(int irq, void* pToken)
{
//	unsigned long flags;
	printk("multitouch: Interrupt comes in\n");

	if(!FirmwareLoaded)
		return IRQ_HANDLED;

	if(!MultitouchIRQ)
		MultitouchIRQ = irq;

//	spin_unlock_wait(&GotATNLock);
//	spin_lock_irqsave(&GotATNLock, flags);
	disable_irq_nosync(MultitouchIRQ);
	++GotATN;
//	spin_unlock_irqrestore(&GotATNLock, flags);
	enable_irq(MultitouchIRQ);
	schedule_work(&multitouch_workqueue);
	printk("multitouch: Interrupt queued\n");
	return IRQ_HANDLED;
}


int mt_spi_tx(const MTSPISetting* setting, const u8* outBuffer, int outLen)
{
	int ret;
	iphone_spi_set_baud(MT_SPI, setting->speed, SPIOption13Setting0, 1, 1, 1);
	iphone_gpio_pin_output(MT_SPI_CS, 0);
	msleep(setting->txDelay);
	ret = iphone_spi_tx(MT_SPI, outBuffer, outLen, true, false);
	iphone_gpio_pin_output(MT_SPI_CS, 1);
	return ret;
}

int mt_spi_txrx(const MTSPISetting* setting, const u8* outBuffer, int outLen, u8* inBuffer, int inLen)
{
	int ret;
	iphone_spi_set_baud(MT_SPI, setting->speed, SPIOption13Setting0, 1, 1, 1);
	iphone_gpio_pin_output(MT_SPI_CS, 0);
	msleep(setting->rxDelay);
	ret = iphone_spi_txrx(MT_SPI, outBuffer, outLen, inBuffer, inLen, true);
	iphone_gpio_pin_output(MT_SPI_CS, 1);
	return ret;
}

static void got_cal(const struct firmware* fw, void *context)
{
	if(!fw)
	{
		printk("multitouch: couldn't get cal, trying again...\n");
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr2_cal.bin", multitouch_dev, NULL, got_cal);
		return;
	}

	cal_fw = kmalloc(fw->size, GFP_KERNEL);
	cal_fw_size = fw->size;
	memcpy(cal_fw, fw->data, fw->size);

	printk("multitouch: initializing multitouch\n");
	multitouch_setup(constructed_fw, constructed_fw_size, proxcal_fw, proxcal_fw_size, cal_fw, cal_fw_size);

	/* caller will call release_firmware */
}

static void got_proxcal(const struct firmware* fw, void *context)
{
	if(!fw)
	{
		printk("multitouch: couldn't get proxcal, trying again...\n");
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr2_proxcal.bin", multitouch_dev, NULL, got_proxcal);
		return;
	}

	proxcal_fw = kmalloc(fw->size, GFP_KERNEL);
	proxcal_fw_size = fw->size;
	memcpy(proxcal_fw, fw->data, fw->size);

	printk("multitouch: requesting cal\n");
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr2_cal.bin", multitouch_dev, NULL, got_cal);

	/* caller will call release_firmware */
}

static void got_constructed(const struct firmware* fw, void *context)
{
	if(!fw)
	{
		printk("multitouch: couldn't get firmware, trying again...\n");
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr2.bin", multitouch_dev, NULL, got_constructed);
		return;
	}

	constructed_fw = kmalloc(fw->size, GFP_KERNEL);
	constructed_fw_size = fw->size;
	memcpy(constructed_fw, fw->data, fw->size);

	printk("multitouch: requesting proxcal\n");
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr2_proxcal.bin", multitouch_dev, NULL, got_proxcal);

	/* caller will call release_firmware */
}

static int iphone_multitouch_probe(struct platform_device *pdev)
{
	/* this driver is such a hack */
	if(multitouch_dev)
		return -EBUSY;

	multitouch_dev = &pdev->dev;

	printk("multitouch: requesting Firmware\n");
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "zephyr2.bin", multitouch_dev, NULL, got_constructed);
}

static int iphone_multitouch_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver iphone_multitouch_driver = {
	.probe = iphone_multitouch_probe,
	.remove = iphone_multitouch_remove,
	.suspend = NULL, /* optional but recommended */
	.resume = NULL,   /* optional but recommended */
	.driver = {
		.owner = THIS_MODULE,
		.name = "iphone-multitouch",
	},
};

static struct platform_device iphone_multitouch_dev = {
	.name = "iphone-multitouch",
	.id = -1,
};

static int __init iphone_multitouch_init(void)
{
	int ret;

	ret = platform_driver_register(&iphone_multitouch_driver);

	if (!ret) {
		ret = platform_device_register(&iphone_multitouch_dev);

		if (ret != 0) {
			platform_driver_unregister(&iphone_multitouch_driver);
		}
	}
	return ret;
}

static void __exit iphone_multitouch_exit(void)
{
	platform_device_unregister(&iphone_multitouch_dev);
	platform_driver_unregister(&iphone_multitouch_driver);
}

module_init(iphone_multitouch_init);
module_exit(iphone_multitouch_exit);

MODULE_DESCRIPTION("iPhone Zephyr multitouch driver");
MODULE_AUTHOR("Yiduo Wang");
MODULE_LICENSE("GPL");

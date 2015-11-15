/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 * bridge.c
 */
#include <linux/delay.h>
#include <linux/gpio.h>
#include <bridge.h>
#include <../../../../arch/arm/mach-tegra/gpio-names.h>

#define SPI_WRITE(value) spi_write9(value);

#define DEBUG_INITIAL

#define en_mipi_pwr			TEGRA_GPIO_PC1
#define mipi_rst_l			TEGRA_GPIO_PW1
#define mipi_shut			TEGRA_GPIO_PB2
#define en_lcd_pwr			TEGRA_GPIO_PL4

struct spi_data {
	unsigned short delay;
	unsigned int   value;
};

int read_bits=1;
int bridge_mode=0;
static struct spi_device *bridge_spi;
static int disable_end = TRUE;
unsigned int high_addr=0,low_addr=0;

#ifdef DEBUG_INITIAL
const char ssd2828_reg_set[]={
0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xC9,0xCA,0xCB,0xCC,0xD9,
0xDE,0xD6,0xC4,0xBA,0xBB,0xB9,0xB7,0xB8,0xBC,0xBF,
0xBC,0xBF,0xB7};
#endif

extern bool IsPansonicPanel(void);

static int spi_read_byte16(u8 value, u16 *data)
{
    u8      tx_buf[2];
    u8      rx_buf[2];
    int     rc;
    struct spi_message  m;
    struct spi_transfer xfer[2];

    memset(&xfer, 0, sizeof(xfer));

    tx_buf[1] = (value & 0xFF00) >> 8;
    tx_buf[0] = (value & 0x00FF);
    xfer[0].tx_buf = tx_buf;
    xfer[0].bits_per_word = 9;
    xfer[0].len = 2;

    xfer[1].rx_buf = rx_buf;
    xfer[1].bits_per_word = 16;
    xfer[1].len = 2;

    spi_message_init(&m);
    spi_message_add_tail(&xfer[0], &m);
    spi_message_add_tail(&xfer[1], &m);

    rc = spi_sync(bridge_spi, &m);
    *data=(rx_buf[1] | (((u16)rx_buf[0])<<8));
    //printk(KERN_INFO "rx_buf[0]:0x%x,rx_buf[1]:0x%x \n", rx_buf[0],rx_buf[1]);
    if (rc)
        printk(KERN_ERR "spi_sync_read failed %d\n", rc);
    return rc;
}

struct spi_data solomon_reg_mipi_read_cmd[] = {   // MIPI READ
    {0, 0x00C0}, // Send 0x0C0
    {0, 0x0101}, // Set 0x101
    {0, 0x0100}, // Set 0x100
    {0, 0x00C4}, // Send 0x0C4
    {0, 0x0101}, // Set 0x101
    {0, 0x0100}, // Set 0x100
    {0, 0x00C1}, // Send 0x0C1
    {0, 0x0102}, // Set 0x10A
    {0, 0x0100}, // Set 0x100
    {0, 0x00B7}, // Send 0x0B7
    {0, 0x018B}, // Set 0x18B                        //bit 0 = 0 LP mode, 1 HS mode
    {0, 0x0103}, // Set 0x103
    {0, 0x00BC}, // Send 0x0B7
    {0, 0x0102}, // Set 0x01FE, FE is runtime paramater what number of byte to be read
    {0, 0x0100}, // Set 0x100	
    {0, 0x00BF}, // Send 0x0BF
    {0, 0x01FF}, // Set 0x01FF, FE is runtime paramater what panel register to be read
    {20, 0x0101}, // Set 0x100	
};

struct spi_data solomon_reg_mipi_read_back[] = {
    {0, 0x00B7},
    {0, 0x0183}, //bit 0 = 0 LP mode, 1 HS mode
    {0, 0x0103},
    {0, 0x00BB},
    {0, 0x0108},
    {0, 0x0100},
    {0, 0x00C1},
    {0, 0x010A},
    {0, 0x0100},
    {0, 0x00C0},
    {0, 0x0101},
    {0, 0x0100},
    {0, 0x00BC}, //Must be 1 byte read request, 0x0001
    {0, 0x01FE},
    {0, 0x0100},
    {0, 0x00BF},
    {0, 0x01FF}, //read panel register
    {16, 0x0100},
    {0, 0x00D4},
    {0, 0x01FA},
    {0, 0x0100},
    {0, 0x00C6},
};

struct spi_data solomon_reg_read_set8[] = {
    {0, 0x00D4},
    {0, 0x01FA},
    {0, 0x0100},
    {0, 0x00B0},
    //{0, 0x00FA},
};

struct spi_data solomon_init_sequence_set_Panasonic[] = {
    //                       continuous                             non-continuous (LowPower Mode)
	{0, 0x00B1}, //   1480MHz/150MHz                       155MHz                               RGB V_1920 x H_1200
	{0, 0x0102}, //       02                                           02                                    HSYNC Act=2, HBP=16. HFP=17 
	{0, 0x0110}, //       10                                           10                                    VSYNC Act=16, VBP=32, VFP=112
	{0, 0x00B2}, //        B2
	{0, 0x0112}, //        12                                          12 
	{0, 0x0130}, //        30                                          30
	{0, 0x00B3}, //        B3
	{0, 0x0111}, //        11                                          11
	{0, 0x0170}, //        70                                          70
	{0, 0x00B4}, //        B4
	{0, 0x01B0}, //        B0                                          B0
	{0, 0x0104}, //        04                                          04
	{0, 0x00B5}, //        B5
	{0, 0x0180}, //        80                                          80
	{0, 0x0107}, //        07                                          07
	{0, 0x00B6}, //        B6
	{0, 0x0117}, //        0B                                          17                                       Change to Burst Mode.
	{200, 0x0120}, //    20                                           20                                       Change to Rising Edge Trigger, no compressed burst   
	//                       continuous                                 non-continuous (LowPower Mode)
	{0, 0x00C9}, // C9   128MHz   154MHz                  155MHz  
	{0, 0x010C}, //          09          0B                          0C                                        was 0A increased HS-prepare delay
	{0, 0x0129}, //          26          2A                          29                                         was 29 increased HS-Zero delay
	{0, 0x00CA}, // CA   128MHz   154MHz
	{0, 0x010A}, //           07          08                         0A                                         was 0A, reducing CLK-Prepare
	{0, 0x013E}, //           38          3D                         3E
	{0, 0x00CB},
	{0, 0x011D},
	{0, 0x0103},
	{0, 0x00CC}, // CC   128MHz   154MHz                  155MHz  
	{0, 0x0114}, //          15              17                       14                                           was 13, increasing HS-Trail
	{0, 0x010F}, //          0E              0F                       0F                                           was 13, reducing CLK-Trail
	//{0, 0x00D9}, // D9   128MHz   154MHz                   
	//{0, 0x01A0}, //          A0              A0                      xx 
	//{0, 0x0167}, //           67              65                      xx                                            was 67, reducing HS driving current to fit MIPI SPEC
	{0, 0x00DE},
	{0, 0x0103},
	{0, 0x0100},
	{0, 0x00D6},
	{0, 0x0108},
	{0, 0x0100},
	{0, 0x00C4},
	{0, 0x0101},   // FBW Setting?
	{1, 0x0100},
    {0, 0x00BA}, // BA   128MHz   154MHz                  155MHz  
    {0, 0x0129}, //           26           29                         29                                               0x0125 for pclk 128000000 // PLL 41X, 984MHz, for 60fps, 154MHz Pixel clock
    {0, 0x01C0}, //           C0          C0                         C0
	{0, 0x00BB}, // BB
    {0, 0x010C}, //                08                                  0C                                                   Please set 0D for TLPX > 50ns
    {0, 0x0100}, //                00                                  00
	{0, 0x00B9},
    {0, 0x0101},
    {0, 0x0100},
	{0, 0x00B7}, // B7
    {0, 0x0143}, //                43                                  43
    {0, 0x0103}, //                00                                  03
	{0, 0x00B8},
    {0, 0x0100},
    {0, 0x0100},
	{0, 0x00BC},
    {0, 0x0101},
    {0, 0x0100},
	{0, 0x00BF},
    {0, 0x0111},
    {200, 0x0100},
    {0, 0x00BC},
    {0, 0x0101},
    {0, 0x0100},
	{0, 0x00BF},
    {0, 0x0129},
    {200, 0x0100},
    {0, 0x00B7}, // B7 
    {0, 0x0149}, //              4B                                  49
    {0, 0x0103}, //              00                                  03
};

struct spi_data solomon_init_sequence_set_IVO[] = {
    //                       continuous                      non-continuous (LowPower Mode)
	{0, 0x00B1}, //       B1                                    B1
	{0, 0x010A}, //       02                                    0A
	{0, 0x0102}, //       01                                    02
	{0, 0x00B2}, //       B2                                    B2
	{0, 0x0140}, //       2C                                    40
	{0, 0x0105}, //       04                                     05
	{0, 0x00B3}, //       B3                                     B3
	{0, 0x0140}, //       2A                                    40
	{0, 0x010F}, //       05                                     0F
	{0, 0x00B4},
	{0, 0x01B0},
	{0, 0x0104},
	{0, 0x00B5},
	{0, 0x0180},
	{0, 0x0107},
	{0, 0x00B6}, //       B6                                   B6
	{0, 0x0117}, //       0B                                   17             Change to Burst Mode.
	{200, 0x0120},//     20                                    20             00, Change to Rising Edge Trigger, no compressed burst 
	{0, 0x00C9}, // C9   128MHz   148MHz     128MHz   148MHz        C9
	{0, 0x010C}, //           06           07             0A             0C             was 0A increased HS-prepare delay
	{0, 0x0129}, //           23           28             24              29             was 29 increased HS-Zero delay
	{0, 0x00CA}, // CA   128MHz   148MHz                         CA
	{0, 0x010A}, //           06          07              08             0A             was 0A, reducing CLK-Prepare
	{0, 0x013E}, //           35          3D              37             3E
	{0, 0x00CB}, // CB   128MHz   148MHz                          CB
	{0, 0x011D}, //           19           1D             1A             1D
	{0, 0x0103}, //           02           03              02              03
	{0, 0x00CC}, // CC   128MHz   148MHz                          CC
	{0, 0x0114}, //           10           12               12             14              was 13, increasing HS-Trail
	{0, 0x010F}, //           0C           0D               0D              0F              was 13, reducing CLK-Trail
	//{0, 0x00D9},   // D9   128MHz   154MHz
	//{0, 0x01A0},   //         xx            xx
	//{0, 0x0167},   //         xx            xx                                    was 67, reducing HS driving current to fit MIPI SPEC
	{0, 0x00DE},
	{0, 0x0103},
	{0, 0x0100},
	{0, 0x00D6},
	{0, 0x0108},
	{0, 0x0100},
	{0, 0x00C4},
	{0, 0x0101},   // FBW Setting?
	{1, 0x0100},
    {0, 0x00BA},  // BA   128MHz   148MHz      128MHz   148MHz 
    {0, 0x0129},  //           23          28              24              29                0x0125 for pclk 128000 // PLL 41X, 984MHz, for 60fps, 154MHz Pixel clock
    {0, 0x01C0},  //           C0          C0             C0              C0                 
	{0, 0x00BB},  //  BB   128MHz   148MHz                          BB
    {0, 0x010C},  //          0A            0C             0A              0C               08, Please set 0D for TLPX > 50ns
    {0, 0x0100},  //          00            00              00              00
	{0, 0x00B9},
    {0, 0x0101},
    {0, 0x0100},
	{0, 0x00B7},
    {0, 0x0143},
    {0, 0x0103},
	{0, 0x00B8},
    {0, 0x0100},
    {0, 0x0100},
	{0, 0x00BC},
    {0, 0x0101},
    {0, 0x0100},
	{0, 0x00BF},
    {0, 0x0111},
    {200, 0x0100},
    {0, 0x00BC},
    {0, 0x0101},
    {0, 0x0100},
	{0, 0x00BF},
    {0, 0x0129},
    {200, 0x0100},
    {0, 0x00B7}, //          B7            B7              B7
    {0, 0x0149}, //          4B            4B              49
    {0, 0x0103}, //          03            03              03
};


#ifdef DEBUG_INITIAL
struct spi_data solomon_re_init_sequence_set[] = {
    {0, 0x00B1}, //RGB start
    {0, 0x0102},
    {0, 0x0110},
    {0, 0x00B2},
    {0, 0x0112},
    {0, 0x0130},
    {0, 0x00B3},
    {0, 0x0111},
    {0, 0x0170},
    {0, 0x00B4},
    {0, 0x01B0},
    {0, 0x0104},
    {0, 0x00B5},
    {0, 0x0180},
    {0, 0x0107},
    {0, 0x00B6},
    {0, 0x0107}, //5 - 18bits 7 24bit
    {200, 0x0100},
    {0, 0x00C9},
    {0, 0x0110},
    {0, 0x0114},
    {0, 0x00CA},
    {0, 0x0110},
    {0, 0x0128},
    {0, 0x00D9},
    {0, 0x01A0},
    {0, 0x0167},
    {0, 0x00DE},
    {0, 0x0103},
    {0, 0x0100},
    {0, 0x00D6},
    {0, 0x0108},
    {0, 0x0100},
    {0, 0x00C4},
    {0, 0x0101},
    {1, 0x0100},
    {0, 0x00BA},
    {0, 0x0125},
    {0, 0x01C0},
    {0, 0x00BB},
    {0, 0x0108},
    {0, 0x0100},
    {0, 0x00B9},
    {0, 0x0101},
    {0, 0x0100},
    {0, 0x00B7},
    {0, 0x0143},
    {0, 0x0100},
    {0, 0x00B8},
    {0, 0x0100},
    {0, 0x0100},
    {0, 0x00BC},
    {0, 0x0101},
    {0, 0x0100},
    {0, 0x00BF},
    {0, 0x0111},
    {200, 0x0100},
    {0, 0x00BC},
    {0, 0x0101},
    {0, 0x0100},
    {0, 0x00BF},
    {0, 0x0129},
    {200, 0x0100},
    {0, 0x00B7},
    {0, 0x014B},
    {0, 0x0100},
};
#endif

static int ssd2828_device_id;

static int spi_write9(u16 value)
{
    struct spi_message m;
    struct spi_transfer xfer;

    u8 w[2];
    int ret;

    spi_message_init(&m);

    memset(&xfer, 0, sizeof(xfer));

    w[1] = (value & 0xFF00) >> 8;
    w[0] = (value & 0x00FF);
    xfer.tx_buf = w;

    xfer.bits_per_word = 9;
    xfer.len = 2;
    spi_message_add_tail(&xfer, &m);

    ret = spi_sync(bridge_spi, &m);

    if (ret < 0)
        dev_warn(&bridge_spi->dev, "failed to write to value (%d)\n", value);
    return ret;
}

static int ssd2828_bridge_spi_get_devices_id(void)
{
    int cnt, read_cnt;
    u16 readdata1;
    struct spi_data *sequence;
    sequence =solomon_reg_read_set8;
	read_cnt=ARRAY_SIZE(solomon_reg_read_set8);
    // read_cnt = SEQUENCE_SIZE(solomon_reg_read_set8);
    // printk("%s *** read_cnt:%d \n",__func__,read_cnt);
    for ( cnt = 0 ; cnt < read_cnt ; cnt++ )
    {
            spi_write9(sequence->value);
            if( sequence->delay ) mdelay( sequence->delay );
            sequence++;
    }
    spi_read_byte16(0xFA,&readdata1);
    ssd2828_device_id = readdata1;
    pr_debug("[%s] device_id=%x \n", __func__, readdata1);
    return readdata1;
}

#ifdef DEBUG_INITIAL
static int ssd2828_bridge_spi_reg(u8 value)
{
	u16 regcmd;
	u16 readdata1;
	regcmd= (0x00FF & value);
	spi_write9(0x00D4);
	spi_write9(0x01FA);
	spi_write9(0x0100);
	spi_write9(regcmd);
	spi_read_byte16(0xFA,&readdata1);
	// printk("ssd2828_bridge_spi_reg success *** spi read finished \n");
	return readdata1;
}
#endif

static ssize_t ssd2828_bridge_show_device_id(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    int r=0;
    /* Read device id */
    pr_debug("%s \n",__func__);
    //printk("%s *** device_id : 0x%x \n",__func__,device_id);
	ssd2828_bridge_spi_get_devices_id();
    r = snprintf(buf, PAGE_SIZE, "0x%x \n",
        ssd2828_device_id);
    return r;
}

static ssize_t ssd2828_bridge_show_mipi_data(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    int loop=5,total_cnt,cnt,res=0;
	unsigned int sendcmd=0;
    struct spi_data *sequence;
    u16 readdata1=0;
    int r=0;
    /* Read device id */
    total_cnt=ARRAY_SIZE(solomon_reg_mipi_read_cmd);
    sequence = solomon_reg_mipi_read_cmd;
    for ( cnt = 0 ; cnt < total_cnt ; cnt++ ) {
		if(sequence->value==0x1FF) {
    	   //sequence->value=sendcmd1;
		   sendcmd = 0x100 | low_addr;
		   SPI_WRITE(sendcmd);
           if( sequence->delay )          // low byte address
              mdelay( sequence->delay );
		   //printk(" rgb_bridge_enable > bridge spi driver : value 0x%04x \n", sendcmd);	
           sequence++;
           sendcmd = 0x100 | high_addr;
		   SPI_WRITE(sendcmd);
           if( sequence->delay )          // high byte address
              mdelay( sequence->delay );				
		   //printk(" rgb_bridge_enable > bridge spi driver : value 0x%04x \n", sendcmd);
		   sequence++;
		   cnt++;
		   continue;
	    }
    	else if(sequence->value==0x1FE) {
            //sequence->value=read_bits;
			sendcmd=read_bits;
			SPI_WRITE(sendcmd);
			//printk(" rgb_bridge_enable > bridge spi driver : value 0x%04x \n", sendcmd);
			sequence++;
			continue;
        }		
    	  
		if(!bridge_mode && sequence->value==0x183) {
        		sequence->value=0x182;
    	    	printk("Nor debug ~~ !!! sequence->value==%x\n",sequence->value);
        } 
		else if(bridge_mode && sequence->value==0x182) {
		        sequence->value=0x183;
    		    printk("Nor debug ~~ !!! sequence->value==%x\n",sequence->value);
	    }

        if(sequence->value==0x183||sequence->value==0x182)
               printk("Nor debug ~~ sequence->value==%x\n",sequence->value);
			
        SPI_WRITE(sequence->value);
        //printk(" rgb_bridge_enable > bridge spi driver : cnt:%02d value 0x%04x \n",cnt, sequence->value);
        if( sequence->delay )
           mdelay( sequence->delay );
        sequence++;
    }
    res=0;
    //printk(" =========== Rx read back DATA ==\n");  // =========== Rx read back DATA ===========================
    //mdelay(16);  // Wait 16ms
    do {       // =========== SPI READ for 0xC6 of SSD2828  ===================
		  readdata1 = ssd2828_bridge_spi_reg(0xC6);  
          res = readdata1;
          //printk(" spi read finished loop=%d 0xC6=%x RDR=%x \n",loop,readdata1,readdata1&0x1);
		  loop--;
	} while ( ((res&0x1) !=1) && loop >= 0);

	if( loop < 0 )
		      printk("MIPI CMD time out !!\n");

	if( (res&0x1)==1 ) {  // =========== SPI READ for 0xFF of SSD2828  ===================
        for ( cnt = 0 ; cnt < read_bits ; cnt++ ) {
            readdata1=ssd2828_bridge_spi_reg(0xFF);
            //printk("Data:%4x\n",readdata1);		
        }
	}
    r = snprintf(buf, PAGE_SIZE, "0x%x",readdata1);
    return r;
}

#ifdef DEBUG_INITIAL
static ssize_t ssd2828_bridge_store_reg_dump(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	int readdata,i,sendcmd;
	sscanf(buf, "%d", &sendcmd);
	if(sendcmd==1)
	{
	    /* ssd2828 all reg dump */
		printk("\n %s *** cmd( data ) *** \n",__func__);
		for(i=0;i<sizeof(ssd2828_reg_set);i++){
			readdata=ssd2828_bridge_spi_reg(ssd2828_reg_set[i]);
			printk(" 0x%x( 0x%x ) , \n",ssd2828_reg_set[i],readdata);
			if(i%10==9) printk("\n");
		}
	    printk("\n");
	}
	return count;
}

static ssize_t ssd2828_bridge_store_set_bits(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int sendcmd;
    sscanf(buf, "%d", &sendcmd);
	read_bits=sendcmd;
    printk("ssd2828 set bits =%d\n",read_bits);
    return count;
}

static ssize_t ssd2828_bridge_show_get_bits(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
	int r;
	printk("ssd2828 ~~ get bits=%d\n",read_bits);
	r = snprintf(buf, PAGE_SIZE,
                        "Nor debug ~~ get read_bits=%d \n",
                        read_bits);
	return r;
}
static ssize_t ssd2828_bridge_store_set_mode(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
        int sendcmd;
        sscanf(buf, "%d", &sendcmd);
        bridge_mode=sendcmd;
        printk("ssd2828 set mode =%d\n",bridge_mode);
        return count;
}
static ssize_t ssd2828_bridge_show_get_mode(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
        int r;
        printk("ssd2828 ~~ get mode=%d\n",bridge_mode);
        r = snprintf(buf, PAGE_SIZE,
                        "Nor debug ~~ get read_bits=%d \n",
                        bridge_mode);
        return r;
}
static ssize_t ssd2828_bridge_store_read_mipi_reg(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    sscanf(buf, "%2x %02x", &low_addr, &high_addr); //read mipi register address
    //printk("### ssd2828_bridge_store_read_mipi_reg read_mipi_reg=%2x %2x, %d Byte\n",low_addr,high_addr,read_bits);
    return count;
}

static ssize_t ssd2828_bridge_store_re_initial(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int sendcmd,total_cnt,cnt,temp=-1;
	struct spi_data *sequence;
    sscanf(buf, "%x", &sendcmd);
    
    printk("ssd2828 re-initial =%x\n",sendcmd);

    ssd2828_bridge_spi_get_devices_id();

    total_cnt=ARRAY_SIZE(solomon_re_init_sequence_set);
    sequence = solomon_re_init_sequence_set;
    for ( cnt = 0 ; cnt < total_cnt ; cnt++ )
    {
	    if(sequence->value==sendcmd)
		{
		    temp=cnt;
		    printk("Nor debug ~~ change %x register\n",sequence->value);
	    }
    	if (temp!=-1 && cnt==temp+1)
		{
	    	sequence->value=bridge_mode;
		    printk("Nor debug ~~ change bit 15-8 =%x register\n",sequence->value);
    	}
	    else if (temp!=-1 && cnt==temp+2)
		{
		    sequence->value=read_bits;
    		printk("Nor debug ~~ change bit 7-0 =%x register\n",sequence->value);
	    }
        SPI_WRITE(sequence->value);
        /* printk(" rgb_bridge_enable > bridge spi driver : value %8x \n", sequence->value); */
        if( sequence->delay )
            mdelay( sequence->delay );
        sequence++;
    }

    return count;
}
#endif

DEVICE_ATTR(device_id, 0660, ssd2828_bridge_show_device_id, NULL);
DEVICE_ATTR(mipi_data, 0660, ssd2828_bridge_show_mipi_data, NULL);
#ifdef DEBUG_INITIAL
DEVICE_ATTR(reg_dump, 0660, NULL, ssd2828_bridge_store_reg_dump);
DEVICE_ATTR(read_bits, 0660, ssd2828_bridge_show_get_bits, ssd2828_bridge_store_set_bits);
DEVICE_ATTR(read_mode, 0660, ssd2828_bridge_show_get_mode, ssd2828_bridge_store_set_mode);
DEVICE_ATTR(read_mipi_reg, 0660, NULL, ssd2828_bridge_store_read_mipi_reg);
DEVICE_ATTR(re_initial, 0660, NULL, ssd2828_bridge_store_re_initial);
#endif

int bridge_power_on_off(bool on){
	if(on){
		gpio_direction_output(en_mipi_pwr, 1);
		gpio_direction_output(en_lcd_pwr, 1);
		gpio_direction_output(mipi_rst_l, 1);
		gpio_direction_output(mipi_shut, 0);
	}
	else
	{
		gpio_direction_output(en_lcd_pwr, 0);
		gpio_direction_output(mipi_rst_l, 0);
		gpio_direction_output(mipi_shut, 1);
		gpio_direction_output(en_mipi_pwr, 1);
	}
	return 0;
}

int bridge_enable(void)
{
    int total_cnt,cnt;
    struct spi_data *sequence;
    pr_debug("[%s]\n", __func__);
    bridge_power_on_off(1);
    ssd2828_bridge_spi_get_devices_id();
    if( IsPansonicPanel() )  { //  Pansonic Panel	
	   printk("[video] bridge_enable() for Pansonic Panel \n");
       total_cnt=ARRAY_SIZE(solomon_init_sequence_set_Panasonic);
       sequence = solomon_init_sequence_set_Panasonic;
	} else {               // IVO
	    printk("[video] bridge_enable() for IVO Panel \n");
       total_cnt=ARRAY_SIZE(solomon_init_sequence_set_IVO);
       sequence = solomon_init_sequence_set_IVO;
    }	
    for ( cnt = 0 ; cnt < total_cnt ; cnt++ )
    {
        SPI_WRITE(sequence->value);
        pr_debug("bridge spi value %8x \n", sequence->value);
        if( sequence->delay )
            mdelay( sequence->delay );
        sequence++;
    }
    return 0;
}

int bridge_disable(void)
{
	bridge_power_on_off(0);
	disable_end = TRUE;
	return 0;
}

static int bridge_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	disable_end = FALSE;
	//bridge_disable();
	return 0;
}

static void bridge_spi_shutdown(struct spi_device *spi)
{
	disable_end = FALSE;
	gpio_direction_output(en_lcd_pwr, 0);
	gpio_direction_output(mipi_rst_l, 0);
	gpio_direction_output(mipi_shut, 0);
	gpio_direction_output(en_mipi_pwr, 0);
	return;
}

static int bridge_spi_resume(struct spi_device *spi)
{
	//bridge_enable();
	return 0;
}

static int bridge_spi_probe(struct spi_device *spi)
{
	struct bridge_platform_data *pdata;
	int ret;
	pdata = spi->dev.platform_data;
	
	if (!pdata) {
		dev_err(&spi->dev, "Nor debug ~~ bridge_spi_probe null pdata\n");
		return 0;
	}

	/* request platform data */
	spi->bits_per_word = pdata->bits_per_word;
	spi->mode = pdata->mode;
	spi->max_speed_hz = pdata->max_speed_hz;

	/* spi set up */
	ret = spi_setup(spi);

	if (ret < 0) {
		dev_err(&spi->dev, "Nor debug ~~ %s spi_setup failed: %d\n",  __func__, ret);
		return ret;
	}

	bridge_spi = spi;
	ret = device_create_file(&spi->dev, &dev_attr_device_id);
   ret = device_create_file(&spi->dev, &dev_attr_mipi_data);
#ifdef DEBUG_INITIAL
	ret = device_create_file(&spi->dev, &dev_attr_reg_dump);
	ret = device_create_file(&spi->dev, &dev_attr_read_bits);
	ret = device_create_file(&spi->dev, &dev_attr_read_mode);
	ret = device_create_file(&spi->dev, &dev_attr_read_mipi_reg);
	ret = device_create_file(&spi->dev, &dev_attr_re_initial);
#endif

    return 0;
}

static int __devexit bridge_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver bridge_spi_driver = {
	.driver = {
		.name	= STR_RGB_BRIDGE,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe = bridge_spi_probe,
	.remove = __devexit_p(bridge_spi_remove),
	.suspend = bridge_spi_suspend,
	.resume = bridge_spi_resume,
	.shutdown = bridge_spi_shutdown,
};

static int __init bridge_init(void)
{
	int ret = spi_register_driver(&bridge_spi_driver);
	return ret;
}

static void __exit bridge_exit(void)
{
	spi_unregister_driver(&bridge_spi_driver);
}
subsys_initcall(bridge_init);
module_exit(bridge_exit);

MODULE_DESCRIPTION("LCD Driver");
MODULE_LICENSE("GPL");

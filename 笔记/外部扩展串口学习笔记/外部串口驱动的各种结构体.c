/*外部扩展串口芯片是挂接在 EBI0 Chip Select 5 下面 即在内存地址0x6000_0000起始的64K内存范围内*/

static struct at91_uart_config __initdata ek_uart_config = {
	.console_tty	= 0,				/* ttyS0 */
	.nr_tty		= 4,
	.tty_map	= { 3, 0, 1, 2, }		/* ttyS0, ..., ttyS3 */
};

static struct resource ebi05_resource[] = {
	[0] = {
		.start	= AT91_CHIPSELECT_5,
		.end	= AT91_CHIPSELECT_5 + 0x10000,	/*64K*/
		.flags	= IORESOURCE_MEM
	},
};

static struct platform_device ebi05_device = {
	.name			= "at91_ebi05",
	.id			= 0,
	.resource		= ebi05_resource,
	.num_resources	= ARRAY_SIZE(ebi05_resource),		=1
};

static void __init ek_add_device_ebi05(void)
{
	/*	配置外扩串口芯片的相关引脚	PE12、PE13、PE14、PE15、PD7五个引脚
	 * Configure Chip-Select 5 on SMC.	AT91_SMC_SETUP(5)=0x400+5 * 0x10=0x450	AT91_SMC_NWESETUP_(5)=5
	 * Note: These timings were calculated for MASTER_CLOCK = 100000000
	 */
	at91_set_gpio_input(AT91_PIN_PE12,1);		/*上拉输入，8250设备的中断引脚*/
	at91_set_gpio_input(AT91_PIN_PE13,1);
	at91_set_gpio_input(AT91_PIN_PE14,1);
	at91_set_gpio_input(AT91_PIN_PE15,1);																	
	at91_set_A_periph(AT91_PIN_PD7, 1);
	at91_sys_write(AT91_SMC_SETUP(5), AT91_SMC_NWESETUP_(5) | AT91_SMC_NCS_WRSETUP_(3) | AT91_SMC_NRDSETUP_(5) | AT91_SMC_NCS_RDSETUP_(3));/*at91_sys_write(0x450, 5|768|327680|50331648)=at91_sys_write(0x450,0x3050305)*/
	at91_sys_write(AT91_SMC_PULSE(5), AT91_SMC_NWEPULSE_(10) | AT91_SMC_NCS_WRPULSE_(15) | AT91_SMC_NRDPULSE_(10) | AT91_SMC_NCS_RDPULSE_(15));
	at91_sys_write(AT91_SMC_CYCLE(5), AT91_SMC_NWECYCLE_(20) | AT91_SMC_NRDCYCLE_(20));
	at91_sys_write(AT91_SMC_MODE(5), AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_BAT_WRITE | AT91_SMC_DBW_8 | AT91_SMC_TDF_(1));

	

	/* Configure Interrupt pin as input, no pull-up */
//	at91_set_gpio_input(AT91_PIN_PD4, 0);
//	platform_device_register(&ebi05_device);
}


/*platform_device_alloc函数给这个结构体动态申请内存空间kzalloc(sizeof(struct platform_object) + strlen(name), GFP_KERNEL);*/
struct platform_object {
	struct platform_device pdev;
	char name[1];					="serial8250"
};

struct platform_device {
	const char	* name;				="serial8250"
	u32		id;						=-1
	struct device	dev;			=dev(初始化了)
	u32		num_resources;			=？(用到了但是没有找到在哪儿赋值的)
	struct resource	* resource;		=?
};

static struct platform_driver serial8250_isa_driver = {
	.probe		= serial8250_probe,
	.remove		= __devexit_p(serial8250_remove),
	.suspend	= serial8250_suspend,
	.resume		= serial8250_resume,
	.driver		= {
		.name	= "serial8250",
		.owner	= THIS_MODULE,
	},
};

/*uart_register_driver函数将这个结构体注册到内核*/
static struct uart_driver serial8250_reg = {
	.owner			= THIS_MODULE,
	.driver_name	= "serial",								/*驱动名*/
	.dev_name		= "ttyS",								/*设备名*/
	.major			= TTY_MAJOR,							/*主设备号*/
	.minor			= 64,									/*次设备号的起始设备号*/
	.nr				= UART_NR(4),							/*次设备的最大数量*/
	.cons			= serial8250_console,					/*控制台*/
	.state			=  kmalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL)
	.tty_driver		= tty_driver(执行动态分配内存的struct tty_driver实例)
};

/*每一个串口设备的结构体，我们这里有4个串口设备，就动态分配了4个这样的结构体缓存*/
struct uart_state {
	unsigned int		close_delay;			=500	/* msec */
	unsigned int		closing_wait;			=30000	/* msec */

#define USF_CLOSING_WAIT_INF	(0)
#define USF_CLOSING_WAIT_NONE	(~0U)

	int			count;
	int			pm_state;
	struct uart_info	*info;
	struct uart_port	*port;					=uart_port结构体

	struct mutex		mutex;					=初始化了
};

struct uart_8250_port {
	struct uart_port	port;
	struct timer_list	timer		=
									{
										.function = serial8250_timeout,
										.entry.next = NULL,
										
									};		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned short		capabilities;	/* port capabilities */
	unsigned short		bugs;		/* port bugs */
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;			=~ALPHA_KLUDGE_MCR				/* mask of user bits */
	unsigned char		mcr_force;			=ALPHA_KLUDGE_MCR				/* mask of forced bits */
	unsigned char		lsr_break_flag;

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};


static struct uart_ops serial8250_pops = {		/*uart的方法*/
	.tx_empty	= serial8250_tx_empty,
	.set_mctrl	= serial8250_set_mctrl,
	.get_mctrl	= serial8250_get_mctrl,
	.stop_tx	= serial8250_stop_tx,
	.start_tx	= serial8250_start_tx,
	.stop_rx	= serial8250_stop_rx,
	.enable_ms	= serial8250_enable_ms,
	.break_ctl	= serial8250_break_ctl,
	.startup	= serial8250_startup,
	.shutdown	= serial8250_shutdown,
	.set_termios	= serial8250_set_termios,
	.pm		= serial8250_pm,
	.type		= serial8250_type,
	.release_port	= serial8250_release_port,
	.request_port	= serial8250_request_port,
	.config_port	= serial8250_config_port,
	.verify_port	= serial8250_verify_port,
};

struct uart_icount {
	__u32	cts;				/**/
	__u32	dsr;
	__u32	rng;
	__u32	dcd;
	__u32	rx;					/*接收字符计数*/
	__u32	tx;					/*发送字符计数*/
	__u32	frame;				/*帧错误计数*/
	__u32	overrun;			/*Rx FIFO溢出计数*/
	__u32	parity;				/*帧校验错误计数*/
	__u32	brk;				/*break计数*/
	__u32	buf_overrun;		/**/
};

struct uart_port {
	spinlock_t			lock;			=上锁了初始化了											/* port lock 串口端口的自旋锁*/
	unsigned int		iobase;			=(unsigned long)ioremap(old_serial_port[i].port, 7<<2+1)/*串口端口的I/O基地址*/
	unsigned char __iomem	*membase;	=(unsigned long)ioremap(old_serial_port[i].port, 7<<2+1);/* read/write[bwl] I/O内存基地址经映射后对应的虚拟基地址*/
	unsigned int		irq;			=irq_canonicalize(old_serial_port[i].irq);				/* irq number 串口需要的中断号*/
	unsigned int		uartclk;		=old_serial_port[i].baud_base * 16=115200*16			/* base uart clock 串口时钟*/
	unsigned int		fifosize;		/* tx fifo size 串口FIFO缓冲的大小*/
	unsigned char		x_char;			/* xon/xoff char xon/xoff字符*/
	unsigned char		regshift;		=old_serial_port[i].iomem_reg_shift=?					/* reg offset shift 寄存器位移*/
	unsigned char		iotype;			=old_serial_port[i].io_type=? /* io access style I/O访问方式，串口端口寄存器的地址类型*/
	unsigned char		unused1;		/*保留*/

#define UPIO_PORT		(0)				/*I/O端口*/
#define UPIO_HUB6		(1)
#define UPIO_MEM		(2)				/*I/O内存*/
#define UPIO_MEM32		(3)
#define UPIO_AU			(4)			/* Au1x00 type IO */
#define UPIO_TSI		(5)			/* Tsi108/109 type IO */

	unsigned int		read_status_mask;						/* driver specific 接收状态*/
	unsigned int		ignore_status_mask;						/* driver specific */
	struct uart_info	*info;			=uart_state.info		/* pointer to parent info */
	struct uart_icount	icount;									/* statistics 计数器*/

	struct console		*cons;			=serial8250_console		/* struct console, if any console结构体*/
#ifdef CONFIG_SERIAL_CORE_CONSOLE
	unsigned long		sysrq;			/* sysrq timeout */
#endif

	upf_t			flags;				=old_serial_port[i].flags=STD_COM_FLAGS

#define UPF_FOURPORT		((__force upf_t) (1 << 1))
#define UPF_SAK			((__force upf_t) (1 << 2))
#define UPF_SPD_MASK		((__force upf_t) (0x1030))
#define UPF_SPD_HI		((__force upf_t) (0x0010))
#define UPF_SPD_VHI		((__force upf_t) (0x0020))
#define UPF_SPD_CUST		((__force upf_t) (0x0030))
#define UPF_SPD_SHI		((__force upf_t) (0x1000))
#define UPF_SPD_WARP		((__force upf_t) (0x1010))
#define UPF_SKIP_TEST		((__force upf_t) (1 << 6))
#define UPF_AUTO_IRQ		((__force upf_t) (1 << 7))
#define UPF_HARDPPS_CD		((__force upf_t) (1 << 11))
#define UPF_LOW_LATENCY		((__force upf_t) (1 << 13))
#define UPF_BUGGY_UART		((__force upf_t) (1 << 14))
#define UPF_MAGIC_MULTIPLIER	((__force upf_t) (1 << 16))
#define UPF_CONS_FLOW		((__force upf_t) (1 << 23))
#define UPF_SHARE_IRQ		((__force upf_t) (1 << 24))
#define UPF_BOOT_AUTOCONF	((__force upf_t) (1 << 28))
#define UPF_DEAD		((__force upf_t) (1 << 30))
#define UPF_IOREMAP		((__force upf_t) (1 << 31))

#define UPF_CHANGE_MASK		((__force upf_t) (0x17fff))
#define UPF_USR_MASK		((__force upf_t) (UPF_SPD_MASK|UPF_LOW_LATENCY))

	unsigned int		mctrl;										/* current modem ctrl settings */
	unsigned int		timeout;									/* character-based timeout */
	unsigned int		type;			=？							/* port type 端口类型*/
	const struct uart_ops	*ops;		=serial8250_pops			/*串口端口操作函数集*/
	unsigned int		custom_divisor;
	unsigned int		line;			=0、1、2、3		 			/* port index 端口索引*/
	unsigned long		mapbase;		=(unsigned long)ioremap(old_serial_port[i].port, 7<<2+1);		/* for ioremap */
	struct device		*dev;			=device结构体							/* parent device 父设备*/
	unsigned char		hub6;			=old_serial_port[i].hub6;	/* this should be in the 8250 driver */
	unsigned char		unused[3];
};

static const struct old_serial_port old_serial_port[] = 
{
	SERIAL_PORT_DFNS 		=		/* defined in asm/serial.h */
	{
	  /*UART BAUD_BASE   PORT IRQ     FLAGS        */			
		{ 0, BASE_BAUD, 0x3F8, 4, STD_COM_FLAGS },	/* ttyS0 BASE_BAUD=115200*/	
		{ 0, BASE_BAUD, 0x2F8, 3, STD_COM_FLAGS },	/* ttyS1 */	
		{ 0, BASE_BAUD, 0x3E8, 4, STD_COM_FLAGS },	/* ttyS2 */	
		{ 0, BASE_BAUD, 0x2E8, 3, STD_COM4_FLAGS },	/* ttyS3 */
	}
};

/*使用alloc_tty_driver函数来给这个结构体动态分配内存alloc_tty_driver(drv->nr)*/
struct tty_driver {
	int	magic;							=TTY_DRIVER_MAGIC(0x5402)					/* magic number for this structure */
	struct cdev cdev;					=struct cdev这个结构体						/*对应的字符设备*/
	struct module	*owner;				=THIS_MODULE								/*这个驱动的模块拥有者*/
	const char	*driver_name;			="serial"									/*驱动名*/
	const char	*name;					="ttyS"										/*设备名，设备节点名*/
	int	name_base;																	/* offset of printed name */
	int	major;							=4											/* major device number 主设备号*/
	int	minor_start;					=64											/* 开始次设备号，一般为0 */
	int	minor_num;						=uart_driver->nr=4							/* number of *possible* devices 设备数量*/
	int	num;							=4											/*  被分配的设备数量*/
	short	type;						=TTY_DRIVER_TYPE_SERIAL(0x0003)				/* type of tty driver tty驱动的类型*/
	short	subtype;					=SERIAL_TYPE_NORMAL(1)						/*  tty驱动的子类型*/
	struct ktermios init_termios; 		=tty_std_termios							/* Initial termios 初始线路设置*/
	int	flags;							=0xC										/* tty driver flags tty驱动标志*/
	int	refcount;																	/* for loadable tty drivers 引用计数*/
	struct proc_dir_entry *proc_entry; 	=指向 struct proc_dir_entry 结构体 			/* /proc fs entry proc文件系统入口*/
	struct tty_driver *other; 														/*   仅对PTY驱动有意义*/

	/*
	 * Pointer to the tty data structures 接口函数
	 */
	struct tty_struct **ttys;			=(tty_struct **)kmalloc(driver->num * 3 * sizeof(void *), GFP_KERNEL)
	struct ktermios **termios;			=(tty_struct **)(kmalloc(driver->num * 3 * sizeof(void *), GFP_KERNEL) + 4)
	struct ktermios **termios_locked;	=(tty_struct **)(kmalloc(driver->num * 3 * sizeof(void *), GFP_KERNEL) + 8)
	void *driver_state;					=serial8250_reg								/* only used for the PTY driver */
	
	/*
	 * Interface routines from the upper tty layer to the tty
	 * driver.	Will be replaced with struct tty_operations.
	 */
	int  (*open)(struct tty_struct * tty, struct file * filp);					=uart_ops->open
	void (*close)(struct tty_struct * tty, struct file * filp);					=uart_ops->close
	int  (*write)(struct tty_struct * tty,
		      const unsigned char *buf, int count);								=uart_ops->write
	void (*put_char)(struct tty_struct *tty, unsigned char ch);					=uart_ops->put_char
	void (*flush_chars)(struct tty_struct *tty);								=uart_ops->flush_chars
	int  (*write_room)(struct tty_struct *tty);									=uart_ops->write_room
	int  (*chars_in_buffer)(struct tty_struct *tty);							=uart_ops->chars_in_buffer
	int  (*ioctl)(struct tty_struct *tty, struct file * file,	
		    unsigned int cmd, unsigned long arg);								=uart_ops->ioctl
	void (*set_termios)(struct tty_struct *tty, struct ktermios * old);			=uart_ops->set_termios
	void (*throttle)(struct tty_struct * tty);									=uart_ops->throttle
	void (*unthrottle)(struct tty_struct * tty);								=uart_ops->unthrottle
	void (*stop)(struct tty_struct *tty);										=uart_ops->stop
	void (*start)(struct tty_struct *tty);										=uart_ops->start
	void (*hangup)(struct tty_struct *tty);										=uart_ops->hangup
	void (*break_ctl)(struct tty_struct *tty, int state);						=uart_ops->break_ctl
	void (*flush_buffer)(struct tty_struct *tty);								=uart_ops->flush_buffer
	void (*set_ldisc)(struct tty_struct *tty);									=uart_ops->set_ldisc
	void (*wait_until_sent)(struct tty_struct *tty, int timeout);				=uart_ops->wait_until_sent
	void (*send_xchar)(struct tty_struct *tty, char ch);						=uart_ops->send_xchar
	int (*read_proc)(char *page, char **start, off_t off,
			  int count, int *eof, void *data);									=uart_ops->read_proc
	int (*write_proc)(struct file *file, const char __user *buffer,
			  unsigned long count, void *data);									=uart_ops->write_proc=NULL
	int (*tiocmget)(struct tty_struct *tty, struct file *file);					=uart_ops->tiocmget
	int (*tiocmset)(struct tty_struct *tty, struct file *file,	
			unsigned int set, unsigned int clear);								=uart_ops->tiocmset

	struct list_head tty_drivers;
};

static const struct file_operations tty_fops = {
	.llseek		= no_llseek,
	.read		= tty_read,
	.write		= tty_write,
	.poll		= tty_poll,
	.ioctl		= tty_ioctl,
	.open		= tty_open,
	.release	= tty_release,
	.fasync		= tty_fasync,
};

static const struct tty_operations uart_ops = {
	.open		= uart_open,
	.close		= uart_close,
	.write		= uart_write,
	.put_char	= uart_put_char,
	.flush_chars	= uart_flush_chars,
	.write_room	= uart_write_room,
	.chars_in_buffer= uart_chars_in_buffer,
	.flush_buffer	= uart_flush_buffer,
	.ioctl		= uart_ioctl,
	.throttle	= uart_throttle,
	.unthrottle	= uart_unthrottle,
	.send_xchar	= uart_send_xchar,
	.set_termios	= uart_set_termios,
	.stop		= uart_stop,
	.start		= uart_start,
	.hangup		= uart_hangup,
	.break_ctl	= uart_break_ctl,
	.wait_until_sent= uart_wait_until_sent,
#ifdef CONFIG_PROC_FS
	.read_proc	= uart_read_proc,
#endif
	.tiocmget	= uart_tiocmget,
	.tiocmset	= uart_tiocmset,
	.write_proc	= NULL
};

struct ktermios tty_std_termios = {	/* for the benefit of tty drivers  */
	.c_iflag = ICRNL | IXON,
	.c_oflag = OPOST | ONLCR,
	.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL,
	.c_lflag = ISIG | ICANON | ECHO | ECHOE | ECHOK |
		   ECHOCTL | ECHOKE | IEXTEN,
	.c_cc = INIT_C_CC,
	.c_ispeed = 9600,
	.c_ospeed = 9600
};

struct device {
	struct klist		klist_children;								/*设备列表中的子列表*/
	struct klist_node	knode_parent;								/* node in sibling list 兄弟列表*/
	struct klist_node	knode_driver;								/*驱动节点*/
	struct klist_node	knode_bus;									/*总线节点*/
	struct device 	* parent;			="platform"					/*指向符设备*/

	struct kobject kobj;											/*内嵌的kobject对象*/
	char	bus_id[BUS_ID_SIZE];		="serial8250"				/* position on parent bus 在总线上的位置*/
	unsigned		is_registered:1;								/*标识该设备是否已经被注册过*/
	struct device_attribute uevent_attr =
	{
			.attr.name = "uevent",
			.attr.mode = S_IWUSR=002000,
			.store = store_uevent,
	};							/**/
	struct device_attribute *devt_attr;

	struct semaphore	sem;										/*用于同步的信号量*/
	
	struct bus_type	* bus;				=platform_bus_type			/*指向总线类型*/
	struct device_driver *driver;		=?(device_add函数)			/*所使用的驱动程序*/
	void		*driver_data;										/* data private to the driver 驱动的私有数据*/
	void		*platform_data;										/*平台的特定数据*/
	struct dev_pm_info	power;

#ifdef CONFIG_NUMA
	int		numa_node;	/* NUMA node this device is close to */
#endif
	u64		*dma_mask;												/*DMA掩码*/
	u64		coherent_dma_mask;										/*设备一致性DMA的屏蔽字*/

	struct list_head	dma_pools;									/*DMA缓冲池*/

	struct dma_coherent_mem	*dma_mem; 								/*指向设备所使用的一致性DMA存储器描述符的指针*/
	struct dev_archdata	archdata;

	/* class_device migration path */
	struct list_head	node;
	struct class		*class;							=tty_class	/*指向属于的类*/
	dev_t			devt;								=设备号		/*=MKDEV(driver->major, driver->minor_start) + index*/
	struct attribute_group	**groups;								/* 附加属性的数组optional groups */

	void	(*release)(struct device * dev);			=platform_device_release	/*释放设备的方法*/
};

struct device_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct device *dev, struct device_attribute *attr,
			char *buf);
	ssize_t (*store)(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);
};

struct attribute {
	const char		* name;					="uevent"
	struct module 		* owner;
	mode_t			mode;					=S_IWUSR=002000
};

struct bus_type platform_bus_type = {
	.name		= "platform",
	.dev_attrs	= platform_dev_attrs,
	.match		= platform_match,
	.uevent		= platform_uevent,
	.suspend	= platform_suspend,
	.suspend_late	= platform_suspend_late,
	.resume_early	= platform_resume_early,
	.resume		= platform_resume,
};

static struct console serial8250_console = {
	.name		= "ttyS",
	.write		= serial8250_console_write,
	.device		= uart_console_device,
	.setup		= serial8250_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial8250_reg,
};

struct resource {
	resource_size_t start;
	resource_size_t end;
	const char *name;							="serial8250"
	unsigned long flags;						=
	struct resource *parent, *sibling, *child;
};

static const struct serial8250_config uart_config[] = {			/*串口类型配置结构体*/
	[PORT_UNKNOWN] = {
		.name		= "unknown",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_8250] = {
		.name		= "8250",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16450] = {
		.name		= "16450",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550] = {
		.name		= "16550",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550A] = {
		.name		= "16550A",
		.fifo_size	= 16,
		.tx_loadsz	= 16,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO,
	},
	[PORT_CIRRUS] = {
		.name		= "Cirrus",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16650] = {
		.name		= "ST16650",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_16650V2] = {
		.name		= "ST16650V2",
		.fifo_size	= 32,
		.tx_loadsz	= 16,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_01 |
				  UART_FCR_T_TRIG_00,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_16750] = {
		.name		= "TI16750",
		.fifo_size	= 64,
		.tx_loadsz	= 64,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10 |
				  UART_FCR7_64BYTE,
		.flags		= UART_CAP_FIFO | UART_CAP_SLEEP | UART_CAP_AFE,
	},
	[PORT_STARTECH] = {
		.name		= "Startech",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16C950] = {
		.name		= "16C950/954",
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO,
	},
	[PORT_16654] = {
		.name		= "ST16654",
		.fifo_size	= 64,
		.tx_loadsz	= 32,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_01 |
				  UART_FCR_T_TRIG_10,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_16850] = {
		.name		= "XR16850",
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_RSA] = {
		.name		= "RSA",
		.fifo_size	= 2048,
		.tx_loadsz	= 2048,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_11,
		.flags		= UART_CAP_FIFO,
	},
	[PORT_NS16550A] = {
		.name		= "NS16550A",
		.fifo_size	= 16,
		.tx_loadsz	= 16,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO | UART_NATSEMI,
	},
	[PORT_XSCALE] = {
		.name		= "XScale",
		.fifo_size	= 32,
		.tx_loadsz	= 32,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO | UART_CAP_UUE,
	},
};

/*cdev_init函数和cdev_add函数共同填充这个结构体，即实例化这个结构体的一个对象*/
struct cdev {
	struct kobject kobj;				=struct kobject结构体
	struct module *owner;				=THIS_MODULE
	const struct file_operations *ops;	=tty_fops(struct file_operations 结构体变量)/*操作这个字符设备文件的方法*/
	struct list_head list;				=初始化					/*与cdev对应的字符设备文件的inode->i_devices的链表头*/
	dev_t dev;							=设备号					/*起始设备编号*/
	unsigned int count;					=4						/*设备号范围大小*/
};

struct kobject {
	const char		* k_name;
	char			name[KOBJ_NAME_LEN];
	struct kref		kref;
	struct list_head	entry;
	struct kobject		* parent;
	struct kset		* kset;					=kset_get(kobj->kset);
	struct kobj_type	* ktype;			=指向 struct kobj_type 这个结构体
	struct dentry		* dentry;
	wait_queue_head_t	poll;
};

struct kobj_type {
	void (*release)(struct kobject *);		=cdev_default_release
	struct sysfs_ops	* sysfs_ops;
	struct attribute	** default_attrs;
};

/*proc文件系统的文件夹入口结构体*/
struct proc_dir_entry {
	unsigned int low_ino;
	unsigned short namelen;
	const char *name;
	mode_t mode;
	nlink_t nlink;
	uid_t uid;
	gid_t gid;
	loff_t size;
	struct inode_operations * proc_iops;
	const struct file_operations * proc_fops;
	get_info_t *get_info;
	struct module *owner;								=THIS_MODULE
	struct proc_dir_entry *next, *parent, *subdir;
	void *data;											=指向 struct tty_driver 结构体
	read_proc_t *read_proc;								=uart_ops->read_proc=uart_read_proc
	write_proc_t *write_proc;							=uart_ops->write_proc=NULL
	atomic_t count;		/* use count */
	int deleted;		/* delete flag */
	void *set;
};


/*注册外部扩展串口设备的驱动，模型是8250，在drivers/serial/8250.c这个文件里面*/
1、 serial8250_init 函数来初始化外部扩展串口芯片：注册设备和注册设备驱动
	2、 spin_lock_init 初始化设备的自旋锁
	2、 uart_register_driver 注册tty设备驱动
	2、 platform_device_alloc 为 struct platform_object结构体动态申请内存，返回指向该结构体内的 struct platform_device 结构体指针
			并给 struct platform_device 结构体的一些元素赋值
	2、 platform_device_add 函数将上面申请的 struct platform_device 结构体变量代表的设备注册到内核里
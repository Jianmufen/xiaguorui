/*Atmel芯片内部外设驱动注册路线分析*/
1、最重要的两个文件是 atmel_serial.c 和 atmel_serial.h 

2、函数执行路线
	1、 atmel_serial_init
		1、 uart_register_driver		绑定的结构体是 struct uart_driver atmel_uart
			1、 结构体 struct uart_driver 里面是一些驱动信息:主设备号、次设备号、设备名、驱动名、设备数量等等一些信息
				2、 alloc_tty_driver函数为 struct tty_driver 结构体动态申请内存，并为里面的元素magic和num赋值
				2、 tty_set_operations 函数是将 struct tty_operations结构体变量里面的操作方法赋值给 struct tty_driver结构体里面
					的函数指针
				2、 tty_register_driver 函数是将 struct tty_driver驱动注册成功的
					3、 register_chrdev_region 函数注册设备号
					3、 cdev_init 和 cdev_add 两个函数是注册字符设备驱动
					3、 list_add 是将刚注册的驱动添加到内核管理的链表上
		2、 platform_driver_register	绑定的结构体是 struct platform_driver atmel_serial_driver
			1、 struct platform_driver结构体里面最重要的函数是 atmel_serial_driver.probe=atmel_serial_probe
				2、 atmel_serial_probe函数调用了 atmel_init_port 和 uart_add_one_port 这两个函数
					3、 atmel_init_port函数里面将struct uart_ops atmel_pops 变量挂载到struct uart_port结构体下面的ops元素
						4、 struct uart_ops atmel_pops 里面是一些硬件操作:申请IO资源、中断号、配置串口等等一些资源操作的方法


/*这个结构体里面封装了串口所使用的资源信息:使用的中断号是多少，具体需要使用到的IO资源在内存中的地址等等*/
struct uart_port {
	spinlock_t		lock;			/* port lock 串口端口的自旋锁*/
	unsigned int		iobase;			/* in/out[bwl] 串口端口的I/O基地址*/
	unsigned char __iomem	*membase;		=0xFEFF_EE00	/* read/write[bwl] I/O内存基地址经映射后对应的虚拟基地址*/
	unsigned int		irq;			/* irq number 串口需要的中断号*/
	unsigned int		uartclk;		/* base uart clock 串口时钟*/
	unsigned int		fifosize;		/* tx fifo size 串口FIFO缓冲的大小*/
	unsigned char		x_char;			/* xon/xoff char xon/xoff字符*/
	unsigned char		regshift;		/* reg offset shift 寄存器位移*/
	unsigned char		iotype;			/* io access style I/O访问方式，串口端口寄存器的地址类型*/
	unsigned char		unused1;		/*保留*/

#define UPIO_PORT		(0)				/*I/O端口*/
#define UPIO_HUB6		(1)
#define UPIO_MEM		(2)				/*I/O内存*/
#define UPIO_MEM32		(3)
#define UPIO_AU			(4)			/* Au1x00 type IO */
#define UPIO_TSI		(5)			/* Tsi108/109 type IO */

	unsigned int		read_status_mask;	/* driver specific 接收状态*/
	unsigned int		ignore_status_mask;	/* driver specific */
	struct uart_info	*info;			/* pointer to parent info */
	struct uart_icount	icount;			/* statistics 计数器*/

	struct console		*cons;			/* struct console, if any console结构体*/
#ifdef CONFIG_SERIAL_CORE_CONSOLE
	unsigned long		sysrq;			/* sysrq timeout */
#endif

	upf_t			flags;

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

	unsigned int		mctrl;			/* current modem ctrl settings */
	unsigned int		timeout;		/* character-based timeout */
	unsigned int		type;			/* port type 端口类型*/
	const struct uart_ops	*ops;		/*串口端口操作函数集*/
	unsigned int		custom_divisor;
	unsigned int		line;			/* port index 端口索引*/
	unsigned long		mapbase;		=AT91SAM9263_BASE_US0=0xFFF8C000	/*PDF的第20页 for ioremap IO物理地址的起始地址*/
	struct device		*dev;			/* parent device 父设备*/
	unsigned char		hub6;			/* this should be in the 8250 driver */
	unsigned char		unused[3];
};

/*内存中资源的信息*/
struct resource {
	resource_size_t start;		/*起始地址*/
	resource_size_t end;		/*结束地址*/
	const char *name;
	unsigned long flags;
	struct resource *parent, *sibling, *child;
};

/*平台总线的驱动结构体，即驱动的操作方法*/
static struct platform_driver atmel_serial_driver = {
	.probe			= atmel_serial_probe,				/*这个函数最重要*/
	.remove			= atmel_serial_remove,
	.suspend		= atmel_serial_suspend,
	.resume			= atmel_serial_resume,
	.driver			= {									/*driver 是 struct device_driver 结构体类型的变量*/
		.name		= "atmel_usart",
		.owner		= THIS_MODULE,
		.bus		= platform_bus_type					/*bus 是 struct bus_type 结构体类型的指针变量*/
		.probe		= platform_drv_probe 		=atmel_serial_probe
		.remove		= platform_drv_remove		=atmel_serial_remove
		.shutdown	= platform_drv_shutdown		=NULL
		.suspend	= platform_drv_suspend		=atmel_serial_suspend	=NULL
		.resume		= platform_drv_resume		=atmel_serial_resume	=NULL
	},
};

/*平台总线的设备结构体，即驱动操作设备所用到的数据,这个结构体是在arch\arm\mach-at91rm9200\at91sam9263_devices.c文件内赋值的*/
struct platform_device {
	const char	* name;				="atmel_usart"
	u32		id;						=0
	struct device	dev;
	u32		num_resources;
	struct resource	* resource;
};

static struct atmel_uart_data dbgu_data = {
	.use_dma_tx	= 0,
	.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
	.regs		= (void __iomem *)(AT91_VA_BASE_SYS + AT91_DBGU),	/*0xFEFF_EE00*/
};

/*Atmel串口使用的资源*/
static struct resource dbgu_resources[] = {
	[0] = {
		.start	= AT91_VA_BASE_SYS + AT91_DBGU,
		.end	= AT91_VA_BASE_SYS + AT91_DBGU + SZ_512 - 1,
		.flags	= IORESOURCE_MEM,		=0x00000200		/*代表内存资源*/
	},
	[1] = {
		.start	= AT91_ID_SYS,
		.end	= AT91_ID_SYS,
		.flags	= IORESOURCE_IRQ,		=0x00000400		/*代表中断资源*/
	},
};

static struct resource uart0_resources[] = {
	[0] = {
		.start	= AT91SAM9263_BASE_US0,		/*0xfff8c000 物理地址*/
		.end	= AT91SAM9263_BASE_US0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,			/*内存资源*/
	},
	[1] = {
		.start	= AT91SAM9263_ID_US0,		/*串口0的中断号是7*/
		.end	= AT91SAM9263_ID_US0,
		.flags	= IORESOURCE_IRQ,			/*资源类型为中断*/
	},
};

static struct atmel_uart_data uart0_data = {
	.use_dma_tx	= 1,		
	.use_dma_rx	= 1,
};

static struct platform_device at91sam9263_uart0_device = {
	.name		= "atmel_usart",
	.id		= 1,
	.dev		= {
				.platform_data	= &uart0_data,
				.coherent_dma_mask = 0xffffffff,
	},
	.resource	= uart0_resources,
	.num_resources	= ARRAY_SIZE(uart0_resources),
};

static inline void configure_usart0_pins(void)
{
	at91_set_A_periph(AT91_PIN_PA26, 1);		/* TXD0 上拉使能*/
	at91_set_A_periph(AT91_PIN_PA27, 0);		/* RXD0 上拉失能*/
	at91_set_A_periph(AT91_PIN_PA28, 0);		/* RTS0 上拉失能*/
	at91_set_A_periph(AT91_PIN_PA29, 0);		/* CTS0 上拉失能*/
}

static struct resource uart1_resources[] = {
	[0] = {
		.start	= AT91SAM9263_BASE_US1,			/*0xfff90000物理地址*/
		.end	= AT91SAM9263_BASE_US1 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AT91SAM9263_ID_US1,			/*串口1的中断号为8*/
		.end	= AT91SAM9263_ID_US1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct atmel_uart_data uart1_data = {
	.use_dma_tx	= 1,
	.use_dma_rx	= 1,
};

static struct platform_device at91sam9263_uart1_device = {
	.name		= "atmel_usart",
	.id		= 2,
	.dev		= {
				.platform_data	= &uart1_data,
				.coherent_dma_mask = 0xffffffff,
	},
	.resource	= uart1_resources,
	.num_resources	= ARRAY_SIZE(uart1_resources),
};

static inline void configure_usart1_pins(void)
{
	at91_set_A_periph(AT91_PIN_PD0, 1);		/* TXD1 上拉使能*/
	at91_set_A_periph(AT91_PIN_PD1, 0);		/* RXD1 上拉失能*/
}

static struct resource uart2_resources[] = {
	[0] = {
		.start	= AT91SAM9263_BASE_US2,		/*0xfff94000*/
		.end	= AT91SAM9263_BASE_US2 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,	
	},
	[1] = {
		.start	= AT91SAM9263_ID_US2,		/*串口2的中断号为9*/
		.end	= AT91SAM9263_ID_US2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct atmel_uart_data uart2_data = {
	.use_dma_tx	= 1,
	.use_dma_rx	= 1,
};

static struct platform_device at91sam9263_uart2_device = {
	.name		= "atmel_usart",
	.id		= 3,
	.dev		= {
		.platform_data	= &uart2_data,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource	= uart2_resources,
	.num_resources	= ARRAY_SIZE(uart2_resources),
};

static inline void configure_usart2_pins(void)
{
	at91_set_A_periph(AT91_PIN_PD2, 1);		/* TXD2 上拉使能*/
	at91_set_A_periph(AT91_PIN_PD3, 0);		/* RXD2 上拉失能*/
}


static struct atmel_uart_data dbgu_data = {
	.use_dma_tx	= 0,
	.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
	.regs		= (void __iomem *)(AT91_VA_BASE_SYS + AT91_DBGU),
};

/*这个结构体在at91sam9263_devices.c文件内的赋值如下*/
static struct platform_device at91sam9263_dbgu_device = {
	.name		= "atmel_usart",
	.id			= 0,
	.dev		= {
				.platform_data	= &dbgu_data,			/*platform_data 是 struct atmel_uart_data 结构体类型的指针变量*/
				.coherent_dma_mask = 0xffffffff,
	},
	.resource	= dbgu_resources,							/*使用的资源*/
	.num_resources	= ARRAY_SIZE(dbgu_resources),		=2	/*资源数组为2，即struct resource dbgu_resources[] 数组元素个数是2 */
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

static struct uart_ops atmel_pops = {
	.tx_empty	= atmel_tx_empty,
	.set_mctrl	= atmel_set_mctrl,
	.get_mctrl	= atmel_get_mctrl,
	.stop_tx	= atmel_stop_tx,
	.start_tx	= atmel_start_tx,
	.stop_rx	= atmel_stop_rx,
	.enable_ms	= atmel_enable_ms,
	.break_ctl	= atmel_break_ctl,
	.startup	= atmel_startup,			/*申请中断号，并注册中断处理函数*/
	.shutdown	= atmel_shutdown,
	.set_termios	= atmel_set_termios,	/*设置串口的一些参数:数据位、停止位、奇偶校验位、*/
	.type		= atmel_type,				/*确定串口的类型*/
	.release_port	= atmel_release_port,
	.request_port	= atmel_request_port,	/*申请串口的IO资源*/
	.config_port	= atmel_config_port,	/*配置串口的IO口*/
	.verify_port	= atmel_verify_port,
	.pm		= atmel_serial_pm,				/*串口时钟电源管理*/
};

static struct uart_driver atmel_uart = {
	.owner			= THIS_MODULE,
	.driver_name	= "atmel_serial",
	.dev_name		= ATMEL_DEVICENAME		="ttyAT",
	.major			= SERIAL_ATMEL_MAJOR	=204,
	.minor			= MINOR_START			=154,
	.nr				= ATMEL_MAX_UART		=4,
	.cons			= ATMEL_CONSOLE_DEVICE	=atmel_console,
	.state			= kmalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL)	drv->nr=atmel_uart.nr=4
	.tty_driver		= 指向 struct tty_driver 结构体
};

struct uart_driver {
	struct module		*owner;		/*一般为THIS_MODULE*/
	const char		*driver_name;	/*串口驱动名*/
	const char		*dev_name;		/*串口设备名*/
	int			 major;				/*主设备号*/
	int			 minor;				/*次设备号*/
	int			 nr;				/*该uart_driver支持的串口个数(最大)*/
	struct console		*cons;		/*对应控制台，为NULL即不支持控制台，否则不为NULL*/

	/*
	 * these are private; the low level driver should not
	 * touch these; they should be initialised to NULL
	 */
	struct uart_state	*state;
	struct tty_driver	*tty_driver;
};

struct uart_state {
	unsigned int		close_delay;			=500	/* msec */
	unsigned int		closing_wait;			=30000	/* msec */

#define USF_CLOSING_WAIT_INF	(0)
#define USF_CLOSING_WAIT_NONE	(~0U)

	int			count;
	int			pm_state;
	struct uart_info	*info;
	struct uart_port	*port;

	struct mutex		mutex;					=初始化了
};

static struct console atmel_console = {
	.name		= ATMEL_DEVICENAME		="ttyAT",
	.write		= atmel_console_write,
	.device		= uart_console_device,
	.setup		= atmel_console_setup,
	.flags		= CON_PRINTBUFFER		=1,
	.index		= -1,
	.data		= &atmel_uart,			指向 struct uart_driver 结构体变量	控制台的私有数据就是这个结构体 struct uart_driver 
};

/*alloc_tty_driver为下面的结构体动态分配内存空间*/
struct tty_driver {
	int	magic;								=0x5402				/* magic number for this structure */
	struct cdev cdev;	/*对应的字符设备*/
	struct module	*owner;					=THIS_MODULE		/*这个驱动的模块拥有者*/
	const char	*driver_name;				="atmel_serial"		/*驱动名*/
	const char	*name;						="ttyAT"			/*设备名，设备节点名*/
	int	name_base;	/* offset of printed name */
	int	major;								=204				/* major device number 主设备号*/
	int	minor_start;						=154				/* start of minor device number 开始次设备号，一般为0 */
	int	minor_num;	/* number of *possible* devices 设备数量*/
	int	num;								=4					/* number of devices allocated 被分配的设备数量*/
	short	type;							=0x0003/*			/* type of tty driver tty驱动的类型*/
	short	subtype;						=1					/* subtype of tty driver tty驱动的子类型*/
	struct ktermios init_termios; 			=tty_std_termios	/* Initial termios 初始线路设置*/
	int	flags;								=0xc				/* tty driver flags tty驱动标志*/
	int	refcount;	/* for loadable tty drivers 引用计数*/
	struct proc_dir_entry *proc_entry; /* /proc fs entry proc文件系统入口*/
	struct tty_driver *other; /* only used for the PTY driver 仅对PTY驱动有意义*/

	/*
	 * Pointer to the tty data structures 接口函数
	 */
	struct tty_struct **ttys;
	struct ktermios **termios;
	struct ktermios **termios_locked;
	void *driver_state;						=指向 struct uart_driver 变量/* only used for the PTY driver */
	
	/*	指向 struct tty_operations 的结构体变量 uart_open
	 * Interface routines from the upper tty layer to the tty
	 * driver.	Will be replaced with struct tty_operations.
	 */
	int  (*open)(struct tty_struct * tty, struct file * filp);		=uart_open
	void (*close)(struct tty_struct * tty, struct file * filp);		=uart_close
	int  (*write)(struct tty_struct * tty,
		      const unsigned char *buf, int count);					=uart_write
	void (*put_char)(struct tty_struct *tty, unsigned char ch);		=uart_put_char
	void (*flush_chars)(struct tty_struct *tty);					=uart_flush_chars
	int  (*write_room)(struct tty_struct *tty);						=uart_write_room
	int  (*chars_in_buffer)(struct tty_struct *tty);				=uart_chars_in_buffer
	int  (*ioctl)(struct tty_struct *tty, struct file * file,
		    unsigned int cmd, unsigned long arg);					=uart_ioctl
	void (*set_termios)(struct tty_struct *tty, struct ktermios * old);		=uart_set_termios
	void (*throttle)(struct tty_struct * tty);								=uart_throttle
	void (*unthrottle)(struct tty_struct * tty);							=uart_unthrottle
	void (*stop)(struct tty_struct *tty);									=uart_stop
	void (*start)(struct tty_struct *tty);									=uart_start
	void (*hangup)(struct tty_struct *tty);									=uart_hangup
	void (*break_ctl)(struct tty_struct *tty, int state);					=uart_break_ctl
	void (*flush_buffer)(struct tty_struct *tty);							=uart_flush_buffer
	void (*set_ldisc)(struct tty_struct *tty);								=
	void (*wait_until_sent)(struct tty_struct *tty, int timeout);			=uart_wait_until_sent
	void (*send_xchar)(struct tty_struct *tty, char ch);					=uart_send_xchar
	int (*read_proc)(char *page, char **start, off_t off,				
			  int count, int *eof, void *data);								=uart_read_proc
	int (*write_proc)(struct file *file, const char __user *buffer,
			  unsigned long count, void *data);								=
	int (*tiocmget)(struct tty_struct *tty, struct file *file);				=uart_tiocmget
	int (*tiocmset)(struct tty_struct *tty, struct file *file,		
			unsigned int set, unsigned int clear);							=uart_tiocmset

	struct list_head tty_drivers;
};

struct ktermios tty_std_termios = {	/* for the benefit of tty drivers  */
	.c_iflag = ICRNL | IXON								=9600,
	.c_oflag = OPOST | ONLCR							=9600,
	.c_cflag 											=B9600 | CS8 | CREAD | HUPCL | CLOCAL,
	.c_lflag = ISIG | ICANON | ECHO | ECHOE | ECHOK |
		   ECHOCTL | ECHOKE | IEXTEN,
	.c_cc = INIT_C_CC,
	.c_ispeed = 38400,
	.c_ospeed = 38400
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
};

/*在\arch\arm\mach-at91rm9200\at91sam9263.c文件里面给这个还给他的三个元素赋值了*/
static struct clk usart0_clk = {
	.name		= "usart0_clk",
	.pmc_mask	= 1 << AT91SAM9263_ID_US0,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk usart1_clk = {
	.name		= "usart1_clk",
	.pmc_mask	= 1 << AT91SAM9263_ID_US1,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk usart2_clk = {
	.name		= "usart2_clk",
	.pmc_mask	= 1 << AT91SAM9263_ID_US2,
	.type		= CLK_TYPE_PERIPHERAL,
};

/*at91_clock_associate函数给这个结构体的两个元素function和dev赋值了，*/
struct clk {
	struct list_head node;
	const char	*name;		/* unique clock name */
	const char	*function;						="usart"						/* function of the clock */
	struct device	*dev;						=&at91sam9263_uart0_device.dev	/* device associated with function */
	unsigned long	rate_hz;
	struct clk	*parent;
	u32		pmc_mask;
	void		(*mode)(struct clk *, int);
	unsigned	id:2;		/* PCK0..3, or 32k/main/a/b */
	unsigned	type;		/* clock type */
	u16		users;
};

/*这个结构体是在at91_init_serial函数中调用了，但是没有找到在哪儿赋值*/
struct at91_uart_config {
	unsigned short	console_tty;	/* tty number of serial console */
	unsigned short	nr_tty;		/* number of serial tty's */
	short		tty_map[];	/* map UART to tty number */
};

/*在\arch\arm\mach-at91rm9200\board-ns-9263e.c文件里面赋值了*/
static struct at91_uart_config __initdata ek_uart_config = {
	.console_tty	= 0,				/* ttyS0 */
	.nr_tty		= 4,
	.tty_map	= { 3, 0, 1, 2, }		/* ttyS0, ..., ttyS3 */
};


/*串口设备注册的重要函数*/
1、 ek_board_init	调用了很多函数来注册设备
	2、 掉了 at91_add_device_serial 注册平台总线设备 串口
		3、调用了 platform_device_register 函数来完成注册平台总线设备
			4、 调用了 device_initialize 和 platform_device_add 函数
				5、 platform_device_add 调用了 device_add 这个函数来完成平台总线设备的添加
			
2、 at91_init_serial 初始化了串口设备
	2、 调用了 configure_usart0_pins 和 at91_clock_associate 来配置串口的工作模式和工作时钟

3、 at91sam9263_register_clocks 函数里面注册了串口时钟
	2、 调用了 clk_register 来完成注册时钟的
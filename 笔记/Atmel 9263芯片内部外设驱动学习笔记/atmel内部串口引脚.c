第一个串口(232)									对应的GPIO口				在内存的地址
	RXD0(R)										PA27/RXD0/EBI1_D27
	TXD0(T)										PA26/TXD0/EBI1_D26
	RTS0										PA28/RTS0/EBI1_D28
	CTS0										PA29/CTS0/EBI1_D29
	
第二个串口(232)									对应的GPIO口
	DRXD(R)								 		PC30/DRXD					AT91_PIN_PC30=(PIN_BASE + 0x40 + 30)
	DTXD(T)								 		PC31/DTXD					AT91_PIN_PC31=(PIN_BASE + 0x40 + 31)
	
第三个串口(485 X)								对应的GPIO口
	RSX_RX(R)			RS485_RXD	RXD1		PD1/RXD1/SPI0_NPCS3 		
	RSX_TX(T)			TXD1					PD0/TXD1/SPI0_NPCS2
	RSX_CTL(控制引脚)	RS485_CTL1				PA23/MCI1_DB1/EBI1_D23
	
第四个串口(485 Y)								对应的GPIO口
	RSY_RX(R)			RS485_RXD2				PD3/RXD2/SPI1_NPCS3
	RSY_TX(T)			TXD2					PD2/TXD2/SPI1_NPCS2
	RSY_CTL(控制引脚)	RS485_CTL2				PA24/MCI1_DB2/EBI1_D24
	
	
内部只有三个串口(USART)
串口名				起始地址			终止地址			大小
USART0				0xFFF8_C000			0xFFF9_0000			16KB
USART1				0xFFF9_0000			0xFFF9_4000			16KB
USART2				0xFFF9_4000			0xFFF9_8000			16KB
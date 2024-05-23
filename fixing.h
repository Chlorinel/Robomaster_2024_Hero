
/* 93C46选默认的16位模式，但SPI总线上每次发送/接收8位数据 */
#include <stm32f10x.h>
 
#define _BV(n) (1 << (n))
 
uint8_t id = 0;
uint16_t num = 0;
const uint8_t seg8[] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90};
 
void delay(void)
{
	uint16_t i;
	for (i = 0; i < 20000; i++);
}
 
void ser_in(uint8_t data)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		GPIOB->BRR = GPIO_BRR_BR9; // SCLK=>PB9
		if (data & 0x80)
			GPIOB->BSRR = GPIO_BSRR_BS7; // DIO=>PB7
		else
			GPIOB->BRR = GPIO_BRR_BR7;
		data <<= 1;
		GPIOB->BSRR = GPIO_BSRR_BS9;
	}
}
 
void par_out(void)
{
	GPIOB->BRR = GPIO_BRR_BR8; // RCLK=>PB8
	GPIOB->BSRR = GPIO_BSRR_BS8;
}
 
void seg_scan(void)
{
	uint8_t i;
	uint32_t n = num;
	for (i = 0; i <= 4; i++)
	{
		ser_in(seg8[n % 10]);
		ser_in(_BV(i));
		par_out();
		delay();
		n /= 10;
	}
	
	n = id;
	for (i = 6; i <= 7; i++)
	{
		ser_in(seg8[n % 10]);
		ser_in(_BV(i));
		par_out();
		delay();
		n /= 10;
	}
}
 
uint16_t _93C46_Read(uint8_t addr)
{
	// SPI中我们配置的是CPOL=0, 即SCK的空闲状态为低电平; CPHA=0, 也就是在SCK的上升沿对数据进行采样
	// 这里会产生一个问题: 根据EEPROM手册的时序图Figure 2, 虽然发送数据没有问题, 但接收数据时,  SCK上升沿后需要等待tPD0或tPD1的时间后本位的数据才会出现在DO上
	// 如果上升沿出现时就抓取数据, 那么读到的不是本位的数据，而是上一位的数据
	// 因此，我们接收到的数据都是右移了一位之后的数据
	uint16_t data = 0;
	uint16_t temp;
	
	// 开始
	GPIOA->BRR = GPIO_BRR_BR3; // CS=0
	GPIOA->BSRR = GPIO_BSRR_BS3; // CS=1
	
	SPI1->CR1 |= SPI_CR1_SPE; // 启用SPI
	SPI1->DR = 0xc0 + ((addr >> 1) & 0x1f); // 发送操作码及地址码前5位 (1)
	while ((SPI1->SR & SPI_SR_TXE) == 0); // 注意: TXE=1并不代表当前字节发送完毕, 有可能只发送了一两个字节
	SPI1->DR = (addr << 7) & 0x80; // 送入下次要发送的内容: 地址码末位 (2)
	while ((SPI1->SR & SPI_SR_RXNE) == 0); // 等待接收数据
	temp = SPI1->DR; // 忽略这次读取的数据, 因为收到的数据恒为0xff
	// 数据的发送和接收是同时进行的
	// 只有当前字节发送完毕了, RXNE才置位, 而TXE早就置位了（参阅手册上的Figure 240）
	// RXNE置位表明(1)已发送完毕, 开始发送(2)
	
	while ((SPI1->SR & SPI_SR_TXE) == 0); // TXE置位后才能放入新数据, 此时(2)还未发送完毕
	SPI1->DR = 0x00; // 送入下次要发的内容: 根据器件手册上的时序图, 地址发送完毕后应发送0x00, 即DI一直为低电平，不是什么都不发 (3)
	while ((SPI1->SR & SPI_SR_RXNE) == 0); // 等待(2)发送完毕
	temp = SPI1->DR; // 收到的数据: 最高位为1(从器件发送的高阻态被视为1), 次高位为0(dummy bit, 空白位), 低6位为所读取数据的第15~10位
	data = (temp & 0x3f) << 10; // 去掉高两位后送入data变量
	
	while ((SPI1->SR & SPI_SR_TXE) == 0);
	SPI1->DR = 0x00; // 送入最后一次要发送的内容 (4)
	while ((SPI1->SR & SPI_SR_RXNE) == 0); // 等待(3)发送完毕
	temp = SPI1->DR; // 第9~2位数据
	data |= temp << 2;
	
	while ((SPI1->SR & SPI_SR_TXE) == 0);
	while ((SPI1->SR & SPI_SR_RXNE) == 0); // 等待(4)发送完毕
	temp = SPI1->DR;
	data |= temp >> 6;
	
	// 结束
	GPIOA->BRR = GPIO_BRR_BR3; // CS=0
	while (SPI1->SR & SPI_SR_BSY);
	SPI1->CR1 &= ~SPI_CR1_SPE; // 关闭SPI
	delay();
	return data;
}
 
int main(void)
{
	uint8_t i;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_SPI1EN;
	
	// CS(1)接PA3, SCK=PA5接SK(2), MISO=PA6接DO(4), MOSI=PA7接DI(3), ORG悬空选16位模式
	// 根据参考手册RM0008_166页的Table25，SCK、MOSI应配置为复用推挽输出(b)，而MISO应配置为带上拉输入(8)
	GPIOA->CRL = 0xb8b03000;
	GPIOA->BSRR = GPIO_BSRR_BS6; // 带上拉输入
	
	// 数码管动态扫描端口PB7~PB9
	GPIOB->CRH = 0x00000033;
	GPIOB->CRL = 0x30000000;
	
	SPI1->CR1 |= SPI_CR1_MSTR; // 设为主模式
	// SPI1->CR1 &= ~SPI_CR1_DFF; // 每次传送的数据位数为8位(DFF=0)
	SPI1->CR1 |= SPI_CR1_BR; // BR=111, 选256分频
	
	// SPI1->CR2 &= ~SPI_CR2_SSOE; // 不使用NSS(=PA4)端口。因为该端口的有效电平是低电平, 而93C46的有效片选信号为高电平
	SPI1->CR1 |= SPI_CR1_SSM; // 使用软件管理NSS端口，PA4可用作普通I/O口
	SPI1->CR1 |= SPI_CR1_SSI; // 设置NSS的状态: 已选中
	
	while (1)
	{
		num = _93C46_Read(id);
		for (i = 0; i < 50; i++)
			seg_scan();
		id++;
		if (id > 63)
			id = 0;
	}
}

/* 93C46选默认的16位模式，SPI每次也发送16位数据 */
#include <stm32f10x.h>
 
#define _BV(n) (1 << (n))
#define CS_0 (GPIOA->BRR = GPIO_BRR_BR3)
#define CS_1 (GPIOA->BSRR = GPIO_BSRR_BS3)
 
uint8_t id = 0;
uint16_t num = 0;
const uint8_t seg8[] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90};
 
void delay(void)
{
	uint16_t i;
	for (i = 0; i < 20000; i++);
}
 
void ser_in(uint8_t data)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		GPIOB->BRR = GPIO_BRR_BR9; // SCLK=>PB9
		if (data & 0x80)
			GPIOB->BSRR = GPIO_BSRR_BS7; // DIO=>PB7
		else
			GPIOB->BRR = GPIO_BRR_BR7;
		data <<= 1;
		GPIOB->BSRR = GPIO_BSRR_BS9;
	}
}
 
void par_out(void)
{
	GPIOB->BRR = GPIO_BRR_BR8; // RCLK=>PB8
	GPIOB->BSRR = GPIO_BSRR_BS8;
}
 
void seg_scan(void)
{
	uint8_t i;
	uint32_t n = num;
	for (i = 0; i <= 4; i++)
	{
		ser_in(seg8[n % 10]);
		ser_in(_BV(i));
		par_out();
		delay();
		n /= 10;
	}
	
	n = id;
	for (i = 6; i <= 7; i++)
	{
		ser_in(seg8[n % 10]);
		ser_in(_BV(i));
		par_out();
		delay();
		n /= 10;
	}
}
 
uint16_t _93C46_Read(uint8_t addr)
{
	// SPI中我们配置的是CPOL=0, 即SCK的空闲状态为低电平; CPHA=0, 也就是在SCK的上升沿对数据进行采样
	// 这里会产生一个问题: 根据EEPROM手册的时序图Figure 2, 虽然发送数据没有问题, 但接收数据时,  SCK上升沿后需要等待tPD0或tPD1的时间后本位的数据才会出现在DO上
	// 如果上升沿出现时就抓取数据, 那么读到的不是本位的数据，而是上一位的数据
	// 因此，我们接收到的数据都是右移了一位之后的数据
	uint16_t data = 0;
	uint16_t temp;
	
	// 开始
	CS_0;
	CS_1;
	
	SPI1->CR1 |= SPI_CR1_SPE; // 启用SPI
	SPI1->DR = 0xc000 | ((addr & 0x3f) << 7); // 发送操作码(110)、地址码 (1)
	while ((SPI1->SR & SPI_SR_TXE) == 0); // 注意: TXE=1并不代表当前字节发送完毕, 有可能只发送了一两个字节
	SPI1->DR = 0x0000; // 送入下次要发的内容: 根据器件手册上的时序图, 地址发送完毕后应发送0x0000, 即DI一直为低电平，不是什么都不发 (2)
	while ((SPI1->SR & SPI_SR_RXNE) == 0); // 等待接收数据
	temp = SPI1->DR; // 收到的数据: 第15~7位全为1(从器件发送的高阻态被视为1), 第6位为0(dummy bit, 空白位), 第5~0位为所读取数据的第15~10位
	// 数据的发送和接收是同时进行的
	// 只有当前字节发送完毕了, RXNE才置位, 而TXE早就置位了(参阅手册上的Figure 240)
	// RXNE置位表明(1)已发送完毕, 开始发送(2)
	data = (temp & 0x3f) << 10; // 去掉第15~6位后送入data变量
	
	while ((SPI1->SR & SPI_SR_TXE) == 0);
	while ((SPI1->SR & SPI_SR_RXNE) == 0); // 等待(2)发送完毕
	temp = SPI1->DR; // 第9~0位数据
	data |= temp >> 6;
	
	// 结束
	CS_0;
	while (SPI1->SR & SPI_SR_BSY);
	SPI1->CR1 &= ~SPI_CR1_SPE; // 关闭SPI
	return data;
}
 
int main(void)
{
	uint8_t i;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_SPI1EN;
	
	// CS(1)接PA3, SCK=PA5接SK(2), MISO=PA6接DO(4), MOSI=PA7接DI(3), ORG悬空选16位模式
	// 根据参考手册RM0008_166页的Table25，SCK、MOSI应配置为复用推挽输出(b)，而MISO应配置为带上拉输入(8)
	GPIOA->CRL = 0xb8b03000;
	GPIOA->BSRR = GPIO_BSRR_BS6; // 带上拉输入
	
	// 数码管动态扫描端口PB7~PB9
	GPIOB->CRH = 0x00000033;
	GPIOB->CRL = 0x30000000;
	
	SPI1->CR1 |= SPI_CR1_MSTR; // 设为主模式
	SPI1->CR1 |= SPI_CR1_DFF; // 每次传送的数据位数为16位(DFF=1)
	SPI1->CR1 |= SPI_CR1_BR; // BR=111, 选256分频
	
	// SPI1->CR2 &= ~SPI_CR2_SSOE; // 不使用NSS(=PA4)端口。因为该端口的有效电平是低电平, 而93C46的有效片选信号为高电平
	SPI1->CR1 |= SPI_CR1_SSM; // 使用软件管理NSS端口，PA4可用作普通I/O口
	SPI1->CR1 |= SPI_CR1_SSI; // 设置NSS的状态: 已选中
	
	while (1)
	{
		num = _93C46_Read(id);
		for (i = 0; i < 50; i++)
			seg_scan();
		id++;
		if (id > 63)
			id = 0;
	}
}



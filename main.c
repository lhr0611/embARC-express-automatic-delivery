#include "arc.h"
#include "arc_builtin.h"
#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "board.h"
#include "dev_uart.h"
#include "embARC.h"
#include "embARC_debug.h"

#include "..\..\drivers\mux\mux.h"
#include "arc_timer.h"
#include "arc_exception.h"
#include "dw_gpio.h"
#include "..\..\drivers\gpio\dw_gpio_obj.h"
#include "..\..\board\emsk\drivers\spi\dw_spi_obj.h"

#define forward 0
#define left_forward 1
#define right_forward 2
#define backward 3
#define left_backward 4
#define right_backward 5
#define EMSK_GPIO_CHECK_EXP_NORTN(EXPR)		CHECK_EXP_NOERCD(EXPR, error_exit)
#define SPI_TEST_CASE		1

#define SPI_MASTER_ID		1

#define SPI_SLAVE_FREQ		100000

DEV_SPI *spi_master;

#define SPI_SLV_CS		BOARD_WIFI_SPI_LINE
#define SPI_SLV_DFS		8

#define BUF_LEN	32
uint32_t tx_data[BUF_LEN];
uint32_t rx_data[BUF_LEN];

//j1 uart spi 0101
//j2 0000
//j3 0000
//j4 0000
//j5 0000
//j6 0000
//j7 00
//reserved 00000
static long int t0,t1,t2,t3,t4,t5,t6,d_time;
static int Duty_cycle0,Duty_cycle1;
static int PWM0,PWM1;
static DEV_GPIO *motor_port;

static void timer0_isr(void *ptr)
{
	timer_int_clear(TIMER_0);  
    if(t0>=100) 
      t0=0;        
    if(t0<=Duty_cycle0)
    {   
      PWM0 = 1;
    }    
    else
    {
	PWM0 = 0;
    }
    if(t0<=Duty_cycle1)
    {   
       PWM1 = 1;
    }    
    else
    {
	PWM1 = 0;
    }
   	 if(t6)
      {
      motor_write(((PWM0<<11)+(0<<10)+(PWM1<<9)+(0<<8)),EMSK_GPIO_C_VAILD_MASK);
      }
      else
      {
	      if(PWM0)
	      {
			if(PWM1)
			{
				motor_write(((0<<11)+(1<<10)+(0<<9)+(1<<8)),EMSK_GPIO_C_VAILD_MASK);
			}
			else
			{
				motor_write(((0<<11)+(1<<10)+(0<<9)+(0<<8)),EMSK_GPIO_C_VAILD_MASK);
			}
	      }
	      else
	      {
		      if(PWM1)
			{
				motor_write(((0<<11)+(0<<10)+(0<<9)+(1<<8)),EMSK_GPIO_C_VAILD_MASK);
			}
			else
			{
				motor_write(((0<<11)+(0<<10)+(0<<9)+(0<<8)),EMSK_GPIO_C_VAILD_MASK);
			}
	      }
      }
	t0++;
	if(t2>=t5) 
	{
          t2=0;  
	int_disable(INTNO_TIMER0);
	}   
	t2++;
	EMBARC_PRINTF("t2 = %d\n",t2);
}

static void timer1_isr(void *ptr)
{
	timer_int_clear(TIMER_1);
	if(t4>=t3) 
	{
          t4=0;  
	int_disable(INTNO_TIMER1);
	}   
	t4++;
	EMBARC_PRINTF("t4 = %d\n",t4);
}

void motor_init(void)
{
	motor_port = gpio_get_dev(0x02);

	EMSK_GPIO_CHECK_EXP_NORTN(motor_port != NULL);

	if (motor_port->gpio_open(DW_GPIO_MASK_ALL) == (-6)) {
		motor_port->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT, (void *)(DW_GPIO_MASK_ALL));
		motor_port->gpio_control(GPIO_CMD_DIS_BIT_INT, (void *)(DW_GPIO_MASK_ALL));
	}
	
	motor_write(0, DW_GPIO_MASK_ALL);

	EMBARC_PRINTF("motor_init_ok");
error_exit:
	return;
}

void motor_write(uint32_t motor_val, uint32_t mask)
{
	EMSK_GPIO_CHECK_EXP_NORTN(motor_port != NULL);

	motor_val = (~motor_val) & mask;
	motor_val = motor_val;
	mask = (mask) & DW_GPIO_MASK_ALL;
	motor_port->gpio_write(motor_val, mask);

error_exit:
	return;
}

void time_init(void)
{
	t0 = 0;
	t1 = 0;
	t2 = 0;
	t3 = 0;
	t4 = 0;
	t5 = 0;
	d_time = 0;
	Duty_cycle0 = 70;
	Duty_cycle1 = 70;
	int_disable(INTNO_TIMER0);
	int_disable(INTNO_TIMER1);
	timer_stop(TIMER_0);
	timer_stop(TIMER_1);
	int_handler_install(INTNO_TIMER0, timer0_isr);
	int_handler_install(INTNO_TIMER1, timer1_isr);
	int_pri_set(INTNO_TIMER0, INT_PRI_MIN);
	int_pri_set(INTNO_TIMER1, INT_PRI_MIN);
	int_enable(INTNO_TIMER0);
	int_enable(INTNO_TIMER1);
	timer_start(TIMER_0, TIMER_CTRL_IE | TIMER_CTRL_NH, 100);
	timer_start(TIMER_1, TIMER_CTRL_IE | TIMER_CTRL_NH, 100);
	int_disable(INTNO_TIMER0);
	int_disable(INTNO_TIMER1);
	EMBARC_PRINTF("time_init_ok");
//	motor_write(0xa00, DW_GPIO_MASK_ALL);
}

void slvdev_init(uint32_t freq)
{
	spi_master = spi_get_dev(SPI_MASTER_ID); /* Get SPI device, DW_SPI_0_ID/DW_SPI_1_ID */

	spi_master->spi_open(DEV_MASTER_MODE, freq); /* Open SPI device in master mode */

	spi_master->spi_control(SPI_CMD_SET_CLK_MODE, CONV2VOID(SPI_CLK_MODE_3)); /* Set SPI clock mode */
	spi_master->spi_control(SPI_CMD_SET_DFS, CONV2VOID(SPI_SLV_DFS)); /* Set SPI data frame size */
}

void dump_data(uint8_t *buf, uint8_t len)
{
	EMBARC_PRINTF("DUMP DATA:");
	for (int i = 0; i < len; i++) {
		EMBARC_PRINTF("%d\t", buf[i]);
	}
	EMBARC_PRINTF("\r\n");
}

DEV_SPI_TRANSFER xfer;

/**
 * \brief	SPI device transfer function
 * \param[in]	*tx_buf	transmit buffer pointer
 * \param[in]	*rx_buf	receive buffer pointer
 * \param[in]	len	buffer length
 */
void slvdev_xfer(uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len)
{
	DEV_SPI_XFER_SET_TXBUF(&xfer, tx_buf, 0, len); /* Set transmit buffer */
	DEV_SPI_XFER_SET_RXBUF(&xfer, rx_buf, 0, len); /* Set receive buffer */
	DEV_SPI_XFER_SET_NEXT(&xfer, NULL); /* Set next SPI transfer */
	spi_master->spi_control(SPI_CMD_MST_SEL_DEV, CONV2VOID(SPI_SLV_CS)); /* Set SPI slave device */
	spi_master->spi_control(SPI_CMD_TRANSFER_POLLING, &xfer); /* Start the transfer by polling */
	spi_master->spi_control(SPI_CMD_MST_DSEL_DEV, CONV2VOID(SPI_SLV_CS)); /* Deset SPI slave device */
}

void slvdev_write(uint8_t *buf, uint8_t len)
{
	spi_master->spi_control(SPI_CMD_MST_SEL_DEV, CONV2VOID(SPI_SLV_CS)); /* Select SPI slave device */
	spi_master->spi_write(buf, len); /* Write operation by polling */
	spi_master->spi_control(SPI_CMD_MST_DSEL_DEV, CONV2VOID(SPI_SLV_CS)); /* Deselect SPI slave device */
}

/**
 * \brief	SPI device read function
 * \param[in]	*buf	buffer pointer
 * \param[in]	len	buffer length
 */
void slvdev_read(uint8_t *buf, uint8_t len)
{
	spi_master->spi_control(SPI_CMD_MST_SEL_DEV, CONV2VOID(SPI_SLV_CS)); /* Select SPI slave device */
	spi_master->spi_read(buf, len); /* Read operation by polling */
	spi_master->spi_control(SPI_CMD_MST_DSEL_DEV, CONV2VOID(SPI_SLV_CS)); /* Deselect SPI slave device */
}



//#define forward 0
//#define left_forward 1
//#define right_forward 2
//#define backward 3
//#define left_backward 4
//#define right_backward 5


void main(void)
 {  
	int i;
	int temp0[10];
	int temp1[10];
	uint8_t *tx_buf = (uint8_t *)tx_data;
	uint8_t *rx_buf = (uint8_t *)rx_data;
	temp0 [0] = forward;
	temp0 [1] = left_forward;
	temp0 [2] = forward;
	temp0 [3] = right_forward;
	temp0 [4] = forward;
	temp0 [5] = backward;
	temp0 [6] = right_backward;
	temp0 [7] = backward;
	temp0 [8] = left_backward;
	temp0 [9] = backward;
	temp1 [0] = forward;
	temp1 [1] = right_forward;
	temp1 [2] = forward;
	temp1 [3] = left_forward;
	temp1 [4] = forward;
	temp1 [5] = backward;
	temp1 [6] = left_backward;
	temp1 [7] = backward;
	temp1 [8] = right_backward;
	temp1 [9] = backward;
	EMBARC_PRINTF("temp0=%p",temp0);
	EMBARC_PRINTF("temp1=%p",temp1);
	cpu_lock();
   	uint32_t temp;
	board_init();
	MUX_REG *mux_regs;
	mux_regs = (MUX_REG *)(MUX_REG *)(_arc_aux_read(0x20a)|0x00000000U);
	mux_init(mux_regs);
	set_pmod_mux(mux_regs,0x10000);
	temp=get_pmod_mux(mux_regs);
	EMBARC_PRINTF("pmod reg = %x",temp);
	motor_init();
	time_init();
	slvdev_init(SPI_SLAVE_FREQ);
	EMBARC_PRINTF("e_ok\n");
	cpu_unlock();
	while(1)
	{ 
		//0x00,0x00,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00
		tx_buf[0] = 0x00;
		tx_buf[1] = 0x17;
		tx_buf[2] = 0x01;
		tx_buf[3] = 0x14;
		tx_buf[4] = 0xd4;
		tx_buf[5] = 0xfd;
		tx_buf[6] = 0x03;
		tx_buf[7] = 0xff;
		tx_buf[8] = 0x00;
		tx_buf[9] = 0x00;
		slvdev_write(tx_buf, 10);
		delay(1000);
		//0x00,0x00,0xFF,0x04,0xFc,0xd4,0x4A,0x02,0x00,0xdf,0x00;
		tx_buf[0] = 0x00;
		tx_buf[1] = 0xdf;
		tx_buf[2] = 0x00;
		tx_buf[3] = 0x02;
		tx_buf[4] = 0x4a;
		tx_buf[5] = 0xd4;
		tx_buf[6] = 0xfc;
		tx_buf[7] = 0x04;
		tx_buf[8] = 0xff;
		tx_buf[9] = 0x00;
		tx_buf[10] = 0x00;
		slvdev_write(tx_buf, 11);
		delay(1000);
		slvdev_read(rx_buf, BUF_LEN);
		delay(1000);
		slvdev_read(rx_buf, BUF_LEN);
		//85206854 85206855
		if( rx_buf[7] == 8 && rx_buf[6] == 5 && rx_buf[5] == 2 && rx_buf[4] == 0 && rx_buf[3] == 6 && rx_buf[2] == 8 && rx_buf[1] == 5 && rx_buf[0] == 4 )
		{
			EMBARC_PRINTF("rf_ok\n");
			for(i = 0;i < 10;i++)
			{
				car_set(temp0[i]);
				delay(1000);
			}
		}
		if( rx_buf[7] == 8 && rx_buf[6] == 5 && rx_buf[5] == 2 && rx_buf[4] == 0 && rx_buf[3] == 6 && rx_buf[2] == 8 && rx_buf[1] == 5 && rx_buf[0] == 5 )
		{
			EMBARC_PRINTF("rf_ok\n");
			for(i = 0;i < 10;i++)
			{
				car_set(temp1[i]);
				delay(1000);
			}
		}
	
	}
}

void car_set(const unsigned int mode)
{
	switch(mode)
	{
	case forward : 	{			
						t6 = 1;
						t5 = 1000;
				    		Duty_cycle0 = 99; 
						Duty_cycle1 = 99; 
						EMBARC_PRINTF("forward\n");
						int_enable(INTNO_TIMER0);
						timer_start(TIMER_0, TIMER_CTRL_IE | TIMER_CTRL_NH, 100);
						motor_write(0x0,EMSK_GPIO_C_VAILD_MASK);
						EMBARC_PRINTF("forward finish\n");
						break;}
	case left_forward : {		 				
			    			t6 = 1;
				    		t5 = 1000;
				    		Duty_cycle0 = 30; 
						Duty_cycle1 = 99; 
						EMBARC_PRINTF("left_forward\n");
						int_enable(INTNO_TIMER0);
						timer_start(TIMER_0, TIMER_CTRL_IE | TIMER_CTRL_NH, 100);
						motor_write(0x0,EMSK_GPIO_C_VAILD_MASK);
						EMBARC_PRINTF("left_forward finish\n");
						break;}
	case right_forward :{			
			    			t6 = 1;	
				    		t5 = 1000;
				    		Duty_cycle0 = 99; 
						Duty_cycle1 = 30; 
						EMBARC_PRINTF("right_forward\n");
						int_enable(INTNO_TIMER0);
						timer_start(TIMER_0, TIMER_CTRL_IE | TIMER_CTRL_NH, 100);
						motor_write(0x0,EMSK_GPIO_C_VAILD_MASK);
						EMBARC_PRINTF("right_forward finish\n");
						break;}
	case backward :{			
		       		  	        t6 = 0;
			       			t5 = 1000;
				    		Duty_cycle0 = 99; 
						Duty_cycle1 = 99; 
						EMBARC_PRINTF("backward\n");
						int_enable(INTNO_TIMER0);
						timer_start(TIMER_0, TIMER_CTRL_IE | TIMER_CTRL_NH, 100);
						motor_write(0x0,EMSK_GPIO_C_VAILD_MASK);
						EMBARC_PRINTF("backward finish\n");
						break;}
	case left_backward : {			
			     			t6 = 0;
			     			t5 = 10000;
				    		Duty_cycle0 = 30; 
						Duty_cycle1 = 99; 
						EMBARC_PRINTF("left_backward\n");
						int_enable(INTNO_TIMER0);
						timer_start(TIMER_0, TIMER_CTRL_IE | TIMER_CTRL_NH, 100);
						motor_write(0x0,EMSK_GPIO_C_VAILD_MASK);
						EMBARC_PRINTF("left_backward finish\n");
						break;}
	case right_backward : {			
			      			t6 = 0;
				      		t5 = 10000;
				    		Duty_cycle0 = 99; 
						Duty_cycle1 = 30; 
						EMBARC_PRINTF("right_backward\n");
						int_enable(INTNO_TIMER0);
						timer_start(TIMER_0, TIMER_CTRL_IE | TIMER_CTRL_NH, 100);
						motor_write(0x0,EMSK_GPIO_C_VAILD_MASK);
						EMBARC_PRINTF("right_backward finish\n");
						break;}
	}
}

void delay(const unsigned long int time)
{
	t3 = time;
	int_enable(INTNO_TIMER1);
	timer_start(TIMER_1, TIMER_CTRL_IE | TIMER_CTRL_NH, 1000);
}

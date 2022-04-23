/*********************************************************************************
 * �ļ���       ��IO_IIC.h
 * ����         ��IIC
 * ����         ��CRP
 * ��ע         ��IO��ģ��IICͨ��
**********************************************************************************/
 
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
 
#include "IO_IIC.h"



void IO_IIC_GPIO_Init(void)
{		
//			GPIO_InitTypeDef GPIO_InitStruct;//F4 ��ʼ������
//			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//�����˿�ʱ��
// 
//			GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT; 	
//			GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;                                    
//			GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;     
//			GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;                                      
//			GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;   
//			GPIO_Init(GPIOB, &GPIO_InitStruct);

	
		My_GPIO_Init(GPIOB,GPIO_Pin_6,GPIO_Mode_Out_OD, GPIO_Speed_50MHz);//�Զ����ʼ������
		My_GPIO_Init(GPIOB,GPIO_Pin_7,GPIO_Mode_Out_OD, GPIO_Speed_50MHz);//�Զ����ʼ������
//		My_GPIO_Init(GPIOB,GPIO_Pin_10,GPIO_Mode_Out_OD, GPIO_Speed_50MHz);//�Զ����ʼ������
//		My_GPIO_Init(GPIOB,GPIO_Pin_11,GPIO_Mode_Out_OD, GPIO_Speed_50MHz);//�Զ����ʼ������
		IO_IIC_Delay();IO_IIC_Delay();
		IO_IIC_Delay();IO_IIC_Delay();
	
}

void SDA_DIR_OUT(void)//����ʹ�ú����ı�IO����
{		
//			GPIO_InitTypeDef GPIO_InitStruct;
//		//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//�����˿�ʱ��
// 
//			GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT; 	
//			GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;                                    
//			GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;     
//			GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;                                      
//			GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;   
//			GPIO_Init(GPIOB, &GPIO_InitStruct);
    
				My_GPIO_Init(GPIOB,GPIO_Pin_7,GPIO_Mode_Out_OD, GPIO_Speed_50MHz);//�Զ����ʼ������
			//	My_GPIO_Init(GPIOB,GPIO_Pin_11,GPIO_Mode_Out_OD, GPIO_Speed_50MHz);//�Զ����ʼ������
}
void SDA_DIR_IN(void)
{		
//			GPIO_InitTypeDef GPIO_InitStruct;
//		//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//�����˿�ʱ��
// 
//			GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN; 	
//			GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;                                    
//			GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;     
//			GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;                                      
//			GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;   
//			GPIO_Init(GPIOB, &GPIO_InitStruct);
				My_GPIO_Init(GPIOB,GPIO_Pin_7,GPIO_Mode_IPU, GPIO_Speed_50MHz);//�Զ����ʼ������
			//My_GPIO_Init(GPIOB,GPIO_Pin_7,GPIO_Mode_IPU, GPIO_Speed_50MHz);//�Զ����ʼ������
}

//----------------------------------------------------------------------------------------------------------//
 
void IO_IIC_Delay(void) //ͨ��Ƶ�� 375 hz
{
	 unsigned char tem_IIC=10;//Ĭ��39
	 
	 while(tem_IIC-- >0);
	  
}
 
void IO_IIC_start(void) //I2C start �ź�
{
        
	SDA_OUT_H;	 				//����������
	SCL_OUT_H;					//����ʱ����
	IO_IIC_Delay();			//��ʱ
	SDA_OUT_L; 					//�����½���
	IO_IIC_Delay();			//��ʱ
	SCL_OUT_L;          //����ʱ����
	IO_IIC_Delay();
 
}
 
void IO_IIC_stop(void) //I2C stop �ź�
{
        
	SDA_OUT_L;					     //����������
	SCL_OUT_H;						   //����ʱ����
	IO_IIC_Delay();				 	 //��ʱ
	SDA_OUT_H; 							 //����������
	IO_IIC_Delay(); 					 //��ʱ
}
 
void IO_IIC_ack(void)    //ack   I2C����Ӧ���ź�
{
		SDA_DIR_IN();          //��SDA�˿�����Ϊ����ģʽ
		//IO_IIC_Delay();IO_IIC_Delay();
		SCL_OUT_H;            //����ʱ����

		IO_IIC_Delay();        //��ʱ
		SCL_OUT_L;            //����ʱ����  
		SDA_DIR_OUT();           
		//IO_IIC_Delay();// IO_IIC_Delay();       //��ʱ

}
 
void IO_IIC_no_ack(void) //not ack 
{
		SDA_OUT_L;
		IO_IIC_Delay();
		SCL_OUT_H;

		IO_IIC_Delay();
		SCL_OUT_L;
		IO_IIC_Delay();   //�õ͵�ƽ����ʱ���Գ� Ŀ�����������߰�����׼����
	 
}
 
void IO_IIC_write_byte(unsigned char dat) //write a byte//��I2C���߷���һ���ֽ�����
{
	unsigned char i,temp;
	
	 
	 for(i = 0; i < 8; i++)
	 {
			 temp=dat & 0x80;
			if(temp==0x80)
				{
					SDA_OUT_H;
				}
			else
				{
					SDA_OUT_L;
				}
				IO_IIC_Delay();
				dat = dat << 1;		
				SCL_OUT_H;
				IO_IIC_Delay();
				SCL_OUT_L;
				 
    } 
	IO_IIC_ack();
}
 
unsigned char IO_IIC_read_byte(void) //read a byte//��I2C���߽���һ���ֽ�����
{
		unsigned char i,dat=0;
	
		SDA_DIR_IN();            //  ��SDA�˿�����Ϊ����ģʽ
		IO_IIC_Delay();IO_IIC_Delay();
		for(i = 0; i < 8; i++)
		{
	
			dat = dat << 1;
			IO_IIC_Delay();
			SCL_OUT_H;
			if(SDA_IN == 1)
			{
				dat++;
			}
			IO_IIC_Delay();
			SCL_OUT_L;
		 
		}
		SDA_DIR_OUT();   
		IO_IIC_Delay();//IO_IIC_Delay();
		return dat;
}
void IO_IIC_write_Command(unsigned char qijian_address,unsigned char reg_address,unsigned char command) //address+register+command
{
	IO_IIC_start();                      //��ʼ�ź�
	IO_IIC_write_byte(qijian_address);   //�����豸��ַ+д�ź�
	IO_IIC_write_byte(reg_address);      //�ڲ��Ĵ�����ַ��
	IO_IIC_write_byte(command);          //�ڲ��Ĵ������ݣ�
	IO_IIC_stop();                       //����ֹͣ�ź�
}  
unsigned char IO_IIC_read_Data(unsigned char qijian_address,unsigned char reg_address) //address(with bit 0 set) + register
{
	unsigned char REG_data=0;
	IO_IIC_start();                          //��ʼ�ź�
	IO_IIC_write_byte(qijian_address);       //�����豸��ַ+д�ź�
	IO_IIC_write_byte(reg_address);          //�ڲ��Ĵ�����ַ��
 // IO_IIC_ack();                          //����Ӧ���ź�   ע�����������Ӧ���ź���û��Ӧ���źŵĽ����ͬ 
	IO_IIC_stop(); 
  IO_IIC_start();                  					//��ʼ�ź�
	IO_IIC_write_byte(qijian_address+1);      //�����豸��ַ+���ź�
//IO_IIC_ack();                            //����Ӧ���ź�
	REG_data = IO_IIC_read_byte();           //�����Ĵ�������
	
	
/*IO_IIC_Delay();
  SCL = 1;		
	IO_IIC_Delay();
	SCL = 0;
	IO_IIC_Delay();  */
	
	IO_IIC_ack();                         //������Ӧ���ź�
	IO_IIC_stop();                            //ֹͣ�ź�
	return REG_data;
} 




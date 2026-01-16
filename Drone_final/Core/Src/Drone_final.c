

#include "NRF24/NRF24.h"
#include "i2c/i2c.h"
#include "MPU_6050/mpu6050.h"
#include "stm32f103xb.h"
#include "pid/pid.h"

/* ---------- Cấu trúc PID ---------- */



/* system tick */
uint32_t mtick = 0;                                     // Biến đếm mili‐giây hệ thống

void SysTick_Init(void)
{
    SysTick->LOAD = 8000 - 1;                            // Reload = 8000 → 1ms (HCLK = 8MHz)
    SysTick->VAL  = 0;                                  // Reset giá trị đếm
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |         // Clock = HCLK
                     SysTick_CTRL_ENABLE_Msk     |       // Enable SysTick
                     SysTick_CTRL_TICKINT_Msk;           // Enable ngắt SysTick
}

void SysTick_Handler(void)
{
    mtick++;                                             // Mỗi 1ms tăng biến đếm
}

void delay(uint32_t ms)
{
    uint32_t s = mtick;                                  // Lưu thời điểm bắt đầu
    while ((mtick - s) < ms);                            // Chờ đủ số ms
}



//============================PWM================================
void PWM_init ()
{
	// Bật clock GPIOA và TIM2
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                   // Enable GPIOA
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                   // Enable TIM2

    // PA0–PA3: Alternate Function Push-Pull, Output 2MHz
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |     // Clear PA0
                    GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |     // Clear PA1
                    GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |     // Clear PA2
                    GPIO_CRL_MODE3 | GPIO_CRL_CNF3);     // Clear PA3

    GPIOA->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1 |  // PA0 → TIM2_CH1
                   GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1 |  // PA1 → TIM2_CH2
                   GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1 |  // PA2 → TIM2_CH3
                   GPIO_CRL_MODE3_1 | GPIO_CRL_CNF3_1);  // PA3 → TIM2_CH4
    // MODEy_1 = 1 → Output 2MHz
    // CNFy_1  = 1 → Alternate Function Push-Pull

    // Timer2: PWM 50Hz = 20ms period
    // Tick = 1µs → PSC = 7 (8MHz / (7+1) = 1MHz)
    // ARR = 20000 - 1 → 20ms
	TIM2->PSC = 7;                                      // Prescaler
	TIM2->ARR = 20000 - 1;                              // Auto-reload

    // --- PWM mode 1 ---
    // CH1 (PA0)
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear OC1 mode
	TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);            // PWM mode 1
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;                     // Enable preload

    // CH2 (PA1)
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;                     // Clear OC2 mode
	TIM2->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);            // PWM mode 1
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;                     // Enable preload

    // CH3 (PA2)
	TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;                     // Clear OC3 mode
	TIM2->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos);            // PWM mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;                     // Enable preload

    // CH4 (PA3)
	TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;                     // Clear OC4 mode
	TIM2->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos);            // PWM mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;                     // Enable preload

    // Enable output cho 4 channel
	TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E |       // Enable CH1 & CH2
	               TIM_CCER_CC3E | TIM_CCER_CC4E);       // Enable CH3 & CH4

    // Enable preload & counter
    TIM2->CR1 |= TIM_CR1_ARPE;                          // Auto-reload preload
	TIM2->EGR |= TIM_EGR_UG;                             // Update event
	TIM2->CR1 |= TIM_CR1_CEN;                            // Start timer
}

void Duty_PWM (uint8_t channel, uint16_t duty)
{
	switch (channel)
	{
	    case 1: TIM2->CCR1 = duty; break;               // Motor 1 → CH1
	    case 2: TIM2->CCR2 = duty; break;               // Motor 2 → CH2
	    case 3: TIM2->CCR3 = duty; break;               // Motor 3 → CH3
	    case 4: TIM2->CCR4 = duty; break;               // Motor 4 → CH4
	}
}


float Handle_Data (uint8_t Data)
{
	float value = (float)Data;

	value -= 40.0f;
	return value;
}


/* ---------- main ---------- */
int main(void)
{
	uint8_t RxData[4];
 	uint8_t RxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};

 	float throttle;
 	float yaw;
 	float roll;
 	float pitch;

    SysTick_Init();
    PWM_init();
    i2c_init();
    MPU6050Init();
    PID_Init();
    NRF24_Init();         // spi_init() called inside
    NRF24_RxMode(RxAddress, 10); // power up and enable CE
    led_init();
    led_off();

    while (1)
    {

    	if(isDataAvailable(2))
    	{
    		NRF24_Receive(RxData,1);
    		if (RxData[0] == 100)
    		{
    		    PID_Reset(&pidTocDoPitch);
    		    PID_Reset(&pidTocDoRoll);
    		    PID_Reset(&pidGocRoll);
    		    PID_Reset(&pidGocPitch);
    		} else {
    			throttle = ((float)RxData[0]) * 10.0f;
    			yaw = Handle_Data(RxData[1]);
    			roll = Handle_Data(RxData[2]);
    			pitch = Handle_Data(RxData[3]);

    		}
    	}

    		if ((RxData[0] > 140))
    		{
    		    led_on();
    		} else {
    			led_off();
    		}
    		MPU6050_Read_G();
    		MPU_Read_A();
    		Filter(Ax, Ay, Az, Gx, Gy, Gz);

    	    vongDieuKhien(0.01, throttle, roll, pitch, yaw);
    	    Duty_PWM(1, m1);
    	    Duty_PWM(2, m2);
    		Duty_PWM(3, m3);
    		Duty_PWM(4, m4);
    }
    return 0;
}



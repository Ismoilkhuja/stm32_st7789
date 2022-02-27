#include "stm32f4xx.h"
#include "st7789.h"
#include "svc.h"
#include <stdlib.h>
#include <stdbool.h>

__IO uint32_t tick = 0;

typedef int64_t float_t;
#define FLOAT_BITS 20
#define FLOAT_FACT ((float_t)1 << FLOAT_BITS)
#define MANDELBROT_MAXITER 1024

typedef int float_fast_t;
#define FLOAT_FAST_BITS 13
#define FLOAT_FAST_FACT ((float_t)1 << FLOAT_FAST_BITS)
#define MANDELBROT_MAXITER_FAST 64


#define PIXEL_BUFFER_LINES 20
#define PIXEL_BUFFER_SIZE (ST7789_LCD_WIDTH * PIXEL_BUFFER_LINES)

#define IMAGE_BUFFER_LINES 1
#define IMAGE_BUFFER_SIZE (ST7789_LCD_WIDTH * IMAGE_BUFFER_LINES)
#define IMAGE_ADDR  ((uint32_t)0x08020000)

void setupTimer(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 0x3200 - 1;
    // TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->CR1 |= TIM_CR1_CEN;
}


void svcLogTime() {
    static char buf[12];
    itoa(TIM1->CNT / 10, buf, 10);
    char *bufPtr = buf;
    while (*bufPtr != 0) {
        bufPtr++;
    }
    *bufPtr = '\n';
    bufPtr++;
    *bufPtr = '\0';
    svcWrite0(buf);
}


void svcLogTimeReset() {
    TIM1->CNT = 0;
}


void svcWriteNumber(int number) {
    static char buf[12];
    itoa(number, buf, 10);
    char *bufPtr = buf;
    while (*bufPtr != 0) {
        bufPtr++;
    }
    *bufPtr = '\n';
    bufPtr++;
    *bufPtr = '\0';
    svcWrite0(buf);
}


void    st7789_GPIOInit(void) {
    // Enable GPIOA, SPI1 and DMA2 clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA2EN;
    // Enable DMA1 channel
//	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Enable SPI
    ST7789_SPI->SR = 0;
    // Reverse polarity?
    ST7789_SPI->CR1 = \
        SPI_CR1_SSM | \
        SPI_CR1_SSI | \
        SPI_CR1_MSTR | \
        SPI_CR1_CPOL | \
        SPI_CR1_BIDIMODE | \
        SPI_CR1_BIDIOE;
    ST7789_SPI->CR1 |= SPI_CR1_SPE;

    // DC and RST signals
    // Maximum output speed

    ST7789_DC_PORT->MODER |= GPIO_MODER_MODE1_0;
    ST7789_RST_PORT->MODER |= GPIO_MODER_MODE9_0;

    ST7789_DC_PORT->OSPEEDR |= GPIO_OSPEEDR_OSPEED1;
    ST7789_RST_PORT->OSPEEDR |= GPIO_OSPEEDR_OSPEED9;

//	ST7789_DC_PORT->CRH |= GPIO_CRH_MODE8;
//	ST7789_RST_PORT->CRH |= GPIO_CRH_MODE9;
    // Output push pull
//	ST7789_DC_PORT->CRH &= ~(GPIO_CRH_CNF8);
//	ST7789_RST_PORT->CRH &= ~(GPIO_CRH_CNF9);


    // SPI pins
    // Black Pill, SPI1: PB3 - MOSI, PB5 - SCK
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE5 | GPIO_MODER_MODE3,
               GPIO_MODER_MODE5_1 | GPIO_MODER_MODE3_1);
    // Max speed
    SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED3);

    // Alternate function PB3, PB5 - SPI1
    MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL3,
            0x5 << GPIO_AFRL_AFSEL5_Pos | 0x5 << GPIO_AFRL_AFSEL3_Pos);



    // Maximum output speed on PA5/PA7
//	GPIOA->CRL |= GPIO_CRL_MODE5;
//	GPIOA->CRL |= GPIO_CRL_MODE7;
    // Alternate mode on PA5/PA7
//	GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_CNF5)) | (GPIO_CRL_CNF5_1);
//	GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_CNF7)) | (GPIO_CRL_CNF7_1);
}


void demoCycleColors(void) {
    for (uint8_t color = 0; color < 248; color += 8) {
        st7789_Clear(st7789_RGBToColor(color, color, color));
    }
    for (uint8_t color = 248; color > 0; color -= 8) {
        st7789_Clear(st7789_RGBToColor(255, color, color));
    }
    for (uint8_t color = 0; color < 248; color += 8) {
        st7789_Clear(st7789_RGBToColor(255, color, 0));
    }
    for (uint8_t color = 248; color > 0; color -= 8) {
        st7789_Clear(st7789_RGBToColor(color, 255, 0));
    }
    for (uint8_t color = 0; color < 248; color += 8) {
        st7789_Clear(st7789_RGBToColor(0, 255, color));
    }
    for (uint8_t color = 248; color > 0; color -= 8) {
        st7789_Clear(st7789_RGBToColor(0, color, 255));
    }
    for (uint8_t color = 248; color > 0; color -= 8) {
        st7789_Clear(st7789_RGBToColor(0, 0, color));
    }
}


void demoCheckboardDisplay(uint16_t checkboardSize, uint16_t startX, uint16_t startY) {
    st7789_StartMemoryWrite();
    bool xPolarity = (startX / checkboardSize) & 1;
    bool yPolarity = (startY / checkboardSize) & 1;
    bool initialXPolarity = (startX / checkboardSize) & 1;
    startX = startX % checkboardSize;
    startY = startY % checkboardSize;
    uint16_t lineBuffer[ST7789_LCD_WIDTH * 2];
    uint16_t *buf = (uint16_t *)(&lineBuffer);
    uint16_t xCounter = startX;
    uint16_t yCounter = startY;

    for (uint16_t line = 0; line < ST7789_LCD_HEIGHT + 1; ++line) {
        uint16_t *frontBuffer = ((line & 1 )== 0) ? (buf + ST7789_LCD_WIDTH) : buf;
        uint16_t *backBuffer = ((line & 1) == 0) ? buf : (buf + ST7789_LCD_WIDTH);
        xPolarity = initialXPolarity;
        if (line > 0) {
            st7789_WriteDMA(frontBuffer, ST7789_LCD_WIDTH * 2);
        }
        if (line < ST7789_LCD_HEIGHT) {
            xCounter = startX;
            for (uint16_t column = 0; column < ST7789_LCD_WIDTH; ++column) {
                backBuffer[column] = (xPolarity ^ yPolarity) ? 0xffff : st7789_RGBToColor(line, column, 255 - line);
                xCounter++;
                if (xCounter == checkboardSize) {
                    xPolarity = !xPolarity;
                    xCounter = 0;
                }
            }
        }
        if (line > 0) {
            st7789_WaitForDMA();
        }
        yCounter++;
        if (yCounter == checkboardSize) {
            yPolarity = !yPolarity;
            yCounter = 0;
        }
    }
}


void demoCheckboard(void) {
    uint16_t size = 1;
    uint16_t posX = 0;
    uint16_t posY = 0;
    for (size_t i = 0; i < 50; ++i) {
        size++;
        demoCheckboardDisplay(size, posX, posY);
    }
    for (size_t i = 0; i < 50; ++i) {
        posX += 3;
        demoCheckboardDisplay(size, posX, posY);
    }
    for (size_t i = 0; i < 50; ++i) {
        posX += 3;
        posY += i / 10;
        demoCheckboardDisplay(size, posX, posY);
    }
    for (size_t i = 0; i < 100; ++i) {
        posX += 3 + i / 25;
        posY += 5 + i / 10;
        demoCheckboardDisplay(size, posX, posY);
    }
    for (size_t i = 0; i < 20; ++i) {
        posX += 7;
        posY += 15;
        demoCheckboardDisplay(size, posX, posY);
    }
    for (size_t i = 0; i < 100; ++i) {
        posX += 7 * (100 - i) / 100;
        posY += 15 * (100 - i) / 100;
        demoCheckboardDisplay(size, posX, posY);
    }
}


uint16_t demoMandelbrotCalculateFast(float_fast_t realInit, float_fast_t imagInit, uint16_t maxiter) {
    float_fast_t realq, imagq, real, imag;
    uint16_t iter;

    real = realInit;
    imag = imagInit;
    for (iter = 0; iter < maxiter; ++iter) {
        realq = (real * real) >> FLOAT_FAST_BITS;
        imagq = (imag * imag) >> FLOAT_FAST_BITS;
        if ((realq + imagq) > (float_fast_t) 4 * FLOAT_FAST_FACT) {
            break;
        }
        imag = ((real * imag) >> (FLOAT_FAST_BITS - 1)) + imagInit;
        real = realq - imagq + realInit;
    }
    return iter;
}


void demoMandelbrotDisplayFast(float_fast_t realmin, float_fast_t imagmin, float_fast_t realmax, float_fast_t imagmax, uint16_t maxiter) {
    st7789_StartMemoryWrite();
    uint16_t lineBuffer[ST7789_LCD_WIDTH * 2];
    uint16_t *buf = (uint16_t *)(&lineBuffer);
    uint16_t colormap[MANDELBROT_MAXITER_FAST];
    for (size_t i = 0; i < maxiter; ++i) {
        colormap[i] = st7789_RGBToColor(
            (maxiter - i - 1) * 256 / maxiter,
            i * 256 / maxiter,
            0
        );
    }

    float_fast_t stepReal, stepImag, real, imag;
    uint8_t column, line;

    stepReal = (realmax - realmin) / ST7789_LCD_WIDTH;
    stepImag = (imagmax - imagmin) / ST7789_LCD_HEIGHT;

    imag = imagmax;
    for (line = 0; line < ST7789_LCD_HEIGHT; ++line) {
        uint16_t *frontBuffer = ((line & 1 )== 0) ? (buf + ST7789_LCD_WIDTH) : buf;
        uint16_t *backBuffer = ((line & 1) == 0) ? buf : (buf + ST7789_LCD_WIDTH);
        if (line > 0) {
            st7789_WriteDMA(frontBuffer, ST7789_LCD_WIDTH * 2);
        }

        real = realmin;
        for (column = 0; column < ST7789_LCD_WIDTH; ++column) {
            uint16_t color = demoMandelbrotCalculateFast(real, imag, maxiter);
            if (color == maxiter) {
                color = 0;
            }
            else {
                color = colormap[color];
            }
            backBuffer[column] = color;
            real += stepReal;
        }
        imag -= stepImag;

        if (line > 0) {
            st7789_WaitForDMA();
        }
        if (line == ST7789_LCD_HEIGHT - 1) {
            st7789_WriteDMA(backBuffer, ST7789_LCD_WIDTH * 2);
            st7789_WaitForDMA();
        }
    }
}


uint16_t demoMandelbrotCalculate(float_t realInit, float_t imagInit) {
    float_t realq, imagq, real, imag;
    uint16_t iter;

    real = realInit;
    imag = imagInit;
    for (iter = 0; iter < MANDELBROT_MAXITER; ++iter) {
        realq = (real * real) >> FLOAT_BITS;
        imagq = (imag * imag) >> FLOAT_BITS;
        if ((realq + imagq) > (float_t) 4 * FLOAT_FACT) {
            break;
        }
        imag = ((real * imag) >> (FLOAT_BITS - 1)) + imagInit;
        real = realq - imagq + realInit;
    }
    return iter;
}


void demoMandelbrotDisplay(float_t realmin, float_t imagmin, float_t realmax, float_t imagmax) {
    st7789_StartMemoryWrite();
    uint16_t lineBuffer[ST7789_LCD_WIDTH * 2];
    uint16_t *buf = (uint16_t *)(&lineBuffer);
    uint16_t colormap[MANDELBROT_MAXITER];
    for (int i = 0; i < MANDELBROT_MAXITER; ++i) {
        colormap[i] = st7789_RGBToColor(
            i < 64 ? (MANDELBROT_MAXITER - i - 1) * 256 / MANDELBROT_MAXITER : 0,
            i < 64 ? (i * 16) * 256 / MANDELBROT_MAXITER : (i < 319 ? 319 - i : 0),
            (i * 4) * 256 / MANDELBROT_MAXITER
        );
    }

    float_t stepReal, stepImag, real, imag;
    uint8_t column, line;

    stepReal = (realmax - realmin) / ST7789_LCD_WIDTH;
    stepImag = (imagmax - imagmin) / ST7789_LCD_HEIGHT;

    imag = imagmax;
    for (line = 0; line < ST7789_LCD_HEIGHT; ++line) {
        uint16_t *frontBuffer = ((line & 1 )== 0) ? (buf + ST7789_LCD_WIDTH) : buf;
        uint16_t *backBuffer = ((line & 1) == 0) ? buf : (buf + ST7789_LCD_WIDTH);
        if (line > 0) {
            st7789_WriteDMA(frontBuffer, ST7789_LCD_WIDTH * 2);
        }

        real = realmin;
        for (column = 0; column < ST7789_LCD_WIDTH; ++column) {
            uint16_t color = demoMandelbrotCalculate(real, imag);
            if (color == MANDELBROT_MAXITER) {
                color = 0;
            }
            else {
                color = colormap[color];
            }
            backBuffer[column] = color;
            real += stepReal;
        }
        imag -= stepImag;

        if (line > 0) {
            st7789_WaitForDMA();
        }
        if (line == ST7789_LCD_HEIGHT - 1) {
            st7789_WriteDMA(backBuffer, ST7789_LCD_WIDTH * 2);
            st7789_WaitForDMA();
        }
    }
}


void demoMandelbrot() {
    float realMin = -2.0;
    float imagMin = -1.35;
    float realMax = 0.7;
    float imagMax = 1.35;
    const float realFinalMin = -0.749;
    const float imagFinalMin = 0.125;
    const float realFinalMax = -0.739;
    const float imagFinalMax = 0.135;

    for (size_t i = 8; i < 16; ++i) {
        demoMandelbrotDisplayFast(realMin * FLOAT_FAST_FACT, imagMin * FLOAT_FAST_FACT, realMax * FLOAT_FAST_FACT, imagMax * FLOAT_FAST_FACT, i);
        st7789_WaitNanosecs(20000);
    }
    for (size_t i = 0; i < 24; ++i) {
        realMin = (realMin * 9 + realFinalMin) / 10;
        realMax = (realMax * 9 + realFinalMax) / 10;
        imagMin = (imagMin * 9 + imagFinalMin) / 10;
        imagMax = (imagMax * 9 + imagFinalMax) / 10;
        demoMandelbrotDisplayFast(realMin * FLOAT_FAST_FACT, imagMin * FLOAT_FAST_FACT, realMax * FLOAT_FAST_FACT, imagMax * FLOAT_FAST_FACT, 16 + i);
    }

    demoMandelbrotDisplay(realFinalMin * FLOAT_FACT, imagFinalMin * FLOAT_FACT, realFinalMax * FLOAT_FACT, imagFinalMax * FLOAT_FACT);
    st7789_WaitNanosecs(4000000);
}


void demoPixmap() {
    uint16_t line = 0;
    uint16_t batch = 0;
    uint16_t pixels[PIXEL_BUFFER_SIZE];

    st7789_Clear(0x0000);
    st7789_StartMemoryWrite();
    while (line < ST7789_LCD_HEIGHT) {
        pixels[0] = batch;
        pixels[1] = PIXEL_BUFFER_SIZE;
        svcCall(0xff, &pixels);
        st7789_WriteDMA(&pixels, PIXEL_BUFFER_SIZE * 2);
        st7789_WaitForDMA();
        line = line + PIXEL_BUFFER_LINES;
        batch += 1;
    }
    st7789_WaitNanosecs(2000000);
}

void demoIcon(uint32_t image_addr){
    uint16_t line = 0;
    uint16_t batch = 0;
    uint16_t pixels[IMAGE_BUFFER_SIZE];
    uint32_t value=0;

    st7789_Clear(0x0000);
    st7789_StartMemoryWrite();
    while (line < ST7789_LCD_HEIGHT) {
//        pixels[0] = batch;
//        pixels[1] = PIXEL_BUFFER_SIZE;
//        svcCall(0xff, &pixels);

        for (uint32_t i=0; i<100; i+=2){
            value = (*(uint32_t *)(image_addr + 2*i + batch * 200));
            pixels[i] = (uint16_t)(value >> 16);
            pixels[i+1] = (uint16_t)value;
        }

        st7789_WriteDMA(&pixels, IMAGE_BUFFER_SIZE * 2);
        st7789_WaitForDMA();
        line = line + IMAGE_BUFFER_LINES;
        batch += 1;
    }
    st7789_WaitNanosecs(2000000);

}



void delay_ms(const uint32_t ms){
    uint32_t start = tick;
    while ((tick - start)<ms){}
}

void GPIO_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
    MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE13_1, GPIO_MODER_MODE13_0);
    CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT13);

}

void SetSysClkTo84(void){

    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS);
    while (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY)!= FLASH_ACR_LATENCY_2WS){}

    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM);
    SET_BIT(RCC->PLLCFGR, 25 << RCC_PLLCFGR_PLLM_Pos);

    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN);
    SET_BIT(RCC->PLLCFGR, 168 << RCC_PLLCFGR_PLLN_Pos);

    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);

    CLEAR_BIT(RCC->CFGR, RCC_CFGR_HPRE_3);    // AHB prescaler - not divided

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1, RCC_CFGR_PPRE1_2);   // APB1 prescaler - divided by 2

    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PPRE2_2);    // APB2 prescaler - not divided

    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY)){}

    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY)){}    

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_0, RCC_CFGR_SW_1);  // System clock - PLL

    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){}

}

//void WriteToFlash(){
//    flash
//}

int main(void){
    SetSysClkTo84();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);
    GPIO_Init();

    setupTimer();

    st7789_GPIOInit();
    st7789_Reset();
    st7789_Init_1_3_LCD();

    while (1) {
        SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
        delay_ms(500);
        SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
        delay_ms(300);

        demoIcon(IMAGE_ADDR);
        demoCycleColors();
        demoCheckboard();
        demoMandelbrot();
    }

    return 0;
}

void SysTick_Handler(void){
    tick++;
}

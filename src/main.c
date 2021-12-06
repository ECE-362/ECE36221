

/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include "lcd.h"
#include "midi.h"
#include "step-array.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>   // for MA_PI

MIDI_Player *mp;
extern uint8_t midifile[];

//#include "maple-leaf-rag.c"

#define LEFT -1
#define RIGHT 1
#define UP -1
#define DOWN 1

#define WIDTH (320/2)
#define HEIGHT (240/2)

#define P1COLOR CYAN
#define P2COLOR 0xFBE1

typedef struct {
    u16 x[3];
    u16 y[3];
    int lr;
    int ud;
    u16 color[2];
} Player;

Player p1;
Player p2;

int grid[HEIGHT][WIDTH/32];

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

void micro_wait(unsigned int n) {
    nano_wait(n*1000);
}

void mille_wait(unsigned int n) {
    nano_wait(n*1000000);
}

void wait(unsigned int n) {
    for(int i = 0; i < n; i++) {
        nano_wait(1000000000);
    }
}

//===========================================================================
// LCD Setup
//===========================================================================
void init_lcd_spi_gpio() {
    // Configure GPIOB for SPI with LCD
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    // Configure pins 8, 11, and 14 as output
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);
    GPIOB->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0;
    // Configure pins 3, 4, and 5 as SCK, MISO, and MOSI
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5);
}

void init_lcd_spi() {
    init_lcd_spi_gpio();

    // Configure SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |= SPI_CR1_BR_0;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 |= SPI_CR2_DS;
    SPI1->CR2 &= ~(SPI_CR2_DS_3);
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR2 |= SPI_CR2_FRXTH;
    SPI1->CR1 |= SPI_CR1_SPE;
}

//===========================================================================
// Music Timer
//===========================================================================

void init_TIM2(int tick){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR2 |= TIM_CR2_MMS_1;
    TIM2->PSC = 48-1;
    TIM2->ARR = tick-1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void init_TIM6(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->CR2 |= TIM_CR2_MMS_1;
    TIM6->PSC = 10-1;
    TIM6->ARR = 160-1;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC_SetPriority(TIM6_DAC_IRQn, 0);
    NVIC->ISER[0] |= 1 << TIM6_DAC_IRQn;
}

//===========================================================================
// Handler Setup
//===========================================================================
struct {
    uint8_t note;
    uint8_t chan;
    uint8_t volume;
    int     step;
    int     offset;
} voice[3];

#define N 1000
#define RATE 30000
short int wavetable[N];
//int step = 0;
//int offset = 0;

void TIM6_DAC_IRQHandler(void)
{
    // TODO: Remember to acknowledge the interrupt right here!
    TIM6->SR &= ~TIM_SR_UIF;

    int sample = 0;

        for(int i=0; i < sizeof voice / sizeof voice[0]; i++) {
         //   if(voice[i].volume != 0){
            sample += (wavetable[voice[i].offset>>16] * voice[i].volume) /*<< 4*/;
            voice[i].offset += voice[i].step;
            if ((voice[i].offset >> 16) >= sizeof wavetable / sizeof wavetable[0])
                voice[i].offset -= (sizeof wavetable / sizeof wavetable[0]) << 16;
    //  }
        }
        sample = (sample >> 16) + 2048;
        DAC->DHR12R1 = sample;
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}

void TIM2_IRQHandler(void)
{
    TIM2->SR &= ~TIM_SR_UIF;

    midi_play();
}

//===========================================================================
// Tempo
//===========================================================================
void set_tempo(int time, int value, const MIDI_Header *hdr)
{
    TIM2->ARR = value/hdr->divisions - 1;
}

void init_DAC(){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 3<<(2*4);
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR &= ~DAC_CR_EN1;
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_TSEL1;
    DAC->CR |= DAC_CR_EN1;
}

void note_on(int time, int chan, int key, int velo)
    {
    for(int i=0; i < sizeof voice / sizeof voice[0]; i++)
        if (voice[i].step == 0) {
        // configure this voice to have the right step and volume
            voice[i].step = step[key];
            voice[i].note = key;
            voice[i].volume = velo;
        break;
        }
    }

void note_off(int time, int chan, int key, int velo)
{
  for(int i=0; i < sizeof voice / sizeof voice[0]; i++)
    if (voice[i].step != 0 && voice[i].note == key) {
    // turn off this voice
    voice[i].step = 0;
    voice[i].note = 0;
    voice[i].volume = 0;
    break;
    }
}

//===========================================================================
// Game Timer
//===========================================================================
void init_tim7() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 4800 - 1;
    TIM7->ARR = 1000 - 1;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] |= 1 << TIM7_IRQn;
}

void stopTimer() {
    TIM7->CR1 &= ~TIM_CR1_CEN;
}

void startTimer() {
    TIM7->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// Player Button Setup
//===========================================================================
void init_buttons() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    GPIOA->MODER &= ~0xff00ff;   //Clear port
    GPIOA->PUPDR &= ~0xff00ff;   //Set PA0, PA1, PA2, PA3 and PA8, PA9, PA10, PA11 as inputs
    GPIOA->PUPDR |= 0xaa00aa;    //Set internal pull down resistor

    SYSCFG->EXTICR[1] = 0;       //Select PA0, PA1, PA2, PA3 for interrupt
    SYSCFG->EXTICR[2] = 0;       //Select PA8, PA9, PA10, PA11 for interrrupt
    EXTI->IMR |= 0xf0f;          //Un-mask each interrupt
    EXTI->RTSR |= 0xf0f;         //Select rising edge trigger i.e. on press

    NVIC->ISER[0] = (1 << EXTI0_1_IRQn) | (1 << EXTI2_3_IRQn) | (1 << EXTI4_15_IRQn);
}

//===========================================================================
// Game Engine
//===========================================================================
bool gameRunning = false;
bool gameReady = false;
bool gameEnd = false;
bool p1Collision;
bool p2Collision;
bool gameStart = false;

void init_grid() {
    for(int i = 0; i<WIDTH; i++) {
        for(int j = 0; j<HEIGHT; j++) {
            grid[i][j] = 0;
        }
    }
}

void init_players() {
    for(int i = 0; i<2; i++) {
        p1.x[i] = 10;
        p1.y[i] = 60;
        p2.x[i] = 160-10;
        p2.y[i] = 60;
    }

    p1.lr = RIGHT;
    p1.ud = 0;
    p1.color[0] = BLUE;
    p1.color[1] = P1COLOR;

    p2.lr = LEFT;
    p2.ud = 0;
    p2.color[0] = RED;
    p2.color[1] = P2COLOR;
}

void updatePosition(Player *p) {
    p->x[2] = p->x[1];
    p->x[1] = p->x[0];
    p->x[0] += p->lr;
    p->y[2] = p->y[1];
    p->y[1] = p->y[0];
    p->y[0] += p->ud;
}

void updateDirection(Player *p, int lr, int ud) {
    if(p->lr == 0 && lr != 0) {
        p->lr = lr;
        p->ud = 0;
    } else if(p->ud == 0 && ud != 0) {
        p->ud = ud;
        p->lr = 0;
    }
}

bool detectCollision(Player *p) {
    if((p->x[0] >= WIDTH) || (p->y[0] >= HEIGHT)) {
        return true;
    } else {
        return (1<<(p->x[0]%32)) == (grid[p->y[0]][p->x[0]/32] & 1<<(p->x[0]%32));
    }
}

void fillGrid(u16 x, u16 y, u16 color) {
    LCD_DrawFillRectangle(2*x, 2*y, 2*x+1, 2*y+1, color);
}

void movePlayer(Player *p) {
    fillGrid(p->x[2], p->y[2], p->color[1]);
    fillGrid(p->x[0], p->y[0], p->color[0]);
    grid[p->y[0]][p->x[0]/32] |= 1<<(p->x[0]%32);
}

void drawTitlescreen() {
    LCD_DrawRectangle(60, 80, 45, 70, CYAN);
    //LCD_DrawRectangle(65, 70, 75, 100, CYAN);
    LCD_DrawLine(65, 80, 65, 120, CYAN);
    LCD_DrawLine(75, 80, 75, 120, CYAN);
    LCD_DrawLine(65, 120, 75, 120, CYAN);
    LCD_DrawLine(75, 80, 135, 80, CYAN);
    LCD_DrawLine(75, 70, 125, 70, CYAN);
    LCD_DrawLine(65, 80, 75, 70, CYAN);
    LCD_DrawLine(135, 80, 125, 70, CYAN);
    LCD_DrawRectangle(90, 85, 100, 120, CYAN);
    LCD_DrawLine(105, 85, 105, 95, CYAN);
    LCD_DrawLine(105, 95, 125, 120, CYAN);
    LCD_DrawLine(125, 120, 135, 120, CYAN);
    LCD_DrawLine(135, 120, 115, 95, CYAN);
    LCD_DrawLine(115, 95, 125, 95, CYAN);
    LCD_DrawLine(125, 95, 135, 85, CYAN);
    LCD_DrawLine(135, 85, 105, 85, CYAN); //TR done
    LCD_Circle(163, 95, 25, 0, CYAN);
    LCD_Circle(163, 95, 15, 0, CYAN);     //O done
    LCD_DrawLine(195, 120, 195, 70, CYAN);
    LCD_DrawLine(195, 70, 200, 70, CYAN);
    LCD_DrawLine(200, 70, 220, 90, CYAN);
    LCD_DrawLine(220, 90, 220, 95, CYAN);
    LCD_DrawLine(220, 95, 205, 95, CYAN);
    LCD_DrawLine(205, 95, 205, 120, CYAN);
    LCD_DrawLine(205, 120, 195, 120, CYAN); //half of n
    LCD_DrawLine(223, 90, 223, 95, CYAN);
    LCD_DrawLine(243, 120, 223, 95, CYAN);
    LCD_DrawLine(243, 120, 248, 120, CYAN);
    LCD_DrawLine(248, 120, 248, 70, CYAN);
    LCD_DrawLine(248, 70, 238, 70, CYAN);
    LCD_DrawLine(238, 70, 238, 90, CYAN);
    LCD_DrawLine(238, 90, 223, 90, CYAN);
    LCD_DrawString(60, 150, WHITE, BLACK, "PRESS ANY BUTTON TO START", 16, 0);
}

void init_gameState() {
    gameRunning = false;
    gameReady = false;
    gameEnd = false;
    p1Collision = false;
    p2Collision = false;
}

void titleScreen() {
    stopTimer();
    init_gameState();
    init_buttons();
    init_grid();
    init_players();
    init_tim7();
    LCD_Clear(BLACK);
    drawTitlescreen();
    gameReady = true;
}

void startGame() {
    LCD_Clear(BLACK);
    startTimer();
    gameRunning = true;


}

void victoryScreen() {
    stopTimer();
    gameRunning = false;
    gameReady = false;
    if (p1Collision && p2Collision) {
        LCD_DrawString(160-4*4, 120-8, WHITE, BLACK, "TIE!", 16, 0);
    } else if (p1Collision) {
        LCD_DrawString(160-14*4, 120-8, P2COLOR, BLACK, "Player 2 Wins!", 16, 0);
    } else if (p2Collision) {
        LCD_DrawString(160-14*4, 120-8, P1COLOR, BLACK, "Player 1 Wins!", 16, 0);
    }
 //   RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
 //   RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
 //   RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
    wait(3);
    titleScreen();
}

//===========================================================================
// Timer Handler
//===========================================================================
void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    updatePosition(&p1);
    updatePosition(&p2);
    p1Collision = detectCollision(&p1);
    p2Collision = detectCollision(&p2);
    if(p1Collision || p2Collision) {
        gameEnd = true;
        return;
    }
    movePlayer(&p1);
    movePlayer(&p2);
}

//===========================================================================
// Button Handlers
//===========================================================================
void EXTI0_1_IRQHandler(void){
    // Acknowledge Interrupts
    EXTI->PR |= 0x1;
    EXTI->PR |= 0x2;

    if(!gameReady) {
        return;
    }

    if(!gameRunning) {
     //   startGame();
        gameStart = 1;
    } else if(GPIOA->IDR & 0x1){            //PA0 Pressed
        updateDirection(&p1, 0, UP);
    } else if(GPIOA->IDR & 0x2){      //PA1 Pressed
        updateDirection(&p1, RIGHT, 0);
    }
}

void EXTI2_3_IRQHandler(void){
    // Acknowledge Interrupts
    EXTI->PR |= 0x4;
    EXTI->PR |= 0x8;

    if(!gameReady) {
        return;
    }

    if(!gameRunning) {
       // startGame();
        gameStart = 1;
    } else if(GPIOA->IDR & 0x4){            //PA2 Pressed
        updateDirection(&p1, LEFT, 0);
    } else if(GPIOA->IDR & 0x8){      //PA3 Pressed
        updateDirection(&p1, 0, DOWN);
    }
}

void EXTI4_15_IRQHandler(void){
    // Acknowledge Interrupts
    EXTI->PR |= 0x100;
    EXTI->PR |= 0x200;
    EXTI->PR |= 0x400;
    EXTI->PR |= 0x800;

    if(!gameReady) {
        return;
    }

    if(!gameRunning) {
        //startGame();
        gameStart = 1;
    } else if(GPIOA->IDR & 0x100){     //PA8 Pressed
        updateDirection(&p2, 0, UP);
    } else if(GPIOA->IDR & 0x200){     //PA9 Pressed
        updateDirection(&p2, LEFT, 0);
    } else if(GPIOA->IDR & 0x400){     //PA10 Pressed
        updateDirection(&p2, 0, DOWN);
    } else if(GPIOA->IDR & 0x800){     //PA11 Pressed
        updateDirection(&p2, RIGHT, 0);
    }
}

//===========================================================================
// Main
//===========================================================================

int main() {
  //  for(int i=0; i < N; i++){
   //   wavetable[i] = i * 65535.0 / N - 32768;
    for(int i=0; i < N; i++){
            wavetable[i] = 32767 * sin(2 * M_PI * i / N);
    }

    init_TIM6();
        init_TIM2(10417);
        init_DAC();
    MIDI_Player *mp = midi_init(midifile);
    LCD_Setup();
    LCD_Clear(BLACK);
    titleScreen();

    for(;;) {

//        // If there is a collision, display the victory screen
        if (gameEnd) {
            victoryScreen();
        }
        if (mp->nexttick >= MAXTICKS){
            midi_init(midifile);
        }
        if(gameStart){
                startGame();
                gameStart = 0;
                }
        asm("wfi");
    }
}






#include <stdint.h>

#define RCC_AHB1ENR    (*(volatile uint32_t*)0x40023830)
#define RCC_APB1ENR    (*(volatile uint32_t*)0x40023840)

#define GPIOA_MODER    (*(volatile uint32_t*)0x40020000)
#define GPIOA_PUPDR    (*(volatile uint32_t*)0x4002000C)
#define GPIOA_IDR      (*(volatile uint32_t*)0x40020010)
#define GPIOA_ODR      (*(volatile uint32_t*)0x40020014)
#define GPIOA_AFRL     (*(volatile uint32_t*)0x40020020)

#define GPIOB_MODER    (*(volatile uint32_t*)0x40020400)
#define GPIOB_ODR      (*(volatile uint32_t*)0x40020414)

#define GPIOC_MODER    (*(volatile uint32_t*)0x40020800)
#define GPIOC_ODR      (*(volatile uint32_t*)0x40020814)

#define USART2_BASE    0x40004400
#define USART2_SR      (*(volatile uint32_t*)(USART2_BASE + 0x00))
#define USART2_DR      (*(volatile uint32_t*)(USART2_BASE + 0x04))
#define USART2_BRR     (*(volatile uint32_t*)(USART2_BASE + 0x08))
#define USART2_CR1     (*(volatile uint32_t*)(USART2_BASE + 0x0C))

typedef enum {
    STATE_OFF = 0,
    STATE_PRECHARGE,
    STATE_READY,
    STATE_FAULT
} vehicle_state_t;

vehicle_state_t state = STATE_OFF;

static void delay(volatile uint32_t t) {
    while(t--);
}

static int read_button_PA5() { return (GPIOA_IDR & (1 << 5)) != 0; } // ignition
static int read_button_PA6() { return (GPIOA_IDR & (1 << 6)) != 0; } // brake
static int read_button_PA7() { return (GPIOA_IDR & (1 << 7)) != 0; } // ready

void uart2_write(char c)
{
    while (!(USART2_SR & (1 << 7))); // TXE
    USART2_DR = c;
}

void uart2_print(char *s)
{
    while (*s)
        uart2_write(*s++);
}

static void uart2_init(void)
{
    RCC_AHB1ENR |= (1 << 0);     // GPIOA
    RCC_APB1ENR |= (1 << 17);    // USART2

    GPIOA_MODER &= ~(3 << (2 * 2));
    GPIOA_MODER |=  (2 << (2 * 2));      // AF mode

    GPIOA_AFRL &= ~(0xF << (2 * 4));
    GPIOA_AFRL |=  (7 << (2 * 4));       // AF7

    USART2_BRR = 0x0683;                 // 115200 baud
    USART2_CR1 |= (1 << 3);              // TE
    USART2_CR1 |= (1 << 13);             // UE
}

static void leds_off() {
    GPIOA_ODR &= ~(1 << 9);  // RED
    GPIOC_ODR &= ~(1 << 7);  // YELLOW
    GPIOB_ODR &= ~(1 << 6);  // GREEN
}

static void update_leds() {
    leds_off();

    switch(state) {
        case STATE_OFF:
            break;

        case STATE_PRECHARGE:
            GPIOC_ODR |= (1 << 7);  // YELLOW
            break;

        case STATE_READY:
            GPIOB_ODR |= (1 << 6);  // GREEN
            break;

        case STATE_FAULT:
            GPIOA_ODR ^= (1 << 9);  // RED blink
            delay(300000);
            break;
    }
}

int main(void) {

    /* Enable GPIO clocks */
    RCC_AHB1ENR |= (1 << 0); // GPIOA
    RCC_AHB1ENR |= (1 << 1); // GPIOB
    RCC_AHB1ENR |= (1 << 2); // GPIOC

    uart2_init();
    uart2_print("\r\n=== VEHICLE STARTUP CONTROLLER ===\r\n");
    uart2_print("Buttons:\r\n");
    uart2_print("  PA5 = Ignition\r\n");
    uart2_print("  PA6 = Brake\r\n");
    uart2_print("  PA7 = Ready\r\n\r\n");

    uart2_print("Startup Instructions:\r\n");
    uart2_print("  1. Press PA5 (Ignition) to enter PRECHARGE\r\n");
    uart2_print("  2. Hold PA6 (Brake)\r\n");
    uart2_print("  3. Press PA7 (Ready) to enter READY\r\n");
    uart2_print("  4. Green LED stays ON even after releasing brake\r\n");
    uart2_print("  5. If FAULT occurs, press PA5 to reset\r\n\r\n");

    /* LED OUTPUTS */
    GPIOA_MODER &= ~(3 << (9*2));
    GPIOA_MODER |=  (1 << (9*2));   // PA9 RED

    GPIOC_MODER &= ~(3 << (7*2));
    GPIOC_MODER |=  (1 << (7*2));   // PC7 YELLOW

    GPIOB_MODER &= ~(3 << (6*2));
    GPIOB_MODER |=  (1 << (6*2));   // PB6 GREEN

    /* BUTTON INPUTS + PULL-DOWNS */
    GPIOA_MODER &= ~(3 << (5*2));   // PA5 ignition
    GPIOA_MODER &= ~(3 << (6*2));   // PA6 brake
    GPIOA_MODER &= ~(3 << (7*2));   // PA7 ready

    GPIOA_PUPDR &= ~(3 << (5*2));
    GPIOA_PUPDR &= ~(3 << (6*2));
    GPIOA_PUPDR &= ~(3 << (7*2));

    GPIOA_PUPDR |=  (2 << (5*2));  // pull-down
    GPIOA_PUPDR |=  (2 << (6*2));  // pull-down
    GPIOA_PUPDR |=  (2 << (7*2));  // pull-down

    while(1) {

        int ignition = read_button_PA5();
        int brake    = read_button_PA6();
        int ready    = read_button_PA7();

        switch(state) {

            case STATE_OFF:
                if (ignition) {
                    uart2_print("[OFF] Ignition pressed -> PRECHARGE\r\n");
                    state = STATE_PRECHARGE;
                    delay(200000);
                }
                break;

            case STATE_PRECHARGE:
                if (ready && brake) {
                    uart2_print("[PRECHARGE] Brake+Ready detected -> READY\r\n");
                    state = STATE_READY;
                    delay(200000);
                }
                if (ready && !brake) {
                    uart2_print("[PRECHARGE] FAULT: Ready pressed without brake\r\n");
                    state = STATE_FAULT;
                }
                break;

            case STATE_READY:
                // Brake no longer required to stay in READY
                if (ignition) {
                    uart2_print("[READY] Ignition pressed -> OFF\r\n");
                    state = STATE_OFF;
                    delay(200000);
                }
                break;

            case STATE_FAULT:
                uart2_print("[FAULT] System locked. Press Ignition to reset.\r\n");

                if (ignition) {
                    uart2_print("[FAULT] Resetting -> OFF\r\n");
                    state = STATE_OFF;
                    delay(200000);
                }
                break;
        }

        update_leds();
    }
}

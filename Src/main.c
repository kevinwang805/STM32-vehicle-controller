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

#define SYSCFG_BASE     0x40013800
#define SYSCFG_EXTICR2  (*(volatile uint32_t*)(SYSCFG_BASE + 0x0C))

#define EXTI_BASE       0x40013C00
#define EXTI_IMR        (*(volatile uint32_t*)(EXTI_BASE + 0x00))
#define EXTI_RTSR       (*(volatile uint32_t*)(EXTI_BASE + 0x08))
#define EXTI_FTSR       (*(volatile uint32_t*)(EXTI_BASE + 0x0C))
#define EXTI_PR         (*(volatile uint32_t*)(EXTI_BASE + 0x14))

#define RCC_APB2ENR     (*(volatile uint32_t*)0x40023844)

#define NVIC_ISER0 (*(volatile uint32_t*)0xE000E100)

#define SYST_CSR (*(volatile uint32_t*)0xE000E010)
#define SYST_RVR (*(volatile uint32_t*)0xE000E014)
#define SYST_CVR (*(volatile uint32_t*)0xE000E018)

typedef enum {
    STATE_OFF = 0,
    STATE_PRECHARGE,
    STATE_READY,
    STATE_FAULT
} vehicle_state_t;

vehicle_state_t state = STATE_OFF;

volatile uint32_t system_time_ms = 0;  // incremented every 1 ms by SysTick

volatile int flag_ignition = 0;
volatile int flag_brake    = 0;
volatile int flag_ready    = 0;

volatile uint32_t last_ignition_time = 0;  // for 50 ms debounce
volatile uint32_t last_brake_time    = 0;
volatile uint32_t last_ready_time    = 0;

void uart2_write(char c)
{
    while (!(USART2_SR & (1 << 7))); // TXE
    USART2_DR = c;
}

void uart2_print(const char *s)
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

static void systick_init(void)
{
    SYST_RVR = 16000 - 1;   // 16 MHz / 1000 Hz = 16000 ticks per ms
    SYST_CVR = 0;           // clear current value
    SYST_CSR = (1 << 2) | (1 << 1) | (1 << 0);  // CLKSOURCE, TICKINT, ENABLE
}

void SysTick_Handler(void)
{
    system_time_ms++;
}

static void exti_init(void)
{
    // 1. Enable SYSCFG clock
    RCC_APB2ENR |= (1 << 14);

    // 2. Route PA5, PA6, PA7 to EXTI5/6/7
    SYSCFG_EXTICR2 &= ~(0xFFF);   // PA = 0000

    // 3. Enable EXTI lines 5, 6, 7
    EXTI_IMR |= (1 << 5) | (1 << 6) | (1 << 7);

    // 4. Rising edge trigger (button press)
    EXTI_RTSR |= (1 << 5) | (1 << 6) | (1 << 7);

    // 5. Falling edge trigger (button release)
    EXTI_FTSR |= (1 << 5) | (1 << 6) | (1 << 7);

    // 6. Enable NVIC interrupt for EXTI5–9
    NVIC_ISER0 |= (1 << 23);
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
            break;
    }
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t now = system_time_ms;

    if (EXTI_PR & (1 << 5)) {
        EXTI_PR |= (1 << 5); // clear pending

        if (GPIOA_IDR & (1 << 5)) {  // rising edge = press
            if ((now - last_ignition_time) >= 50) {  // 50 ms debounce
                flag_ignition = 1;
                last_ignition_time = now;
            }
        }
    }

    if (EXTI_PR & (1 << 6)) {
        EXTI_PR |= (1 << 6); // clear pending

        if (GPIOA_IDR & (1 << 6)) {  // rising edge = press
            if ((now - last_brake_time) >= 50) {  // 50 ms debounce
                flag_brake = 1;
                last_brake_time = now;
            }
        }
    }

    if (EXTI_PR & (1 << 7)) {
        EXTI_PR |= (1 << 7); // clear pending

        if (GPIOA_IDR & (1 << 7)) {  // rising edge = press
            if ((now - last_ready_time) >= 50) {  // 50 ms debounce
                flag_ready = 1;
                last_ready_time = now;
            }
        }
    }
}

int main(void) {

    /* Enable GPIO clocks */
    RCC_AHB1ENR |= (1 << 0); // GPIOA
    RCC_AHB1ENR |= (1 << 1); // GPIOB
    RCC_AHB1ENR |= (1 << 2); // GPIOC

    uart2_init();
    systick_init();

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

    uart2_print("50 ms debounce ENABLED\r\n");
    uart2_print("Rising + falling EXTI ENABLED\r\n\r\n");

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

    exti_init();
    update_leds();  // initial LED state

    while (1)
    {
        int ignition = flag_ignition;
        int brake    = flag_brake;
        int ready    = flag_ready;

        flag_ignition = 0;
        flag_brake    = 0;
        flag_ready    = 0;

        switch(state)
        {
            case STATE_OFF:
                if (ignition) {
                    uart2_print("[INT] Ignition -> PRECHARGE\r\n");
                    state = STATE_PRECHARGE;
                }
                break;

            case STATE_PRECHARGE:
                // Ready pressed: check if brake is CURRENTLY held
                if (ready) {
                    if (GPIOA_IDR & (1 << 6)) {
                        uart2_print("[INT] Brake + Ready -> READY\r\n");
                        state = STATE_READY;
                    } else {
                        uart2_print("[FAULT] Ready pressed without brake\r\n");
                        state = STATE_FAULT;
                    }
                }

                // Brake event alone doesn't transition state
                if (brake) {
                    uart2_print("[INT] Brake pressed\r\n");
                }
                break;

            case STATE_READY:
                if (ignition) {
                    uart2_print("[INT] Ignition -> OFF\r\n");
                    state = STATE_OFF;
                }
                break;

            case STATE_FAULT:
                if (ignition) {
                    uart2_print("[INT] Fault reset -> OFF\r\n");
                    state = STATE_OFF;
                }
                break;
        }

        update_leds();
    }
}

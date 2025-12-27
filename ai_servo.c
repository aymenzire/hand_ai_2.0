#include <MKL25Z4.H>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* ==========================
 * Prototypes
 * ========================== */
static void initServoPwm(void);
static void setServoAngleDeg(int angle_deg, uint8_t servo_id);

static void initUart0(void);
static uint8_t uart0ReceiveByteBlocking(void);

static uint8_t extractServoId(uint8_t rx_byte);
static uint8_t extractTurnCode(uint8_t rx_byte);

static void busyDelay(volatile unsigned int ticks);

/* ==========================
 * TODO:
 * - Remplacer les "magic numbers" (0x0C|0x20, 20.97152, etc.) par des #define clairs.
 * - Mettre la logique finger/turn -> angle dans une table (lookup table) au lieu de 15 if.
 * - Ajouter une validation: angle_deg borné [0..180] + gérer valeurs de turn non supportées.
 * - Ajouter une gestion d'erreur UART (timeout, trame invalide, etc.).
 * ========================== */

int main(void)
{
    uint8_t rx_byte = 0;

    initUart0();
    initServoPwm();

    /* Position initiale des 5 servos */
    for (uint8_t servo_id = 0; servo_id < 5; servo_id++) {
        setServoAngleDeg(45, servo_id);
    }

    while (1) {
        rx_byte = uart0ReceiveByteBlocking();

        /* Début de trame: 0xFF, ensuite 5 commandes */
        if (rx_byte == 0xFF) {

            for (uint8_t frame_idx = 0; frame_idx < 5; frame_idx++) {
                rx_byte = uart0ReceiveByteBlocking();

                uint8_t servo_id  = extractServoId(rx_byte);
                uint8_t turn_code = extractTurnCode(rx_byte);

                /* Petite logique (même comportement que ton code, mais plus lisible) */
                if (servo_id == 0x0) {
                    if (turn_code == 0x1) setServoAngleDeg(30, 0x0);
                    if (turn_code == 0x2) setServoAngleDeg(65, 0x0);
                    if (turn_code == 0x4) setServoAngleDeg(180, 0x0);
                }

                if (servo_id == 0x1) {
                    if (turn_code == 0x1) setServoAngleDeg(45, 0x1);
                    if (turn_code == 0x2) setServoAngleDeg(120, 0x1);
                    if (turn_code == 0x4) setServoAngleDeg(180, 0x1);
                }

                if (servo_id == 0x2) {
                    if (turn_code == 0x1) setServoAngleDeg(30, 0x2);
                    if (turn_code == 0x2) setServoAngleDeg(90, 0x2);
                    if (turn_code == 0x4) setServoAngleDeg(180, 0x2);
                }

                if (servo_id == 0x3) {
                    if (turn_code == 0x1) setServoAngleDeg(45, 0x3);
                    if (turn_code == 0x2) setServoAngleDeg(90, 0x3);
                    if (turn_code == 0x4) setServoAngleDeg(180, 0x3);
                }

                if (servo_id == 0x4) {
                    if (turn_code == 0x1) setServoAngleDeg(45, 0x4);
                    if (turn_code == 0x2) setServoAngleDeg(100, 0x4);
                    if (turn_code == 0x4) setServoAngleDeg(180, 0x4);
                }

                /* TODO: si servo_id > 4 -> ignorer / flagger une erreur */
            }
        }
    }
}

/* ==========================
 * PWM/Servo
 * ========================== */
static void initServoPwm(void)
{
    /* Activer l'horloge du PORTD (sorties PWM sur PTD0..PTD4) */
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    /* MUX = ALT4 (TPM0_CHx) sur PTD0..PTD4 */
    PORTD->PCR[0] = 0x0400;
    PORTD->PCR[1] = 0x0400;
    PORTD->PCR[2] = 0x0400;
    PORTD->PCR[3] = 0x0400;
    PORTD->PCR[4] = 0x0400;

    /* Activer TPM0 */
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

    /* Clock TPM = MCGFLLCLK (selon ta config) */
    SIM->SOPT2 |= 0x01000000;

    TPM0->SC = 0; /* Timer off pendant la config */

    /* Mode PWM (center-aligned) + pulse high */
    TPM0->CONTROLS[0].CnSC = 0x20 | 0x08;
    TPM0->CONTROLS[1].CnSC = 0x20 | 0x08;
    TPM0->CONTROLS[2].CnSC = 0x20 | 0x08;
    TPM0->CONTROLS[3].CnSC = 0x20 | 0x08;
    TPM0->CONTROLS[4].CnSC = 0x20 | 0x08;
}

static void setServoAngleDeg(int angle_deg, uint8_t servo_id)
{
    int duty_percent;
    int mod_value;
    int cnv_value;

    /* TODO: valider angle_deg (0..180) et saturer si besoin */

    /* Même calcul que ton code, juste renommé */
    mod_value    = 20.97152 * 1000000 / (2 * 16 * 50);
    duty_percent = (13 * angle_deg - 2 * angle_deg + 360) / 180;
    cnv_value    = (duty_percent * mod_value) / 100;

    /* On garde ton comportement: MOD écrit + CnV sur le channel choisi */
    TPM0->MOD = mod_value;

    switch (servo_id) {
        case 0x0:
            TPM0->SC = 0; /* disable timer */
            TPM0->CONTROLS[0].CnV = cnv_value;
            TPM0->SC = 0x0C | 0x20;
            break;

        case 0x1:
            TPM0->CONTROLS[1].CnV = cnv_value;
            TPM0->SC = 0x0C | 0x20;
            break;

        case 0x2:
            TPM0->CONTROLS[2].CnV = cnv_value;
            TPM0->SC = 0x0C | 0x20;
            break;

        case 0x3:
            TPM0->CONTROLS[3].CnV = cnv_value;
            TPM0->SC = 0x0C | 0x20;
            break;

        case 0x4:
            TPM0->CONTROLS[4].CnV = cnv_value;
            TPM0->SC = 0x0F; /* comme ton code */
            break;

        default:
            /* TODO: gérer servo_id invalide */
            break;
    }
}

/* ==========================
 * Décodage trame
 * ========================== */
static uint8_t extractServoId(uint8_t rx_byte)
{
    /* Bits [7:5] -> id servo (0..7) */
    return (uint8_t)(0x07 & (rx_byte >> 5));
}

static uint8_t extractTurnCode(uint8_t rx_byte)
{
    /* Bits [4:0] -> code angle/tour */
    return (uint8_t)(0x1F & rx_byte);
}

/* ==========================
 * UART0
 * ========================== */
static void initUart0(void)
{
    /* Horloge PORTA */
    SIM->SCGC5 |= SIM_SCGC5_PORTA(1);

    /* PTA1=UART0_RX, PTA2=UART0_TX (ALT2) */
    PORTA_PCR1 |= PORT_PCR_MUX(2);
    PORTA_PCR2 |= PORT_PCR_MUX(2);

    /* Source clock UART0 = MCGFLLCLK */
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);

    /* Horloge UART0 */
    SIM->SCGC4 |= SIM_SCGC4_UART0(1);

    /* Baud 115200 (tel que tu l'avais) */
    UART0->BDL = 0x0B;
    UART0->BDH = 0x00;

    /* 8-N-1 */
    UART0->C1 = 0x00;

    /* Interrupt RX activée (même si tu ne l'utilises pas en ISR ici) */
    UART0->C2 |= UART_C2_RIE(1);

    /* Activer Tx/Rx */
    UART0->C2 |= UART_C2_TE(1);
    UART0->C2 |= UART_C2_RE(1);

    /* TODO: si tu veux vraiment l'interrupt RX, ajouter NVIC + ISR UART0 */
}

static uint8_t uart0ReceiveByteBlocking(void)
{
    /* Attente bloquante jusqu'à un byte reçu */
    while (!(UART0->S1 & UART0_S1_RDRF_MASK)) {
        /* TODO: ajouter un timeout pour éviter blocage infini */
    }
    return (uint8_t)UART0->D;
}

/* ==========================
 * Delay simple
 * ========================== */
static void busyDelay(volatile unsigned int ticks)
{
    while (ticks--) {
        /* nop */
    }
}

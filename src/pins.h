
/******************************************************************************
 ****** NUCLEO ****************************************************************
 ******************************************************************************/
#define USER_BUTTON PC13
#define GREEN_LED PB0
#define BLUE_LED PB7
#define RED_LED PB14

/******************************************************************************
 ****** MOSFET OUTPUTS ********************************************************
 ******************************************************************************/
#define MOSFET_1 PD0
#define MOSFET_2 PG0

/******************************************************************************
 ****** RELAY OUTPUTS *********************************************************
 ******************************************************************************/
#define RELAY_1 PE1
#define RELAY_2 PG12

/******************************************************************************
 ****** RC SERVOS *************************************************************
 ******************************************************************************/
#define SERVO_1_PWM PC9
#define SERVO_2_PWM PC6

/******************************************************************************
 ****** ADC INPUTS ************************************************************
 ******************************************************************************/
#define BATT_SENSE PC0
#define TEMP_NTC PA4


// "MAIN" I2C (SPECTRO)
#define I2C1_SCL PB8
#define I2C1_SDA PB9

// "MAIN" SPI (SPECTRO)
#define SPI5_SCK PF7
#define SPI5_MISO PF8
#define SPI5_MOSI PF9

// "MAIN" SPI CHIP SELECT PINS
#define SPI_SS_1 PF2
#define SPI_SS_2 PD3
#define SPI_SS_3 PG1
#define SPI_SS_4 PF12

/******************************************************************************
 ****** EXTRA I2C FOR TEMP SENSOR (I2C2) **************************************
 ******************************************************************************/
#define EXTRA_I2C_SDA PF0
#define EXTRA_I2C_SCL PF1

/******************************************************************************
 ****** EXTRA I/O *************************************************************
 ******************************************************************************/
#define SPI_I2C_EXTRA_1 PE10
#define SPI_I2C_EXTRA_2 PE12
#define SPI_I2C_EXTRA_3 PE14
#define SPI_I2C_EXTRA_4 PE15

/******************************************************************************
 ****** LIMIT SWITCHES ********************************************************
 ******************************************************************************/
#define SW1 PC10
#define SW2 PC11
#define SW3 PC12
#define SW4 PG2
#define SW5 PG3
#define SW6 PD7
#define SW7 PD6
#define SW8 PA15

/******************************************************************************
 ****** DC_PUMP ***************************************************************
 ******************************************************************************/
#define DC_PUMP_1_EN_1 PD1
#define DC_PUMP_1_EN_2 PE3
#define DC_PUMP_1_PWM PA3

#define DC_PUMP_2_EN_1 PD5
#define DC_PUMP_2_EN_2 PD4
#define DC_PUMP_2_PWM PB3

#define DC_PUMP_3_EN_1 PC3
#define DC_PUMP_3_EN_2 PC2
//#define DC_PUMP_3_PWM PA0 / WKUP  this is the problem (wake-up pin is always high...)

#define DC_PUMP_4_EN_1 PD2
#define DC_PUMP_4_EN_2 PF6
#define DC_PUMP_4_PWM PA5

/******************************************************************************
 ****** STEPPERS **************************************************************
 ******************************************************************************/
#define STEPPER_1_EN PB12
#define STEPPER_1_DIR PF5
#define STEPPER_1_STEP PC8

#define STEPPER_2_EN PB2 / BOOT1
#define STEPPER_2_DIR PF4
#define STEPPER_2_STEP PA6

#define STEPPER_3_EN PE8
#define STEPPER_3_DIR PD11
#define STEPPER_3_STEP PB11

#define STEPPER_4_EN PF10
#define STEPPER_4_DIR PE7
#define STEPPER_4_STEP PC7

#define STEPPER_5_STEP PB15
#define STEPPER_5_EN PF14
#define STEPPER_5_DIR PE13

#define STEPPER_6_EN PE9
#define STEPPER_6_DIR PE11
#define STEPPER_6_STEP PB1

#define STEPPER_7_EN PF3
#define STEPPER_7_DIR PF11
#define STEPPER_7_STEP PD13

#define STEPPER_8_EN PF15
#define STEPPER_8_DIR PE0
#define STEPPER_8_STEP PB10

#define STEPPER_9_EN PG8
#define STEPPER_9_DIR PG5
#define STEPPER_9_STEP PD12

#define STEPPER_10_EN PD10
#define STEPPER_10_DIR PG4
#define STEPPER_10_STEP PD14

/******************************************************************************
 ****** GSM PINS **************************************************************
 ******************************************************************************/
#define MCU_UART_RX PG9
#define MCU_UART_TX PG14
#define MCU_GSM_PWRKEY PD15
#define MCU_UART_RI PB4
#define MCU_UART1_DTR PB5

/******************************************************************************
 ****** SD_CARD (SPI4) ********************************************************
 ******************************************************************************/
#define SD_CARD_SCK PE2
#define SD_CARD_CS PE4
#define SD_CARD_MISO PE5
#define SD_CARD_MOSI PE6

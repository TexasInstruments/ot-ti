/*
 *  ======== ti_drivers_config.h ========
 *  Configured TI-Drivers module declarations
 *
 *  The macros defines herein are intended for use by applications which
 *  directly include this header. These macros should NOT be hard coded or
 *  copied into library source code.
 *
 *  Symbols declared as const are intended for use with libraries.
 *  Library source code must extern the correct symbol--which is resolved
 *  when the application is linked.
 *
 *  DO NOT EDIT - This file is generated for the LP_CC1352P7_4
 *  by the SysConfig tool.
 */
#ifndef ti_drivers_config_h
#define ti_drivers_config_h

#define CONFIG_SYSCONFIG_PREVIEW

#define CONFIG_LP_CC1352P7_4
#ifndef DeviceFamily_CC13X2X7
#define DeviceFamily_CC13X2X7
#endif

#include <ti/devices/DeviceFamily.h>

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== CCFG ========
 */


/*
 *  ======== AESECB ========
 */

extern const uint_least8_t                  CONFIG_AESECB_MBEDTLS_CONST;
#define CONFIG_AESECB_MBEDTLS               0
#define CONFIG_TI_DRIVERS_AESECB_COUNT      1


/*
 *  ======== ECDH ========
 */

extern const uint_least8_t              CONFIG_ECDH_0_CONST;
#define CONFIG_ECDH_0                   0
#define CONFIG_TI_DRIVERS_ECDH_COUNT    1


/*
 *  ======== ECDSA ========
 */

extern const uint_least8_t                  CONFIG_ECDSA_0_CONST;
#define CONFIG_ECDSA_0                      0
#define CONFIG_TI_DRIVERS_ECDSA_COUNT       1


/*
 *  ======== ECJPAKE ========
 */

extern const uint_least8_t                  CONFIG_ECJPAKE_0_CONST;
#define CONFIG_ECJPAKE_0                    0
#define CONFIG_TI_DRIVERS_ECJPAKE_COUNT     1


/*
 *  ======== GPIO ========
 */
extern const uint_least8_t CONFIG_SPINEL_INT_CONST;
#define CONFIG_SPINEL_INT 16

/* Owned by CONFIG_SPI_1 as  */
extern const uint_least8_t CONFIG_GPIO_SPI_1_SCLK_CONST;
#define CONFIG_GPIO_SPI_1_SCLK 26

/* Owned by CONFIG_SPI_1 as  */
extern const uint_least8_t CONFIG_GPIO_SPI_1_POCI_CONST;
#define CONFIG_GPIO_SPI_1_POCI 24

/* Owned by CONFIG_SPI_1 as  */
extern const uint_least8_t CONFIG_GPIO_SPI_1_PICO_CONST;
#define CONFIG_GPIO_SPI_1_PICO 25

/* Owned by CONFIG_UART2_0 as  */
extern const uint_least8_t CONFIG_GPIO_UART2_0_TX_CONST;
#define CONFIG_GPIO_UART2_0_TX 13

/* Owned by CONFIG_UART2_0 as  */
extern const uint_least8_t CONFIG_GPIO_UART2_0_RX_CONST;
#define CONFIG_GPIO_UART2_0_RX 12

/* Owned by /ti/drivers/RF as  */
extern const uint_least8_t CONFIG_RF_24GHZ_CONST;
#define CONFIG_RF_24GHZ 29

/* Owned by CONFIG_SPI_1 as  */
extern const uint_least8_t CONFIG_GPIO_SPI_1_CSN_CONST;
#define CONFIG_GPIO_SPI_1_CSN 27

/* Owned by /ti/drivers/RF as  */
extern const uint_least8_t CONFIG_RF_HIGH_PA_CONST;
#define CONFIG_RF_HIGH_PA 28

/* Owned by /ti/drivers/RF as  */
extern const uint_least8_t CONFIG_RF_SUB1GHZ_CONST;
#define CONFIG_RF_SUB1GHZ 30

/* The range of pins available on this device */
extern const uint_least8_t GPIO_pinLowerBound;
extern const uint_least8_t GPIO_pinUpperBound;

/* LEDs are active high */
#define CONFIG_GPIO_LED_ON  (1)
#define CONFIG_GPIO_LED_OFF (0)

#define CONFIG_LED_ON  (CONFIG_GPIO_LED_ON)
#define CONFIG_LED_OFF (CONFIG_GPIO_LED_OFF)


/*
 *  ======== NVS ========
 */

extern const uint_least8_t              CONFIG_NVSINTERNAL_CONST;
#define CONFIG_NVSINTERNAL              0
extern const uint_least8_t              CONFIG_NVS_DBG_INTERNAL_CONST;
#define CONFIG_NVS_DBG_INTERNAL         1
#define CONFIG_TI_DRIVERS_NVS_COUNT     2


/*
 *  ======== RF ========
 */
#define Board_DIO_30_RFSW 0x0000001e


/*
 *  ======== SHA2 ========
 */

extern const uint_least8_t              CONFIG_SHA2_0_CONST;
#define CONFIG_SHA2_0                   0
#define CONFIG_TI_DRIVERS_SHA2_COUNT    1


/*
 *  ======== SPI ========
 */

/*
 *  PICO: DIO25
 *  POCI: DIO24
 *  SCLK: DIO26
 *  CSN: DIO27
 */
extern const uint_least8_t              CONFIG_SPI_1_CONST;
#define CONFIG_SPI_1                    0
#define CONFIG_TI_DRIVERS_SPI_COUNT     1


/*
 *  ======== TRNG ========
 */

extern const uint_least8_t              CONFIG_TRNG_THREAD_CONST;
#define CONFIG_TRNG_THREAD              0
#define CONFIG_TI_DRIVERS_TRNG_COUNT    1


/*
 *  ======== UART2 ========
 */

/*
 *  TX: DIO13
 *  RX: DIO12
 *  XDS110 UART
 */
extern const uint_least8_t                  CONFIG_UART2_0_CONST;
#define CONFIG_UART2_0                      0
#define CONFIG_TI_DRIVERS_UART2_COUNT       1


/*
 *  ======== Watchdog ========
 */

extern const uint_least8_t                  CONFIG_WATCHDOG0_CONST;
#define CONFIG_WATCHDOG0                    0
#define CONFIG_TI_DRIVERS_WATCHDOG_COUNT    1


/*
 *  ======== Board_init ========
 *  Perform all required TI-Drivers initialization
 *
 *  This function should be called once at a point before any use of
 *  TI-Drivers.
 */
extern void Board_init(void);

/*
 *  ======== Board_initGeneral ========
 *  (deprecated)
 *
 *  Board_initGeneral() is defined purely for backward compatibility.
 *
 *  All new code should use Board_init() to do any required TI-Drivers
 *  initialization _and_ use <Driver>_init() for only where specific drivers
 *  are explicitly referenced by the application.  <Driver>_init() functions
 *  are idempotent.
 */
#define Board_initGeneral Board_init

#ifdef __cplusplus
}
#endif

#endif /* include guard */

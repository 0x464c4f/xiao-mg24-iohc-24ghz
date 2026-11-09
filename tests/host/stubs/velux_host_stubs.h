/*
 * Host-side stand-ins for the Silicon Labs SDK / RAILtest symbols used by
 * firmware/app.c, so the real firmware source can be compiled and exercised
 * on a PC. The fake radio records every transmitted frame with its channel
 * and (virtual) timestamp, and can feed raw demodulator bytes back in.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef VELUX_HOST_STUBS_H
#define VELUX_HOST_STUBS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------- em_core.h
#define CORE_DECLARE_IRQ_STATE  int velux_irq_state_ = 0
#define CORE_ENTER_CRITICAL()   (void)velux_irq_state_
#define CORE_EXIT_CRITICAL()    (void)velux_irq_state_

// ---------------------------------------------------------------- sl_gpio.h
typedef enum { SL_GPIO_PORT_A, SL_GPIO_PORT_B, SL_GPIO_PORT_C, SL_GPIO_PORT_D } sl_gpio_port_t;
typedef enum { SL_GPIO_MODE_PUSH_PULL = 4 } sl_gpio_mode_t;
typedef struct { sl_gpio_port_t port; uint8_t pin; } sl_gpio_t;
int sl_gpio_set_pin_mode(const sl_gpio_t *gpio, sl_gpio_mode_t mode, bool out);

// ---------------------------------------------------------------- sl_cli.h
typedef struct {
  int argc;
  const char *const *argv;
} sl_cli_command_arg_t;
int sl_cli_get_argument_count(sl_cli_command_arg_t *a);
const char *sl_cli_get_argument_string(sl_cli_command_arg_t *a, int i);

// ---------------------------------------------------------------- rail.h
typedef void *RAIL_Handle_t;
typedef int RAIL_Status_t;
#define RAIL_STATUS_NO_ERROR 0

typedef enum { RAIL_IDLE_ABORT = 1 } RAIL_IdleMode_t;
typedef uint32_t RAIL_RadioState_t;
#define RAIL_RF_STATE_IDLE  0x1U
#define RAIL_RF_STATE_RX    0x2U
#define RAIL_RF_STATE_TX    0x4U
#define RAIL_TX_OPTION_REMOVE_CRC 0x10U

typedef enum { PACKET_MODE = 0, FIFO_MODE = 1 } RAIL_DataMethod_t;
typedef struct {
  int txSource;
  int rxSource;
  RAIL_DataMethod_t txMethod;
  RAIL_DataMethod_t rxMethod;
} RAIL_DataConfig_t;

typedef struct {
  uint8_t  syncWordBits;
  uint32_t syncWord1;
  uint32_t syncWord2;
} RAIL_SyncWordConfig_t;

typedef struct {
  RAIL_RadioState_t success;
  RAIL_RadioState_t error;
} RAIL_StateTransitions_t;

typedef void *RAIL_RxPacketHandle_t;
#define RAIL_RX_PACKET_HANDLE_INVALID          ((RAIL_RxPacketHandle_t)0)
#define RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE  ((RAIL_RxPacketHandle_t)2)

typedef struct {
  int      packetStatus;
  uint16_t packetBytes;
} RAIL_RxPacketInfo_t;

typedef struct {
  struct { uint32_t packetTime; } timeReceived;
  int8_t  rssi;
  uint8_t lqi;
  uint8_t subPhyId;
} RAIL_RxPacketDetails_t;

RAIL_Status_t RAIL_Idle(RAIL_Handle_t h, RAIL_IdleMode_t mode, bool wait);
uint16_t RAIL_SetFixedLength(RAIL_Handle_t h, uint16_t len);
void RAIL_ResetFifo(RAIL_Handle_t h, bool tx, bool rx);
uint16_t RAIL_WriteTxFifo(RAIL_Handle_t h, const uint8_t *data, uint16_t len, bool reset);
uint32_t RAIL_GetTime(void);
RAIL_Status_t RAIL_StartTx(RAIL_Handle_t h, uint16_t ch, uint32_t opts, const void *sched);
RAIL_RadioState_t RAIL_GetRadioState(RAIL_Handle_t h);
RAIL_Status_t RAIL_ConfigData(RAIL_Handle_t h, const RAIL_DataConfig_t *cfg);
RAIL_Status_t RAIL_ConfigSyncWords(RAIL_Handle_t h, const RAIL_SyncWordConfig_t *cfg);
RAIL_Status_t RAIL_SetTxPowerDbm(RAIL_Handle_t h, int16_t ddbm);
RAIL_Status_t RAIL_SetTxTransitions(RAIL_Handle_t h, const RAIL_StateTransitions_t *t);
RAIL_Status_t RAIL_StartRx(RAIL_Handle_t h, uint16_t ch, const void *sched);
RAIL_RxPacketHandle_t RAIL_GetRxPacketInfo(RAIL_Handle_t h, RAIL_RxPacketHandle_t which, RAIL_RxPacketInfo_t *info);
RAIL_Status_t RAIL_GetRxPacketDetailsAlt(RAIL_Handle_t h, RAIL_RxPacketHandle_t p, RAIL_RxPacketDetails_t *d);
void RAIL_CopyRxPacket(uint8_t *dst, const RAIL_RxPacketInfo_t *info);
RAIL_Status_t RAIL_ReleaseRxPacket(RAIL_Handle_t h, RAIL_RxPacketHandle_t p);
uint16_t RAIL_GetRxFifoBytesAvailable(RAIL_Handle_t h);
uint16_t RAIL_ReadRxFifo(RAIL_Handle_t h, uint8_t *dst, uint16_t len);

// ---------------------------------------------------------------- app_common.h (RAILtest)
#define ASYNC_RESPONSE 0x1U
extern RAIL_Handle_t railHandle;
extern uint16_t channel;
extern bool rxHeld;
extern bool rxProcessHeld;
extern volatile uint32_t packetsHeld;
extern uint32_t logLevel;
extern int32_t rxFreqOffset;
extern RAIL_DataConfig_t railDataConfig;
extern bool rxFifoManual;

// ---------------------------------------------------------------- registers / flash
extern volatile uint32_t fake_reg_fcd[4];
extern volatile uint32_t fake_reg_modem_ctrl1, fake_reg_sync0, fake_reg_sync1, fake_reg_timing;
#define REG_FRC_FCD0      (fake_reg_fcd[0])
#define REG_FRC_FCD1      (fake_reg_fcd[1])
#define REG_FRC_FCD2      (fake_reg_fcd[2])
#define REG_FRC_FCD3      (fake_reg_fcd[3])
#define REG_MODEM_CTRL1   (fake_reg_modem_ctrl1)
#define REG_MODEM_SYNC0   (fake_reg_sync0)
#define REG_MODEM_SYNC1   (fake_reg_sync1)
#define REG_MODEM_TIMING  (fake_reg_timing)

#define FAKE_FLASH_PAGE_SIZE 8192U
extern uint8_t fake_flash[FAKE_FLASH_PAGE_SIZE];
extern unsigned fake_flash_erase_count;
#define VELUX_NVM_FLASH_PAGE_ADDR ((uintptr_t)fake_flash)

// Replacement flash driver (NOR semantics: erase -> 0xFF, program can only clear bits)
void velux_flash_erase_page(uintptr_t page_addr);
void velux_flash_write_64b(uintptr_t slot_addr, const void *slot64);

#endif

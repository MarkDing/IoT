/***************************************************************************//**
 * @file
 * @brief Code related to letting the chip sleep when using Micrium RTOS.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include PLATFORM_HEADER
#include "app/framework/include/af.h"
#include "rtcdriver.h"
#include <kernel/include/os.h>
#include "bsp_tick_rtcc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "sleep.h"
#include "rail.h"

#if (defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_POLL_CLI) && defined(ZA_CLI_FULL))
#define ENABLE_POLL_CLI
#define POLL_CLI_TASK_PERIOD_MS                 250
#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_POLL_CLI && ZA_CLI_FULL

//------------------------------------------------------------------------------
// Static variables.

static bool zigbeeTaskDeepSleepAllowed = false;
static uint32_t zigbeeTaskNextWakeUpTimeMs;
static RTCDRV_TimerID_t wakeupTimerId;
static bool wakeupTimerInited = false;

//------------------------------------------------------------------------------
// Extern and forward declarations.

extern void emAfPluginMicriumRtosWakeUpZigbeeStackTask(void);
static uint16_t EM2WakeupProcessTime(void);

//------------------------------------------------------------------------------
// Sleep handler - invoked from the idle task hook.

void emAfPluginMicriumRtosSleepHandler(void)
{
  bool railCanDeepSleep = false;
  bool bleCanDeepSleep = true;
  uint32_t zigbeeSleepTimeMs, callTimeMs;

  if (!wakeupTimerInited) {
    assert(RTCDRV_AllocateTimer(&wakeupTimerId) == ECODE_EMDRV_RTCDRV_OK);
    wakeupTimerInited = true;
  }

  INTERRUPTS_OFF();

#if defined(EMBER_AF_PLUGIN_BLE)
  // The BLE stacks keeps the SLEEP manager updated with the current lowest
  // energy mode allowed, so we just query the SLEEP manager to determine
  // whether BLE can deep sleep or not.
  bleCanDeepSleep = (SLEEP_LowestEnergyModeGet() != sleepEM1);
#endif

  callTimeMs = halCommonGetInt32uMillisecondTick();

  if (timeGTorEqualInt32u(callTimeMs, zigbeeTaskNextWakeUpTimeMs)) {
    // ZigBee next wake up time is in the past, don't sleep at all and post the
    // ZigBee task so that it can refresh the next wake up time.
    INTERRUPTS_ON();
    emAfPluginMicriumRtosWakeUpZigbeeStackTask();
    return;
  } else {
    zigbeeSleepTimeMs =
      elapsedTimeInt32u(callTimeMs, zigbeeTaskNextWakeUpTimeMs);
  }

  if (bleCanDeepSleep && zigbeeTaskDeepSleepAllowed) {
    RAIL_Status_t status = RAIL_Sleep(EM2WakeupProcessTime(),
                                      &railCanDeepSleep);
    // RAIL_Sleep() returns a non-success status when the radio is not idle. We
    // do not deep sleep in that case.
    if (status != RAIL_STATUS_NO_ERROR) {
      railCanDeepSleep = false;
    }
  }

  if (zigbeeTaskDeepSleepAllowed && bleCanDeepSleep && railCanDeepSleep) {
    RTOS_ERR err;
    RAIL_Status_t status;

    // Lock the OS scheduler so that we can get to the RAIL_Wake() call once we
    // are done deep sleeping.
    OSSchedLock(&err);
    assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

    halPowerDown();

    // Start an RTC timer to get us out of EM2.
    assert(RTCDRV_StartTimer(wakeupTimerId,
                             rtcdrvTimerTypeOneshot,
                             zigbeeSleepTimeMs,
                             NULL,
                             NULL) == ECODE_EMDRV_RTCDRV_OK);
    // Enter EM2.
    halSleepPreserveInts(SLEEPMODE_WAKETIMER);

    assert(RAIL_Wake(0) == RAIL_STATUS_NO_ERROR);

    INTERRUPTS_ON();

    halPowerUp();

    assert(RTCDRV_StopTimer(wakeupTimerId) == ECODE_EMDRV_RTCDRV_OK);

    OSSchedUnlock(&err);
    assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);
  } else {
    halCommonIdleForMilliseconds(&zigbeeSleepTimeMs);
  }
}

//------------------------------------------------------------------------------
// Idle/sleep rtos callback - invoked from the idle/sleep plugin whenever the
// Zigbee stack can idle or sleep.

bool emberAfPluginIdleSleepRtosCallback(uint32_t *durationMs, bool sleepOk)
{
  uint32_t actualDurationMs = *durationMs;
  uint32_t deltaMs, startTicks = RTCDRV_GetWallClockTicks32();
  OS_TICK yieldTimeTicks;
  RTOS_ERR err;
  CPU_TS ts;

#if defined(ENABLE_POLL_CLI)
  if (actualDurationMs > POLL_CLI_TASK_PERIOD_MS) {
    actualDurationMs = POLL_CLI_TASK_PERIOD_MS;
  }
#endif // ENABLE_POLL_CLI

  yieldTimeTicks = (OSCfg_TickRate_Hz * actualDurationMs) / 1000;

  zigbeeTaskDeepSleepAllowed = sleepOk;
  zigbeeTaskNextWakeUpTimeMs
    = halCommonGetInt32uMillisecondTick() + actualDurationMs;

  INTERRUPTS_ON();

  // Yield the stack task.
  OSTaskSemPend(yieldTimeTicks, OS_OPT_PEND_BLOCKING, &ts, &err);

  zigbeeTaskDeepSleepAllowed = false;

  deltaMs = RTCDRV_TicksToMsec(RTCDRV_GetWallClockTicks32() - startTicks);

  if ( deltaMs <= actualDurationMs ) {
    *durationMs = actualDurationMs - deltaMs;
  } else {
    *durationMs = 0;
  }

  return true;
}

//------------------------------------------------------------------------------
// Static functions.

// Jira:EMZIGBEE-2690: EM2_WAKEUP_PROCESS_TIME_OVERHEAD_US value is set to 215
// in the BLE code (which is where this code comes from). On Jumbo, this
// translates to EM2WakeupProcessTime() returning 575us, which turned out to be
// too close. Bumping the overall EM2WakeupProcessTime() to 750 (by bumbing
// EM2_WAKEUP_PROCESS_TIME_OVERHEAD_US to 390) gives RAIL enough time to keep up
// with BLE events.

// Time required by the hardware to come out of EM2 in microseconds.
// This value includes HW startup, emlib and sleepdrv execution time.
// Voltage scaling, HFXO startup and HFXO steady times are excluded from
// this because they are handled separately. RTCCSYNC time is also
// excluded and it is handled by RTCCSYNC code itself.
#define EM2_WAKEUP_PROCESS_TIME_OVERHEAD_US (390)

// Time it takes to upscale voltage after EM2 in microseconds.
#define EM2_WAKEUP_VSCALE_OVERHEAD_US (30)

/* one cycle is 83 ns */
#define TIMEOUT_PERIOD_US(cycles) (cycles * 83 / 1000)
static const uint16_t timeoutPeriodTable[] =
{
  TIMEOUT_PERIOD_US(2),       /* 0 = 2 cycles */
  TIMEOUT_PERIOD_US(4),       /* 1 = 4 cycles */
  TIMEOUT_PERIOD_US(16),      /* 2 = 16 cycles */
  TIMEOUT_PERIOD_US(32),      /* 3 = 32 cycles */
  TIMEOUT_PERIOD_US(256),     /* 4 = 256 cycles */
  TIMEOUT_PERIOD_US(1024),    /* 5 = 1024 cycles */
  TIMEOUT_PERIOD_US(2048),    /* 6 = 2048 cycles */
  TIMEOUT_PERIOD_US(4096),    /* 7 = 4096 cycles */
  TIMEOUT_PERIOD_US(8192),    /* 8 = 8192 cycles */
  TIMEOUT_PERIOD_US(16384),   /* 9 = 16384 cycles */
  TIMEOUT_PERIOD_US(32768),   /* 10 = 32768 cycles */
};

static bool isHfxoAutoSelectEnabled(void)
{
#ifdef _CMU_HFXOCTRL_AUTOSTARTSELEM0EM1_MASK
  if (CMU->HFXOCTRL & _CMU_HFXOCTRL_AUTOSTARTSELEM0EM1_MASK) {
    return true;
  }
#endif
  return false;
}

static bool isEm23VScaleEnabled(void)
{
#ifdef _EMU_CTRL_EM23VSCALE_MASK
  if (EMU->CTRL & _EMU_CTRL_EM23VSCALE_MASK) {
    return true;
  }
#endif
  return false;
}

static uint16_t em2WakeupVScaleOverhead(void)
{
  if (!isEm23VScaleEnabled()) {
    return 0;
  }

  if (!isHfxoAutoSelectEnabled()) {
    // If HFXO auto select is disabled, the voltage upscaling is done in
    // EMLIB while waiting for HFXO to stabilize, thus adding no overhead
    // to the overall wakeup time.
    return 0;
  }

  return EM2_WAKEUP_VSCALE_OVERHEAD_US;
}

static uint16_t EM2WakeupProcessTime(void)
{
#ifndef _SILICON_LABS_32B_SERIES_2
  uint8_t steady = ((CMU->HFXOTIMEOUTCTRL
                     & _CMU_HFXOTIMEOUTCTRL_STEADYTIMEOUT_MASK)
                    >> _CMU_HFXOTIMEOUTCTRL_STEADYTIMEOUT_SHIFT);
  uint8_t startup = ((CMU->HFXOTIMEOUTCTRL
                      & _CMU_HFXOTIMEOUTCTRL_STARTUPTIMEOUT_MASK)
                     >> _CMU_HFXOTIMEOUTCTRL_STARTUPTIMEOUT_SHIFT);
  return timeoutPeriodTable[steady] + timeoutPeriodTable[startup]
         + EM2_WAKEUP_PROCESS_TIME_OVERHEAD_US + em2WakeupVScaleOverhead();
#else
  // Note: (EMHAL-1521) return some safe value (1ms) until we have all the
  // proper registers for EFR series 2.
  return 1000;
#endif
}

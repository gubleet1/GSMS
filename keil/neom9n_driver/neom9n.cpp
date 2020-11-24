#include "stm32f4xx_hal.h"
#include "tm_stm32_usart.h"
#include "ublox.h"
#include "neom9n.h"

// neo-m9n buffer
neom9n_buf_t neom9n_buf;

// neo-m9n handle
static SFE_UBLOX_GPS neom9n;

uint8_t neom9n_init(void)
{
  // connect to device using UART
  if (connect())
    return 1u;
  // configure device
  if (configure())
    return 1u;
  // initialization successful
  return 0u;
}

static uint8_t connect(void)
{
  uint32_t t0 = HAL_GetTick();
  // timeout after 2s
  while(HAL_GetTick() - t0 < 2000u)
  {
    // initialize UART7 (default baud rate)
    TM_USART_Init(UART7, TM_USART_PinsPack_1, NEOM9N_DEFAULT_BAUD_RATE);
    // try to connect
    if (neom9n.begin())
    {
      // successfully connected at default baud rate, set configured baud rate
      neom9n.setSerialRate(NEOM9N_BAUD_RATE);
      HAL_Delay(100);
    }
    // initialize UART7 (configured baud rate)
    TM_USART_Init(UART7, TM_USART_PinsPack_1, NEOM9N_BAUD_RATE);
    // try to connect
    if (neom9n.begin())
    {
      // successfully connected at configured baud rate
      return 0u;
    }
    // retry after 100ms
    HAL_Delay(100);
  }
  // initialization failed due to timeout
  return 1u;
}

static uint8_t configure(void)
{
  // activate UBX protocol on UART1 port
  if (!neom9n.setUART1Output(COM_TYPE_UBX))
    return 1u;
  // activate UBX-NAV-PVT message on UART1 port
  if (!neom9n.configureMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_UART1, 1))
    return 1u;
  // configure output rate
  if (!neom9n.setNavigationFrequency(NEOM9N_OUTPUT_RATE))
    return 1u;
  // enable auto PVT without implicit update
  neom9n.assumeAutoPVT(true, false);
  // configuration successful
  return 0u;
}

void neom9n_check(void)
{
  // check for new bytes in UART buffer and process them
  neom9n.checkUblox(UBX_CLASS_NAV, UBX_NAV_PVT);
}

uint8_t neom9n_data_ready(uint8_t update_buf)
{
  // check if unread raw data is available
  uint8_t data_ready = neom9n.dataReady() ? 1u : 0u;
  if (data_ready && update_buf)
  {
    // update buffer with raw data
    neom9n_update_buf();
  }
  return data_ready;
}

void neom9n_update_buf(void)
{
  // update NED velocity
  neom9n_buf.v_N = neom9n.getVelocityN();
  neom9n_buf.v_E = neom9n.getVelocityE();
  neom9n_buf.v_D = neom9n.getVelocityD();
}

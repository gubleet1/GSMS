#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "neom9n.h"
#include "processing.h"
#include "att_kf.h"
#include "app_main.h"

// hal handles
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c1;

// bno055 buffers
uint8_t bno055_curr_buf_index = 0u;
uint8_t bno055_next_buf_index = 0u;
bno055_buf_t bno055_buf[2];

// bno055 handle
static struct bno055_t bno055;

void app_init(void)
{
  // initialize kalman filters
  att_kf();
  // setup bno055 and neo-m9n
  bno055_setup();
  neom9n_setup();
  // start reading bno055 at 100Hz
  bno055_start();
  // start reading neo-m9n at 25Hz
  neom9n_start();
  // enable data processing
  processing_enabled = 1u;
}

static void bno055_setup(void)
{
  // initialize buffers
  bno055_buf_init();
  // initialize handle
  bno055.bus_read = bno055_i2c_bus_read;
  bno055.bus_write = bno055_i2c_bus_write;
  bno055.delay_msec = HAL_Delay;
  bno055.dev_addr = BNO055_I2C_ADDR2;
  // reset device
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(1u);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(1000u);
  // initialize driver
  if (bno055_init(&bno055) != BNO055_SUCCESS)
  {
    Error_Handler();
  }
  // set clock source to external
  if (bno055_set_clk_src(BNO055_BIT_ENABLE) != BNO055_SUCCESS)
  {
    Error_Handler();
  }
  // set page to 0
  if (bno055.page_id != BNO055_PAGE_ZERO)
  {
    if (bno055_write_page_id(BNO055_PAGE_ZERO) != BNO055_SUCCESS)
    {
      Error_Handler();
    }
  }
  // set power mode to normal
  if (bno055_set_power_mode(BNO055_POWER_MODE_NORMAL) != BNO055_SUCCESS)
  {
    Error_Handler();
  }
  // set operation mode to NDOF
  if (bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF) != BNO055_SUCCESS)
  {
    Error_Handler();
  }
  // set gyroscope unit to rad/s
  uint8_t gyro_unit = BNO055_INIT_VALUE;
  if (bno055_get_gyro_unit(&gyro_unit) != BNO055_SUCCESS)
  {
    Error_Handler();
  }
  if (gyro_unit != BNO055_GYRO_UNIT_RPS)
  {
    if (bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS) != BNO055_SUCCESS)
    {
      Error_Handler();
    }
  }
}

static void bno055_start(void)
{
  // start tim3 at 100Hz (algorithm base timer)
  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void bno055_buf_init(void)
{
  // initialize 2 buffers
  for (int i = 0; i < 2; i++)
  {
    // allocate memory
    bno055_buf[i].data = (uint8_t*) malloc(BNO055_BUF_SIZE * sizeof(uint8_t));
    // set empty flag
    bno055_buf[i].empty = 1u;
  }
}

void bno055_buf_inc(uint8_t* bno055_buf_index)
{
  // increment the buffer index
  *bno055_buf_index = (*bno055_buf_index + 1u) % 2;
}

int8_t bno055_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  int32_t status = BNO055_SUCCESS;
  // read bno055 registers in blocking mode
  if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(dev_addr << 1u), reg_addr, 1u, reg_data, cnt, HAL_MAX_DELAY) != HAL_OK)
  {
    status = BNO055_ERROR;
  }
  return (int8_t)status;
}

int8_t bno055_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  int32_t status = BNO055_SUCCESS;
  // write bno055 registers in blocking mode
  if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(dev_addr << 1u), reg_addr, 1u, reg_data, cnt, HAL_MAX_DELAY) != HAL_OK)
  {
    status = BNO055_ERROR;
  }
  return (int8_t)status;
}

static void neom9n_setup(void)
{
  // initialize driver
  if (neom9n_init())
  {
    Error_Handler();
  }
}

static void neom9n_start(void)
{
  // start tim4 at 625Hz
  // Note: This rate is used to continuously read neo-m9n data from the UART
  //       buffer. The actual sample output rate of the neo-m9n is set to 25Hz.
  if(HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // instance is TIM3
    // request bno055 samples at 100Hz (algorithm base timer)
    // check if i2c is ready
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
      Error_Handler();
    }
    // check if next buffer is empty
    if (!bno055_buf[bno055_next_buf_index].empty)
    {
      Error_Handler();
    }
    // i2c read parameters
    uint16_t dev_addr = bno055.dev_addr;
    uint8_t reg_addr = BNO055_BUF_BASE_REG_ADDR;
    uint8_t *reg_data = bno055_buf[bno055_next_buf_index].data;
    uint16_t cnt = BNO055_BUF_SIZE;
    // start DMA transfer
    if (HAL_I2C_Mem_Read_DMA(&hi2c1, (uint16_t) (dev_addr << 1u), reg_addr, 1u, reg_data, cnt) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else if (htim->Instance == TIM4)
  {
    // instance is TIM4
    // check for new neo-m9n data in UART buffer at 625Hz
    // Note: This will process received characters and update internal neo-m9n
    //       state variables if a full packet has been received.
    neom9n_check();
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    // instance is I2C1
    // bno055 sample received
    // transfer into next buffer is complete, clear the empty flag
    bno055_buf[bno055_next_buf_index].empty = 0u;
    // increment next buffer index
    bno055_buf_inc(&bno055_next_buf_index);
  }
}

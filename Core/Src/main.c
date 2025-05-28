/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - CS43L22 Happy Birthday Player
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t note_freq;    // Frequency register value for CS43L22
    uint16_t duration;    // Duration in milliseconds
} Note_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// CS43L22 I2C Address
#define CS43L22_ADDR        0x94  // 0x4A << 1

// CS43L22 Register Addresses
#define CS43L22_REG_ID              0x01
#define CS43L22_REG_POWER_CTRL1     0x02
#define CS43L22_REG_POWER_CTRL2     0x04
#define CS43L22_REG_CLOCKING_CTRL   0x05
#define CS43L22_REG_INTERFACE_CTRL1 0x06
#define CS43L22_REG_INTERFACE_CTRL2 0x07
#define CS43L22_REG_PASSTHROUGH_A   0x08
#define CS43L22_REG_PASSTHROUGH_B   0x09
#define CS43L22_REG_ANALOG_ZC_SC    0x0A
#define CS43L22_REG_PASSTHROUGH_GANG_CTRL 0x0C
#define CS43L22_REG_PLAYBACK_CTRL1  0x0D
#define CS43L22_REG_MISC_CTRL       0x0E
#define CS43L22_REG_PLAYBACK_CTRL2  0x0F
#define CS43L22_REG_PASSTHROUGH_A_VOL 0x14
#define CS43L22_REG_PASSTHROUGH_B_VOL 0x15
#define CS43L22_REG_PCMA_VOL        0x1A
#define CS43L22_REG_PCMB_VOL        0x1B
#define CS43L22_REG_BEEP_FREQ_TIME  0x1C
#define CS43L22_REG_BEEP_VOL_TONE   0x1D
#define CS43L22_REG_BEEP_TONE_CFG   0x1E
#define CS43L22_REG_TONE_CTRL       0x1F
#define CS43L22_REG_MASTER_A_VOL    0x20
#define CS43L22_REG_MASTER_B_VOL    0x21
#define CS43L22_REG_HP_A_VOL        0x22
#define CS43L22_REG_HP_B_VOL        0x23
#define CS43L22_REG_SPEAK_A_VOL     0x24
#define CS43L22_REG_SPEAK_B_VOL     0x25
#define CS43L22_REG_CH_MIX_SWAP     0x26
#define CS43L22_REG_LIMIT_CTRL1     0x27
#define CS43L22_REG_LIMIT_CTRL2     0x28
#define CS43L22_REG_LIMIT_ATTACK    0x29
#define CS43L22_REG_OVERFLOW_CLK    0x2E
#define CS43L22_REG_BATTERY_COMP    0x2F

// Beep Generator Frequency Values (approximate mapping)
#define BEEP_OFF    0x00
#define BEEP_C4     0x40  // ~261 Hz - C4
#define BEEP_CS4    0x44  // ~277 Hz - C#4
#define BEEP_D4     0x48  // ~294 Hz - D4
#define BEEP_DS4    0x4C  // ~311 Hz - D#4
#define BEEP_E4     0x50  // ~330 Hz - E4
#define BEEP_F4     0x54  // ~349 Hz - F4
#define BEEP_FS4    0x58  // ~370 Hz - F#4
#define BEEP_G4     0x5C  // ~392 Hz - G4
#define BEEP_GS4    0x60  // ~415 Hz - G#4
#define BEEP_A4     0x64  // ~440 Hz - A4
#define BEEP_AS4    0x68  // ~466 Hz - A#4
#define BEEP_B4     0x6C  // ~494 Hz - B4
#define BEEP_C5     0x70  // ~523 Hz - C5
#define BEEP_CS5    0x74  // ~554 Hz - C#5
#define BEEP_D5     0x78  // ~587 Hz - D5
#define BEEP_DS5    0x7C  // ~622 Hz - D#5
#define BEEP_E5     0x80  // ~659 Hz - E5
#define BEEP_F5     0x84  // ~698 Hz - F5
#define BEEP_FS5    0x88  // ~740 Hz - F#5
#define BEEP_G5     0x8C  // ~784 Hz - G5
#define BEEP_GS5    0x90  // ~831 Hz - G#5
#define BEEP_A5     0x94  // ~880 Hz - A5
#define BEEP_AS5    0x98  // ~932 Hz - A#5
#define BEEP_B5     0x9C  // ~988 Hz - B5

// LED pins for light show
#define LED_GREEN   GPIO_PIN_12  // GPIOD
#define LED_ORANGE  GPIO_PIN_13  // GPIOD
#define LED_RED     GPIO_PIN_14  // GPIOD
#define LED_BLUE    GPIO_PIN_15  // GPIOD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

/* USER CODE BEGIN PV */
// Happy Birthday melody notes and durations
// Key of C major, tempo: moderato
const Note_t happy_birthday[] = {
    // "Happy Birthday to you"
    {BEEP_C4, 300},   // Hap-
    {BEEP_C4, 200},   // py
    {BEEP_D4, 500},   // Birth-
    {BEEP_C4, 500},   // day
    {BEEP_F4, 500},   // to
    {BEEP_E4, 1000},  // you

    {BEEP_OFF, 200},  // Brief pause

    // "Happy Birthday to you"
    {BEEP_C4, 300},   // Hap-
    {BEEP_C4, 200},   // py
    {BEEP_D4, 500},   // Birth-
    {BEEP_C4, 500},   // day
    {BEEP_G4, 500},   // to
    {BEEP_F4, 1000},  // you

    {BEEP_OFF, 200},  // Brief pause

    // "Happy Birthday dear [Name]"
    {BEEP_C4, 300},   // Hap-
    {BEEP_C4, 200},   // py
    {BEEP_C5, 500},   // Birth-
    {BEEP_A4, 500},   // day
    {BEEP_F4, 500},   // dear
    {BEEP_E4, 500},   // [Na-
    {BEEP_D4, 1000},  // me]

    {BEEP_OFF, 200},  // Brief pause

    // "Happy Birthday to you"
    {BEEP_AS4, 300},  // Hap-
    {BEEP_AS4, 200},  // py
    {BEEP_A4, 500},   // Birth-
    {BEEP_F4, 500},   // day
    {BEEP_G4, 500},   // to
    {BEEP_F4, 1200},  // you
};

const uint8_t melody_length = sizeof(happy_birthday) / sizeof(Note_t);
static uint8_t play_count = 0;
static uint8_t current_note = 0;
static uint32_t note_start_time = 0;
static uint8_t is_playing = 0;
static uint32_t led_last_update = 0;
static uint8_t led_pattern = 0;
static uint8_t melody_finished = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
// CS43L22 Functions
HAL_StatusTypeDef CS43L22_Init(void);
HAL_StatusTypeDef CS43L22_WriteReg(uint8_t reg, uint8_t value);
uint8_t CS43L22_ReadReg(uint8_t reg);
void CS43L22_PlayBeep(uint8_t frequency, uint8_t volume);
void CS43L22_StopBeep(void);

// Music and LED Functions
void Play_Happy_Birthday(void);
void Update_LED_Show(uint8_t note_frequency);
void Reset_LEDs(void);
void Celebration_LED_Pattern(void);
void Error_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Write register to CS43L22
  * @param  reg: Register address
  * @param  value: Value to write
  * @retval HAL status
  */
HAL_StatusTypeDef CS43L22_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDR, data, 2, 1000);
}

/**
  * @brief  Read register from CS43L22
  * @param  reg: Register address
  * @retval Register value
  */
uint8_t CS43L22_ReadReg(uint8_t reg)
{
    uint8_t value = 0;
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDR, &reg, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c1, CS43L22_ADDR, &value, 1, 1000);
    return value;
}

/**
  * @brief  Initialize CS43L22 DAC
  * @retval HAL status
  */
HAL_StatusTypeDef CS43L22_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    // Reset CS43L22 via hardware pin
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(5);

    // Check device ID
    uint8_t id = CS43L22_ReadReg(CS43L22_REG_ID);
    if ((id & 0xF8) != 0xE0) {
        return HAL_ERROR; // Wrong device ID
    }

    // Power management sequence
    status |= CS43L22_WriteReg(CS43L22_REG_POWER_CTRL1, 0x01); // Power up

    // Set clocking control
    status |= CS43L22_WriteReg(CS43L22_REG_CLOCKING_CTRL, 0x81); // Auto detect clock

    // Interface control
    status |= CS43L22_WriteReg(CS43L22_REG_INTERFACE_CTRL1, 0x04); // I2S format

    // Set master volume
    status |= CS43L22_WriteReg(CS43L22_REG_MASTER_A_VOL, 0x00); // 0dB
    status |= CS43L22_WriteReg(CS43L22_REG_MASTER_B_VOL, 0x00); // 0dB

    // Set headphone volume
    status |= CS43L22_WriteReg(CS43L22_REG_HP_A_VOL, 0x00); // 0dB
    status |= CS43L22_WriteReg(CS43L22_REG_HP_B_VOL, 0x00); // 0dB

    // Enable beep generator
    status |= CS43L22_WriteReg(CS43L22_REG_BEEP_TONE_CFG, 0x80); // Enable tone generator

    // Power up required blocks
    status |= CS43L22_WriteReg(CS43L22_REG_POWER_CTRL2, 0xAF); // Power up HP and speakers

    return status;
}

/**
  * @brief  Play beep tone
  * @param  frequency: Beep frequency register value
  * @param  volume: Beep volume (0x00 to 0x0F)
  */
void CS43L22_PlayBeep(uint8_t frequency, uint8_t volume)
{
    // Set beep frequency and time
    CS43L22_WriteReg(CS43L22_REG_BEEP_FREQ_TIME, frequency);

    // Set beep volume and tone control
    CS43L22_WriteReg(CS43L22_REG_BEEP_VOL_TONE, (volume & 0x0F) | 0x80);

    // Enable beep
    CS43L22_WriteReg(CS43L22_REG_BEEP_TONE_CFG, 0xC0); // Enable beep and tone
}

/**
  * @brief  Stop beep tone
  */
void CS43L22_StopBeep(void)
{
    CS43L22_WriteReg(CS43L22_REG_BEEP_TONE_CFG, 0x80); // Disable beep, keep tone enabled
    CS43L22_WriteReg(CS43L22_REG_BEEP_VOL_TONE, 0x00); // Set volume to 0
}

/**
  * @brief  Create celebratory LED pattern
  */
void Celebration_LED_Pattern(void)
{
    // Sparkle effect - random LEDs
    for (int i = 0; i < 20; i++) {
        Reset_LEDs();

        // Light up random combination of LEDs
        uint8_t pattern = (HAL_GetTick() + i) % 16;
        if (pattern & 0x01) HAL_GPIO_WritePin(GPIOD, LED_GREEN, GPIO_PIN_SET);
        if (pattern & 0x02) HAL_GPIO_WritePin(GPIOD, LED_ORANGE, GPIO_PIN_SET);
        if (pattern & 0x04) HAL_GPIO_WritePin(GPIOD, LED_RED, GPIO_PIN_SET);
        if (pattern & 0x08) HAL_GPIO_WritePin(GPIOD, LED_BLUE, GPIO_PIN_SET);

        HAL_Delay(100);
    }

    // Final all-lights-on moment
    HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE, GPIO_PIN_SET);
    HAL_Delay(500);
    Reset_LEDs();
}

/**
  * @brief  Update LED light show based on note frequency
  * @param  note_frequency: Current note frequency value
  */
void Update_LED_Show(uint8_t note_frequency)
{
    uint32_t current_time = HAL_GetTick();

    // Update LED pattern every 150ms for a gentle, birthday-appropriate effect
    if (current_time - led_last_update > 150) {
        led_last_update = current_time;

        // Reset all LEDs first
        HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE, GPIO_PIN_RESET);

        if (note_frequency != BEEP_OFF) {
            // Birthday-themed LED patterns based on note frequency
            if (note_frequency >= BEEP_C5) {
                // High notes - Bright celebration (Red and Blue)
                HAL_GPIO_WritePin(GPIOD, LED_RED | LED_BLUE, GPIO_PIN_SET);
                if (led_pattern % 2) {
                    HAL_GPIO_WritePin(GPIOD, LED_ORANGE, GPIO_PIN_SET);
                }
            } else if (note_frequency >= BEEP_G4) {
                // Mid-high notes - Warm glow (Orange and Green)
                HAL_GPIO_WritePin(GPIOD, LED_ORANGE | LED_GREEN, GPIO_PIN_SET);
                if (led_pattern % 3 == 0) {
                    HAL_GPIO_WritePin(GPIOD, LED_BLUE, GPIO_PIN_SET);
                }
            } else if (note_frequency >= BEEP_E4) {
                // Mid notes - Happy colors (Green and Orange)
                HAL_GPIO_WritePin(GPIOD, LED_GREEN, GPIO_PIN_SET);
                if (led_pattern % 2) {
                    HAL_GPIO_WritePin(GPIOD, LED_ORANGE | LED_RED, GPIO_PIN_SET);
                }
            } else {
                // Lower notes - Gentle glow (Blue and Green)
                HAL_GPIO_WritePin(GPIOD, LED_BLUE | LED_GREEN, GPIO_PIN_SET);
            }

            // Special pattern for the "Happy Birthday" start notes (C4)
            if (note_frequency == BEEP_C4) {
                // Rotating birthday candle effect
                switch (led_pattern % 4) {
                    case 0: HAL_GPIO_WritePin(GPIOD, LED_GREEN, GPIO_PIN_SET); break;
                    case 1: HAL_GPIO_WritePin(GPIOD, LED_ORANGE, GPIO_PIN_SET); break;
                    case 2: HAL_GPIO_WritePin(GPIOD, LED_RED, GPIO_PIN_SET); break;
                    case 3: HAL_GPIO_WritePin(GPIOD, LED_BLUE, GPIO_PIN_SET); break;
                }
            }

            // Special highlight for the high C5 note (birthday climax)
            if (note_frequency == BEEP_C5) {
                HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE, GPIO_PIN_SET);
            }
        }

        led_pattern++;
    }
}

/**
  * @brief  Reset all LEDs
  */
void Reset_LEDs(void)
{
    HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE, GPIO_PIN_RESET);
}

/**
  * @brief  Play Happy Birthday melody
  */
void Play_Happy_Birthday(void)
{
    uint32_t current_time = HAL_GetTick();

    // Check if we've already played 2 times
    if (melody_finished) {
        return;
    }

    if (!is_playing) {
        // Start playing if we haven't played 2 times yet
        if (play_count < 2) {
            is_playing = 1;
            current_note = 0;
            note_start_time = current_time;
        } else {
            melody_finished = 1;
            // Special celebration at the end
            HAL_Delay(500);
            Celebration_LED_Pattern();
            return;
        }
    }

    // Check if current note duration has elapsed
    if (current_time - note_start_time >= happy_birthday[current_note].duration) {
        // Stop current note
        CS43L22_StopBeep();

        // Move to next note
        current_note++;

        if (current_note >= melody_length) {
            // End of melody reached
            current_note = 0;
            play_count++;
            is_playing = 0;
            Reset_LEDs();

            if (play_count < 2) {
                HAL_Delay(1500); // Longer pause between repetitions for birthday song
            } else {
                melody_finished = 1;
            }
            return;
        }

        // Start next note
        note_start_time = current_time;
    }

    // Play current note and update LEDs
    if (happy_birthday[current_note].note_freq != BEEP_OFF) {
        CS43L22_PlayBeep(happy_birthday[current_note].note_freq, 0x0C); // Slightly higher volume for celebration
    }

    Update_LED_Show(happy_birthday[current_note].note_freq);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  // Initialize CS43L22 DAC
  if (CS43L22_Init() != HAL_OK) {
      // Initialization failed - blink red LED
      while(1) {
          HAL_GPIO_WritePin(GPIOD, LED_RED, GPIO_PIN_SET);
          HAL_Delay(200);
          HAL_GPIO_WritePin(GPIOD, LED_RED, GPIO_PIN_RESET);
          HAL_Delay(200);
      }
  }

  // Initialization successful - birthday greeting LED show
  for (int i = 0; i < 5; i++) {
      // Simulate blowing out candles effect
      HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE, GPIO_PIN_SET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(GPIOD, LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE, GPIO_PIN_RESET);
      HAL_Delay(200);
  }

  // Brief moment before the birthday song
  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Play Happy Birthday with light show
    Play_Happy_Birthday();

    // Small delay to prevent excessive CPU usage
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                        |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

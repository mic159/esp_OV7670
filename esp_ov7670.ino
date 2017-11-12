#include "sccb.h"
#include <driver/ledc.h>
#include <rom/lldesc.h>
//#include <FreeRTOS.h>
#include "OV7670.h"
extern "C" {
  #include <soc/i2s_reg.h>
  #include <soc/i2s_struct.h>
}

#define PIN_XCLK 18  // Clock provided to camera
#define PIN_RESET 5  // Reset pin.

#define PIN_D0    GPIO_NUM_12
#define PIN_D1    GPIO_NUM_14
#define PIN_D2    GPIO_NUM_27
#define PIN_D3    GPIO_NUM_26
#define PIN_D4    GPIO_NUM_25
#define PIN_D5    GPIO_NUM_35
#define PIN_D6    GPIO_NUM_34
#define PIN_D7    GPIO_NUM_33
#define PIN_VSYNC GPIO_NUM_32
#define PIN_HREF  GPIO_NUM_39
#define PIN_PCLK  GPIO_NUM_36

#define OV7670_ADDR (0x21)
#define OV7670_PID (0x76)

#define DMA_NUM_BUFFERS 10
#define DMA_BUFFER_SIZE 4092
#define DMA_DECODED_SIZE (DMA_BUFFER_SIZE / 4)

uint8_t* dma_buffer[DMA_NUM_BUFFERS];
lldesc_t dma_buffer_desc[DMA_NUM_BUFFERS];
volatile int      dma_current_buffer = 0;
volatile int      dma_last_buffer = 0;
uint8_t  dma_decoded[DMA_DECODED_SIZE];
intr_handle_t dma_intr_handle;
volatile bool dma_interrupt = false;


void setup() {
  Serial.begin(115200);
  Serial.println("\n\nOV7670 TEST\nStarting...");
  
  // Set pin modes
  pinMode(PIN_D0, INPUT); pinMode(PIN_D1, INPUT); pinMode(PIN_D2, INPUT); pinMode(PIN_D3, INPUT);
  pinMode(PIN_D4, INPUT); pinMode(PIN_D5, INPUT); pinMode(PIN_D6, INPUT); pinMode(PIN_D7, INPUT);
  pinMode(PIN_VSYNC, INPUT);
  pinMode(PIN_HREF, INPUT);
  pinMode(PIN_PCLK, INPUT);


  printf("Enable clock on pin %i\n", PIN_XCLK);
  enable_clock();
  printf("Start SCCB  SDA: %i SCL: %i\n", SDA, SCL);
  SCCB_Init(SDA, SCL);
  printf("Toggle reset on pin %i\n", PIN_RESET);
  pinMode(PIN_RESET, OUTPUT);
  gpio_pulldown_en(GPIO_NUM_5);
  digitalWrite(PIN_RESET, LOW);
  delay(10);
  gpio_pulldown_dis(GPIO_NUM_5);
  digitalWrite(PIN_RESET, HIGH);
  delay(100);

  Serial.println("Checking for camera...");
  if (readRegister(OV7670_REG_PID) != OV7670_PID) {
    Serial.println("ERROR: OV7670 not detected.");
    return;
  }

  Serial.println("Enabling test pattern");
  setTestPattern(OV7670_TESTPATTERN_GRAY_FADE);
  updateRegister(OV7670_REG_COM7, OV7670_FORMAT_MASK, OV7670_FORMAT_RGB);
  updateRegister(OV7670_REG_COM15, OV7670_FORMAT_RGB_MASK, OV7670_FORMAT_RGB_565);
  check_state();

  Serial.println("Configuring I2S");
  dma_configure();
  Serial.println("Wait for VSYNC");
  while (gpio_get_level(PIN_VSYNC) == 0) {}
  while (gpio_get_level(PIN_VSYNC) != 0) {}
  dma_start();
  Serial.println("Started");
}

inline uint8_t readRegister(uint8_t addr) {
  return SCCB_Read(OV7670_ADDR, addr);
}

void updateRegister(uint8_t reg, uint8_t mask, uint8_t value) {
  SCCB_Write(
    OV7670_ADDR,
    reg,
    (readRegister(reg) & ~mask) | (value & mask)
  );
}

void setTestPattern(uint8_t type) {
  if (type) {
    updateRegister(OV7670_REG_COM7, OV7670_TESTPATTERN_ENABLE_MASK, OV7670_TESTPATTERN_ENABLE_MASK);
  } else {
    updateRegister(OV7670_REG_COM7, OV7670_TESTPATTERN_ENABLE_MASK, 0);
  }
  updateRegister(
    OV7670_REG_SCALING_XSC,
    OV7670_TESTPATTERN_MASK,
    type & 0b10 ? OV7670_TESTPATTERN_MASK: 0
  );
  updateRegister(
    OV7670_REG_SCALING_YSC,
    OV7670_TESTPATTERN_MASK,
    type & 0b01 ? OV7670_TESTPATTERN_MASK: 0
  );
}

void check_state() {
  printf(
    "PID: 0x%.2x, VER: 0x%.2x, MID: 0x%.4x\n",
    readRegister(OV7670_REG_PID),
    readRegister(OV7670_REG_VER),
    readRegister(OV7670_REG_MIDH) << 8 | readRegister(OV7670_REG_MIDL)
  );
  uint8_t com7 = readRegister(OV7670_REG_COM7);
  switch (com7 & OV7670_FORMAT_MASK) {
    case OV7670_FORMAT_YUV:
      Serial.println("Format: YUV");
      break;
    case OV7670_FORMAT_RGB:
    {
      uint8_t com15 = readRegister(OV7670_REG_COM15);
      switch (com15 & OV7670_FORMAT_RGB_MASK) {
        case OV7670_FORMAT_RGB_NORMAL:
          Serial.println("RGB Normal");
          break;
        case OV7670_FORMAT_RGB_565:
          Serial.println("RGB 565");
          break;
        case OV7670_FORMAT_RGB_555:
          Serial.println("RGB 555");
          break;
        default:
          Serial.println("Unknown");
      }
    }
      break;
    default:
      printf("Format: Unknown %x\n", com7);
  }
  printf("Test Pattern: %s\n", (com7 & 0b11)?"Enabled":"Disabled");
}

void loop() {
  if (!dma_interrupt) return;
  dma_interrupt = false;
  printf("Got data. last: %i current: %i\n", dma_last_buffer, dma_current_buffer);
  // decode data
  unsigned int length = min(
    dma_buffer_desc[dma_last_buffer].length / 4,
    DMA_DECODED_SIZE
  );
  uint8_t* input_cursor = dma_buffer[dma_last_buffer];
  uint8_t* output_cursor = dma_decoded;
  // b1 00 b0 00  b3 00 b2 00  b5 00 b4 00  b7 00 b6 00
  // b0 b1 b2 b3  b4 b5 b6 b7 ...
  for (int i = 0; i < length; i += 2) {
    *(output_cursor++) = input_cursor[2];
    *(output_cursor++) = input_cursor[0];
    input_cursor += 4;
  }

  for (int i = 0; i < length; i += 2) {
    uint8_t byte1 = dma_decoded[i];
    uint8_t byte2 = dma_decoded[i];
    uint8_t red   = (byte1 & 0b11111000);
    uint8_t green = (((byte1 & 0b00000111) << 3) | ((byte2 & 0b11100000) >> 5)) << 2;
    uint8_t blue  = (byte2 & 0b00011111) << 3;
    Serial.print((red + green + blue)/3);
    Serial.print(' ');
  }
  Serial.println('.');
}

static void IRAM_ATTR i2s_isr(void* arg) {
  I2S0.int_clr.val = I2S0.int_raw.val;
  dma_last_buffer = dma_current_buffer;
  dma_current_buffer = (dma_current_buffer + 1) % DMA_NUM_BUFFERS;
  dma_interrupt = true;
}

int enable_clock() {
  periph_module_enable(PERIPH_LEDC_MODULE);

  ledc_timer_config_t timer_conf;
  timer_conf.bit_num    = (ledc_timer_bit_t)1;
  timer_conf.freq_hz    = 16000000;  // 10MHz
  timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  timer_conf.timer_num  = LEDC_TIMER_0;
  esp_err_t err = ledc_timer_config(&timer_conf);
  if (err != ESP_OK) {
      Serial.print("ledc_timer_config failed, rc=");
      Serial.println(err);
      return err;
  }

  ledc_channel_config_t ch_conf;
  ch_conf.channel    = LEDC_CHANNEL_0;
  ch_conf.timer_sel  = LEDC_TIMER_0;
  ch_conf.intr_type  = LEDC_INTR_DISABLE;
  ch_conf.duty       = 1;
  ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  ch_conf.gpio_num   = PIN_XCLK;

  err = ledc_channel_config(&ch_conf);
  if (err != ESP_OK) {
      Serial.print("ledc_channel_config failed, rc=");
      Serial.println(err);
      return err;
  }
  Serial.println("Clock enabled");
}

void dma_configure() {
  const uint32_t const_high = 0x38;
  gpio_matrix_in(PIN_D0,    I2S0I_DATA_IN0_IDX, false);
  gpio_matrix_in(PIN_D1,    I2S0I_DATA_IN1_IDX, false);
  gpio_matrix_in(PIN_D2,    I2S0I_DATA_IN2_IDX, false);
  gpio_matrix_in(PIN_D3,    I2S0I_DATA_IN3_IDX, false);
  gpio_matrix_in(PIN_D4,    I2S0I_DATA_IN4_IDX, false);
  gpio_matrix_in(PIN_D5,    I2S0I_DATA_IN5_IDX, false);
  gpio_matrix_in(PIN_D6,    I2S0I_DATA_IN6_IDX, false);
  gpio_matrix_in(PIN_D7,    I2S0I_DATA_IN7_IDX, false);
  gpio_matrix_in(PIN_VSYNC, I2S0I_V_SYNC_IDX,   true);
  gpio_matrix_in(PIN_HREF,  I2S0I_H_SYNC_IDX,   false);
  gpio_matrix_in(const_high,I2S0I_H_ENABLE_IDX, false);
  gpio_matrix_in(PIN_PCLK,  I2S0I_WS_IN_IDX,    false);

  periph_module_enable(PERIPH_I2S0_MODULE);
  i2s_conf_reset();
  I2S0.conf.rx_slave_mod = 1; // Switch on Slave mode.
  I2S0.conf2.lcd_en = 1; // Enable parallel mode
  I2S0.conf2.camera_en = 1; // Use HSYNC/VSYNC/HREF to control sampling
  
  // Configure clock divider
  I2S0.clkm_conf.clkm_div_a   = 1;
  I2S0.clkm_conf.clkm_div_b   = 0;
  I2S0.clkm_conf.clkm_div_num = 2;

  I2S0.fifo_conf.dscr_en = 1; // FIFO will sink data to DMA
  I2S0.fifo_conf.rx_fifo_mod = 1; // FIFO configuration, 0-3???
  I2S0.fifo_conf.rx_fifo_mod_force_en = 1;

  I2S0.conf_chan.rx_chan_mod = 1;

  I2S0.sample_rate_conf.rx_bits_mod = 0; // Clear flags which are used in I2S serial mode

  I2S0.conf.rx_right_first = 0;
  I2S0.conf.rx_msb_right   = 0;
  I2S0.conf.rx_msb_shift   = 0;
  I2S0.conf.rx_mono        = 0;
  I2S0.conf.rx_short_sync  = 0;

  I2S0.timing.val          = 0;

  // DMA buffer setup
  for (int i = 0; i < DMA_NUM_BUFFERS; ++i) {
    printf("Creating buffer %i\n", i);
    dma_buffer[i] = new uint8_t[DMA_BUFFER_SIZE];
    dma_buffer_desc[i].size = DMA_BUFFER_SIZE;
    dma_buffer_desc[i].length = 0;
    dma_buffer_desc[i].owner  = 1;
    dma_buffer_desc[i].sosf   = 1;
    dma_buffer_desc[i].offset = 0;
    dma_buffer_desc[i].eof    = 1;
    dma_buffer_desc[i].empty  = 0;
    dma_buffer_desc[i].buf    = dma_buffer[i];
    //dma_buffer_desc[i].qe.stqe_next = &dma_buffer_desc[(i + 1) % DMA_NUM_BUFFERS];
  }

  I2S0.rx_eof_num = 360*2; // sample_count

  // Pointer to first buffer description, cast to a number?
  I2S0.in_link.addr  = (uint32_t)dma_buffer_desc;
  I2S0.in_link.start = 1;

  I2S0.int_clr.val     = I2S0.int_raw.val;
  I2S0.int_ena.val     = 0;
  I2S0.int_ena.in_done = 1;

  // Register the interrupt handler.
  esp_intr_alloc(
    ETS_I2S0_INTR_SOURCE,
    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
    &i2s_isr,
    NULL,
    &dma_intr_handle
  );
}

void dma_start() {
  //dma_semaphore.take();
  dma_interrupt = false;
  esp_intr_enable(dma_intr_handle); // Start the interrupt handler
  I2S0.conf.rx_start = 1; // Start I2S!
}

static void i2s_conf_reset() {
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M
            | I2S_AHBM_FIFO_RST_M;
    I2S0.lc_conf.val |= lc_conf_reset_flags;
    I2S0.lc_conf.val &= ~lc_conf_reset_flags;

    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M
            | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
    while (I2S0.state.rx_fifo_reset_back) {
        ;
    }
}


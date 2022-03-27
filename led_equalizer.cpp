#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico.h"

// For ADC input:
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "kiss_fft.hpp"
#include "kiss_fftr.hpp"
#include "PicoLED/PicoLed.hpp"

#define CAPTURE_CHANNEL 0
#define CAPTURE_DEPTH 512
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

#define SPEED_BIN 9
#define ADC_CLOCK_DIV 2500
#define FREQ_BIN_STEP 48000000.0/ADC_CLOCK_DIV/CAPTURE_DEPTH

#define MIN_MAGNITUDE_RANGE 0.04 //This is the lowest we will drop the fullscale range for outputPower
#define MAG_RANGE_ADJUST_SPEED 1 // This is a value between 0 and 1 to adjust how fast the display adjusts to volume changes
#define WS2812B_LED_PIN 2
#define LED_LENGTH 135
#define LED_ROWS 15
#define MAX_LED_BRIGHTNESS 255
#define MIN_LED_BRIGHTNESS 50
#define BRIGHTNESS_FADE 30

#define DISPLAY_ROWS 15
#define DISPLAY_COL 9


// Vector Magnitude Approximation Variables https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
#define ALPHA 0.9604
#define BETA 0.3978
#define PI 3.14159265

uint32_t i = 0;
uint dma_chan;
dma_channel_config cfg;
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
uint8_t bright[LED_LENGTH];
float magnitude_range = MIN_MAGNITUDE_RANGE;

auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 0, WS2812B_LED_PIN, LED_LENGTH, PicoLed::FORMAT_GRB);

void setup();
void sample(uint16_t *capture);

void setup()
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // disable for now. We can enable at a later date
        false     // Don't shift FIFO contents to one value or else you get some screwy numbers since its only bit 11:4 that are reported
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    adc_set_clkdiv(ADC_CLOCK_DIV);

    printf("Arming DMA\n");
    sleep_ms(1000);
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    dma_chan = dma_claim_unused_channel(true);
    cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);
}

void sample(uint16_t *capture)
{
    dma_channel_configure(dma_chan, &cfg,
            capture,    // dst
            &adc_hw->fifo,  // src
            CAPTURE_DEPTH,  // transfer count
            true            // start immediately
        );

    adc_run(true);


    // Once DMA finishes, stop any new conversions from starting, and clean up
    // the FIFO in case the ADC was still mid-conversion.
    dma_channel_wait_for_finish_blocking(dma_chan);
    //printf("Capture finished\n");
    adc_run(false);
    adc_fifo_drain();
}

void displayUpdate(uint8_t *Intensity) {
  int color_hsv = 0;
  
  PicoLed::Color leds[LED_LENGTH];
  for (int i = 0; i < DISPLAY_COL; i++) {
    for (int j = 1; j <= DISPLAY_ROWS; j++) {
      if (j <= Intensity[i]) {                              // Light everything within the intensity range
          bright[i*DISPLAY_ROWS + j-1] = MAX_LED_BRIGHTNESS; //((bright[i*DISPLAY_ROWS + j-1] + FADE) <= BRIGHTNESS) ? bright[i*DISPLAY_ROWS + j-1] + FADE : BRIGHTNESS;
          ledStrip.setPixelColor(i*DISPLAY_ROWS + j-1,PicoLed::HSV(color_hsv,255,MAX_LED_BRIGHTNESS));
          //This line is left here for reference
          //leds[i*DISPLAY_ROWS + j-1] = PicoLed::HSV(color_hsv,255,MAX_LED_BRIGHTNESS);

      }
      else {                                                 // Everything outside the range goes dark
        bright[i*DISPLAY_ROWS + j-1] = ((bright[i*DISPLAY_ROWS + j-1] - BRIGHTNESS_FADE) > MIN_LED_BRIGHTNESS) ? bright[i*DISPLAY_ROWS + j-1] - BRIGHTNESS_FADE : 0;
        ledStrip.setPixelColor(i*DISPLAY_ROWS + j-1,PicoLed::HSV(color_hsv,255,bright[i*DISPLAY_ROWS + j-1]));
        //This line is left here for reference
        //leds[i*DISPLAY_ROWS + j-1] = PicoLed::HSV(color_hsv,255,bright[i*DISPLAY_ROWS + j-1]);
      }
    }
    color_hsv += 255 / DISPLAY_COL;                                    // Increment the Hue to get the Rainbow
  }
}

int main() {
    float hammingWindow[CAPTURE_DEPTH];
    uint16_t capture_buf[CAPTURE_DEPTH];
    kiss_fft_scalar fft_in[CAPTURE_DEPTH]; // kiss_fft_scalar is a float
    kiss_fft_cpx fft_out[CAPTURE_DEPTH];
    float power[CAPTURE_DEPTH/2];
    float outputPower[SPEED_BIN];
    uint8_t intensityLED[SPEED_BIN];
    float a0 = 25.0/46.0;

    kiss_fftr_cfg confg = kiss_fftr_alloc(CAPTURE_DEPTH,false,0,0);

    uint64_t startTime = time_us_64();

    
    ledStrip.setBrightness(64);
    ledStrip.setPixelColor(20,PicoLed::RGB(255,255,255));
    ledStrip.show();

    setup();

    for (i = 0; i< CAPTURE_DEPTH; i++)
    {
        hammingWindow[i] = a0 - (1 - a0)*cos(2 * PI * i / CAPTURE_DEPTH);
    }

    while(true)
    {   
        startTime = time_us_64();
        sample(capture_buf);

        // Compute DC offset value (should be 3.3V/s)
        uint64_t sum = 0;
        for (int i=0;i<CAPTURE_DEPTH;i++) {sum+=capture_buf[i];}
        float avg = (float)sum/CAPTURE_DEPTH * ADC_CONVERT;

        // Fill FFT input subtracting DC offset
        // Consider applying the hamming window to the sample
        for (int i=0;i<CAPTURE_DEPTH;i++) {fft_in[i]=((float)capture_buf[i]*ADC_CONVERT-avg)*hammingWindow[i];}

        //computer fast fourier transform
        kiss_fftr(confg , fft_in, fft_out);

        // Computer power and calculate max freq component
        float max_power = 0;
        int max_idx = 0;
        int j = 0;
        float current_freq = 0;
        outputPower[0] = 0;
        uint16_t current_freq_bin = 32;
        // any frequency bin over CAPTURE_DEPTH/2 is aliased (nyquist sampling theorum)
        for (int i = 0; i < CAPTURE_DEPTH/2; i++) {
            current_freq = i * FREQ_BIN_STEP;
            power[i] = (fft_out[i].r > fft_out[i].i)? ALPHA * fft_out[i].r + BETA * fft_out[i].i : ALPHA * fft_out[i].i + BETA * fft_out[i].r;
            power[i] = power[i] > 0 ? power[i]*2/CAPTURE_DEPTH: -power[i]*2/CAPTURE_DEPTH;
            
            if (power[i]>max_power) {
                max_power=power[i];
                max_idx = i;
            }

            if(j == SPEED_BIN)
            {
                outputPower[j-1] = (power[i] > outputPower[j-1]) ? power[i] : outputPower[j-1];
                if (i == CAPTURE_DEPTH/2 -1)
                {
                    intensityLED[j-1] = (outputPower[j-1] / magnitude_range * 15 > LED_ROWS) ? 15 : outputPower[j-1] / magnitude_range * 15;
                }
            }
            if (current_freq > current_freq_bin) 
            {
                intensityLED[j] = (outputPower[j] / magnitude_range * 15 > LED_ROWS) ? 15 : outputPower[j] / magnitude_range * 15;
                j++;
                current_freq_bin = pow(2,j+5);
                if(j < SPEED_BIN)
                    outputPower[j] = power[i];
            }
            else if(j < SPEED_BIN)
            {
                outputPower[j] = (power[i] > outputPower[j]) ? power[i] : outputPower[j];
            }            
            
        }

        if (max_power > magnitude_range*0.75)
        {
            // last cycle's max power exceeded magnitude range, time to increase magnitude range slowly
            magnitude_range = magnitude_range * (LED_ROWS+1.0*MAG_RANGE_ADJUST_SPEED)/LED_ROWS;
            gpio_put(LED_PIN, 1);
            //DEBUG
            // ledStrip.setPixelColor(5,PicoLed::RGB(0,255,0));
            // ledStrip.show();
        }
        else if (max_power < magnitude_range / 2)
        {
            // last cycle's max power was less than half of the magnitude range. time to decrease magnitude range
            magnitude_range = magnitude_range * (LED_ROWS-1.0*MAG_RANGE_ADJUST_SPEED)/LED_ROWS;
            if (magnitude_range < MIN_MAGNITUDE_RANGE)
                magnitude_range = MIN_MAGNITUDE_RANGE;
            gpio_put(LED_PIN, 0);
            //DEBUG
            // ledStrip.setPixelColor(5,PicoLed::RGB(255,0,0));
            // ledStrip.show();
        }

        displayUpdate(intensityLED);
        ledStrip.show();

        // DEBUG - output the outputPower array to read from USB
        // for (int i = 0; i < SPEED_BIN; i++)
        // {
        //     outputPower[i] = power[(int)(i * CAPTURE_DEPTH / SPEED_BIN)];
        //     for (int j = (int)(i * CAPTURE_DEPTH / 2 / SPEED_BIN); j < (int)((i+1) * CAPTURE_DEPTH/ 2 / SPEED_BIN); j++)
        //     {
        //         outputPower[i] = (outputPower[i] > power[j]) ? outputPower[i]:power[j];
        //     }
        //     printf("%.2f\n", (outputPower[i]));
        // }

        //Print samples to stdout so you can display them in pyplot, excel, matlab
        // for (int i = 0; i < CAPTURE_DEPTH/2; i++) {
        //     printf("%.5f\n", (power[i]));   
        // }
        // printf("done\n");
        // sleep_ms(1000);

        // for (int i = 0; i<SPEED_BIN;i++)
        // {
        //     printf("%.5f\n", (outputPower[i]));
        //     //printf("%d\n", (intensityLED[i]));
        // }
        // printf("done\n");
        // sleep_ms(10);
        
    }
}
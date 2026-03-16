#include <Arduino.h>
#include <SPI.h>
#include "esp_dsp.h"
#include "esp_timer.h"
#include <math.h>

#define PIN_CS 5
#define PIN_SCLK 18
#define PIN_MISO 23
#define PIN_MOSI 25

#define MUX_IN_PIN 19

// ---- PARÂMETROS DO SISTEMA ----

#define ADC_VREF 3.3f
#define ADC_MAX 65535.0f

#define VIRTUAL_GND 1.65f
#define AMP_GAIN 33000.0f

#define FFT_SIZE 256
#define FS 256.0f

// ---- BANDAS EEG (Agora em Frequência Real, não índices) ----

#define DELTA_MIN_FREQ 1.0f
#define DELTA_MAX_FREQ 3.0f

#define THETA_MIN_FREQ 4.0f
#define THETA_MAX_FREQ 7.0f

#define ALPHA_MIN_FREQ 8.0f
#define ALPHA_MAX_FREQ 12.0f

#define SMR_MIN_FREQ 13.0f
#define SMR_MAX_FREQ 15.0f

#define BETA_MIN_FREQ 16.0f
#define BETA_MAX_FREQ 20.0f

#define BETA_HIGH_MIN_FREQ 21.0f
#define BETA_HIGH_MAX_FREQ 35.0f

// ---- DOUBLE BUFFER ----

float buffer_A[FFT_SIZE * 2];
float buffer_B[FFT_SIZE * 2];

float* write_buffer = buffer_A;
float* read_buffer = buffer_B;

float fft_magnitude[FFT_SIZE / 2];
float window[FFT_SIZE];

volatile int sample_index = 0;
volatile bool fft_ready = false;

// ---- TIMING ----

const uint64_t sample_interval_us = 1000000ULL / (int)FS;
esp_timer_handle_t sampling_timer;

SPISettings adsSettings(6000000, MSBFIRST, SPI_MODE0);

// ---- FUNÇÕES ----

void ads_init();
uint16_t ads_read();
float adc_to_microvolts(uint16_t sample);
int freq_to_bin(float freq);
float calcular_potencia(float* magnitude_array, float min_freq, float max_freq);
void IRAM_ATTR sampling_callback(void* arg);

// ------------------------------------------------

void setup()
{
    Serial.begin(921600);

    ads_init();

    pinMode(MUX_IN_PIN, OUTPUT);
    digitalWrite(MUX_IN_PIN, LOW);

    dsps_fft2r_init_fc32(NULL, FFT_SIZE);
    dsps_wind_hann_f32(window, FFT_SIZE);

    const esp_timer_create_args_t timer_args = {
        .callback = &sampling_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "eeg_sampler"
    };
    
    esp_timer_create(&timer_args, &sampling_timer);
    esp_timer_start_periodic(sampling_timer, sample_interval_us);

    Serial.println("Sistema EEG iniciado: PSD + Potência Relativa");
}

// ------------------------------------------------

void loop()
{
    if (fft_ready)
    {
        fft_ready = false; 

        // DESCOMENTE ABAIXO PARA VER O GRÁFICO NO PYTHON
        for(int i = 0; i < FFT_SIZE; i++) {
            Serial.print("T,");
            Serial.println(read_buffer[2 * i]);
        }
        
        float media = 0;

        for(int i = 0; i < FFT_SIZE; i++)
            media += read_buffer[2 * i];

        media /= FFT_SIZE;

        for(int i = 0; i < FFT_SIZE; i++)
        {
            read_buffer[2 * i] -= media;
            read_buffer[2 * i] *= window[i];
        }

        dsps_fft2r_fc32(read_buffer, FFT_SIZE);
        dsps_bit_rev_fc32(read_buffer, FFT_SIZE);

        // --- CÁLCULO DA PSD (Power Spectral Density) ---
        for(int i = 0; i < FFT_SIZE / 2; i++)
        {
            float re = read_buffer[2 * i];
            float im = read_buffer[2 * i + 1];

            // Cálculo correto da PSD: (Re² + Im²) / (N * FS)
            fft_magnitude[i] = (re * re + im * im) / (FS * FFT_SIZE);
        }

        // --- CÁLCULO DAS BANDAS (Potência Absoluta) ---
        float delta     = calcular_potencia(fft_magnitude, DELTA_MIN_FREQ, DELTA_MAX_FREQ);
        float theta     = calcular_potencia(fft_magnitude, THETA_MIN_FREQ, THETA_MAX_FREQ);
        float alpha     = calcular_potencia(fft_magnitude, ALPHA_MIN_FREQ, ALPHA_MAX_FREQ);
        float smr       = calcular_potencia(fft_magnitude, SMR_MIN_FREQ, SMR_MAX_FREQ);
        float beta      = calcular_potencia(fft_magnitude, BETA_MIN_FREQ, BETA_MAX_FREQ);
        float high_beta = calcular_potencia(fft_magnitude, BETA_HIGH_MIN_FREQ, BETA_HIGH_MAX_FREQ);

        // --- NORMALIZAÇÃO (Potência Relativa / Ratio) ---
        float total_power = delta + theta + alpha + smr + beta + high_beta;

        // Evita divisão por zero caso o sinal seja nulo
        if (total_power > 0.000001f) {
            delta     /= total_power;
            theta     /= total_power;
            alpha     /= total_power;
            smr       /= total_power;
            beta      /= total_power;
            high_beta /= total_power;
        }

        // Imprime os valores relativos (de 0.0 a 1.0). 
        // Ex: 0.45 em Alpha significa que 45% da atividade cerebral atual é Alpha.
        Serial.print("B,");
        Serial.print(delta, 4); Serial.print(",");
        Serial.print(theta, 4); Serial.print(",");
        Serial.print(alpha, 4); Serial.print(",");
        Serial.print(smr, 4); Serial.print(",");
        Serial.print(beta, 4); Serial.print(",");
        Serial.println(high_beta, 4);
    }
}

// ------------------------------------------------
void IRAM_ATTR sampling_callback(void* arg)
{
    uint16_t sample = ads_read();
    float microvolts = adc_to_microvolts(sample);

    write_buffer[2 * sample_index] = microvolts;
    write_buffer[2 * sample_index + 1] = 0; 

    sample_index++;

    if(sample_index >= FFT_SIZE)
    {
        sample_index = 0;

        float* temp = write_buffer;
        write_buffer = read_buffer;
        read_buffer = temp;

        fft_ready = true;
    }
}

// ------------------------------------------------
float adc_to_microvolts(uint16_t sample)
{
    float voltage = (sample * ADC_VREF) / ADC_MAX;
    float amp_output = voltage - VIRTUAL_GND;
    float eeg_voltage = amp_output / AMP_GAIN;
    return eeg_voltage * 1000000.0f; // Mantém a conversão para uV
}

// ------------------------------------------------
void ads_init()
{
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
}

// ------------------------------------------------
uint16_t ads_read()
{
    uint8_t b1, b2, b3;

    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_CS, LOW);

    b1 = SPI.transfer(0x00);
    b2 = SPI.transfer(0x00);
    b3 = SPI.transfer(0x00);

    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    
    uint16_t result = 0;
    result |= (b1 & 0x03) << 14;
    result |= (b2) << 6;
    result |= (b3) >> 2;

    return result;
}

// ------------------------------------------------
int freq_to_bin(float freq)
{
    // Converte a frequência em Hz para o índice correto do array da FFT
    return (int)((freq * FFT_SIZE) / FS);
}

// ------------------------------------------------
float calcular_potencia(float* magnitude_array, float min_freq, float max_freq)
{
    int bin_min = freq_to_bin(min_freq);
    int bin_max = freq_to_bin(max_freq);
    
    float soma = 0;
    for(int i = bin_min; i <= bin_max; i++)
        soma += magnitude_array[i];

    return soma;
}
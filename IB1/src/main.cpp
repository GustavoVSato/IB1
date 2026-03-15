#include <Arduino.h>
#include <SPI.h>
#include "esp_dsp.h"

#define PIN_CS 5
#define PIN_SCLK 18
#define PIN_MISO 23
#define PIN_MOSI 25 // Pino fantasma para o SPI.transfer não travar o ESP32

#define MUX_IN_PIN 19

#define DELTA_MIN 1
#define DELTA_MAX 3

#define THETA_MIN 4
#define THETA_MAX 7

#define ALPHA_MIN 8
#define ALPHA_MAX 12

#define SMR_MIN 13
#define SMR_MAX 15

#define BETA_MIN 16
#define BETA_MAX 20

#define BETA_HIGH_MIN 21
#define BETA_HIGH_MAX 35

#define FFT_SIZE 256
#define FS 256

// --- IMPLEMENTAÇÃO DE DOUBLE BUFFERING ---
float buffer_A[FFT_SIZE * 2];
float buffer_B[FFT_SIZE * 2];
float* write_buffer = buffer_A; // Onde os dados novos são gravados
float* read_buffer = buffer_B;  // Onde a FFT é processada

float fft_magnitude[FFT_SIZE / 2];
float window[FFT_SIZE];

int sample_index = 0;
bool fft_ready = false;

unsigned long last_sample_time = 0;
const unsigned long sample_interval = 1000000 / FS;

SPISettings adsSettings(6000000, MSBFIRST, SPI_MODE0);

// Declaração das funções
void ads_init();
uint16_t ads_read();
float calcular_potencia(float* magnitude_array, int min_freq, int max_freq);

void setup()
{
    Serial.begin(115200);

    ads_init();

    pinMode(MUX_IN_PIN, OUTPUT);

    // Trava o MUX no canal 1
    digitalWrite(MUX_IN_PIN, LOW);

    dsps_fft2r_init_fc32(NULL, FFT_SIZE);

    // Cria janela Hamming
    dsps_wind_hann_f32(window, FFT_SIZE);

    Serial.println("Sistema FFT iniciado");
}

void loop()
{
    unsigned long now = micros();
    
    // 1. ROTINA DE AMOSTRAGEM (Não é bloqueada pela FFT)
    if (now - last_sample_time >= sample_interval)
    {
        last_sample_time += sample_interval;

        uint16_t sample = ads_read();

        // Converte o valor bruto de 16 bits (0 a 65535) diretamente para microvolts reais.
        // Matemática: (3.3V / 33000 de ganho) * 1.000.000 uV = 100 uV de escala total.
        float s = ((float)sample / 65535.0) * 100.0;

        write_buffer[2 * sample_index] = s;
        write_buffer[2 * sample_index + 1] = 0;

        sample_index++;

        // Quando o buffer enche, troca os ponteiros e avisa que a FFT pode rodar
        if(sample_index >= FFT_SIZE)
        {
            sample_index = 0;
            
            float* temp = write_buffer;
            write_buffer = read_buffer;
            read_buffer = temp;
            
            fft_ready = true;
        }
    }

    // 2. ROTINA DE PROCESSAMENTO DA FFT
    if (fft_ready)
    {
        // A) Calcula a média (Nível DC / Offset do Terra Virtual de 1.65V)
        float media = 0;
        for(int i = 0; i < FFT_SIZE; i++)
        {
            media += read_buffer[2 * i];
        }
        media /= FFT_SIZE;

        // B) Remove o DC e aplica a Janela de Hann
        for(int i = 0; i < FFT_SIZE; i++)
        {
            read_buffer[2 * i] -= media;         // Filtra a corrente contínua automaticamente
            read_buffer[2 * i] *= window[i];     // Aplica a janela para evitar vazamento espetral
        }

        // C) Calcula a FFT
        dsps_fft2r_fc32(read_buffer, FFT_SIZE);
        dsps_bit_rev_fc32(read_buffer, FFT_SIZE);
        dsps_cplx2reC_fc32(read_buffer, FFT_SIZE);

        // D) Extrai as Magnitudes (agora na escala de microvolts ao quadrado)
        for(int i = 0; i < FFT_SIZE / 2; i++)
        {
            float re = read_buffer[2 * i];
            float im = read_buffer[2 * i + 1];

            fft_magnitude[i] = (re * re + im * im) / FFT_SIZE;
        }

        // E) Calcula e agrupa as potências (Ritmos Cerebrais em uV^2)
        float delta     = calcular_potencia(fft_magnitude, DELTA_MIN, DELTA_MAX);
        float theta     = calcular_potencia(fft_magnitude, THETA_MIN, THETA_MAX);
        float alpha     = calcular_potencia(fft_magnitude, ALPHA_MIN, ALPHA_MAX);
        float smr       = calcular_potencia(fft_magnitude, SMR_MIN, SMR_MAX);
        float beta      = calcular_potencia(fft_magnitude, BETA_MIN, BETA_MAX);
        float high_beta = calcular_potencia(fft_magnitude, BETA_HIGH_MIN, BETA_HIGH_MAX);

        // F) Envia via Serial no formato do Teleplot
        Serial.print(">D:"); Serial.println(delta);
        Serial.print(">T:"); Serial.println(theta);
        Serial.print(">A:"); Serial.println(alpha);
        Serial.print(">S:"); Serial.println(smr);
        Serial.print(">B:"); Serial.println(beta);
        Serial.print(">HB:"); Serial.println(high_beta);

        // G) Limpa a flag para o próximo ciclo
        fft_ready = false;
    }
}

float calcular_potencia(float* magnitude_array, int min_freq, int max_freq)
{
    float soma_potencia = 0;

    for (int i = min_freq; i <= max_freq; i++)
    {
        soma_potencia += magnitude_array[i];
    }

    return soma_potencia;
}

void ads_init()
{
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);

    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
}

uint16_t ads_read()
{
    uint8_t b1, b2, b3;

    SPI.beginTransaction(adsSettings);
    
    // Inicia a conversão baixando o CS
    digitalWrite(PIN_CS, LOW);

    // O ADS8326 exige 24 ciclos de clock (3 bytes transferidos)
    b1 = SPI.transfer(0x00); // Clocks 1-8: Amostragem + Bit Nulo + Bits 15 e 14
    b2 = SPI.transfer(0x00); // Clocks 9-16: Bits 13 a 6
    b3 = SPI.transfer(0x00); // Clocks 17-24: Bits 5 a 0 + rastro

    // Finaliza a comunicação
    digitalWrite(PIN_CS, HIGH);

    SPI.endTransaction();

    // Remontando o quebra-cabeça dos 16 bits
    uint16_t result = 0;
    
    // b1 guarda os bits B15 e B14 nas duas últimas posições (máscara 0x03 isola eles)
    result |= (b1 & 0x03) << 14; 
    
    // b2 guarda os bits B13 até B6
    result |= (b2) << 6;
    
    // b3 guarda os bits B5 até B0 nas 6 primeiras posições (precisamos descartar as 2 últimas)
    result |= (b3) >> 2;

    return result;
}
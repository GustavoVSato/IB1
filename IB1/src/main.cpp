#include <Arduino.h>
#include <SPI.h>
#include "esp_dsp.h"

#define PIN_CS 5      // CS do ADS8326
#define PIN_SCLK 18   // SCLK
#define PIN_MISO 19   // DOUT

#define MUX_IN_PIN 25 // controle do MUX

// Faixas de Frequência (Resolução 1 Hz por índice)
#define DELTA_MIN 1
#define DELTA_MAX 3

#define THETA_MIN 4
#define THETA_MAX 7

#define ALPHA_MIN 8
#define ALPHA_MAX 12

#define SMR_MIN 12
#define SMR_MAX 15

#define BETA_MIN 16     // Começa em 16 para não colidir com o SMR
#define BETA_MAX 20

#define BETA_HIGH_MIN 21
#define BETA_HIGH_MAX 35

#define FFT_SIZE 256
#define FS 256          // Frequência de amostragem em Hz

// Criação de buffers duplos (Um para cada canal)
float fft_buffer_ch1[FFT_SIZE * 2];
float fft_buffer_ch2[FFT_SIZE * 2];

float fft_magnitude_ch1[FFT_SIZE/2];
float fft_magnitude_ch2[FFT_SIZE/2];

int sample_index = 0;

// Controle de tempo para garantir exatos 256 Hz de amostragem
unsigned long last_sample_time = 0;
const unsigned long sample_interval = 1000000 / FS; // 3906 microssegundos

SPISettings adsSettings(6000000, MSBFIRST, SPI_MODE0);

// Declaração das funções
void ads_init();
uint16_t ads_read();
float calcular_potencia(float* magnitude_array, int min_freq, int max_freq);

void setup() {
  Serial.begin(115200);

  ads_init();     // inicializa SPI e ADC
  pinMode(MUX_IN_PIN, OUTPUT);

  // Inicializa DSP para FFT
  dsps_fft2r_init_fc32(NULL, FFT_SIZE); 
  
  Serial.println("Sistema de Captação e Processamento FFT Iniciado.");
}

void loop() {
  // Executa exatamente na frequência de amostragem (FS = 256 Hz)
  if (micros() - last_sample_time >= sample_interval) {
    last_sample_time = micros();

    // ---- DESMULTIPLEXAÇÃO SÍNCRONA ----
    
    // 1. Seleciona e lê o Canal 1 
    digitalWrite(MUX_IN_PIN, LOW);
    delayMicroseconds(2); // Tempo para o sinal do MUX estabilizar no ADC
    uint16_t sample1 = ads_read();

    // 2. Seleciona e lê o Canal 2 
    digitalWrite(MUX_IN_PIN, HIGH);
    delayMicroseconds(2); // Tempo para estabilizar
    uint16_t sample2 = ads_read();

    // Alimenta os buffers (Parte Real = sinal lido, Parte Imaginária = 0)
    fft_buffer_ch1[2 * sample_index] = (float)sample1;
    fft_buffer_ch1[2 * sample_index + 1] = 0;

    fft_buffer_ch2[2 * sample_index] = (float)sample2;
    fft_buffer_ch2[2 * sample_index + 1] = 0;

    sample_index++;

    // ---- PROCESSAMENTO DA FFT (A cada 1 segundo completo de dados) ----
    if(sample_index >= FFT_SIZE){
      sample_index = 0;

      // Executa FFT para o Canal 1
      dsps_fft2r_fc32(fft_buffer_ch1, FFT_SIZE);
      dsps_bit_rev_fc32(fft_buffer_ch1, FFT_SIZE);
      dsps_cplx2reC_fc32(fft_buffer_ch1, FFT_SIZE);

      // Executa FFT para o Canal 2
      dsps_fft2r_fc32(fft_buffer_ch2, FFT_SIZE);
      dsps_bit_rev_fc32(fft_buffer_ch2, FFT_SIZE);
      dsps_cplx2reC_fc32(fft_buffer_ch2, FFT_SIZE);

      // Calcula a magnitude (potência) de cada bin para os dois canais
      for(int i = 0; i < FFT_SIZE/2; i++){
        fft_magnitude_ch1[i] = (fft_buffer_ch1[2*i] * fft_buffer_ch1[2*i]) + (fft_buffer_ch1[2*i+1] * fft_buffer_ch1[2*i+1]); 
        fft_magnitude_ch2[i] = (fft_buffer_ch2[2*i] * fft_buffer_ch2[2*i]) + (fft_buffer_ch2[2*i+1] * fft_buffer_ch2[2*i+1]); 
      }

      // ---- SEPARAÇÃO DAS ONDAS (Exemplo focado no Canal 1) ----
      float delta     = calcular_potencia(fft_magnitude_ch1, DELTA_MIN, DELTA_MAX);
      float theta     = calcular_potencia(fft_magnitude_ch1, THETA_MIN, THETA_MAX);
      float alpha     = calcular_potencia(fft_magnitude_ch1, ALPHA_MIN, ALPHA_MAX);
      float smr       = calcular_potencia(fft_magnitude_ch1, SMR_MIN, SMR_MAX);
      float beta      = calcular_potencia(fft_magnitude_ch1, BETA_MIN, BETA_MAX);
      float high_beta = calcular_potencia(fft_magnitude_ch1, BETA_HIGH_MIN, BETA_HIGH_MAX);

      // Plota os dados brutos das bandas na porta Serial para monitoramento ou interface gráfica
      Serial.print("D:"); Serial.print(delta); Serial.print(",");
      Serial.print("T:"); Serial.print(theta); Serial.print(",");
      Serial.print("A:"); Serial.print(alpha); Serial.print(",");
      Serial.print("S:"); Serial.print(smr); Serial.print(",");
      Serial.print("B:"); Serial.print(beta); Serial.print(",");
      Serial.print("HB:"); Serial.println(high_beta);
    }
  }
}

// Função auxiliar para somar as potências dentro dos índices exatos da matriz
float calcular_potencia(float* magnitude_array, int min_freq, int max_freq) {
  float soma_potencia = 0;
  for (int i = min_freq; i <= max_freq; i++) {
    soma_potencia += magnitude_array[i];
  }
  return soma_potencia;
}

// Inicialização do hardware ADC
void ads_init() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);   // ADC inativo
  SPI.begin(PIN_SCLK, PIN_MISO, -1, PIN_CS);
}

// Leitura do chip ADS8326
uint16_t ads_read() {
  uint8_t hi, lo;

  SPI.beginTransaction(adsSettings);
  digitalWrite(PIN_CS, LOW);

  hi = SPI.transfer(0x00);
  lo = SPI.transfer(0x00);

  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  return (uint16_t)(hi << 8) | lo;
}
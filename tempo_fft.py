import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
from scipy.signal import iirnotch, lfilter, welch

# --- CONFIGURAÇÕES ---
PORTA = '/dev/ttyACM0'  
BAUD_RATE = 921600      
FS = 256                
AMOSTRAS_TEMPO = 512    

buffer_tempo = np.zeros(AMOSTRAS_TEMPO)

try:
    ser = serial.Serial(PORTA, BAUD_RATE, timeout=0.05)
    ser.dtr = False; ser.rts = False; time.sleep(0.1)
    ser.dtr = True; ser.rts = True; time.sleep(0.1)
    ser.reset_input_buffer()
except Exception as e:
    print(f"Erro ao conectar: {e}")
    sys.exit()

# Filtro Notch 60Hz
b_notch, a_notch = iirnotch(w0=60.0, Q=30.0, fs=FS)

# --- CONFIGURAÇÃO DA FIGURA ---
fig, (ax_tempo, ax_fft) = plt.subplots(2, 1, figsize=(10, 6))
fig.canvas.manager.set_window_title('EEG - Tempo e FFT')

# Eixo do Tempo
linha_tempo, = ax_tempo.plot(np.arange(AMOSTRAS_TEMPO), np.zeros(AMOSTRAS_TEMPO), color='black')
ax_tempo.set_title("Sinal EEG no Tempo (Notch 60Hz)")
ax_tempo.set_ylabel("Amplitude (uV)")
ax_tempo.set_ylim(-50, 50)
ax_tempo.grid(True, linestyle='--', alpha=0.5)

# Eixo da FFT
linha_fft, = ax_fft.plot(np.linspace(0, FS/2, 129), np.zeros(129), color='red')
ax_fft.set_title("Densidade Espectral de Potência (FFT)")
ax_fft.set_xlabel("Frequência (Hz)")
ax_fft.set_ylabel("Potência")
ax_fft.set_xlim(0, 70)
ax_fft.set_ylim(0, 5)
ax_fft.grid(True, linestyle='--', alpha=0.5)

plt.tight_layout()

def ler_serial():
    linhas_lidas = 0
    while ser.in_waiting > 0 and linhas_lidas < 500:
        try:
            linha = ser.readline().decode('utf-8').strip()
            linhas_lidas += 1
            if linha.startswith('T,'):
                valor = float(linha.split(',')[1])
                buffer_tempo[:-1] = buffer_tempo[1:]
                buffer_tempo[-1] = valor
        except Exception:
            pass 

def atualizar_grafico(frame):
    ler_serial()
    
    # Tratamento e filtro
    sinal_sem_dc = buffer_tempo - np.mean(buffer_tempo)
    sinal_limpo = lfilter(b_notch, a_notch, sinal_sem_dc)
    
    linha_tempo.set_ydata(sinal_limpo)
    
    # FFT (Welch)
    freqs, psd = welch(sinal_limpo, FS, nperseg=256)
    linha_fft.set_ydata(psd)
    
    # Auto-escala do gráfico de FFT
    max_psd = np.max(psd)
    if max_psd > ax_fft.get_ylim()[1]:
        ax_fft.set_ylim(0, max_psd * 1.2)
        
    return [linha_tempo, linha_fft]

ani = animation.FuncAnimation(fig, atualizar_grafico, interval=50, blit=False)
plt.show()
ser.close()

import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys
from scipy.signal import butter, sosfilt, iirnotch, lfilter, welch

# --- CONFIGURAÇÕES ---
PORTA = '/dev/ttyACM0'  
BAUD_RATE = 921600      
FS = 256                
AMOSTRAS_TEMPO = 512    

buffer_tempo = np.zeros(AMOSTRAS_TEMPO)

# Conexão Serial
try:
    ser = serial.Serial(PORTA, BAUD_RATE, timeout=0.05)
    ser.dtr = False; ser.rts = False; time.sleep(0.1)
    ser.dtr = True; ser.rts = True; time.sleep(0.1)
    ser.reset_input_buffer()
except Exception as e:
    print(f"Erro: {e}")
    sys.exit()

# --- FILTROS DIGITAIS ---
b_notch, a_notch = iirnotch(w0=60.0, Q=30.0, fs=FS)

def criar_filtro(lowcut, highcut, fs, order=4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    return butter(order, [low, high], analog=False, btype='band', output='sos')

sos_delta = criar_filtro(1.0, 3.0, FS)
sos_theta = criar_filtro(4.0, 7.0, FS)
sos_alpha = criar_filtro(8.0, 12.0, FS)
sos_smr   = criar_filtro(13.0, 15.0, FS)
sos_beta  = criar_filtro(16.0, 35.0, FS)

# --- ESTÉTICA DO DASHBOARD ---
# Criamos 7 subplots (6 de ondas no tempo + 1 de FFT)
fig = plt.figure(figsize=(12, 10))
grid = plt.GridSpec(7, 1, hspace=0.6)

eixos = [fig.add_subplot(grid[i, 0]) for i in range(6)]
ax_fft = fig.add_subplot(grid[6, 0]) # Gráfico da FFT na última linha

fig.canvas.manager.set_window_title('EEG Dashboard - Tempo e Frequência (FFT)')

titulos = [
    "Sinal Limpo (Notch 60Hz)", "Delta (1-3 Hz)", "Theta (4-7 Hz)", 
    "Alpha (8-12 Hz)", "SMR (13-15 Hz)", "Beta (16-35 Hz)"
]
cores = ['#000000', '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
linhas = []

for i, ax in enumerate(eixos):
    linha, = ax.plot(np.arange(AMOSTRAS_TEMPO), np.zeros(AMOSTRAS_TEMPO), color=cores[i], linewidth=1.2)
    linhas.append(linha)
    ax.set_title(titulos[i], loc='right', fontsize=9, color='#333333', pad=2)
    ax.set_ylabel("uV", fontsize=8)
    ax.grid(True, linestyle='--', alpha=0.5)
    
    # Ajuste de escala sugerido pela Luiza (mais zoom)
    if i == 0: ax.set_ylim(-40, 40)
    else: ax.set_ylim(-20, 20)

# Configuração específica do gráfico de FFT
ax_fft.set_title("Espectro de Frequência (FFT / PSD)", loc='right', fontsize=9, color='darkred')
ax_fft.set_xlabel("Frequência (Hz)")
ax_fft.set_ylabel("Potência", fontsize=8)
ax_fft.set_xlim(0, 70) # Mostra até 70Hz para ver o Notch em 60
ax_fft.set_ylim(0, 5)  # Ajuste conforme a potência do seu sinal
linha_fft, = ax_fft.plot(np.linspace(0, FS/2, 129), np.zeros(129), color='red', linewidth=1.5)
ax_fft.grid(True, linestyle='--', alpha=0.5)

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
    
    # 1. Tratamento do sinal no tempo
    sinal_sem_dc = buffer_tempo - np.mean(buffer_tempo)
    sinal_limpo = lfilter(b_notch, a_notch, sinal_sem_dc)
    
    # Atualiza ondas no tempo
    linhas[0].set_ydata(sinal_limpo)
    linhas[1].set_ydata(sosfilt(sos_delta, sinal_limpo))
    linhas[2].set_ydata(sosfilt(sos_theta, sinal_limpo))
    linhas[3].set_ydata(sosfilt(sos_alpha, sinal_limpo))
    linhas[4].set_ydata(sosfilt(sos_smr, sinal_limpo))
    linhas[5].set_ydata(sosfilt(sos_beta, sinal_limpo))
    
    # 2. Cálculo da FFT para o gráfico de frequência
    # Usamos o sinal limpo para mostrar o efeito dos filtros
    freqs, psd = welch(sinal_limpo, FS, nperseg=256)
    linha_fft.set_ydata(psd)
    
    # Ajuste automático simples de escala da FFT se o sinal subir muito
    max_psd = np.max(psd)
    if max_psd > ax_fft.get_ylim()[1]:
        ax_fft.set_ylim(0, max_psd * 1.2)
        
    return linhas + [linha_fft]

ani = animation.FuncAnimation(fig, atualizar_grafico, interval=50, blit=False)
plt.show()
ser.close()
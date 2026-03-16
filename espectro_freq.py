import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

# --- CONFIGURAÇÕES ---
PORTA = '/dev/ttyACM0'  
BAUD_RATE = 921600      

try:
    ser = serial.Serial(PORTA, BAUD_RATE, timeout=0.05)
    ser.dtr = False; ser.rts = False; time.sleep(0.1)
    ser.dtr = True; ser.rts = True; time.sleep(0.1)
    ser.reset_input_buffer()
except Exception as e:
    print(f"Erro ao conectar: {e}")
    sys.exit()

bandas_nomes = ['Delta\n(1-3 Hz)', 'Theta\n(4-7 Hz)', 'Alpha\n(8-12 Hz)', 'SMR\n(13-15 Hz)', 'Beta\n(16-20 Hz)', 'High Beta\n(21-35 Hz)']
valores_bandas = [0.0] * 6
cores = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

fig, ax = plt.subplots(figsize=(10, 5))
fig.canvas.manager.set_window_title('Potência Relativa das Bandas EEG')

barras = ax.bar(bandas_nomes, valores_bandas, color=cores)
ax.set_ylim(0, 1.0) # Vai de 0 a 100%
ax.set_ylabel("Potência Relativa (0.0 a 1.0)")
ax.set_title("Espectro de Frequência do EEG")
ax.grid(True, axis='y', linestyle='--', alpha=0.7)

def atualizar_grafico(frame):
    global valores_bandas
    while ser.in_waiting > 0:
        try:
            linha = ser.readline().decode('utf-8').strip()
            if linha.startswith('B,'):
                partes = linha.split(',')
                # Converte os 6 valores de string para float
                valores_bandas = [float(x) for x in partes[1:7]]
        except Exception:
            pass

    # Atualiza a altura de cada barra
    for barra, valor in zip(barras, valores_bandas):
        barra.set_height(valor)
        
    return barras

ani = animation.FuncAnimation(fig, atualizar_grafico, interval=50, blit=False)
plt.tight_layout()
plt.show()
ser.close()

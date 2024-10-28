import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import pandas as pd
import matplotlib.pyplot as plt

# Carica i dati dal file CSV
data = pd.read_csv('/home/andrea/cable_plots_ws/src/cable_model/cable_model_pkg/data/positions_data.csv')


# Estrai l'ultima riga
last_row = data.iloc[-1]

# Estrai le posizioni x e z
positions_x = [last_row[f'position_x_{i}'] for i in range(10)]
positions_z = [last_row[f'position_z_{i}'] for i in range(10)]

# Crea il grafico a dispersione
plt.figure(figsize=(10, 6))
plt.scatter(positions_x, positions_z, color='blue', marker='o', s= 200)  # Usa pallini blu #aumenta la size
plt.plot(positions_x, positions_z, color='red', linewidth=2)  # Usa una linea ro

# Imposta le etichette degli assi e ingrandiscile
plt.xlabel('Position X', fontsize=24)
plt.ylabel('Position Z', fontsize=24)

# Ingrandisci gli x-ticks e gli y-ticks
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)

# Aggiungi griglia
plt.grid(True)

# Mostra il grafico
plt.show()


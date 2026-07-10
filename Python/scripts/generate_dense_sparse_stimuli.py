import numpy as np
import os

# =====================================================================
# 1. CONFIGURAZIONE GEOMETRIA TILE E SPARSITY
# =====================================================================
M = 16        # Righe della Matrice A (Matrice Sparsa)
K = 32        # Colonne di A / Righe della Matrice B (Matrice Densa)
N = 16        # Colonne della Matrice B

SPARSITY = 0.80  # 80% di zeri nella Matrice A (ma mantenuta in formato denso)
AXI_WIDTH = 64 # Cambia in 64 se hai modificato il parametro nel sottosistema RTL
BYTE_WIDTH = 4   # 4 Byte = Word a 32-bit (int32 / float32)

PATH_STIMULI = "../../test/stimuli/" # Assicurati che il percorso sia coerente con la tua cartella

# Calcolo di quanti elementi a 32-bit stanno in un singolo beat AXI
DATA_PER_BEAT = AXI_WIDTH // (BYTE_WIDTH * 8)

# =====================================================================
# 2. GENERAZIONE DELLE MATRICI (A è sparsa nei valori, B è densa)
# =====================================================================
print(f"Generazione matrici: A ({M}x{K}) Sparsa, B ({K}x{N}) Densa...")

# Matrice A: Genera valori casuali e applica una maschera per imporre la sparsity
A_dense_vals = np.random.randint(1, 10, size=(M, K)).astype(np.int32)
mask = np.random.rand(M, K) > SPARSITY
A = A_dense_vals * mask  # Questa è la matrice sparsa in contenitore denso

# Matrice B: Interamente densa
B = np.random.randint(1, 10, size=(K, N)).astype(np.int32)

# Calcolo del risultato Golden ideale (C = A x B)
C_golden = np.dot(A, B)

# =====================================================================
# 3. FUNZIONE DI DATA PACKING PER IL BUS AXI
# =====================================================================
def pack_matrix_to_axi(matrix, data_per_beat):
    flattened = matrix.flatten()
    packed_lines = []
    
    # Raggruppa gli elementi in base alla larghezza del bus AXI
    for i in range(0, len(flattened), data_per_beat):
        beat_elements = flattened[i:i+data_per_beat]
        # Se gli elementi rimanenti sono meno della larghezza del bus, fai padding con zeri
        if len(beat_elements) < data_per_beat:
            beat_elements = np.pad(beat_elements, (0, data_per_beat - len(beat_elements)), 'constant')
        
        # Inverte l'ordine per rispettare l'esadecimale Little-Endian nel bus
        hex_str = "".join(f"{int(x):08x}" for x in reversed(beat_elements))
        packed_lines.append(hex_str)
    return packed_lines

# Pacchettizzazione dei segmenti di memoria
hex_A = pack_matrix_to_axi(A, DATA_PER_BEAT)
hex_B = pack_matrix_to_axi(B, DATA_PER_BEAT)
hex_C = pack_matrix_to_axi(C_golden, DATA_PER_BEAT)

# =====================================================================
# 4. SCRITTURA DEI FILE PER VERILATOR (initial_dram.txt e gold_dram.txt)
# =====================================================================
os.makedirs(PATH_STIMULI, exist_ok=True)

# Definiamo gli indirizzi base fittizi nella DRAM (es. a step di 0x10000 parole)
# Esprimiamo gli indirizzi in linee esadecimali per la sintassi @ di Verilog
line_address_A = 0x000000
line_address_B = 0x010000
line_address_C = 0x020000

# Scrittura di initial_dram.txt (Contiene A e B)
with open(os.path.join(PATH_STIMULI, "initial_dram.txt"), "w") as f:
    f.write(f"@{line_address_A:06X}\n")
    for line in hex_A:
        f.write(f"{line}\n")
        
    f.write(f"@{line_address_B:06X}\n")
    for line in hex_B:
        f.write(f"{line}\n")

# Scrittura di gold_dram.txt (Contiene il risultato atteso C)
with open(os.path.join(PATH_STIMULI, "gold_dram.txt"), "w") as f:
    f.write(f"@{line_address_C:06X}\n")
    for line in hex_C:
        f.write(f"{line}\n")

# =====================================================================
# 5. GENERAZIONE DI TSTCFG.TXT E GOLDENSTIMULI.TXT NATIVI
# =====================================================================
# tstcfg.txt definisce le aree di memoria da verificare a fine test
with open(os.path.join(PATH_STIMULI, "tstcfg.txt"), "w") as f:
    # Formato standard nativo: [start_line_hex] [end_line_hex]
    f.write(f"{line_address_C:08x} {line_address_C + len(hex_C):08x}\n")

# GoldenStimuli.txt nativo per SAURIA
# NOTA: Qui dovrai fare affidamento sui comandi generati nativamente dal 
# vecchio script di SAURIA per configurare i registri del dma_top nativo 
# e del df_controller nativo (es. basi di A, B, C e le dimensioni M, K, N).
with open(os.path.join(PATH_STIMULI, "GoldenStimuli.txt"), "w") as f:
    # Eseguiamo una sequenza fittizia di NOP per far scorrere il tempo,
    # oppure inserisci qui i comandi AXI-Lite nativi estratti dall'esempio originale di SAURIA.
    # Formato: [data] [addr] [wr] [rd] [wait] [exp] [check]
    for _ in range(100):
        f.write("00000000 00000000 0 0 0 00000000 0\n")
    
    # Riga finale con check_flag attivo per terminare la simulazione
    f.write("00000000 00000000 0 0 0 00000000 1\n")

print("Generazione completata con successo! I file di test sono pronti in:", PATH_STIMULI)
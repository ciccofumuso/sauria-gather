import numpy as np

# Nuova geometria controllata per l'isolamento debug
M, K, N = 8, 8, 8

# Indirizzi DRAM (Coerenti con l'Azione 3, espressi in Byte)
ADDR_DENSE_MATRIX   = 0x80000000
ADDR_COMP_IDX       = 0x80200000
ADDR_UNCOMP_IDX     = 0x80400000
ADDR_OUTPUT_RESULT  = 0x80600000

WORD_OFFS_DENSE   = (ADDR_DENSE_MATRIX - 0x80000000) // 8
WORD_OFFS_COMP    = (ADDR_COMP_IDX - 0x80000000) // 8
WORD_OFFS_UNCOMP  = (ADDR_UNCOMP_IDX - 0x80000000) // 8
WORD_OFFS_OUTPUT  = (ADDR_OUTPUT_RESULT - 0x80000000) // 8

print("--- Generazione Test Isolato 8x8 Deterministico ---")

# Creiamo una matrice sparsa fissa: ogni riga ha esattamente 2 elementi non-zero (es. alle colonne 1 e 4)
A_dense = np.zeros((M, K), dtype=np.int32)
for i in range(M):
    A_dense[i, 1] = i + 1
    A_dense[i, 4] = i + 2

nnzs_per_row = np.array([2] * M, dtype=np.uint16)      # Ogni riga ha 2 elementi
uncompressed_idx = np.array([1, 4] * M, dtype=np.uint16) # Indici di colonna fissi

# Matrice densa B (8x8) -> Trasposta diventa B^T (8x8)
B_dense = np.ones((K, N), dtype=np.int32) * 5 # Riempita di 5 per un controllo matematico rapido
B_transposed = B_dense.T

# Formattazione stringhe esadecimali per Verilator
def to_hex_64bit_matrix(data_array):
    flat = data_array.flatten()
    return [f"{(int(flat[i+1]) << 32) | (int(flat[i]) & 0xFFFFFFFF):016X}" for i in range(0, len(flat), 2)]

def to_hex_64bit_indices(data_array):
    flat = data_array.flatten()
    while len(flat) % 4 != 0: flat = np.append(flat, 0)
    return [f"{(int(flat[i+3]) << 48) | (int(flat[i+2]) << 32) | (int(flat[i+1]) << 16) | (int(flat[i]) & 0xFFFF):016X}" for i in range(0, len(flat), 4)]

hex_dense = to_hex_64bit_matrix(B_transposed)
hex_comp = to_hex_64bit_indices(nnzs_per_row)
hex_uncomp = to_hex_64bit_indices(uncompressed_idx)

# Scrittura dei file di memoria
with open("initial_dram.txt", "w") as f:
    f.write(f"@{WORD_OFFS_DENSE:08X}\n")
    for line in hex_dense: f.write(f"{line}\n")
    f.write(f"@{WORD_OFFS_COMP:08X}\n")
    for line in hex_comp: f.write(f"{line}\n")
    f.write(f"@{WORD_OFFS_UNCOMP:08X}\n")
    for line in hex_uncomp: f.write(f"{line}\n")

# File degli stimoli composto solo da NOP
stimuli_lines = ["00000000 00000000 0 0 0 00000000 0"] * 3000
stimuli_lines.append("00000000 00000000 0 0 0 00000000 1")

with open("GoldenStimuli.txt", "w") as f: f.write("\n".join(stimuli_lines) + "\n")
with open("tstcfg.txt", "w") as f:
    f.write(f"{ADDR_DENSE_MATRIX:08X}\n")
    f.write(f"{ADDR_OUTPUT_RESULT:08X}\n")
    f.write(f"{ADDR_OUTPUT_RESULT + 0x0500:08X}\n")

print(f"File generati! Word dense: {len(hex_dense)}, Comp: {len(hex_comp)}, Uncomp: {len(hex_uncomp)}")
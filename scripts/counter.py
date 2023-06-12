from collections import Counter

def controlla_elemento_piu_frequente(tupla):
    # Conta le occorrenze di ciascun elemento nella tupla
    conteggio = Counter(tupla)
    
    # Trova l'elemento più frequente e il numero di volte che appare
    elemento_piu_frequente, frequenza_piu_alta = conteggio.most_common(1)[0]
    
    # Calcola la percentuale di frequenza rispetto alla lunghezza della tupla
    percentuale_frequenza = (frequenza_piu_alta / len(tupla)) * 100
    
    # Verifica se l'elemento più frequente è presente per almeno l'80% dei casi
    if percentuale_frequenza >= 80:
        print(f"L'elemento {elemento_piu_frequente} si ripete per almeno l'80% dei casi.")
    else:
        print(f"L'elemento {elemento_piu_frequente} non si ripete per almeno l'80% dei casi.")

# Esempio di utilizzo
tupla = (1, 1, 1, 1, 1, 2, 2, 2, 2, 2)
controlla_elemento_piu_frequente(tupla)

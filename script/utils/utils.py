from collections import Counter

def count_occurrences(self, vector):

    """ Count Occurrences of Each Element """

    # Count Occurrences of Each Element in the Vector
    count = Counter(vector)

    # Find the More Frequent Element
    more_frequent_el, high_freq = count.most_common(1)[0]

    # Compute Frequency Percentage in respect to Vector Length
    freq_percentage = (high_freq / len(vector)) * 100

    # Verifica se l'elemento più frequente è presente per almeno l'80% dei casi
    if percentuale_frequenza >= self.recognition_percentage:
        print("Ho riconosciuto il gesto {}".format(elemento_piu_frequente))
        return elemento_piu_frequente

    else:
        print("Comando Gestuale Non Riconosciuto")
    
    self.gesture_vector = None

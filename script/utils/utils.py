import os, shutil, threading
import torch, rospy

from collections import Counter

# Project Folder (ROOT Project Location)
FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

def count_occurrences(vector, recognition_percentage):

    """ Count Occurrences of Each Element """

    # Count Occurrences of Each Element in the Vector
    count = Counter(vector)

    # Find the Most Frequent Element
    more_frequent_el, high_freq = count.most_common(1)[0]

    # Compute Frequency Percentage in respect to Vector Length
    freq_percentage = (high_freq / len(vector)) * 100

    # Check if the Most Frequent Element is Present for at least 80% of the Cases
    if freq_percentage >= recognition_percentage:

        print(f'Gesture Recognized {more_frequent_el}')
        return more_frequent_el

    else:

        # Clean Gesture Vector
        print('Gesture Not Recognized')
        return None

def classifierNetwork(input, model, device):

    """ Classify the Gesture """

    input = torch.tensor(input, dtype=torch.float32, device=device)
    output = model(input)
    prediction = output.argmax(dim=0)
    command = prediction.item()
    print(f'Prediction: {command}')

    return command

class MyThread(threading.Thread):

    def __init__(self):
        super().__init__()
        self.stop_event = threading.Event()
        self.paused = threading.Event()
        self.paused.set()
        self.time = 0

    def run(self):

        while not self.stop_event.is_set() and not rospy.is_shutdown():
            self.paused.wait()
            self.time += 1
            rospy.sleep(1)
            if not self.time  == 0: print("Counter", self.time)

    def resume(self):
        self.paused.set()

    def pause(self):
        self.paused.clear()

    def stop(self):
        self.stop_event.set()
        self.time = 0

def delete_pycache_folders(verbose:bool=False):

  """ Delete Python `__pycache__` Folders Function """
  
  if verbose: print('\n\nDeleting `__pycache__` Folders...\n\n')

  # Walk Through the Project Folders
  for root, dirs, files in os.walk(FOLDER):

    if "__pycache__" in dirs:

      # Get `__pycache__` Path
      pycache_folder = os.path.join(root, "__pycache__")
      if verbose: print(f"{pycache_folder}")

      # Delete `__pycache__`
      try: shutil.rmtree(pycache_folder)
      except Exception as e: print(f"An error occurred while deleting {pycache_folder}: {e}")

  if verbose: print('\n\n')

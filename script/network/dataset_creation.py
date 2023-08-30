import os, csv
import random

""" 

Labelling:

    0: Empty Command
    1: Place Object + Given Area (Voice Only)
    2: Place Object + Given Area + Point-At
    3: Place Object + Point-At
    4: Point-At (Gesture Only)
    5: Stop (Gesture Only)
    6: Pause (Gesture Only)
    7: Start Experiment (Voice Only)
    8: Object Moved (Voice Only)
    9: User Moved (Voice Only)
    10: User Can't Move (Voice Only)
    11: Replan Trajectory (Voice Only)
    12: Wait For Command (Voice Only)
    13: Can Go (Voice Only)
    14: Wait Time (Voice Only)

Voice Commands:

    0:  NULL
    1:  EXPERIMENT_START
    2:  MOVED_OBJECT
    3:  PUT_OBJECT_IN_AREA
    4:  PUT_OBJECT_IN_GIVEN_AREA
    5:  PUT_OBJECT_IN_AREA_GESTURE
    6:  USER_MOVED
    7:  USER_CANT_MOVE
    8:  REPLAN_TRAJECTORY
    9:  WAIT_FOR_COMMAND
    10: CAN_GO
    11: WAIT_TIME

Gestures:

    0: No Gesture
    1: Pause
    2: Point-At
    3: Stop

"""

# Dataset Parameters
n_dataset_row = 50001
last_scenario = 14

# Project Folder (ROOT Project Location)
PROJECT_FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

def writeCSV():

    with open(f'{PROJECT_FOLDER}/dataset/fusion_dataset.csv', 'w', newline='') as csv_file:

        # Prepare CSV Header
        csv_writer = csv.writer(csv_file, delimiter=',')
        csv_writer.writerow(["Command","Gesture","Voice"])

        for i in range(n_dataset_row):

            # Get Random Label
            label = random.randint(0, last_scenario)

            # 0: Empty Command
            if   label == 0: gesture, voice = 0, 0

            # 1: Place Object + Given Area (Voice Only)
            elif label == 1: gesture, voice = 0, 4

            # 2: Place Object + Given Area + Point-At
            elif label == 2: gesture, voice = 2, 4

            # 3: Place Object + Point-At
            elif label == 3: gesture, voice = 2, 5

            # 4: Point-At (Gesture Only)
            elif label == 4: gesture, voice = 2, 0

            # 5: Stop (Gesture Only)
            elif label == 5: gesture, voice = 3, 0

            # 6: Pause (Gesture Only)
            elif label == 6: gesture, voice = 1, 0

            # 7: Start Experiment (Voice Only)
            elif label == 7: gesture, voice = 0, 1

            # 8: Object Moved (Voice Only)
            elif label == 8: gesture, voice = 0, 2

            # 9: User Moved (Voice Only)
            elif label == 9: gesture, voice = 0, 6

            # 10: User Can't Move (Voice Only)
            elif label == 10: gesture, voice = 0, 7

            # 11: Replan Trajectory (Voice Only)
            elif label == 11: gesture, voice = 0, 8

            # 12: Wait For Command (Voice Only)
            elif label == 12: gesture, voice = 0, 9

            # 13: Can Go (Voice Only)
            elif label == 13: gesture, voice = 0, 10

            # 14: Wait Time (Voice Only)
            elif label == 14: gesture, voice = 0, 11

            # Write CSV Row
            csv_writer.writerow([label, gesture, voice])

        print('CSV Dataset Created')

if __name__ == '__main__':

    writeCSV()

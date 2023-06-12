import csv, re
import random

# codici sotto agli oggetti es pelati = 12

n_dataset_row = 50001
last_scenario = 34


def writeCSV():

    with open('/home/alberto/catkin_ws/src/fusion/dataset/fusion_dataset.csv', 'w', newline='') as csv_file:
        
        csv_writer = csv.writer(csv_file, delimiter=',')
        csv_writer.writerow(["Command","gesture", "voice"])

        for i in range(n_dataset_row):
        
            input_array = [random.randint(0, last_scenario)]                      
            Label = input_array[0]

            if Label == 0:   #Amen non ho preso nulla il timer è andato a caso 

                gesture = 0
                voice = 0 

            elif Label == 1:   #point at + prendi il pomodoro

                gesture = 1
                voice = 10

            elif Label == 2:   #point at + prendi il sale

                gesture = 1
                voice = 20

            elif Label == 3:   #point at + prendi la pasta

                gesture = 1
                voice = 30

            elif Label == 4:   #point at + prendi la forchetta

                gesture = 1
                voice = 40

            elif Label == 5:   #point at + prendi il bicchiere

                gesture = 1
                voice = 60
            
            elif Label == 6:   #0 + prendi il pomodoro

                gesture = 0
                voice = 10

            elif Label == 7:   #0 + prendi il sale

                gesture = 0
                voice = 20

            elif Label == 8:   #0 + prendi la pasta

                gesture = 0
                voice = 30

            elif Label == 9:   #0 + prendi la forchetta

                gesture = 0
                voice = 40

            elif Label == 10:   #0 + prendi il bicchiere

                gesture = 0
                voice = 60

            elif Label == 11:   #0 + prendi la passata

                gesture = 0
                voice = 11

            elif Label == 12:   #0 + prendi i pelati

                gesture = 0
                voice = 12

            elif Label == 13:   #0 + prendi il sale grosso

                gesture = 0
                voice = 21

            elif Label == 14:   #0 + prendi il sale iodato

                gesture = 0
                voice = 22

            elif Label == 15:   #0 + prendi le pennette

                gesture = 0
                voice = 31

            elif Label == 16:   #0 + prendi gli spaghetti

                gesture = 0
                voice = 32

            elif Label == 17:   #0 + prendi la forchetta di legno

                gesture = 0
                voice = 41

            elif Label == 18:   #0 + prendi la forchetta di plastica

                gesture = 0
                voice = 42

            elif Label == 19:   #0 + prendi il coltello

                gesture = 0
                voice = 51

            elif Label == 20:   #0 + prendi il bicchiere bianco

                gesture = 0
                voice = 61

            elif Label == 21:   #0 + prendi il bicchiere trasparente

                gesture = 0
                voice = 62

            elif Label == 22: #point at + prendi la passata

                gesture = 1
                voice = 11

            elif Label == 23: #point at + prendi i pelati

                gesture = 1
                voice = 12

            elif Label == 24: #point at + prendi il sale grosso

                gesture = 1
                voice = 21
            
            elif Label == 25: #point at + prendi il sale iodato

                gesture = 1
                voice = 22
            
            elif Label == 26: #point at + prendi le pennette

                gesture = 1
                voice = 31

            elif Label == 27: #point at + prendi gli spaghetti

                gesture = 1
                voice = 32

            elif Label == 28: #point at + prendi la forchetta di legno

                gesture = 1
                voice = 41

            elif Label == 29: #point at + prendi la forchetta di plastica

                gesture = 1
                voice = 42

            elif Label == 30:   #point at + prendi il coltello

                gesture = 1
                voice = 51

            elif Label == 31: #point at + prendi il bicchiere bianco

                gesture = 1
                voice = 61

            elif Label == 32: #point at + prendi il bicchiere trasparente

                gesture = 1
                voice = 62
            
            elif Label == 33: #point at + 0

                gesture = 1
                voice = 0

            elif Label == 34: #Point at + che oggetti ci sono lì

                gesture = 1
                voice = 100

            input_array.append(gesture)
            input_array.append(voice)
            csv_writer.writerow(input_array) 
        print('CSV file created')



if __name__ == '__main__':
     
    writeCSV()
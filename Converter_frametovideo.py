import cv2
import os

# Define the codec (non so cosa sia)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')

# Create the output folder if it does not exist
if not os.path.exists("Video_dataset_mp4"):
    os.makedirs("Video_dataset_mp4")

# Iterate through the subfolders in "Frame_gesti" (contiene tutte le cartelle con tutte le immagini)
for subfolder in os.listdir("Frame_gesti"):
    if os.path.isdir(os.path.join("Frame_gesti", subfolder)):

        # Define the input and output paths
        input_path = os.path.join("Video Minimi", subfolder)
        output_path = os.path.join("Video_dataset_mp4", subfolder + ".mp4")

        # Create VideoWriter object
        out = cv2.VideoWriter(output_path, fourcc, 12.0, (176,100))


        # Iterate through the images in the subfolder
        for filename in os.listdir(input_path):
            if filename.endswith('.jpg'):
                # Read the image
                img = cv2.imread(os.path.join(input_path, filename))
                # Write the image to the VideoWriter
                out.write(img)

        # Release the VideoWriter
        out.release()

print("Video creati")

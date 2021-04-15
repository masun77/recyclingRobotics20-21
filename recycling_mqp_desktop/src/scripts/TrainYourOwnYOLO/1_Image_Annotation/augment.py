import pandas as pd
from PIL import Image, ImageOps, ImageDraw
import csv

# Read the name of your annotations csv in vott format - replace Annotations-export with your filename
df = pd.read_csv('Annotations-export.csv')
tocsv = []

# Open the csv to write to - replace augmentedAnnotations.csv with your filename
# could also change it to appending to the original annotations csv
file = open('augmentedAnnotations.csv', 'w+', newline='')
dict = {}

# The name of the folder the images are in and will be saved to
img_folder = "aug_images/"

# For each image, do 5 augmentations: flip vertically, mirror horizontally, and rotate 90, 180, 280 degrees
for image in range(df.shape[0]):
    for i in range(5):
        tocsv.append(["","","","",""])

    name = df.iloc[image][0]
    xMin = df.iloc[image][1]
    yMin = df.iloc[image][2]
    xMax = df.iloc[image][3]
    yMax = df.iloc[image][4]

    firsttime = False
    if (not name in dict) : # only save each reflected/rotated image once, regardless of how many bounding boxes it has
        firsttime = True
        dict[name] = name

    im = Image.open(img_folder + name)
    width, height = im.size

    im2 = im.rotate(90, expand = True)    # (yMin, width - xMax), (yMax, width - xMin)
    tocsv[image * 5][0] = "r90_" + name
    tocsv[image * 5][1] = yMin
    tocsv[image * 5][2] = width - xMax
    tocsv[image * 5][3] = yMax
    tocsv[image * 5][4] = width - xMin
    if firsttime:
        im2.save(img_folder + "r90_" + name, "JPEG")

    im2 = im.rotate(180, expand = True)    # (width - xMax, height - yMax), (width - xMin, height - yMin)
    tocsv[image * 5 + 1][0] = "r180_"+ name
    tocsv[image * 5 + 1][1] = width - xMax
    tocsv[image * 5 + 1][2] = height - yMax
    tocsv[image * 5 + 1][3] = width - xMin
    tocsv[image * 5 + 1][4] = height - yMin
    if firsttime:
        im2.save(img_folder + "r180_" + name, "JPEG")

    im2 = im.rotate(270, expand = True)    # (height - yMax, xMin), (height - yMin, xMax)
    tocsv[image * 5 + 2][0] = "r270_" + name
    tocsv[image * 5 + 2][1] = height - yMax
    tocsv[image * 5 + 2][2] = xMin
    tocsv[image * 5 + 2][3] = height - yMin
    tocsv[image * 5 + 2][4] = xMax
    if firsttime:
        im2.save(img_folder + "r270_" + name, "JPEG")

    im2 = ImageOps.flip(im)    # xMin, heigh - yMax; xMax, heigh - yMin
    tocsv[image * 5 + 3][0] = "flip_" + name
    tocsv[image * 5 + 3][1] = xMin
    tocsv[image * 5 + 3][2] = height - yMax
    tocsv[image * 5 + 3][3] = xMax
    tocsv[image * 5 + 3][4] = height - yMin
    if firsttime:
        im2.save(img_folder + "flip_" + name, "JPEG")

    im2 = ImageOps.mirror(im)  # width - xMax, yMin; width - xMin, yMax
    tocsv[image * 5 + 4][0] = "mirror_" + name
    tocsv[image * 5 + 4][1] = width - xMax
    tocsv[image * 5 + 4][2] = yMin
    tocsv[image * 5 + 4][3] = width - xMin
    tocsv[image * 5 + 4][4] = yMax
    if firsttime:
        im2.save(img_folder + "mirror_" + name, "JPEG")

# writing the data into the file
with file:
    write = csv.writer(file)
    write.writerows(tocsv)

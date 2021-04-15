import pandas as pd

# Read the ground-truth csv into .txt files for each image
def readgt():
    df = pd.read_csv('gt.csv')   # name of ground-truth annotations csv
    names = {}

    # assume the csv is in the vott format: name, xmin, ymin, xmax, ymax, label
    for image in range(df.shape[0]):
        name = df.iloc[image][0][:-5]
        xMin = str(df.iloc[image][1])
        yMin = str(df.iloc[image][2])
        xMax = str(df.iloc[image][3])
        yMax = str(df.iloc[image][4])
        label = 'cardboard'
        img = [label, xMin, yMin, xMax, yMax]

        if (not name in names) :
            names[name] = []
        names[name].append(img)

    for imgName in names:
        file = open("input/ground-truth/" + imgName + ".txt", 'w')
        contents = names[imgName]
        for c in contents:
            file.write(c[0] + " " + c[1] + " " + c[2] + " "
                       + c[3] + " " + c[4] + " "
                       + "\n")
        file.close()

# Read the detections csv into text files
def read_detections():
    df = pd.read_csv('gt.csv')  # name of detections annotations csv
    names = {}

    # assume the csv is in the Detector format: name, path, xmin, ymin, xmax, ymax, label, confidence
    for image in range(df.shape[0]):
        name = df.iloc[image][0][:-5]
        xMin = str(df.iloc[image][1])
        yMin = str(df.iloc[image][2])
        xMax = str(df.iloc[image][3])
        yMax = str(df.iloc[image][4])
        label = 'cardboard'
        confidence = str(df.iloc[image][7])
        img = [label, confidence, xMin, yMin, xMax, yMax]

        if (not name in names):
            names[name] = []
        names[name].append(img)

    for imgName in names:
        file = open("input/detection-results/" + imgName + ".txt", 'w')
        contents = names[imgName]
        for c in contents:
            file.write(c[0] + " " + c[1] + " " + c[2] + " "
                       + c[3] + " " + c[4] + " "
                       + c[5]
                       + "\n")
        file.close()

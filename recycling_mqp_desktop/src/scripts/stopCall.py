# Change the value in the state.txt file to not 'y' so the robot
# while loop reading from the file stops
with open('state.txt', 'w') as file:
    file.write("n")
    file.close()
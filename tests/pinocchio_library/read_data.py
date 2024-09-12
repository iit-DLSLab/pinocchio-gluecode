import numpy as np

import matplotlib.pyplot as plt

numbers = []

# Open the file in read mode
file_path = ""
file_name = ""
with open(file_path+file_name+".txt", 'r') as file:
    # Read each line and convert it to a number
    for line in file:
        number = float(line.strip())
        numbers.append(number)

    # Compute the mean value of numbers
    mean_value = np.mean(numbers)

    # Plot the variable numbers
    plt.plot(numbers)
    plt.plot([mean_value] * len(numbers), color='red', linestyle='--', label='Mean')
    plt.title(file_name)
    plt.xlabel('Sample')
    plt.ylabel('Computation time (ms)')
    plt.legend()
    plt.savefig(file_path+file_name+".png")
    plt.show()
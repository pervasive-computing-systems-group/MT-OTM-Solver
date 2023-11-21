import errno
import math
import os
import random

FILE_PATH = "Test_Set_V/"
NUM_PLOTS = 50

# Fixed Plot Values
FIXED_PLOTS = False
FIXED_PLOT_SHIP_VELOCITY = 2.5
FIXED_PLOT_INCREMENT = 20
FIXED_PLOT_MIN_POINTS = 60


# # Test-set 2
# VELOCITY_FIXED = 2.5
# UPPER_X_LIMIT = 0
# LOWER_X_LIMIT = -50
# UPPER_Y_LIMIT = 500
# LOWER_Y_LIMIT = -500
# STEP_SIZE = 250

# Test-set 3 (I think this is the original data set)
# VELOCITY_FIXED = 2.5
# UPPER_X_LIMIT = 0
# LOWER_X_LIMIT = -50
# UPPER_Y_LIMIT = 200
# LOWER_Y_LIMIT = -200
# STEP_SIZE = 80

# Test-set 4 (Test_Set_III)
# VELOCITY_FIXED = 2.5
# UPPER_X_LIMIT = 1280
# LOWER_X_LIMIT = 1200
# UPPER_Y_LIMIT = 200
# LOWER_Y_LIMIT = -200
# STEP_SIZE = 80

# Test-set 5 (Test_Set_IV)
# VELOCITY_FIXED = 2.5
# UPPER_X_LIMIT = 0
# LOWER_X_LIMIT = -80
# UPPER_Y_LIMIT = 200
# LOWER_Y_LIMIT = -200
# STEP_SIZE = 16

# Test-set 5 (Test_Set_V)
VELOCITY_FIXED = 2.5
UPPER_X_LIMIT = 1280
LOWER_X_LIMIT = 1200
UPPER_Y_LIMIT = 200
LOWER_Y_LIMIT = -200
STEP_SIZE = 16


def rand_plot(num_points, file):
    # Generate a plot of randomly place points within the defined limits
    for points in range(0, num_points):
        x_coord = random.randrange(LOWER_X_LIMIT, UPPER_X_LIMIT)
        y_coord = random.randrange(LOWER_Y_LIMIT, UPPER_Y_LIMIT)

        file.write(f"{x_coord} {y_coord}\n")


def main(num_plots, num_points):
    if not os.path.exists(os.path.dirname(FILE_PATH)):
        try:
            os.makedirs(os.path.dirname(FILE_PATH))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

    # Generate num_plots of completely random graphs
    for i in range(num_plots):
        file_name = f"{FILE_PATH}plot_{num_points}_{i}.txt"
        with open(file_name, 'w') as file:
            file.write(f"{num_points}\n")
            rand_plot(num_points, file)
            file.write("0.0 0.0\n")
            file.write(f"{VELOCITY_FIXED} 0.0")


if __name__ == '__main__':
    for x in range(30):
        # n = 5 + 5*x
        n = 3 + x
        UPPER_X_LIMIT += STEP_SIZE
        print(n, UPPER_X_LIMIT)
        main(NUM_PLOTS, n)

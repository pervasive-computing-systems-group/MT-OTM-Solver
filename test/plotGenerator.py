import errno
import math
import os
import random

FILE_PATH = "Test_Set_03/"
RADIUS = 30.0
NUM_PLOTS = 50
N = 25

# Fixed Plot Values
FIXED_PLOTS = False
FIXED_PLOT_SHIP_VELOCITY = 2.5
FIXED_PLOT_INCREMENT = 20
FIXED_PLOT_MIN_POINTS = 60

# Random Plot Values
TRANSLATION_MIN = 50
TRANSLATION_MAX = 400
RADIUS_MIN = 10
RADIUS_MAX = 50
VELOCITY_MAX = 7.50

# Easier parameters (incase the above doesn't play well)
# TRANSLATION_MIN = 40
# TRANSLATION_MAX = 100
# RADIUS_MIN = 10
# RADIUS_MAX = 25
# VELOCITY_MAX = 2.0

# # Test-set 2
# VELOCITY_FIXED = 2.5
# UPPER_X_LIMIT = 0
# LOWER_X_LIMIT = -50
# UPPER_Y_LIMIT = 500
# LOWER_Y_LIMIT = -500
# STEP_SIZE = 250

# Test-set 3
COMPLETELY_RANDOM = True
VELOCITY_FIXED = 2.5
UPPER_X_LIMIT = 0
LOWER_X_LIMIT = -50
UPPER_Y_LIMIT = 200
LOWER_Y_LIMIT = -200
STEP_SIZE = 80


def normal_plot(num_points, radius_coverage, file):
    # Calculates the Random Parameters #
    horizontal_offset = 150
    horizontal_direction = 1
    vertical_offset = 150
    orientation_offset = 0

    # Calculate the Grid Length to Width Ratio#
    square_ratio = round(num_points ** 0.5)
    # ratio_threshold = round(square_ratio * 0.25)
    # if ratio_threshold < 1:
    #     ratio_threshold = 1

    number_of_cols = square_ratio # random.randrange(square_ratio - ratio_threshold, square_ratio + ratio_threshold)
    current_col = 0
    current_row = 0

    for points in range(0, num_points):
        if current_col == number_of_cols:
            current_col = 0
            current_row += 1

        x_coord = (current_col * radius_coverage * math.cos(math.radians(orientation_offset)))  # Equation for x-coord
        y_coord = (current_col * radius_coverage * math.sin(math.radians(orientation_offset)))  # Equation for y-coord

        x_coord += (current_row * (radius_coverage * (3 ** 0.5) / 2) * math.cos(math.radians(orientation_offset + 90)))
        y_coord += (current_row * (radius_coverage * (3 ** 0.5) / 2) * math.sin(math.radians(orientation_offset + 90)))

        if current_row % 2 == 1:  # Odd Row
            x_coord += radius_coverage * 0.5 * math.cos(math.radians(orientation_offset))
            y_coord += radius_coverage * 0.5 * math.sin(math.radians(orientation_offset))

        x_coord += horizontal_offset * horizontal_direction
        y_coord += vertical_offset

        file.write(f"{x_coord} {y_coord}\n")
        current_col += 1


def rectangular(num_points, radius_coverage, file):
    # Calculates the Random Parameters #
    horizontal_offset = random.randrange(TRANSLATION_MIN, TRANSLATION_MAX)
    horizontal_direction = random.randrange(-1, 2, 2)  # Gets a direction randomly (-1 or 1)
    vertical_offset = random.randrange(TRANSLATION_MIN, TRANSLATION_MAX)
    orientation_offset = random.randrange(0, 90)

    # Calculate the Grid Length to Width Ratio#
    square_ratio = round(num_points ** 0.5)
    ratio_threshold = round(square_ratio * 0.25)
    if ratio_threshold < 1:
        ratio_threshold = 1

    number_of_cols = random.randrange(square_ratio - ratio_threshold, square_ratio + ratio_threshold)
    current_col = 0
    current_row = 0

    for points in range(0, num_points):
        if current_col == number_of_cols:
            current_col = 0
            current_row += 1

        x_coord = (current_col * radius_coverage * math.cos(math.radians(orientation_offset)))  # Equation for x-coord
        y_coord = (current_col * radius_coverage * math.sin(math.radians(orientation_offset)))  # Equation for y-coord

        x_coord += (current_row * (radius_coverage * (3 ** 0.5) / 2) * math.cos(math.radians(orientation_offset + 90)))
        y_coord += (current_row * (radius_coverage * (3 ** 0.5) / 2) * math.sin(math.radians(orientation_offset + 90)))

        if current_row % 2 == 1:  # Odd Row
            x_coord += radius_coverage * 0.5 * math.cos(math.radians(orientation_offset))
            y_coord += radius_coverage * 0.5 * math.sin(math.radians(orientation_offset))

        x_coord += horizontal_offset * horizontal_direction
        y_coord += vertical_offset

        file.write(f"{x_coord} {y_coord}\n")
        current_col += 1


def triangular(num_points, radius_coverage, file):
    # Calculates the Random Parameters
    horizontal_offset = random.randrange(TRANSLATION_MIN, TRANSLATION_MAX)
    horizontal_direction = random.randrange(-1, 2, 2)  # Gets a direction randomly (-1 or 1)
    vertical_offset = random.randrange(TRANSLATION_MIN, TRANSLATION_MAX)
    orientation_offset = random.randrange(0, 90)

    current_col = 0
    current_row = 0

    points_per_row = 1
    point_per_row_increase = random.randrange(1, 5)

    for points in range(0, num_points):
        if current_col == points_per_row:
            current_col = 0
            current_row += 1
            points_per_row += point_per_row_increase

        x_coord = (current_col * radius_coverage * math.cos(math.radians(orientation_offset)))  # Equation for x-coord
        y_coord = (current_col * radius_coverage * math.sin(math.radians(orientation_offset)))  # Equation for y-coord

        x_coord += (current_row * (radius_coverage * (3 ** 0.5) / 2) * math.cos(math.radians(orientation_offset + 90)))
        y_coord += (current_row * (radius_coverage * (3 ** 0.5) / 2) * math.sin(math.radians(orientation_offset + 90)))

        if current_row % 2 == 1:  # Odd Row
            x_coord += radius_coverage * 0.5 * math.cos(math.radians(orientation_offset))
            y_coord += radius_coverage * 0.5 * math.sin(math.radians(orientation_offset))

        x_coord += horizontal_offset * horizontal_direction
        y_coord += vertical_offset

        file.write(f"{x_coord} {y_coord}\n")
        current_col += 1


def rand_plot(num_points, file):
    # Generate a plot of randomly place points within the defined limits
    for points in range(0, num_points):
        x_coord = random.randrange(LOWER_X_LIMIT, UPPER_X_LIMIT)
        y_coord = random.randrange(LOWER_Y_LIMIT, UPPER_Y_LIMIT)

        file.write(f"{x_coord} {y_coord}\n")


def n_faced(num_points, radius_coverage, file):
    n = random.randrange(3, 10)

    horizontal_offset = random.randrange(TRANSLATION_MIN, TRANSLATION_MAX)
    horizontal_direction = random.randrange(-1, 2, 2)  # Gets a direction randomly (-1 or 1)
    vertical_offset = random.randrange(TRANSLATION_MIN, TRANSLATION_MAX)
    orientation_offset = random.randrange(0, 90)

    current_col = 0
    current_row = 0

    points_per_row = 1
    point_per_row_increase = random.randrange(1, 5)


def incremental_plots(num_plots, radius_coverage):
    for i in range(num_plots):
        num_points = FIXED_PLOT_MIN_POINTS + (i * FIXED_PLOT_INCREMENT)
        file_name = f"{FILE_PATH}plot_{num_points}.txt"

        with open(file_name, 'w') as file:
            file.write(f"{num_points} {radius_coverage}\n")

            normal_plot(num_points, radius_coverage, file)

            file.write("0.0 0.0\n")
            file.write(f"{FIXED_PLOT_SHIP_VELOCITY} 0.0")


def main(num_plots, num_points):
    if not os.path.exists(os.path.dirname(FILE_PATH)):
        try:
            os.makedirs(os.path.dirname(FILE_PATH))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

    if FIXED_PLOTS:
        incremental_plots(num_plots, RADIUS)

    elif COMPLETELY_RANDOM:
        # Generate num_plots of completely random graphs
        for i in range(num_plots):
            file_name = f"{FILE_PATH}plot_{num_points}_{i}.txt"
            with open(file_name, 'w') as file:
                file.write(f"{num_points} {RADIUS_MAX}\n")
                rand_plot(num_points, file)
                file.write("0.0 0.0\n")
                file.write(f"{VELOCITY_FIXED} 0.0")

    else:
        for i in range(num_plots):
            file_name = f"{FILE_PATH}plot_{num_points}_{i}.txt"
            shape = random.randrange(0, 2)
            radius_coverage = random.randrange(RADIUS_MIN, RADIUS_MAX)

            with open(file_name, 'w') as file:
                file.write(f"{num_points} {radius_coverage}\n")

                if shape == 0:  # Rectangular Creation
                    rectangular(num_points, radius_coverage, file)
                elif shape == 1:  # Triangular Creation
                    triangular(num_points, radius_coverage, file)

                file.write("0.0 0.0\n")
                file.write(f"{round(random.uniform(00.00, VELOCITY_MAX), 2)} 0.0")


if __name__ == '__main__':
    UPPER_X_LIMIT = 0
    for x in range(30):
        n = 5 + 5*x
        UPPER_X_LIMIT += STEP_SIZE
        print(n, UPPER_X_LIMIT)
        main(NUM_PLOTS, n)
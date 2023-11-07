#This file takes the plot and the forest and creates a graphical representation of them. User will be asked for file paths

import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter
from IPython import display
from matplotlib import rc
rc('font',**{'family':'serif','serif':['Times']})
rc('text', usetex=True)

do_animation = False
#Uncomment to use in a notebook
#%matplotlib notebook

#Use this to change the step size
step_size = 1

plot_file = open("graph_output.txt")
plot_raw = plot_file.readlines()
plot_file.close()

xs = []
ys = []
for n in plot_raw:
    temp = n.split()
    xs.append(float(temp[0]))
    ys.append(float(temp[1]))

m = int(xs.pop(0))
n = int(ys.pop(0))


def calculate_distance(x1,y1,x2,y2):
    return ((x2 - x1)**2 + (y2 - y1)**2)**.5


# Un-comment the following to print min-spanning tree
# forest_file = open("forest_output.txt")
# forest_raw = forest_file.readlines()
# forest_file.close()
#
# start = []
# end = []
# for i in forest_raw:
#     temp = i.split()
#     start.append(int(temp[0]))
#     end.append(int(temp[1]))
#
# fig2, forest2 = plt.subplots()
# forest2.scatter(xs[0:n],ys[0:n])
# forest2.scatter(xs[n:n+m], ys[n:n+m],marker = "s")
# forest2.scatter(xs[n+m:], ys[n+m:], marker = "D")

# for i in range(0, len(start)):
#     tempx = [xs[start[i]], xs[end[i]]]
#     tempy = [ys[start[i]], ys[end[i]]]
#     forest2.plot(tempx, tempy)

path_file = open("path_output.txt")
path_raw = path_file.readlines()
path_file.close()

start_path = []
end_path = []
for i in path_raw:
    temp = i.split()
    start_path.append(int(temp[0]))
    end_path.append(int(temp[1]))

fig1, forest1 = plt.subplots()
forest1.scatter(xs[0:n],ys[0:n])
forest1.scatter(xs[n:n+m], ys[n:n+m],marker = "s")
forest1.scatter(xs[n+m:], ys[n+m:], marker = "D")

plt.xlabel('$\it{x}$ coordinate ($\it{meters}$)')
plt.ylabel('$\it{y}$ coordinate ($\it{meters}$)')

# forest1.title('Problem Setup')
# forest1.xlabel('$\it{m}$')
# forest1.ylabel('$\it{m}$')

fig3, forest3 = plt.subplots()
forest3.scatter(xs[0:n],ys[0:n])
forest3.scatter(xs[n:n+m], ys[n:n+m],marker = "s")
forest3.scatter(xs[n+m:], ys[n+m:], marker = "D")

for i in range(0,len(start_path)):
    tempx = [xs[start_path[i]],xs[end_path[i]]]
    tempy = [ys[start_path[i]],ys[end_path[i]]]
    forest3.plot(tempx,tempy, color ="red")

fig4, forest4 = plt.subplots()
forest4.scatter(xs[0:n],ys[0:n])
forest4.scatter(xs[n:n+m], ys[n:n+m],marker = "s")
forest4.scatter(xs[n+m:], ys[n+m:], marker = "D")

carrier_start_xs = xs[n+m:]
carrier_start_ys = ys[n+m:]
carrier_end_xs = xs[n:n+m]
carrier_end_ys = ys[n:n+m]


for i in range(0,len(start_path)):
    tempx = [xs[start_path[i]],xs[end_path[i]]]
    tempy = [ys[start_path[i]],ys[end_path[i]]]
    forest4.plot(tempx,tempy, color ="red")
    
for i in range(0,len(carrier_start_xs)):
    tempx = carrier_start_xs[i]
    tempy = carrier_start_ys[i]
    tempdx = carrier_end_xs[i] - carrier_start_xs[i]
    tempdy = carrier_end_ys[i] - carrier_start_ys[i]
    forest4.arrow(tempx,tempy,tempdx,tempdy, color ="black", linestyle = 'dashed',length_includes_head=True,
          head_width=0.1, head_length=0.1)
    
for i in range(0, len(carrier_end_xs)-1):
    tempx = carrier_end_xs[i]
    tempy = carrier_end_ys[i]
    tempdx = carrier_start_xs[i+1] - carrier_end_xs[i]
    tempdy = carrier_start_ys[i+1] - carrier_end_ys[i]
    forest4.arrow(tempx,tempy,tempdx,tempdy, color ="black", linestyle = 'dashed',length_includes_head=True,
          head_width=0.1, head_length=0.1)


for i in range(0,len(carrier_start_xs)):
    tempx = carrier_start_xs[i]
    tempy = carrier_start_ys[i]
    tempdx = carrier_end_xs[i] - carrier_start_xs[i]
    tempdy = carrier_end_ys[i] - carrier_start_ys[i]
    forest3.arrow(tempx,tempy,tempdx,tempdy, color ="black", linestyle = 'dashed',length_includes_head=True,
          head_width=0.1, head_length=0.1)
    
for i in range(0, len(carrier_end_xs)-1):
    tempx = carrier_end_xs[i]
    tempy = carrier_end_ys[i]
    tempdx = carrier_start_xs[i+1] - carrier_end_xs[i]
    tempdy = carrier_start_ys[i+1] - carrier_end_ys[i]
    forest3.arrow(tempx,tempy,tempdx,tempdy, color ="black", linestyle = 'dashed',length_includes_head=True,
          head_width=0.1, head_length=0.1)

if(do_animation):        
    carrier_length = []      
    for i in range(len(carrier_start_xs)):
        carrier_length.append(((carrier_end_xs[i] - carrier_start_xs[i])**2 + (carrier_end_ys[i] - carrier_start_ys[i])**2)**.5)
        if(i < len(carrier_start_xs) - 1):
            carrier_length.append(((carrier_end_xs[i] - carrier_start_xs[i+1])**2 + (carrier_end_ys[i] - carrier_start_ys[i+1])**2)**.5)

    num_steps = []
    for i in range(len(carrier_length)):
        num_steps.append(math.ceil(carrier_length[i]/step_size))



    carrier_waypoints_xs = []
    carrier_waypoints_ys = []



    for i in range(len(carrier_start_xs)):
        carrier_waypoints_xs.append(carrier_start_xs[i])
        carrier_waypoints_ys.append(carrier_start_ys[i])
        carrier_waypoints_xs.append(carrier_end_xs[i])
        carrier_waypoints_ys.append(carrier_end_ys[i])

    adjacencies = []
    for i in range(len(xs)):
        adjacencies.append([])

    for (starts, stops) in zip(start_path, end_path):
        adjacencies[starts].append(stops)
        adjacencies[stops].append(starts)



    starting_points = []
    for i in range(m+n, len(xs)):
        starting_points.append(i)


    UAV_waypoints_xs = []
    UAV_waypoints_ys = []

    UAV_partitions = []



    for start in starting_points:

        partition_count = 0
        previous_point = 0
        next_point = start
        cont = True


        while cont:
            partition_count+=1
            UAV_waypoints_xs.append(xs[next_point])
            UAV_waypoints_ys.append(ys[next_point])
            previous_point = next_point
            next_point = adjacencies[previous_point][0]
            adjacencies[previous_point].pop(0)
            adjacencies[next_point].pop(adjacencies[next_point].index(previous_point))

            if (len(adjacencies[next_point]) == 0):
                cont = False
                UAV_waypoints_xs.append(xs[next_point])
                UAV_waypoints_ys.append(ys[next_point])
                partition_count+=1

        UAV_partitions.append(partition_count)


    UAV_length = []


    temp_length = 0
    j = 1
    k = 0

    for i in range(1,len(UAV_waypoints_xs)):
        temp_length += calculate_distance(UAV_waypoints_xs[i], UAV_waypoints_ys[i], UAV_waypoints_xs[i-1], UAV_waypoints_ys[i-1])
        j+=1
        if(j == UAV_partitions[k]):
            UAV_length.append(temp_length)
            j = 0
            k += 1
            temp_length = 0

    true_length = 0
    for i in range(1, len(UAV_waypoints_xs)):
        true_length+= calculate_distance(UAV_waypoints_xs[i], UAV_waypoints_ys[i], UAV_waypoints_xs[i-1], UAV_waypoints_ys[i-1])




    UAV_step_size = []
    for i in range(0,len(carrier_length),2):
        length = UAV_length[math.floor(i/2)]
        UAV_step_size.append(length /num_steps[i])





    carrier_xs = [carrier_waypoints_xs[0]]
    carrier_ys = [carrier_waypoints_ys[0]] 

    next_UAV_point = 0
    UAV_xs = [UAV_waypoints_xs[next_UAV_point]]
    UAV_ys = [UAV_waypoints_ys[next_UAV_point]]
    next_carrier = 0


    for i in range(len(num_steps)): 
        next_carrier +=1
        #print("Carrier: ", next_carrier)
        #print()
        next_UAV_point += 1
        #print("Next UAV point: ", next_UAV_point,UAV_waypoints_xs[next_UAV_point],UAV_waypoints_ys[next_UAV_point] )
        for j in range(num_steps[i]):
            #print("Step Number: ", j)
            #Generate carrier coordinates
            carrier_ys.append(carrier_ys[0])
            remaining_carrier_distance = carrier_waypoints_xs[i+1] - carrier_xs[len(carrier_xs)-1]
            if(remaining_carrier_distance<=step_size):
                carrier_xs.append(carrier_waypoints_xs[i+1])
                #print("Reached end")
            else:
                carrier_xs.append(carrier_xs[len(carrier_xs)-1] + step_size)

            #Generate UAV coordinates       
            if(i%2 != 0):
                UAV_xs.append(carrier_xs[len(carrier_xs)-1])
                UAV_ys.append(carrier_ys[len(carrier_ys)-1])
            else:
                previous_point = (UAV_xs[len(UAV_xs)-1], UAV_ys[len(UAV_ys)-1])
                remaining_UAV_distance = calculate_distance(previous_point[0], previous_point[1], UAV_waypoints_xs[next_UAV_point], UAV_waypoints_ys[next_UAV_point])
                #print("Remaining distance, Step size: ", remaining_UAV_distance, UAV_step_size[math.floor(i/2)])
                if(remaining_UAV_distance < UAV_step_size[math.floor(i/2)]):
                    if(next_UAV_point + 1 < len(UAV_waypoints_xs)):
                        ratio = (UAV_step_size[math.floor(i/2)] - remaining_UAV_distance)/calculate_distance(UAV_waypoints_xs[next_UAV_point], UAV_waypoints_ys[next_UAV_point],UAV_waypoints_xs[next_UAV_point+1], UAV_waypoints_ys[next_UAV_point+1])
                        step_x = (UAV_waypoints_xs[next_UAV_point+1] - UAV_waypoints_xs[next_UAV_point])*ratio
                        step_y = (UAV_waypoints_ys[next_UAV_point+1]- UAV_waypoints_ys[next_UAV_point])*ratio
                        UAV_xs.append(UAV_waypoints_xs[next_UAV_point]+step_x)
                        UAV_ys.append(UAV_waypoints_ys[next_UAV_point]+step_y)
                        #print("ratio: ",ratio)
                    else:
                        UAV_xs.append(UAV_waypoints_xs[next_UAV_point])
                        UAV_ys.append(UAV_waypoints_ys[next_UAV_point])
                    next_UAV_point += 1
                    #print("Next UAV point: ", next_UAV_point ,UAV_waypoints_xs[next_UAV_point],UAV_waypoints_ys[next_UAV_point], calculate_distance(UAV_waypoints_xs[next_UAV_point-1], UAV_waypoints_ys[next_UAV_point-1],UAV_waypoints_xs[next_UAV_point], UAV_waypoints_ys[next_UAV_point]))

                elif(remaining_UAV_distance == UAV_step_size[math.floor(i/2)]):
                    UAV_xs.append(UAV_waypoints_xs[next_UAV_point])
                    UAV_ys.append(UAV_waypoints_ys[next_UAV_point])
                    next_UAV_point += 1
                    #print("Next UAV point: ", next_UAV_point ,UAV_waypoints_xs[next_UAV_point],UAV_waypoints_ys[next_UAV_point], calculate_distance(UAV_waypoints_xs[next_UAV_point-1], UAV_waypoints_ys[next_UAV_point-1],UAV_waypoints_xs[next_UAV_point], UAV_waypoints_ys[next_UAV_point]) )

                else:
                    ratio = UAV_step_size[math.floor(i/2)]/remaining_UAV_distance
                    step_x = (UAV_waypoints_xs[next_UAV_point] - UAV_xs[len(UAV_xs)-1])*ratio
                    step_y = (UAV_waypoints_ys[next_UAV_point]- UAV_ys[len(UAV_ys)-1])*ratio
                    UAV_xs.append(UAV_xs[len(UAV_xs)-1]+step_x)
                    UAV_ys.append(UAV_ys[len(UAV_ys)-1]+step_y)
                    #print("ratio: ",ratio)

    def add_background ():
        forest3.scatter(xs[0:n],ys[0:n])
        forest3.scatter(xs[n:n+m], ys[n:n+m],marker = "s")
        forest3.scatter(xs[n+m:], ys[n+m:], marker = "D")

        for i in range(0,len(start_path)):
            tempx = [xs[start_path[i]],xs[end_path[i]]]
            tempy = [ys[start_path[i]],ys[end_path[i]]]
            forest3.plot(tempx,tempy, color ="red")

        for i in range(0,len(carrier_start_xs)):
            tempx = carrier_start_xs[i]
            tempy = carrier_start_ys[i]
            tempdx = carrier_end_xs[i] - carrier_start_xs[i]
            tempdy = carrier_end_ys[i] - carrier_start_ys[i]
            forest3.arrow(tempx,tempy,tempdx,tempdy, color ="black", linestyle = 'dashed',length_includes_head=True,
              head_width=0.1, head_length=0.1)

        for i in range(0, len(carrier_end_xs)-1):
            tempx = carrier_end_xs[i]
            tempy = carrier_end_ys[i]
            tempdx = carrier_start_xs[i+1] - carrier_end_xs[i]
            tempdy = carrier_start_ys[i+1] - carrier_end_ys[i]
            forest3.arrow(tempx,tempy,tempdx,tempdy, color ="black", linestyle = 'dashed',length_includes_head=True,
                  head_width=0.1, head_length=0.1)






    def animate (i):
        forest3.clear()
        add_background()

        forest3.scatter(carrier_xs[i], carrier_ys[i], s=70, marker = "8")
        forest3.scatter(UAV_xs[i], UAV_ys[i],s=70, marker = "v")
        #return line, time_text, energy_text

    animation_frames = 0

    for num in num_steps:
        animation_frames += num

    from time import time
    dt = 1./60 # 60 fps
    t0 = time()
    animate(0)
    t1 = time()
    interval = 1000 * dt - (t1 - t0)



    ani = animation.FuncAnimation(fig3, animate, frames=animation_frames,
                                  interval=interval, blit=False)



    ani.save("movie.gif", writer=PillowWriter(fps=24))

plt.xlabel('$\it{x}$ coordinate ($\it{meters}$)')
plt.ylabel('$\it{y}$ coordinate ($\it{meters}$)')

plt.show()

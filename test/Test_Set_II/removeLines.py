import sys
print (sys.argv[1])

try:
    file = open(sys.argv[1])
except IOError:
    print("Failed to read file.")
lines = file.readlines()
countLines = len(lines)
#print(countLines)
#print(lines)

#lines = lines[:-2]
#print(lines)

f = open("temp/"+sys.argv[1], "w")
for i in range(len(lines)-2):
	f.write(lines[i])
f.write("0\n")
f.write(lines[len(lines)-2])
f.write(lines[len(lines)-1])
f.close()


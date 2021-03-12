import matplotlib.pyplot as plt
import sys
import random

def main():
    filename = sys.argv[1]
    file_handle = open(filename,'r')
    lines_list = file_handle.readlines();
    lines_list = lines_list[2:]
    lines_list_stripped = []

    colors = ['r','g','b','c','m']
    col_it = iter(colors)

    for line in lines_list:
        lines_list_stripped.append(line.strip().rstrip(', \n'))

    data = []
    for line in lines_list_stripped:
        data_line = [0,0]
        for pair in line.split(", "):
            vals = pair.split()
            data_line.append(float(vals[0]))
            data_line.append(float(vals[1]))
        data_line.append(0)
        data_line.append(0)
        data.append(data_line)

   #data = [[float(val) for val in line.split(", ")] for line in lines_list_stripped[3:]]

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)


    for line in data:
        it = iter(line)
        x_coords = []
        y_coords = []

        for i in range(int(len(line) / 2)):
            x_coords.append(next(it))
            y_coords.append(next(it))

        r = random.random()
        b = random.random()
        g = random.random()
        color = next(col_it)

        ax.plot(x_coords, y_coords, 'k.')
        ax.plot(x_coords, y_coords, '-', color=color)

        ax.set_ylabel('Lattitude [m]')
        ax.set_xlabel('Longitude [m]')

    fig.savefig('TestPdf.pdf')
    fig.savefig('TestEps.eps')

    ax.plot(0, 0, 'rx')
    #subprocess.check_call(['epstopdf', 'TestEps.eps'])
    plt.show()

if __name__ == "__main__":
    main()
import numpy as np
from ast import literal_eval

filename = '../../results/IMDP_gridPlot_2.py'
filename_write = '../../results/IMDP_gridPlotData_2.csv'
f = open(filename)
h = open(filename_write,'w')

patch_counter = 0;
ready_to_write = False

for line in f:
    line = line.strip()
    if line.startswith('z '):
        data = line.split(' = ');
        prob = eval(data[1])
    if line.startswith('v'):
        data = line.split('=')
        vertex_coordinate = eval(data[1])
        if data[0].endswith('0'):
            vertex_tuple_x = np.sum(vertex_coordinate)/4
            ready_to_write = False

        if data[0].endswith('1'):
            vertex_tuple_y = np.sum(vertex_coordinate)/4
            ready_to_write = True

        if ready_to_write:
            h.write("{0:.4f}".format(vertex_tuple_x) + ', '\
                    + "{0:.4f}".format(vertex_tuple_y) + ', '\
                    + "{0:.4f}".format(prob[patch_counter]) + '\n')
            patch_counter += 1
f.close()
h.close()

# This reads all the py files in a folder, extracts the z value,
# and tabulates the volume.
import numpy as np
from os import walk


mypath = './Figure9_stochy/'
python_files = []
walk_tuple = [v for v in walk(mypath)][0]
d = 0
cov_list = [0.05, 0.1, 0.2, 0.5]
grid_step_size_list = [1, 0.2, 0.15]
probability_threshold = 0.8
safety_set_bound = 3
volume_safe_set = (2 * safety_set_bound) ** 2

volume_matrix = np.inf * np.ones((len(cov_list), len(grid_step_size_list)))
time_matrix = np.inf * np.ones((len(cov_list), len(grid_step_size_list)))
for filename in walk_tuple[2]:
    if '.py' in filename:
        d += 1
        # print(d, filename)
        info_list = filename.split('_')
        cov = int(info_list[7])/1000.0
        grid_step_size = int(info_list[11])/1000.0
        cov_index = np.where(np.isclose(cov_list, cov))[0]
        grid_step_index = np.where(np.isclose(grid_step_size_list,
                                                   grid_step_size))[0]
        if not (len(cov_index) and len(grid_step_index)):
            print('cov              :', info_list[7], cov_index)
            print('grid_step_size   : ', info_list[11], grid_step_index)
            print('Skipping ', filename)
            continue
        f = open(mypath + '/' + filename)
        for line in f:
            if line.startswith('z = '):
                z = np.zeros((0, ))
                exec(line)
                volume_matrix[cov_index[0]][grid_step_index[0]] \
                    = (np.sum(z >= probability_threshold) / z.shape[0]) \
                      * volume_safe_set
        f.close()
    elif 'Runtime' in filename:
        info_list = filename.split('_')
        cov = int(info_list[7])/1000.0
        grid_step_size = int(info_list[11])/1000.0
        cov_index = np.where(np.isclose(cov_list, cov))[0]
        grid_step_index = np.where(np.isclose(grid_step_size_list,
                                              grid_step_size))[0]

        f = open(mypath + '/' + filename)
        for line in f:
            time_matrix[cov_index[0]][grid_step_index[0]] = eval(line)
        f.close()
    else:
        pass

print('Grid sizes: ', grid_step_size_list)
print(volume_matrix)
print(time_matrix)

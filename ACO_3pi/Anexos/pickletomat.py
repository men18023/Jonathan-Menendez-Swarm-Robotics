import pickle
import scipy.io as sio

# Load the .pickle file
with open('markers_alineados_zyx2.pickle', 'rb') as f:
    data = pickle.load(f)

# Convert and save as a .mat file
sio.savemat('markers_alineados_zyx.mat', {'data': data})

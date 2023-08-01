########## load osim model parameters ##################

import pickle

file_path = '/home/ml/myoConverter/models/converted/RajagopalModel/Step2_MomentArmOpt/glmax2_r.pkl'

with open(file_path, "rb") as input_file:
    muscle_ma_data = pickle.load(input_file)

file_path2 = '/home/ml/myoConverter/models/converted/RajagopalModel/Step3_MuscleForceOpt/glmax2_r.pkl'

with open(file_path2, "rb") as input_file2:
    muscle_mf_data = pickle.load(input_file2)

a = 1
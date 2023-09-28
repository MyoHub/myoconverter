import argparse
from myoconverter.O2MPipeline import O2MPipeline

parser = argparse.ArgumentParser(description='Convert OpenSim model to Mujoco', prog='O2M')
parser.add_argument('--osim_file', type=str, help='OpenSim model file')
parser.add_argument('--geometry_folder', type=str, help='Folder containing geometry files')
parser.add_argument('--output_folder', type=str, help='Folder to store the converted model')
parser.add_argument('--convert_steps', type=list, default=[1,2,3], help='List of steps to convert')
parser.add_argument('--muscle_list', type=list, default=None, help='List of muscles to convert')
parser.add_argument('--osim_data_overwrite', type=bool, default=True, help='Overwrite the Osim model state files')
parser.add_argument('--conversion', type=bool, default=True, help='Perform "Cvt#" process')
parser.add_argument('--validation', type=bool, default=True, help='Perform "Vlt#" process')
parser.add_argument('--speedy', type=bool, default=False, help='Do not reduce the checking notes to increase speed')
parser.add_argument('--generate_pdf', type=bool, default=True, help='Generate validation pdf report')
parser.add_argument('--add_ground_geom', type=bool, default=True, help='Add ground to the model')
parser.add_argument('--treat_as_normal_path_point', type=bool, default=False, help='Using constraints to represent moving and conditional path points')
args = parser.parse_args()
O2MPipeline(**vars(args))
import argparse
from myoconverter.O2MPipeline import (
    O2MPipeline,
    parse_convert_steps,
    parse_muscle_list,
    str2bool,
)


def build_cli_parser():
    parser = argparse.ArgumentParser(description="Convert OpenSim model to Mujoco", prog="O2M")
    parser.add_argument("--osim_file", type=str, required=True, help="OpenSim model file")
    parser.add_argument("--geometry_folder", type=str, required=True, help="Folder containing geometry files")
    parser.add_argument("--output_folder", type=str, required=True, help="Folder to store the converted model")
    parser.add_argument("--convert_steps", nargs="+", default=[1, 2, 3], help="List of steps to convert")
    parser.add_argument("--muscle_list", nargs="+", type=str, default=None, help="List of muscles to convert")
    parser.add_argument(
        "--osim_data_overwrite",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="Overwrite the Osim model state files",
    )
    parser.add_argument("--conversion", type=str2bool, nargs="?", const=True, default=True, help='Perform "Cvt#" process')
    parser.add_argument("--validation", type=str2bool, nargs="?", const=True, default=True, help='Perform "Vlt#" process')
    parser.add_argument(
        "--speedy",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
        help="Reduce the checking nodes to increase speed",
    )
    parser.add_argument(
        "--generate_pdf",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="Generate validation pdf report",
    )
    parser.add_argument(
        "--add_ground_geom",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="Add ground to the model",
    )
    parser.add_argument(
        "--treat_as_normal_path_point",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
        help="Treat moving and conditional path points as fixed path points",
    )
    return parser


def main(argv=None):
    args = build_cli_parser().parse_args(argv)
    args.convert_steps = parse_convert_steps(args.convert_steps)
    args.muscle_list = parse_muscle_list(args.muscle_list)
    O2MPipeline(**vars(args))


if __name__ == "__main__":
    main()

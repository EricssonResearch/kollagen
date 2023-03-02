import sys
import argparse

def gen():
    import kollagen
    parser = argparse.ArgumentParser(prog="kollagenr8",description="Generate data from json file using kollagen")
    parser.add_argument("json_file", action="store", type=str, help="Path to json file used for data generation")
    parser.add_argument("output_dir", action="store", nargs="?", default="./output", type=str, help="Output directory, defaults to './output'")
    parser.add_argument("-s", "--singleg2o", action="store_true", default=False, help="Save singleg2o")
    parser.add_argument("-m", "--multig2o", action="store_true", default=True, help="Save multig2o (default)")
    parser.add_argument("--no-singleg2o", action="store_false", dest="singleg2o", help="Don't save singleg2o (default)")
    parser.add_argument("--no-multig2o", action="store_false", dest="multig2o", help="Don't save multig2o")

    args = parser.parse_args()

    kollagen.generate(args.json_file, args.output_dir, args.multig2o, args.singleg2o)

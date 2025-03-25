import argparse

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--method', type=str, default='ours', help='method name', choices=['ours', 'random', 'distance', 'price'])
    parser.add_argument('--num_drones', type=int, default=4, help='number of drones')
    parser.add_argument('--resource', type=int, default=100, help='demand of resource')

    return parser.parse_args()
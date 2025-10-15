#!/usr/bin/env python3

import argparse
import functools
import inspect
import importlib
import os
import rclpy
import traceback
from typing import List
import yaml
from rich.console import Console
from rich.panel import Panel
from rich.syntax import Syntax

from bag_record_pid.bag_record_node import BagRecorderNode

console = Console()

# Routines
def load_routines(pre_routines: List[callable], post_routines: List[callable], config: dict):
    """
    Load routines from config and initialize as partial functions
    with kwargs. Filter any non-existent kwargs from the function
    """
    def import_callable(path: str):
        """
        Import a function inside it.
        """
        parts = path.split(".")
        for i in range(len(parts), 0, -1):
            module_name = ".".join(parts[:i])
            try:
                mod = importlib.import_module(module_name)
                break
            except ModuleNotFoundError as e:
                if e.name == module_name:
                    # The module does not exist. Try shortening
                    # ex: `routines.freq.bag_freq` not found but `routines.freq` is
                    continue  # try a shorter prefix
                else:
                    # The module exists, but failed to load submodule properly
                    print(f"[!] Failed inside valid module: {module_name} has missing submodule '{e.name}'")
                    raise  # this is a real internal import error — stop trying
        else:
            raise ModuleNotFoundError(f"Cannot import {path}")

        fn_name = parts[-1]
        if not hasattr(mod, fn_name):
            raise AttributeError(f"Module {module_name} has no attribute {fn_name}")
        return getattr(mod, fn_name)


    routines = config['common'].get("routines", {})
    pre_h = routines.get('pre_logger', [])
    post_h = routines.get('post_logger', [])

    for ref in pre_h:
        fn_type = ref.get('type', "")
        kwargs = ref.get('args', {})
        fn = import_callable(fn_type)

        if callable(fn):
            kwargs = filter_kwargs(fn, kwargs)
            pre_routines.append(functools.partial(fn, **kwargs))
            print(f"Registered pre_logger function {fn_type}")
        else:
            raise ValueError(f"{ref} is not callable")

    for ref in post_h:
        fn_type = ref.get('type', "")
        kwargs = ref.get('args', {})
        fn = import_callable(fn_type)
        if callable(fn):
            kwargs = filter_kwargs(fn, kwargs)
            post_routines.append(functools.partial(fn, **kwargs))
            print(f"Registered post_logger function {fn_type}")
        else:
            raise ValueError(f"{fn_type} is not callable")

def run_routines(routines: List[callable]):
    """
    Run any desired code before bagging
    """
    for fn in routines:
        fn()

# Nodes
def run_node(config: dict):
    """
    Run ROS2 node by instantiating its class and spinning it. The node can die
    in two ways:
    1. User ctrl+C's it to stop bagging (success)
    2. Node crashes because bagging fails (not success)
    """
    rclpy.init()
    node = None
    success = True
    # Spin not during bagging
    try:
        node = BagRecorderNode(config)
        rclpy.spin(node)
    # If Ctrl+C human interrupt, bag success
    except KeyboardInterrupt:
        if node is not None:
            node.interrupt()
    # If other exception, bag failure (Don't do post proc)
    except Exception:
        # Catch bag failure or other runtime errors
        if node is not None:
            node.interrupt()
        traceback.print_exc()
        success = False
    # Ensure node is dead
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()
    return success

# CLI/config stuff
def next_foldername(path: str, prefix: bool=False, step: int=1, digits: int=6):
    path = path.rstrip('/')
    dir, base_name = os.path.split(path)
    num = next_num(dir, prefix, digits, step)
    if prefix:
        return os.path.join(dir, f"{num:0{digits}d}_{base_name}")
    return os.path.join(dir, f"{base_name}_{num:0{digits}d}")

def next_num(dir: str, prefix: bool, digits: int, step: int = 1, base_name=None):
    """
    Find the next number to create inside a dir framed like so:
    dir
    - 000_foldername | 000_filename.ext | filename_000.ext
    - 001_foldername | 001_filename.ext | filename_001.ext
    ...

    Assumes number is separated by "_".

    Args:
        dir: Directory to search
        prefix: If number is a prefix or suffix to content inside dir
        digits: How many digits the number has
        step: How much to increase number by
        base_name: Base name of the file to search for. If provided, will
                      only search files with base_name in string.
    """
    ni = 0 if prefix else -1
    dir = os.getcwd() if dir == "" else dir
    for root, dirs, files in os.walk(dir):
        search = dirs

        # Filter search by base_name:
        if base_name is not None:
            # Search for base name explicitly. Previously base_name in s would
            # yield bad results i.e. "mse_full" is in "rmse_full" and so the
            # number retrieval would be wrong. Still keep full filename in
            # search though, so the number can be parsed and updated
            excl_num = slice(1, None) if prefix else slice(None, -1)
            search = [s for s in search if base_name == "_".join(s.split('_')[excl_num])]
            
        # find valid candidates
        valid_nums = [int(tok[:digits]) for s in search for tok in [s.split('_')[ni]] if len(tok) == digits and tok[:digits].isdigit()]
        if len(valid_nums) == 0:
            return 0 # First content in folder

        # Find the max file number given using prefix/suffix logic with number of digits of all valid candidates
        return step + max(valid_nums, default=0)

def filter_kwargs(func: callable, kwargs: dict):
    """
    Because common args are copied into all sub-dictionary params, filter only
    valid arms in the function definition to avoid error.
    """
    sig = inspect.signature(func)
    return {k: v for k, v in kwargs.items() if k in sig.parameters}

def convert_key(key: str):
    """
    Convert to proper key representation
    """
    short_to_long_args = {
        's': 'storage',
        'o': 'output',
        'd': 'max-bag-duration',
        'b': 'max-bag-size'
    }
    key = key.strip('-').replace('_','-') # get into ros2 verb cli fmt
    if key in short_to_long_args:
        return short_to_long_args[key.strip('-')]
    return key

def parse_config(config: dict, args: argparse.Namespace, rargs: argparse.Namespace):
    """
    Pre-proc of config. Do things like
    1. Convert any key representations to base ('-s' -> 'storage')
    2. Override config with CLI args
    3. Update routine and bag args with config common args

    Args:
        args (argparse.Namespace): script args not used for ros2 bag record command
        rargs (argparse.Namespace): ros2 bag record args
    """
    common_config = config['common']
    for key in list(common_config['args'].keys()):
        common_config['args'][convert_key(key)] = common_config['args'].pop(key)

    cli_args = {convert_key(k): v for k, v in vars(rargs).items() if v is not None}

    # Handle pre-pending to bag name
    if not args.no_prepend:
        cli_args['output'] = next_foldername(cli_args['output'], prefix=True, digits=2)

    # Precedence order: cli > local cfg args > common cfg args (> means more precedence)
    for _, routines in common_config['routines'].items():
        for hook in routines:
            hook['args'] = common_config['args'] | hook.get('args', {}) | cli_args

    for _, bag_cfg in config['bags'].items():
        bag_cfg['args'] = common_config['args'] | bag_cfg.get('args', {}) | cli_args

    return config

def main(args: argparse.Namespace, rargs: argparse.Namespace):
    """
    Run bagger wrapper

    Args:
        args (argparse.Namespace): script args not used for ros2 bag record command
        rargs (argparse.Namespace): ros2 bag record args
    """
    # Load config
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)
    config = parse_config(config, args, rargs)

    # Routines
    pre_routines = []
    post_routines = []
    load_routines(pre_routines, post_routines, config)

    run_routines(pre_routines)
    success = run_node(config)
    if not success:
        return
    run_routines(post_routines)

def print_help_with_rich(script_parser, ros_parser):
    """Pretty-print combined help for both parsers using Rich."""
    console.print(Panel.fit(
        "[bold cyan]ROS2 Bag Record Wrapper[/bold cyan]\n"
        "Wrapper around `ros2 bag record` with pre/post routines.",
        title="[bold white]Description[/bold white]",
        border_style="cyan",
    ))

    # Script args section
    console.print("[bold yellow]Script (ROS-Agnostic) Arguments:[/bold yellow]\n")
    script_help = script_parser.format_help()
    console.print(Syntax(script_help, "text", theme="ansi_dark", background_color="default"))

    # ROS args section
    console.print("\n[bold magenta]ROS2 Bag Record Arguments:[/bold magenta]\n")
    ros_help = ros_parser.format_help()
    console.print(Syntax(ros_help, "text", theme="ansi_dark", background_color="default"))


if __name__ == "__main__":
    # Want to add more script args not used in the ros2 bag command? Edit this.
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-c", "--config",
                        type=str,
                        required=True,
                        help="Path to YAML config with bag definitions")
    parser.add_argument("--no-prepend",
                        action='store_true',
                        required=False,
                        help="If no-prepend, will not preprend a number to bag name")
    parser.add_argument("-h", "--help", action="store_true")
    args, remaining = parser.parse_known_args()
    
    # Want to add more ROS args? Edit this.
    ros_parser = argparse.ArgumentParser(add_help=False)
    ros_parser.add_argument("-o", "--output",
                        type=str,
                        default=None,
                        required=False,
                        help="Path to YAML config with bag definitions")
    ros_parser.add_argument("-s", "--storage",
                        type=str,
                        default=None,
                        required=False,
                        help="Storage type")
    ros_parser.add_argument("-d", "--max-bag-duration",
                        type=int,
                        default=None,
                        required=False,
                        help="Max bag duration")
    ros_parser.add_argument("-b", "--max-bag-size",
                        type=int,
                        default=None,
                        required=False,
                        help="Max bag size (in bytes) before bagfile will be split")
    ros_parser.add_argument("--storage-config-file",
                        type=str,
                        default=None,
                        required=False,
                        help="Path to a yaml file defining storaeg specific configurations.")

    if args.help:
        print_help_with_rich(parser, ros_parser)
        exit(0)

    ros_args = ros_parser.parse_args(remaining)
    main(args, ros_args)

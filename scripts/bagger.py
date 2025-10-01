#!/usr/bin/env python3

import argparse
import os
import sys
import yaml
import importlib
import functools
import inspect
import rclpy
import traceback

from ament_index_python.packages import get_package_share_directory
from bag_record_pid.bag_record_node import BagRecorderNode

def load_hooks(pre_hooks, post_hooks, config):
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
            except ModuleNotFoundError:
                continue
        else:
            raise ModuleNotFoundError(f"Cannot import {path}")

        fn_name = parts[-1]
        if not hasattr(mod, fn_name):
            raise AttributeError(f"Module {module_name} has no attribute {fn_name}")
        return getattr(mod, fn_name)


    hooks = config['common'].get("hooks", {})
    pre_h = hooks.get('pre_logger', [])
    post_h = hooks.get('post_logger', [])

    for ref in pre_h:
        fn_type = ref.get('type', "")
        kwargs = ref.get('args', {})
        fn = import_callable(fn_type)

        if callable(fn):
            kwargs = filter_kwargs(fn, kwargs)
            pre_hooks.append(functools.partial(fn, **kwargs))
            print(f"Registered pre_logger function {fn_type}")
        else:
            raise ValueError(f"{ref} is not callable")

    for ref in post_h:
        fn_type = ref.get('type', "")
        kwargs = ref.get('args', {})
        fn = import_callable(fn_type)
        if callable(fn):
            kwargs = filter_kwargs(fn, kwargs)
            post_hooks.append(functools.partial(fn, **kwargs))
            print(f"Registered post_logger function {fn_type}")
        else:
            raise ValueError(f"{fn_type} is not callable")

def pre_logger(pre_hooks):
    """
    Run any desired code before bagging
    """
    for pre_fn in pre_hooks:
        pre_fn()

def post_logger(post_hooks):
    """
    Run any desired code after bagging
    """
    for post_fn in post_hooks:
        post_fn()

def filter_kwargs(func, kwargs):
    """
    Because common args are copied into all sub-dictionary params, filter only
    valid arms in the function definition to avoid error.
    """
    sig = inspect.signature(func)
    return {k: v for k, v in kwargs.items() if k in sig.parameters}

def convert_key(key):
    short_to_long_args = {
        's': 'storage',
        'o': 'output',
        'd': 'max_bag_duration',
        'b': 'max_bag_size'
    }
    key = key.strip('-')
    if key in short_to_long_args:
        return short_to_long_args[key.strip('-')]
    return key

def parse_config(config, args):
    cargs = config['common']['args']
    for key in list(cargs.keys()):
       cargs[convert_key(key)] = cargs.pop(key)

    # Override with CLI args
    for key in vars(args).keys():
        val = getattr(args, key, None)
        if val is not None:
            cargs[key] = val
    
    # Update block params with common
    # Give hooks default args
    for key, hooks in config['common']['hooks'].items():
        for hook in hooks:
            hook['args'] = config['common']['args'] | hook.get('args', {})
    # Give bags default args
    for key in config['bags'].keys():
        config['bags'][key] = config['common'] | config['bags'][key]
    return config # not actually needed

def main():
    parser = argparse.ArgumentParser(description="ROS2 bag record wrapper with pre/post hooks")
    parser.add_argument("-c", "--config",
                        type=str,
                        required=True,
                        help="Path to YAML config with bag definitions")
    parser.add_argument("-o", "--output",
                        type=str,
                        default=None,
                        required=False,
                        help="Path to YAML config with bag definitions")
    parser.add_argument("-s", "--storage",
                        type=str,
                        default=None,
                        required=False,
                        help="Storage type")
    parser.add_argument("-d", "--max-bag-duration",
                        type=int,
                        default=None,
                        required=False,
                        help="Max bag duration")
    parser.add_argument("-b", "--max-bag-size",
                        type=int,
                        default=None,
                        required=False,
                        help="Max bag size (in bytes) before bagfile will be split")
    parser.add_argument("--storage-config-file",
                        type=str,
                        default=None,
                        required=False,
                        help="Path to a yaml file defining storaeg specific configurations.")
    args = parser.parse_args()

    # Load config
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)
    config = parse_config(config, args)

    # Hooks
    pre_hooks = []
    post_hooks = []
    load_hooks(pre_hooks, post_hooks, config)

    # Pre-hooks
    pre_logger(pre_hooks)

    # Start ROS node
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

    # Post-hooks
    if not success:
        return
    post_logger(post_hooks)

if __name__ == "__main__":
    main()

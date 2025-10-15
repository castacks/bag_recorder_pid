#!/usr/bin/env python3
import yaml
import argparse
import csv
import os
import subprocess
import shutil

from rich.syntax import Syntax
from rich.console import Console
from rich.panel import Panel

#name for metadata (since metadata.yaml was rudely taken by ros2 bag)
METANAME = 'info.yaml'

console = Console()

def pretty_print_yaml(data):
    """Pretty-print YAML with syntax highlighting."""
    yaml_str = yaml.dump(data, sort_keys=False, default_flow_style=False)
    console.print(Panel(Syntax(yaml_str, "yaml", theme="ayu", line_numbers=False),
                        title="Current Configuration", expand=False))

def edit_in_editor(yaml_path):
    """Open YAML file in user's preferred editor (defaults to vim)."""
    editor = os.environ.get("EDITOR", "vim")

    # For editors like VS Code, use the '-w' flag to wait
    if "code" in editor:
        subprocess.run([editor, "-w", yaml_path])
    else:
        subprocess.run([editor, yaml_path])

    
def update_metadata(metadata_dir: str, template: str):
    metadata_path = os.path.join(metadata_dir, template)
    with open(metadata_path, 'r') as f:
        metadata = yaml.safe_load(f)

    console.print("[bold cyan]Current metadata:[/bold cyan]")
    pretty_print_yaml(metadata)

    choice = input("Is this configuration correct? (y/n/edit) ").strip().lower()

    if choice in ("n", "edit", ""):
        edit_in_editor(metadata_path)

def store_metadata(output: str, metadata_dir: str, template: str):
    metadata_path = os.path.join(metadata_dir, template)
    outpath = os.path.join(output, METANAME)
    shutil.copy(metadata_path, outpath)


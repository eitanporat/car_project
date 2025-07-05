#!/usr/bin/env python3
"""
Generate results table from batch sweep JSON files.
"""

import json
import os
import glob
from pathlib import Path

def parse_filename(filename):
    """Extract length and width from filename like 'L17.0_W7.0.json'"""
    basename = os.path.basename(filename)
    if basename.startswith('L') and '_W' in basename:
        parts = basename.replace('.json', '').split('_')
        length = float(parts[0][1:])  # Remove 'L' prefix
        width = float(parts[1][1:])   # Remove 'W' prefix
        return length, width
    return None, None

def analyze_results(results_dir):
    """Analyze all JSON files in results directory"""
    results = {}
    
    # Find all JSON files
    json_files = glob.glob(os.path.join(results_dir, "L*.json"))
    
    for json_file in json_files:
        length, width = parse_filename(json_file)
        if length is None or width is None:
            continue
            
        with open(json_file, 'r') as f:
            data = json.load(f)
            
        visited_states = data.get('visited_states', 0)
        states = data.get('states')
        actions = data.get('actions')
        
        if states is None or actions is None:
            status = f"No Path Found (visited {visited_states} states)"
        else:
            num_parts = len(actions)
            status = f"Found Path with {num_parts} parts (visited {visited_states} states)"
            
        if length not in results:
            results[length] = {}
        results[length][width] = status
    
    return results

def generate_table(results):
    """Generate markdown table from results"""
    if not results:
        return "No results found."
    
    # Get all unique lengths and widths
    lengths = sorted(results.keys())
    widths = set()
    for length_data in results.values():
        widths.update(length_data.keys())
    widths = sorted(widths)
    
    # Create table header
    table = "| Length \\ Width |"
    for width in widths:
        table += f" {width:.1f} |"
    table += "\n"
    
    # Add separator
    table += "|" + "---|" * (len(widths) + 1) + "\n"
    
    # Add data rows
    for length in lengths:
        table += f"| {length:.1f} |"
        for width in widths:
            if width in results[length]:
                status = results[length][width]
                # Truncate if too long
                if len(status) > 50:
                    status = status[:47] + "..."
                table += f" {status} |"
            else:
                table += " - |"
        table += "\n"
    
    return table

def main():
    # Find the most recent results directory
    results_dirs = glob.glob("results/*")
    if not results_dirs:
        print("No results directories found.")
        return
    
    latest_dir = max(results_dirs, key=os.path.getctime)
    print(f"Analyzing results from: {latest_dir}")
    
    results = analyze_results(latest_dir)
    table = generate_table(results)
    
    print("\nResults Table:")
    print("=" * 80)
    print(table)
    
    # Save to file
    output_file = os.path.join(latest_dir, "results_table.md")
    with open(output_file, 'w') as f:
        f.write("# Parking Sweep Results\n\n")
        f.write(table)
    
    print(f"\nTable saved to: {output_file}")

if __name__ == "__main__":
    main() 
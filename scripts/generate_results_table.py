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
            success = False
        else:
            num_parts = len(actions)
            status = f"Found Path with {num_parts} parts (visited {visited_states} states)"
            success = True
            
        if length not in results:
            results[length] = {}
        results[length][width] = {'status': status, 'success': success, 'visited_states': visited_states}
    
    return results

def generate_markdown_table(results):
    """Generate markdown table from results"""
    if not results:
        return "No results found."
    
    # Get all unique lengths and widths
    lengths = sorted(results.keys())
    widths = set()
    for length_data in results.values():
        widths.update(length_data.keys())
    widths = sorted(widths)
    
    # Calculate statistics
    total_configs = 0
    success_count = 0
    failure_count = 0
    
    for length in lengths:
        for width in widths:
            if width in results[length]:
                total_configs += 1
                if results[length][width]['success']:
                    success_count += 1
                else:
                    failure_count += 1
    
    # Create statistics section
    table = f"""
### 📊 Sweep Statistics
- **Total Configurations**: {total_configs}
- **✅ Successful**: {success_count}
- **❌ Failed**: {failure_count}
- **Success Rate**: {(success_count/total_configs*100):.1f}%

### 📋 Results Table
| Length \\ Width |"""
    
    # Add width headers
    for width in widths:
        table += f" {width:.0f}m |"
    table += "\n"
    
    # Add separator
    table += "|" + "---|" * (len(widths) + 1) + "\n"
    
    # Add data rows
    for length in lengths:
        table += f"| **{length:.0f}m** |"
        for width in widths:
            if width in results[length]:
                result = results[length][width]
                if result['success']:
                    # Success case - green with checkmark
                    table += f" ✅ {result['visited_states']} states |"
                else:
                    # Failure case - red with X
                    table += f" ❌ {result['visited_states']} states |"
            else:
                table += " - |"
        table += "\n"
    
    return table

def generate_html_table(results):
    """Generate HTML table with colored cells"""
    if not results:
        return "No results found."
    
    # Get all unique lengths and widths
    lengths = sorted(results.keys())
    widths = set()
    for length_data in results.values():
        widths.update(length_data.keys())
    widths = sorted(widths)
    
    # HTML table with CSS styling
    html = """
<style>
.results-table {
    border-collapse: collapse;
    margin: 20px 0;
    font-family: Arial, sans-serif;
    font-size: 12px;
    width: 100%;
    max-width: 1200px;
}

.results-table th {
    background-color: #2c3e50;
    color: white;
    padding: 12px 8px;
    text-align: center;
    font-weight: bold;
    border: 1px solid #34495e;
}

.results-table td {
    padding: 10px 6px;
    text-align: center;
    border: 1px solid #bdc3c7;
    vertical-align: middle;
    line-height: 1.3;
}

.results-table .length-header {
    background-color: #34495e;
    color: white;
    font-weight: bold;
    text-align: center;
}

.results-table .success {
    background-color: #d5f4e6;
    color: #27ae60;
    font-weight: 500;
}

.results-table .failure {
    background-color: #fadbd8;
    color: #e74c3c;
    font-weight: 500;
}

.results-table .success:hover {
    background-color: #a9dfbf;
}

.results-table .failure:hover {
    background-color: #f5b7b1;
}

.results-table .length-header:hover {
    background-color: #2c3e50;
}

.stats {
    margin: 20px 0;
    padding: 15px;
    background-color: #ecf0f1;
    border-radius: 5px;
    font-family: Arial, sans-serif;
}

.stats h3 {
    margin-top: 0;
    color: #2c3e50;
}

.stats .stat-item {
    display: inline-block;
    margin-right: 30px;
    font-weight: bold;
}

.stats .success-count {
    color: #27ae60;
}

.stats .failure-count {
    color: #e74c3c;
}

.stats .total-count {
    color: #3498db;
}
</style>

<div class="stats">
    <h3>📊 Sweep Statistics</h3>
"""
    
    # Calculate statistics
    total_configs = 0
    success_count = 0
    failure_count = 0
    
    for length in lengths:
        for width in widths:
            if width in results[length]:
                total_configs += 1
                if results[length][width]['success']:
                    success_count += 1
                else:
                    failure_count += 1
    
    html += f"""
    <div class="stat-item total-count">Total Configurations: {total_configs}</div>
    <div class="stat-item success-count">Successful: {success_count}</div>
    <div class="stat-item failure-count">Failed: {failure_count}</div>
    <div class="stat-item">Success Rate: {(success_count/total_configs*100):.1f}%</div>
</div>

<table class="results-table">
    <thead>
        <tr>
            <th>Length \\ Width</th>
"""
    
    # Add width headers
    for width in widths:
        html += f'            <th>{width:.0f}m</th>\n'
    
    html += "        </tr>\n    </thead>\n    <tbody>\n"
    
    # Add data rows
    for length in lengths:
        html += f"        <tr>\n            <td class='length-header'>{length:.0f}m</td>\n"
        for width in widths:
            if width in results[length]:
                result = results[length][width]
                cell_class = "success" if result['success'] else "failure"
                status = result['status']
                # Make it more concise for HTML
                if result['success']:
                    status = f"✅ {result['visited_states']} states"
                else:
                    status = f"❌ {result['visited_states']} states"
                html += f'            <td class="{cell_class}">{status}</td>\n'
            else:
                html += '            <td>-</td>\n'
        html += "        </tr>\n"
    
    html += "    </tbody>\n</table>"
    
    return html

def main():
    # Find the most recent results directory
    results_dirs = glob.glob("results/*")
    if not results_dirs:
        print("No results directories found.")
        return
    
    latest_dir = max(results_dirs, key=os.path.getctime)
    print(f"Analyzing results from: {latest_dir}")
    
    results = analyze_results(latest_dir)
    
    # Generate both formats
    markdown_table = generate_markdown_table(results)
    html_table = generate_html_table(results)
    
    print("\nMarkdown Results Table:")
    print("=" * 80)
    print(markdown_table)
    
    # Save markdown table
    output_file = os.path.join(latest_dir, "results_table.md")
    with open(output_file, 'w') as f:
        f.write("# Parking Sweep Results\n\n")
        f.write(markdown_table)
    
    # Save HTML table
    html_output_file = os.path.join(latest_dir, "results_table.html")
    with open(html_output_file, 'w') as f:
        f.write("<!DOCTYPE html>\n<html>\n<head>\n")
        f.write("<title>Parking Sweep Results</title>\n")
        f.write("<meta charset='utf-8'>\n")
        f.write("</head>\n<body>\n")
        f.write("<h1>🚗 Parking Sweep Results</h1>\n")
        f.write(html_table)
        f.write("\n</body>\n</html>")
    
    print(f"\nMarkdown table saved to: {output_file}")
    print(f"HTML table saved to: {html_output_file}")

if __name__ == "__main__":
    main() 
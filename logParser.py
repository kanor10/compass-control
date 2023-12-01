import re
import sys
import os

# Function to construct the output file name
def get_output_filename(input_filename):
    name, ext = os.path.splitext(input_filename)
    return f"{name}_coords{ext}"

# Function to extract numbers from a line
def extract_numbers(line):
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
    return numbers[:2]  # Return only the first two numbers

# Check if an input file name was provided
if len(sys.argv) < 2:
    print("Usage: python script.py <input_file>")
    sys.exit(1)

# Get the input file name from command-line arguments
input_file = sys.argv[1]

# Construct the output file name
output_file_name = get_output_filename(input_file)

# Open the source file and create a new file for the output
with open(input_file, 'r') as source_file, open(output_file_name, 'w') as output_file:
    for line in source_file:
        if line.startswith('Coordinates:'):
            first_number, second_number = extract_numbers(line)
            output_file.write(f"{first_number}, {second_number}\n")

print("Processing complete.")

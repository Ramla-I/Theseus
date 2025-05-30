#!/bin/bash

# Check if an argument (file or folder) is provided
if [ -z "$1" ]; then
  echo "Usage: $0 file_or_folder_path"
  exit 1
fi

# Path to file or folder
TARGET="$1"

# Check if the target is a file or a folder
if [ -f "$TARGET" ]; then
  SEARCH_PATH="$TARGET"
elif [ -d "$TARGET" ]; then
  SEARCH_PATH="$TARGET"
else
  echo "Error: '$TARGET' is neither a valid file nor a folder."
  exit 1
fi

# Initialize line count
total_lines=0

# Find all .dfy files and count non-empty and non-comment lines
for file in $(find "$FOLDER" -name "*.dfy"); do
  lines_in_file=$(grep -v '^\s*$' "$file" | grep -v '^\s*//' | wc -l)
  total_lines=$((total_lines + lines_in_file))
done

# Output the total line count
echo "Total lines of code (LOC) in Dafny files in '$FOLDER': $total_lines"

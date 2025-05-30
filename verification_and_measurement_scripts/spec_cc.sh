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

# Find all occurrences, ignoring lines starting with //
count_not_impl=$(grep -rOh "assert_not_impl_any!" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_fields_type=$(grep -rOh "assert_fields_type!" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_private_fields=$(grep -rOh "#\[private_fields(" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_consumes=$(grep -rOh "#\[consumes(" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_no_mutate=$(grep -rOh "#\[nomutates(" "$SEARCH_PATH" | grep -v "^//" | wc -l)

# Sum all counts
total_count=$((count_not_impl + count_fields_type + count_private_fields + count_consumes + count_no_mutate))

# Display the results
echo "Occurrences of the following tags in '$TARGET':"
echo "--------------------------------------------"
echo "assert_not_impl_any!        : $count_not_impl"
echo "assert_fields_type!         : $count_fields_type"
echo "#[private_fields]           : $count_private_fields"
echo "#[consumes]                 : $count_consumes"
echo "#[nomutates]                : $count_no_mutate"
echo "--------------------------------------------"
echo "Total Spec LOC   : $total_count"

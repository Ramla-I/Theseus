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
count_ensures=$(grep -rOh "#\[ensures" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_requires=$(grep -rOh "#\[requires" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_extern=$(grep -rOh "#\[extern_spec" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_trusted=$(grep -rOh "#\[trusted\]" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_pure=$(grep -rOh "#\[pure\]" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_verified=$(grep -rOh "#\[verified\]" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_type_invariant=$(grep -rOh "#\[invariant" "$SEARCH_PATH" | grep -v "^//" | wc -l)
count_body_invariant=$(grep -rOh "body_invariant!(" "$SEARCH_PATH" | grep -v "^//" | wc -l)

# Sum all counts
total_count=$((count_ensures + count_requires + count_extern + count_trusted + count_pure + count_verified + count_type_invariant))

# Display the results
echo "Occurrences of the following tags in '$TARGET':"
echo "--------------------------------------------"
echo "#[ensures]       : $count_ensures"
echo "#[requires]      : $count_requires"
echo "#[extern_spec]   : $count_extern"
echo "#[trusted]       : $count_trusted"
echo "#[pure]          : $count_pure"
echo "#[verified]      : $count_verified"
echo "#[invariant]     : $count_type_invariant"
echo "--------------------------------------------"
echo "Total Spec LOC   : $total_count"

echo "Total Proof LOC  : $count_body_invariant"

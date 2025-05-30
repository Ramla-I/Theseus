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

# Initialize counts for each specification keyword
count_requires=0
count_ensures=0
count_invariant=0
count_decreases=0
count_modifies=0
count_ghost=0
count_lemma=0
count_function=0
total_lemma_lines=0
total_function_lines=0

# Find all .dfy files and count specification-related lines
for file in $(find "$FOLDER" -name "*.dfy"); do
  inside_lemma=false
  inside_function=false
  lemma_line_count=0
  function_line_count=0

  # Read the file line by line
  while IFS= read -r line; do
    # Ignore empty lines and single-line comments
    if [[ -z "$line" || "$line" =~ ^[[:space:]]*// ]]; then
      continue
    fi
    
    # Check for lemma and count lines within braces
    if [[ "$line" =~ lemma\  ]]; then
      ((count_lemma++))
      inside_lemma=true
      lemma_line_count=0  # Reset the counter for new lemma
    fi
    
    # If we are inside a lemma, count lines until we exit the braces
    if [ "$inside_lemma" = true ]; then
      lemma_line_count=$((lemma_line_count + 1))
      
    #   # Check if the line contains an opening brace {
    #   if [[ "$line" =~ \{ ]]; then
    #     Increment line count for opening brace
    #     lemma_line_count=$((lemma_line_count + 1))
    #   fi
      
      # Check for closing brace }
      if [[ "$line" =~ \} ]]; then
        inside_lemma=false
        total_lemma_lines=$((total_lemma_lines + lemma_line_count))
        lemma_line_count=0  # Reset for the next lemma
      fi
     
      continue
    fi

    # Check for function and count lines within braces
    # Count occurrences of 'function', excluding 'function method'
    if [[ "$line" =~ function\  && ! "$line" =~ function\ method ]]; then
      ((count_function++))
      inside_function=true
      function_line_count=0  # Reset the counter for new function
    fi
    
    # If we are inside a function, count lines until we exit the braces
    if [ "$inside_function" = true ]; then
      function_line_count=$((function_line_count + 1))
      
    #   # Check if the line contains an opening brace {
    #   if [[ "$line" =~ \{ ]]; then
    #     # Increment line count for opening brace
    #     function_line_count=$((function_line_count + 1))
    #   fi
      
      # Check for closing brace }
      if [[ "$line" =~ \} ]]; then
        inside_function=false
        total_function_lines=$((total_function_lines + function_line_count))
        function_line_count=0  # Reset for the next function
      fi

      continue
    fi

    # Count occurrences of specification keywords
    if [[ "$line" =~ requires ]]; then
      ((count_requires++))
    fi
    if [[ "$line" =~ ensures ]]; then
      ((count_ensures++))
    fi
    if [[ "$line" =~ invariant ]]; then
      ((count_invariant++))
    fi
    if [[ "$line" =~ decreases ]]; then
      ((count_decreases++))
    fi
    if [[ "$line" =~ modifies ]]; then
      ((count_modifies++))
    fi
    if [[ "$line" =~ ghost ]]; then
      ((count_ghost++))
    fi

  done < "$file"
done

# Print the results in a formatted table
echo "---------------------------------------"
echo "| Keyword       | Occurrences        |"
echo "---------------------------------------"
printf "| %-12s  | %-17d |\n" "requires"            $count_requires
printf "| %-12s  | %-17d |\n" "ensures"             $count_ensures
printf "| %-12s  | %-17d |\n" "invariant"           $count_invariant
printf "| %-12s  | %-17d |\n" "decreases"           $count_decreases
printf "| %-12s  | %-17d |\n" "modifies"            $count_modifies
printf "| %-12s  | %-17d |\n" "ghost"               $count_ghost
printf "| %-12s  | %-17d |\n" "lemma"               $count_lemma
printf "| %-12s  | %-17d |\n" "lemma lines"         $total_lemma_lines # includes lemma keyword
printf "| %-12s  | %-17d |\n" "function"            $count_function
printf "| %-12s  | %-17d |\n" "function lines"      $total_function_lines # includes function keyword
echo "---------------------------------------"

# Calculate total proof lines
total_proof_lines=$((count_invariant + count_decreases + count_ghost + total_lemma_lines))

# Calculate total specifications
total_specifications=$((count_requires + count_ensures + count_modifies + total_function_lines))

# # Output the total lines of code within lemma braces
# echo "Total lines of code within lemma braces in Dafny files in '$FOLDER': $total_lemma_lines"

echo "Total proof lines (invariant + decreases + ghost + lemma): $total_proof_lines"

echo "Total specifications (requires + ensures + modifies + function): $total_specifications"
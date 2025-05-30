import subprocess
import time
import os
import shutil

def remove_folder(path):
    if os.path.exists(path):
        try:
            shutil.rmtree(path)
            # print(f"{path} folder removed successfully.")
        except Exception as e:
            print(f"Failed to remove {path} folder: {e}")
            return
    else:
        print(f"No {path} folder found in the directory.")


def measure_verification_time(directory, command):
    """
    Measures the time it takes to run a given command in a specified directory using perf_counter.
    
    :param directory: The directory to change into before running the command.
    :param command: List of command and arguments, e.g., ['ls', '-la']
    :return: The time taken to execute the command in seconds.
    """
    # Change to the target directory
    try:
        os.chdir(directory)
    except FileNotFoundError:
        print(f"Directory '{directory}' not found.")
        return

    # print(f"Changed to directory: {directory}")

    # Attempt to remove the 'target' folder if it exists
    remove_folder('target')
    remove_folder('../../target')

    # Start timing
    start_time = time.perf_counter()
    
    # Run the command
    try:
        result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        end_time = time.perf_counter()

        elapsed_time = end_time - start_time
        # print(f"Command executed successfully. Output:\n{result.stdout.decode('utf-8')}")
        print(f"Directory: {directory}, Time taken: {elapsed_time:.6f} seconds")
        return elapsed_time
    
    except subprocess.CalledProcessError as e:
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        print(f"Command failed with error:\n{e.stderr.decode('utf-8')}")
        print(f"Directory: {directory}, Time taken: {elapsed_time:.6f} seconds")
        return elapsed_time

# Example usage
cargo_prusti_rustc = ['../../../prusti_versions/prusti_rustc/prusti-dev/target/release/cargo-prusti']
cargo_prusti_vir = ['../../../prusti_versions/prusti_vir/prusti-dev/target/release/cargo-prusti']
cargo_prusti_complete = ['../../../prusti_versions/prusti_complete/prusti-dev-v-2023-08-22-1715/target/release/cargo-prusti']

# use when we want to print out log, make sure it's set in the Prusti.toml files
cargo_prusti_complete_debug = ['../../../prusti_versions/prusti_complete/prusti-dev-v-2023-08-22-1715/target/debug/cargo-prusti']

command_to_run = cargo_prusti_complete_debug

measure_verification_time('kernel/prusti_representation_creator', command_to_run)
os.chdir("../../")
measure_verification_time('kernel/prusti_frame_chunk', command_to_run)
os.chdir("../../")
measure_verification_time('kernel/prusti_page_chunk', command_to_run)
os.chdir("../../")
measure_verification_time('kernel/memory_structs', command_to_run)
os.chdir("../../")
measure_verification_time('kernel/memory', command_to_run)
os.chdir("../../")
measure_verification_time('kernel/prusti_memory_buffer', command_to_run)
os.chdir("../../")
measure_verification_time('kernel/prusti_borrowed_shared_mp', command_to_run)
os.chdir("../../")
measure_verification_time('kernel/pci', command_to_run)
os.chdir("../../")
measure_verification_time('kernel/ixgbe_flexible', command_to_run)
os.chdir("../../")

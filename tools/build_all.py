import os
import shutil
import glob
import sys
import subprocess
import time

subprocess.run("rm -rf _build/", shell=True)
subprocess.run("rm -rf bin/", shell=True)

success_count = 0
fail_count = 0
exit_status = 0

build_format = '| {:32} | {:9} | {:5} | {:6} | {:6} |'
build_separator = '-' * 74

# All supported boards
all_boards = []
for entry in os.scandir("src/boards"):
    if entry.is_dir():
        all_boards.append(entry.name)
all_boards.sort()

#sha, version = build_info.get_version_info()

total_time = time.monotonic()

print(build_separator)
print(build_format.format('Board', 'Result', 'Time', 'Flash', 'SRAM'))
print(build_separator)

for board in all_boards:
    bin_directory = "bin/{}/".format(board)
    os.makedirs(bin_directory, exist_ok=True)

    start_time = time.monotonic()
    make_result = subprocess.run("make -j 4 BOARD={} all".format(board), shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    build_duration = time.monotonic() - start_time

    flash_size = "-"
    sram_size = "-"

    if make_result.returncode == 0:
        success = "\033[32msucceeded\033[0m"
        success_count += 1
        
        out_file = glob.glob('_build/build-{}/*.out'.format(board))[0]
        size_output = subprocess.run('size {}'.format(out_file), shell=True, stdout=subprocess.PIPE).stdout.decode("utf-8")
        size_list = size_output.split('\n')[1].split('\t')
        flash_size = int(size_list[0])
        sram_size = int(size_list[1]) + int(size_list[2])
    else:
        exit_status = make_result.returncode
        success = "\033[31mfailed\033[0m   "
        fail_count += 1

    for entry in os.scandir("_build/build-{}".format(board)):
        for extension in ["zip", "hex", "uf2"]:
            if entry.name.endswith(extension):
                if ("nosd" in entry.name) or ("s140" in entry.name) or ("s132" in entry.name):
                    shutil.copy(entry.path, bin_directory)

    print(build_format.format(board, success, "{:.2f}s".format(build_duration), flash_size, sram_size))

    if make_result.returncode != 0:
        print(make_result.stdout.decode("utf-8"))

# Build Summary
total_time = time.monotonic() - total_time
print(build_separator)
print("Build Sumamary: {} \033[32msucceeded\033[0m, {} \033[31mfailed\033[0m and took {:.2f}s".format(success_count, fail_count, total_time))
print(build_separator)

sys.exit(exit_status)

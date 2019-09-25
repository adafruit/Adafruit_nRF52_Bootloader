import os
import shutil
import sys
import subprocess
import time

subprocess.run("rm -rf _build*", shell=True)
subprocess.run("rm -rf bin/*", shell=True)

travis = False
if "TRAVIS" in os.environ and os.environ["TRAVIS"] == "true":
    travis = True

success_count = 0
fail_count = 0
exit_status = 0

build_format = '| {:30} | {:9} '
build_separator = '-' * 54

all_boards = []
for entry in os.scandir("src/boards"):
    all_boards.append(entry.name)

#sha, version = build_info.get_version_info()

total_time = time.monotonic()

print(build_separator)
print((build_format + '| {:5} |').format('Board', 'Result', 'Time'))
print(build_separator)

for board in all_boards:
    bin_directory = "bin/{}/".format(board)
    os.makedirs(bin_directory, exist_ok=True)

    start_time = time.monotonic()
    make_result = subprocess.run("make -j 4 BOARD={} combinehex genpkg".format(board), shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    build_duration = time.monotonic() - start_time

    if make_result.returncode == 0:
        success = "\033[32msucceeded\033[0m"
        success_count += 1
    else:
        exit_status = make_result.returncode
        success = "\033[31mfailed\033[0m   "
        fail_count += 1

    for entry in os.scandir("_build-{}".format(board)):
        for extension in ["zip", "hex"]:
            if entry.name.endswith(extension) and "nosd" not in entry.name:
                shutil.copy(entry.path, bin_directory)

    if travis:
        print('travis_fold:start:build-{}\\r'.format(board))

    print((build_format + '| {:.2f}s |').format(board, success, build_duration))

    if make_result.returncode != 0:
        print(make_result.stdout.decode("utf-8"))
    if travis:
        print('travis_fold:end:build-{}\\r'.format(board))

# Build Summary
total_time = time.monotonic() - total_time
print(build_separator)
print("Build Sumamary: {} \033[32msucceeded\033[0m, {} \033[31mfailed\033[0m and took {:.2f}s".format(success_count, fail_count, total_time))
print(build_separator)

sys.exit(exit_status)

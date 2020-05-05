#! /usr/bin/env python3

import os
import os.path
import sys
import uritemplate
import glob

sys.path.append("adabot")
import github_requests as github

exit_status = 0

filepaths = list(glob.iglob('../bin/*/*', recursive=True))
filepaths.sort()

for full_filename in filepaths:
    filename = os.path.basename(full_filename)
    url_vars = {}
    url_vars["name"] = filename
    url = uritemplate.expand(os.environ["UPLOAD_URL"], url_vars)
    headers = {"content-type": "application/octet-stream"}
    print(url)
    with open(full_filename, "rb") as f:
        response = github.post(url, data=f, headers=headers)
    if not response.ok:
        if response.status_code == 422 and response.json().get("errors", [{"code":""}])[0]["code"] == "already_exists":
            print("File already uploaded. Skipping.")
            continue
        print("Upload of {} failed with {}.".format(filename, response.status_code))
        print(response.text)
        sys.exit(response.status_code)

sys.exit(exit_status)

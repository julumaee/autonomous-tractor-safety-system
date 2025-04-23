import os

LICENSE_HEADER = """\
# Copyright 2024 Eemil Kulmala, University of Oulu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""

def should_add_header(file_path):
    try:
        with open(file_path, 'r') as f:
            content = f.read(500)
            return 'Apache License' not in content
    except Exception as e:
        print(f"Skipping {file_path}: {e}")
        return False

def add_license_header(file_path):
    try:
        with open(file_path, 'r') as f:
            content = f.read()
        with open(file_path, 'w') as f:
            f.write(LICENSE_HEADER + content)
        print(f"Added license to: {file_path}")
    except Exception as e:
        print(f"Failed to add license to {file_path}: {e}")

def walk_and_process(src_root):
    for root, _, files in os.walk(src_root):
        # Skip any directory that contains 'depthai-ros'
        if 'depthai-ros' in root.split(os.sep):
            continue

        for file in files:
            if file.endswith('.py'):
                full_path = os.path.join(root, file)
                if should_add_header(full_path):
                    add_license_header(full_path)

if __name__ == "__main__":
    walk_and_process(os.path.abspath("src"))

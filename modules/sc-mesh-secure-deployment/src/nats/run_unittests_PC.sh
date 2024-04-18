#!/bin/bash

# python virtualenv
python3 -m venv unittest
source unittest/bin/activate

# install dependencies to virtualenv
pip install coverage==7.4.4  # this is for testing purpose
pip install -r requirements.txt

# discover and run unittests
coverage run -m unittest discover -v
report=$(coverage report -m)

# print report lines starting with "TOTAL"
echo "$report" | grep -e "^src" -e "^mdm" -e "^tests"

# deactivate virtualenv
deactivate


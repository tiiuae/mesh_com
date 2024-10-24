#!/bin/bash

# preconditions
if [ ! -f "$(pwd)/$(basename $0)" ]; then
    echo "Script is not being executed in the same folder"
    exit 1
fi

# python virtualenv
python3 -m venv unittest_cbma
source unittest_cbma/bin/activate

# install dependencies to virtualenv
pip install -r ./cbma/requirements.txt
pip install -r ./requirements.txt
# install testing only related dependencies
pip install pytest pytest-cov

# discover and run unittests
pytest --cov=cbma --cov-report term-missing -v --ignore=lucius/unittests --ignore=debug_tests --ignore=cbma/unittest --ignore=cbma/tests --ignore=tests |& tee ./cbma_coverage_report.txt

## deactivate virtualenv
deactivate

# Clean up __pycache__ directories
find . -type d -name '__pycache__' -exec rm -rf {} +
# Clean up unittest venv
rm -rf unittest_cbma
# Clean up coverage tool's SQL database
rm -f .coverage


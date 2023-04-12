"""
Helper context import for tests
"""
# pylint: disable=import-error, wrong-import-position, unused-import
import sys
import os
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..')))
import validations.validation as validation

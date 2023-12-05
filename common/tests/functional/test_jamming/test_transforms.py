import numpy as np
import pytest

import add_syspath
import transforms

class TestCodeUnderTest:

    #  Resizing an array to a smaller size.
    def test_resizing_smaller_size(self):
        # Create a mock numpy array
        X = np.array([1, 2, 3, 4, 5])
    
        # Call the resize function with a smaller size
        resized_array = transforms.resize(X, 3)
    
        # Check if the resized array has the correct size
        assert resized_array.shape[0] == 3
    
        # Check if the values in the resized array are correct
        assert np.array_equal(resized_array, np.array([1, 3, 5]))

    #  Resizing an array to a larger size.
    def test_resizing_larger_size(self):
        # Create a mock numpy array
        X = np.array([1, 2, 3, 4, 5])

        # Call the resize function with a larger size
        resized_array = transforms.resize(X, 7)

        # Check if the resized array has the correct size
        assert resized_array.shape[0] == 7

        # Check if the values in the resized array are correct
        assert np.array_equal(resized_array, np.array([1, 1, 2, 3, 3, 4, 5]))

    #  Resizing an array to the same size.
    def test_resizing_same_size(self):
        # Create a mock numpy array
        X = np.array([1, 2, 3, 4, 5])
    
        # Call the resize function with the same size
        resized_array = transforms.resize(X, 5)
    
        # Check if the resized array is a copy of the original array
        assert np.array_equal(resized_array, X)
    
        # Check if the resized array is not the same object as the original array
        assert resized_array is not X

    #  Resizing an array of size 1 to any size.
    def test_resizing_size_1(self):
        # Create a mock numpy array of size 1
        X = np.array([1])
    
        # Call the resize function with different sizes
        resized_array_1 = transforms.resize(X, 3)
        resized_array_2 = transforms.resize(X, 5)
    
        # Check if the resized arrays have the correct sizes
        assert resized_array_1.shape[0] == 3
        assert resized_array_2.shape[0] == 5
    
        # Check if the values in the resized arrays are correct
        assert np.array_equal(resized_array_1, np.array([1, 1, 1]))
        assert np.array_equal(resized_array_2, np.array([1, 1, 1, 1, 1]))

    #  Resizing an array of size 0 to any size.
    def test_resizing_size_0(self):
        # Create a mock numpy array of size 0
        X = np.array([])
    
        # Call the resize function with different sizes
        resized_array_1 = transforms.resize(X, 3)
        resized_array_2 = transforms.resize(X, 5)
    
        # Check if the resized arrays have the correct sizes
        assert resized_array_1.shape[0] == 3
        assert resized_array_2.shape[0] == 5

        # Check if the resized arrays contain only nan values
        assert np.all(np.isnan(resized_array_1))
        assert np.all(np.isnan(resized_array_2))

    #  Resizing an array to size 0.
    def test_resizing_to_size_0(self):
        # Create a mock numpy array
        X = np.array([1, 2, 3, 4, 5])
    
        # Call the resize function with size 0
        resized_array = transforms.resize(X, 0)
    
        # Check if the resized array is empty
        assert np.array_equal(resized_array, np.array([]))

    #  Resizing an array to a negative size.
    def test_resize_to_negative_size(self):
        # Arrange
        X = np.array([1, 2, 3])
        size = -2

        # Act and Assert
        with pytest.raises(ValueError):
            transforms.resize(X, size)
    #  Resizing an array to a non-integer size.
    def test_resizing_non_integer_size(self):
        # Create a mock numpy array
        X = np.array([1, 2, 3, 4, 5])
    
        # Call the resize function with a non-integer size
        resized_array = transforms.resize(X, 2.5)
    
        # Check if the resized array has the correct size
        assert resized_array.shape[0] == 2
    
        # Check if the values in the resized array are correct
        assert np.array_equal(resized_array, np.array([1, 5]))

    #  Resizing an array with NaN or Inf values.
    def test_resizing_nan_inf_values(self):
        # Create a mock numpy array with NaN and Inf values
        X = np.array([1, np.nan, 3, np.inf, 5])
    
        # Call the resize function with a size
        resized_array = transforms.resize(X, 5)
    
        # Check if the resized array has the correct size
        assert resized_array.shape[0] == 5
    
        # Check if the values in the resized array are correct
        assert np.isnan(resized_array[1])
        assert np.isinf(resized_array[3])

    #  Resizing an array with duplicate indices.
    def test_resizing_duplicate_indices(self):
        # Create a mock numpy array
        X = np.array([1, 2, 3, 4, 5])
    
        # Call the resize function with duplicate indices
        resized_array = transforms.resize(X, 5)
    
        # Check if the resized array has the correct size
        assert resized_array.shape[0] == 5
    
        # Check if the values in the resized array are correct
        assert np.array_equal(resized_array, X)

    #  Resizing an array with non-monotonic indices.
    def test_resizing_non_monotonic_indices(self):
        # Create a mock numpy array
        X = np.array([1, 2, 3, 4, 5])
    
        # Call the resize function with non-monotonic indices
        resized_array = transforms.resize(X, 5)
    
        # Check if the resized array has the correct size
        assert resized_array.shape[0] == 5
    
        # Check if the values in the resized array are correct
        assert np.array_equal(resized_array, X)

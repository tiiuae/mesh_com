import os
from unittest.mock import MagicMock

import numpy as np
import pytest
import torch

from ldpi import TrainedModel
from options import LDPIOptions


class TestTrainedModel:
    @pytest.fixture
    def mock_options(self):
        options = LDPIOptions()
        options.threshold_type = 'max'
        options.n = 4
        options.l = 60
        options.model_name = 'ResCNN'
        return options

    class TestTrainedModelFileExistence:
        def test_model_files_existence(self, mock_options):
            model = TrainedModel(mock_options)
            model_path = model.store_models_path

            # Define the expected file paths
            expected_files = [
                os.path.join(model_path, 'best_model_with_center.pth'),
                os.path.join(model_path, 'scripted_quantized_model.pth'),
                os.path.join(model_path, 'traced_model.pth')
            ]

            # Check if each file exists and can be loaded with PyTorch
            for file_path in expected_files:
                full_path = os.path.abspath(file_path)
                print(f"Checking file: {full_path}")
                assert os.path.exists(full_path), f"File not found: {file_path}"

                # Attempt to load the model file with PyTorch
                try:
                    if 'scripted_quantized_model.pth' in full_path or 'traced_model.pth' in full_path:
                        # For model files, use torch.jit.load
                        torch.jit.load(full_path)
                    else:
                        # For other .pth files, use torch.load
                        torch.load(full_path)
                except Exception as e:
                    assert False, f"Failed to load file {file_path} with PyTorch: {e}"

    @pytest.mark.parametrize("threshold_type, expected_threshold", [
        ('ninety_nine', 0.99),
        ('near_max', 0.9999),
        ('max', 1.0),
        ('hundred_one', 1.01),
    ])
    def test_initialize_threshold(self, mock_options, threshold_type, expected_threshold):
        mock_options.threshold_type = threshold_type
        model = TrainedModel(mock_options)

        # Mocking the thresholds as they are not set in the __init__ method
        model.ninety_nine_threshold = 0.99
        model.near_max_threshold = 0.9999
        model.max_threshold = 1.0
        model.hundred_one_threshold = 1.01

        assert model._initialize_threshold() == expected_threshold, (f"Threshold initialization failed for threshold_type: {threshold_type}")

    @pytest.fixture(scope="function")
    def flow_data(self, mock_options):
        num_packets = mock_options.n
        len_packets = mock_options.l

        # Create mock flow data
        flow_key_1 = (b'\xC0\xA8\x00\x01', 12345, b'\xC0\xA8\x00\x02', 80, 6)
        flow_key_2 = (b'\xC0\xA8\x00\x03', 12345, b'\xC0\xA8\x00\x04', 80, 17)

        # Example numpy arrays representing packet data
        flow_data_1 = [np.random.rand(len_packets).astype(np.float32) for _ in range(num_packets)]
        flow_data_2 = [np.random.rand(len_packets).astype(np.float32) for _ in range(num_packets)]

        return [(flow_key_1, flow_data_1), (flow_key_2, flow_data_2)]

    @pytest.fixture
    def black_list(self):
        # Define a set of blacklisted IPs
        return {b'\xC0\xA8\x00\x01', b'\xC0\xA8\x00\x05'}

    def test_prepare_tensors(self, mock_options, flow_data, black_list):
        mock_model = TrainedModel(mock_options)
        flow_data_copy = flow_data.copy()
        keys, tensor = mock_model.prepare_tensors(flow_data, black_list)

        # Diagnostic print statements (optional, for debugging purposes)
        print("Processed Keys:", keys)
        print("Blacklist:", black_list)
        print("Tensor Shape:", tensor.shape if tensor.numel() > 0 else "Empty Tensor")

        # Check if blacklisted flows are excluded
        for key in keys:
            assert key[0] not in black_list, "Blacklisted flow key found in the processed keys"

        # Check if the output tensor has the correct shape and type
        assert isinstance(tensor, torch.Tensor), "Output is not a torch.Tensor"
        assert tensor.dim() == 2, "Output tensor does not have the correct number of dimensions"

        expected_non_blacklisted_flows = len([f for f, _ in flow_data_copy if f[0] not in black_list])

        # Check if the number of non-blacklisted flows matches the tensor's first dimension
        assert tensor.shape[0] == expected_non_blacklisted_flows, f"Output tensor does not match the number of non-blacklisted flows {tensor.shape}"

        # Check if the second dimension of the tensor matches the expected shape per flow
        assert tensor.shape[1] == mock_model.args.l * mock_model.args.n, "Output tensor does not have the correct shape per flow"

    def test_infer_anomalies(self, mock_options):
        # Create a mock TrainedModel instance
        model = TrainedModel(mock_options)

        # Mock the model's encode method
        model.model = MagicMock()
        mock_encoded_output = torch.tensor([[0.1, 0.2], [0.3, 0.4], [0.5, 0.6]])
        model.model.encode.return_value = mock_encoded_output

        # Define the center and threshold for testing
        model.center = torch.tensor([0.2, 0.3])
        model.chosen_threshold = 0.05

        # Prepare mock network flows tensor
        mock_network_flows = torch.rand((3, model.args.n * model.args.l))

        # Invoke the infer method
        anomalies = model.infer(mock_network_flows)

        # Calculate expected anomalies based on mock data
        expected_distance = torch.sum((mock_encoded_output - model.center) ** 2, dim=1)
        expected_anomalies = (expected_distance >= model.chosen_threshold)

        assert torch.all(anomalies.eq(expected_anomalies)), "Infer method did not correctly identify anomalies"

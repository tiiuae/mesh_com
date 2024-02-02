from unittest.mock import Mock, patch

import dpkt
import pytest

from options import SnifferOptions
from sniffer import Sniffer


class TestSniffer:
    @pytest.fixture
    def mock_options(self):
        return SnifferOptions()

    @pytest.fixture
    def sniffer(self, mock_options):
        return Sniffer(mock_options)

    def test_initialization(self, sniffer, mock_options):
        assert sniffer.args == mock_options
        assert sniffer.timeout_ns == int(mock_options.timeout * 1e+9)
        assert isinstance(sniffer.flows_tcp, dict)
        assert isinstance(sniffer.flows_udp, dict)

    @pytest.mark.parametrize("eth_type, is_batman, is_broadcast, nested_eth_type, should_unpack", [
        (dpkt.ethernet.ETH_TYPE_IP, False, False, None, True),
        (dpkt.ethernet.ETH_TYPE_IP6, False, False, None, True),
        (17157, True, True, None, False),  # batman type, broadcast (should return early)
    ])
    def test_process_packet(self, eth_type, is_batman, is_broadcast, nested_eth_type, should_unpack, sniffer):
        len_eth = 14  # length of Ethernet header
        len_bat = 10  # length of Batman header
        mock_eth_data = b'\x00' * 60

        # Creating separate mock buffers for batman and normal scenarios
        batman_mock_buf = b'\x00' * len_eth + b'\x00' * len_bat + mock_eth_data
        normal_mock_buf = b'\x00' * 60

        mock_buf = batman_mock_buf if is_batman else normal_mock_buf
        mock_eth = Mock()
        mock_eth.type = eth_type

        if is_batman:
            mock_eth.dst = b'\xff\xff\xff\xff\xff\xff' if is_broadcast else b'\x00' * 6
            if not is_broadcast and nested_eth_type is not None:
                # Create a nested Ethernet frame
                mock_nested_eth = Mock()
                mock_nested_eth.type = nested_eth_type
                mock_eth.data = mock_nested_eth

        with patch('dpkt.ethernet.Ethernet', return_value=mock_eth):
            if should_unpack:
                with patch.object(sniffer, 'unpack_ip') as mock_unpack_ip:
                    sniffer.process_packet(123456789, mock_buf)
                    if nested_eth_type is not None and not is_broadcast:
                        mock_unpack_ip.assert_called_with(mock_eth.data, 123456789)
                    else:
                        mock_unpack_ip.assert_called_with(mock_eth, 123456789)
            else:
                sniffer.process_packet(123456789, mock_buf)

    @patch('dpkt.ethernet.Ethernet')
    @patch('logging.warning')
    def test_process_packet_exception(self, mock_logging, mock_ethernet, sniffer):
        # Simulate a dpkt.dpkt.NeedData exception
        mock_ethernet.side_effect = dpkt.dpkt.NeedData
        sniffer.process_packet(123456789, b'')
        mock_logging.assert_called_once_with('Packet -1 in PCAP file is truncated')

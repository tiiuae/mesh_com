import unittest
import os, json
import warnings
from unittest.mock import patch, MagicMock, mock_open
from src.comms_command import Command
from src.comms_settings import CommsSettings
from src.comms_common import STATUS
from src.comms_status import CommsStatus


cmd_dict_org = {
    "api_version": 1,
    "role": "drone",  # sleeve, drone, gcs
    "radios": [
        {
            "radio_index": "0",
            "ssid": "test_mesh2",
            "key": "1234567890",
            "country": "US",  # all radios must have the same country
            "frequency": "2412",
            "frequency_mcc": "2412",  # multiradio not supporting
            "priority": "long_range",
            "tx_power": "15",
            "mptcp": "disable",
            "slaac": "usb0 wlp3s0",
            "mode": "mesh",  # ap+mesh_scc, mesh, halow
            "mesh_vif": "wlp2s0",
        },
        {
            "radio_index": "1",
            "ssid": "test_mesh",
            "key": "1234567890",
            "country": "US",  # all radios must have the same country
            "frequency": "5220",
            "frequency_mcc": "2412",  # multiradio not supporting
            "priority": "long_range",
            "slaac": "usb0 wlp3s0",
            "tx_power": "15",
            "mptcp": "disable",
            "mode": "mesh",  # ap+mesh_scc, mesh, halow
            "mesh_vif": "wlp3s0",  # this needs to be correct
        },
        {
            "radio_index": "2",
            "ssid": "test_mesh3",
            "key": "1234567890",
            "country": "US",  # all radios must have the same country
            "frequency": "5190",
            "frequency_mcc": "2412",  # multiradio not supporting
            "priority": "long_range",
            "tx_power": "30",
            "slaac": "usb0 wlp3s0",
            "mptcp": "disable",
            "mode": "halow",  # ap+mesh_scc, mesh, halow
            "mesh_vif": "halow1",
        },
    ],
}


class CommandTests(unittest.TestCase):
    """
    Test the Command class
    """

    @classmethod
    def tearDownClass(cls):
        current_path = os.getcwd()
        test_files = ["0_mesh.conf", "1_mesh.conf", "2_mesh.conf"]

        # Clean up the test files
        for file_name in test_files:
            file_path = os.path.join(current_path, "tests", file_name)
            if os.path.exists(file_path):
                os.remove(file_path)

    @patch("src.validation.is_valid_interface")
    def setUp(self, mock_is_valid_interface):
        """
        Setup the test
        """
        logger = MagicMock()

        self.cs: [CommsStatus, ...] = [
            CommsStatus(logger, "0"),
            CommsStatus(logger, "1"),
            CommsStatus(logger, "2"),
        ]
        self.settings = CommsSettings(self.cs, logger)

        with warnings.catch_warnings():
            # interface validation will be tested separately
            mock_is_valid_interface.return_value = True

            warnings.simplefilter("ignore", ResourceWarning)

            jsoned = json.dumps(cmd_dict_org)
            ret, mesh_status = self.settings.handle_mesh_settings(
                jsoned, "./tests", "mesh.conf"
            )
            self.assertEqual(ret, "OK", msg=f"ret: {ret}, mesh_status: {mesh_status}")

        self.comms_status = MagicMock()
        logger = MagicMock()
        nats_mode = False
        self.command = Command(self.comms_status, logger, nats_mode)
        self.assertEqual(self.command.comms_status, self.comms_status)
        self.assertEqual(self.command.logger, logger)
        self.assertEqual(self.command.nats_mode, nats_mode)


    @patch('src.comms_command.subprocess.run')
    @patch('src.comms_command.os.replace')
    @patch('src.comms_command.open')
    def test_handle_command_for_not_supported(self, mock_subprocess, mock_replace, mock_open):
        """
        Test the handle_command method
        """
        index = "0"

        cc = MagicMock()
        mock_subprocess.return_value = MagicMock()
        mock_replace.return_value = MagicMock()
        mock_open.return_value = MagicMock()
        msg = '{"api_version": 1, "cmd": "revoke"}'  # lower case revoke not supported
        ret, _, _ = self.command.handle_command(msg, cc)
        self.assertEqual(ret, "FAIL")

        msg = '{"api_version": 1, "cmd": "REBOOT"}'
        ret, _, _ = self.command.handle_command(msg, cc)
        self.assertEqual(ret, "FAIL") # command not supported

    @patch('src.comms_command.subprocess.run')
    @patch('src.comms_command.os.replace')
    def test_handle_command_for_settings(self, mock_subprocess, mock_replace):
        index = "0"

        cc = MagicMock()
        mock_subprocess.return_value = MagicMock()
        mock_replace.return_value = MagicMock()

        msg = '{"api_version": 1, "cmd": "APPLY", "radio_index":' + f"{index}" + '}'
        ret, _, _ = self.command.handle_command(msg, cc)
        # should fail as no settings are set
        self.assertEqual(ret, "FAIL")

        msg = ('{ "api_version": 1, "role": "sleeve", "radios": [ {"radio_index": "0", '
               '"ssid": "test_mesh2", "key": "1234567890", "country": "US", "frequency": "2412", '
               '"frequency_mcc": "2412", "mptcp": "disable", "priority": "high_throughput", '
               '"tx_power": "15", "mode": "mesh", "mesh_vif": "wlp2s0", "slaac": ""}]}')
        ret, _ = self.settings.handle_mesh_settings(msg, "/tmp/")
        self.assertEqual(ret, "OK")

    @patch('subprocess.run')
    def test_apply_mission_config_success(self, mock_subprocess):
        logger = MagicMock()

        cs: [CommsStatus, ...] = [
            CommsStatus(logger, "0"),
            CommsStatus(logger, "1"),
            CommsStatus(logger, "2"),
        ]
        settings = CommsSettings(self.cs, logger)

        msg = ('{ "api_version": 1, "role": "sleeve", "radios": [ {"radio_index": "0", '
               '"ssid": "test_mesh2", "key": "1234567890", "country": "US", "frequency": "2412", '
               '"frequency_mcc": "2412", "mptcp": "disable", "priority": "high_throughput", '
               '"tx_power": "15", "mode": "mesh", "mesh_vif": "wlp2s0", "slaac": ""}]}')
        ret, _ = settings.handle_mesh_settings(msg, "/opt/")
        self.assertEqual(ret, "OK")

        os.popen('cp /opt/0_mesh_stored.conf /opt/0_mesh.conf')

        mock_subprocess.return_value.returncode = 0

        cc = MagicMock()
        command = Command(cc, MagicMock(), nats_mode=False)
        command.ctrl_mptcp = True
        command.comms_status[0].mesh_cfg_status = STATUS.mesh_cfg_stored
        ret, info, _ = command.handle_command('{"api_version": 1, "cmd": "APPLY", "radio_index": "0"}', cc)
        self.assertEqual(ret, "OK", msg=f"ret: {ret}, info: {info}")
        self.assertEqual(info, "Mission configurations applied")

    @patch('os.replace')
    @patch('subprocess.run')
    def test_apply_mission_config_fail_replace(self, mock_subprocess, mock_replace):
        logger = MagicMock()

        cs: [CommsStatus, ...] = [
            CommsStatus(logger, "0"),
            CommsStatus(logger, "1"),
            CommsStatus(logger, "2"),
        ]
        settings = CommsSettings(self.cs, logger)

        msg = ('{ "api_version": 1, "role": "sleeve", "radios": [ {"radio_index": "0", '
               '"ssid": "test_mesh2", "key": "1234567890", "country": "US", "frequency": "2412", '
               '"frequency_mcc": "2412", "mptcp": "disable", "priority": "high_throughput", '
               '"tx_power": "15", "mode": "mesh", "mesh_vif": "wlp2s0", "slaac": ""}]}')
        ret, _ = settings.handle_mesh_settings(msg, "/opt/")
        self.assertEqual(ret, "OK")

        os.popen('cp /opt/0_mesh_stored.conf /opt/0_mesh.conf')

        mock_subprocess.return_value.returncode = 0

        mock_replace.side_effect = Exception("Error replacing active config file")
        cc = MagicMock()
        command = Command(cc, MagicMock(), nats_mode=False)
        command.comms_status[0].mesh_cfg_status = STATUS.mesh_cfg_stored
        command.radio_index = "0"
        ret, info, _ = command.handle_command(
            '{"api_version": 1, "cmd": "APPLY", "radio_index": "0"}', cc)
        self.assertEqual(ret, "FAIL")
        self.assertEqual(info, "Error replacing active config file")

    @patch('os.replace')
    @patch('subprocess.run')
    def test_apply_mission_config_fail_subprocess(self, mock_subprocess, mock_replace):
        logger = MagicMock()

        cs: [CommsStatus, ...] = [
            CommsStatus(logger, "0"),
            CommsStatus(logger, "1"),
            CommsStatus(logger, "2"),
        ]
        settings = CommsSettings(self.cs, logger)

        msg = ('{ "api_version": 1, "role": "sleeve", "radios": [ {"radio_index": "0", '
               '"ssid": "test_mesh2", "key": "1234567890", "country": "US", "frequency": "2412", '
               '"frequency_mcc": "2412", "mptcp": "disable", "priority": "high_throughput", '
               '"tx_power": "15", "mode": "mesh", "mesh_vif": "wlp2s0", "slaac": ""}]}')
        ret, _ = settings.handle_mesh_settings(msg, "/opt/")
        self.assertEqual(ret, "OK")

        os.popen('cp /opt/0_mesh_stored.conf /opt/0_mesh.conf')

        mock_subprocess.return_value.returncode = 1
        mock_replace.return_value = None
        cc = MagicMock()
        cc.mesh_cfg_status[0] = STATUS.mesh_cfg_stored
        command = Command(cc, MagicMock(), nats_mode=False)
        command.comms_status[0].mesh_cfg_status = STATUS.mesh_cfg_stored
        command.radio_index = "0"
        ret, info, _ = command.handle_command(
            '{"api_version": 1, "cmd": "APPLY", "radio_index": "0"}', cc)
        self.assertEqual(ret, "FAIL")
        self.assertTrue(info.startswith("mesh starting failed"), msg=f"info: {info}")

    def test_apply_mission_config_no_config(self):
        cc = MagicMock()
        cc.mesh_cfg_status[0] = STATUS.mesh_cfg_not_stored
        command = Command(cc, MagicMock(), nats_mode=False)
        command.radio_index = "0"
        ret, info = command._Command__apply_mission_config()
        self.assertEqual(ret, "FAIL")
        self.assertEqual(info, "No setting to apply")


    @patch('src.comms_command.subprocess.run')
    def test_handle_commands_down(self, mock_subprocess):

        cc = MagicMock()
        mock_subprocess.return_value = MagicMock(returncode=0, stdout=b'', stderr=b'')

        #cmd_dict = '{"api_version": 1, "cmd": "DOWN", "radio_index": "0"}'
        self.command.radio_index = "0"
        ret, _ = self.command._Command__radio_down_single()

        mock_subprocess.assert_has_calls([
            unittest.mock.call(['/opt/S9011sNatsMesh', 'stop', 'id0'], shell=False, check=True, capture_output=True),
            unittest.mock.call(['/opt/S90APoint', 'stop', 'id0'], shell=False, check=True, capture_output=True)
        ], any_order=True)

    @patch('src.comms_command.subprocess.run')
    def test_handle_commands_down_exception(self, mock_subprocess):

        cmd_dict = '{"api_version": 1, "cmd": "DOWN", "radio_index": "0"}'
        cc = MagicMock()

        mock_subprocess.side_effect = [
            FileNotFoundError("No such file or directory: '/opt/S9011sNatsMesh'"),
        ]

        with self.assertRaises(FileNotFoundError):
            ret, _ = self.command.handle_command(cmd_dict, cc)

    @patch('src.comms_command.subprocess.run')
    def test_handle_commands_up(self, mock_subprocess):

        cc = MagicMock()
        mock_subprocess.return_value = MagicMock(returncode=0, stdout=b'', stderr=b'')
        self.command.radio_index = "0"

        ret, _ = self.command._Command__radio_up_single()

        mock_subprocess.assert_has_calls([
            unittest.mock.call(['/opt/S9011sNatsMesh', 'start', 'id0'], shell=False, check=True, capture_output=True),
            unittest.mock.call(['/opt/S90APoint', 'start', 'id0'], shell=False, check=True, capture_output=True)
        ], any_order=True)

    @patch('src.comms_command.subprocess.run')
    def test_handle_commands_up_exception(self, mock_subprocess):

        cmd_dict = '{"api_version": 1, "cmd": "UP", "radio_index": "0"}'
        cc = MagicMock()

        mock_subprocess.side_effect = [
            FileNotFoundError("No such file or directory: '/opt/S9011sNatsMesh'"),
        ]

        with self.assertRaises(FileNotFoundError):
            ret, _ = self.command.handle_command(cmd_dict, cc)

    @patch('src.comms_command.subprocess.run')
    def test_handle_commands_down_all(self, mock_subprocess):
        cc = MagicMock()
        mock_subprocess.return_value = MagicMock(returncode=0, stdout=b'', stderr=b'')

        self.command.radio_index = "0"
        cc.settings.radio_index =[0]

        ret, _ = self.command._Command__radio_down_all(cc)

        mock_subprocess.assert_has_calls([
            unittest.mock.call(['/opt/S9011sNatsMesh', 'stop', 'id0'], shell=False, check=True,
                               capture_output=True),
            unittest.mock.call(['/opt/S90APoint', 'stop', 'id0'], shell=False, check=True,
                               capture_output=True)
        ], any_order=True)

    @patch('src.comms_command.subprocess.run')
    def test_handle_commands_up_all(self, mock_subprocess):
        cc = MagicMock()
        mock_subprocess.return_value = MagicMock(returncode=0, stdout=b'', stderr=b'')

        self.command.radio_index = "0"
        cc.settings.radio_index = [0]

        ret, _ = self.command._Command__radio_up_all(cc)

        mock_subprocess.assert_has_calls([
            unittest.mock.call(['/opt/S9011sNatsMesh', 'start', 'id0'], shell=False, check=True,
                               capture_output=True),
            unittest.mock.call(['/opt/S90APoint', 'start', 'id0'], shell=False, check=True,
                               capture_output=True)
        ], any_order=True)

    @patch('src.comms_command.subprocess.run')
    def test_handle_commands_revoke(self, mock_subprocess):

        cc = MagicMock()
        mock_subprocess.return_value = MagicMock(returncode=0, stdout=b'', stderr=b'')
        self.command.radio_index = "0"
        self.command.ctrl_mptcp = True

        ret, _ = self.command._Command__revoke(cc)

        self.assertEqual(ret, "OK")

        mock_subprocess.assert_has_calls([
            unittest.mock.call(['/opt/S9011sNatsMesh', 'restart', 'id0'], shell=False, check=True,
                               capture_output=True),
            unittest.mock.call(['/opt/S90APoint', 'restart', 'id0'], shell=False, check=True,
                               capture_output=True),
            unittest.mock.call(['/opt/S90mptcp', 'stop'], shell=False, check=True,
                               capture_output=True)
        ], any_order=True)

    @patch('subprocess.run')
    def test_debug_success(self, mock_subprocess):
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = b"Debug output"
        cc = MagicMock()
        cc.debug_mode_enabled = True
        command = Command([], MagicMock(), nats_mode=False)
        ret, info, data = self.command._Command__debug(cc, "'ls'")
        self.assertEqual(ret, "OK")
        self.assertEqual(info, "'['ls']' DEBUG COMMAND done")
        self.assertEqual(data, "RGVidWcgb3V0cHV0")

    @patch('subprocess.run')
    def test_debug_fail(self, mock_subprocess):
        mock_subprocess.return_value.returncode = 1
        cc = MagicMock()
        cc.debug_mode_enabled = True
        command = Command([], MagicMock(), nats_mode=False)
        ret, info, data = self.command._Command__debug(cc, "'ls'")
        self.assertEqual(ret, "FAIL")
        self.assertEqual(info, "'{p}' DEBUG COMMAND failed")
        self.assertEqual(data, "")

    def test_debug_disabled(self):
        cc = MagicMock()
        cc.debug_mode_enabled = False
        command = Command([], MagicMock(), nats_mode=False)
        ret, info, data = self.command._Command__debug(cc, "'ls'")
        self.assertEqual(ret, "FAIL")
        self.assertEqual(info, "DEBUG COMMAND disabled: cc.debug_mode_enabled: False")
        self.assertEqual(data, "")


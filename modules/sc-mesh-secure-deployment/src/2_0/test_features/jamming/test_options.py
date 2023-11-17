import add_syspath
from options import Options


class TestCodeUnderTest:

    #  Options object is created successfully
    def test_options_object_created_successfully(self):
        options = Options()
        assert isinstance(options, Options)

    #  Validate configuration fails when mesh interface channel width is not set to 20 MHz
    def test_validate_configuration_fails_mesh_interface_channel_width_not_set_to_20MHz(self, mocker):
        options = Options()
        options.validate_configuration = mocker.Mock(return_value=False)
        assert options.validate_configuration() == False

    #  Validate configuration fails when channels5 list includes invalid channels
    def test_validate_configuration_fails_channels5_list_includes_invalid_channels(self, mocker):
        options = Options()
        options.channels5 = [36, 40, 44, 48, 149, 153, 157, 162]
        assert options.validate_configuration() == False

    #  Options object attributes are set correctly
    def test_options_object_attributes_set_correctly(self):
        options = Options()
        options = Options()
        options.jamming_osf_orchestrator = 'fd01::1'
        options.port = 8080
        options.starting_channel = 36
        options.waiting_time = 15
        options.channels5 = [36, 40, 44, 48, 149, 153, 157, 161]
        options.threshold = 0.0
        options.osf_interface = 'tun0'
        options.scan_interface = 'wlp1s0'
        options.mesh_interface = 'wlp1s0'
        options.debug = True
        options.min_rows = 16
        options.periodic_scan = 60
        options.periodic_recovery_switch = 20
        options.periodic_target_freq_broadcast = 10
        options.spectrum_data_expiry_time = 5

        assert options.jamming_osf_orchestrator == 'fd01::1'
        assert options.port == 8080
        assert options.starting_channel == 36
        assert options.waiting_time == 15
        assert options.channels5 == [36, 40, 44, 48, 149, 153, 157, 161]
        assert options.threshold == 0.0
        assert options.osf_interface == 'tun0'
        assert options.scan_interface == 'wlp1s0'
        assert options.mesh_interface == 'wlp1s0'
        assert options.debug == True
        assert options.min_rows == 16
        assert options.periodic_scan == 60
        assert options.periodic_recovery_switch == 20
        assert options.periodic_target_freq_broadcast == 10
        assert options.spectrum_data_expiry_time == 5

    #  Validate configuration succeeds when mesh interface channel width is set to 20 MHz
    def test_validate_configuration_succeeds_mesh_interface_channel_width_set_to_20MHz(self, mocker):
        options = Options()
        options.validate_configuration = mocker.Mock(return_value=True)
        assert options.validate_configuration() == True

    #  Validate configuration succeeds when channels5 list includes valid channels
    def test_validate_configuration_succeeds_channels5_list_includes_valid_channels(self, mocker):
        options = Options()
        options.channels5 = [36, 40, 44, 48, 149, 153, 157, 161]
        assert options.validate_configuration() == True

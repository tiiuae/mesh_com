import add_syspath
from options import Options


class TestCodeUnderTest:

    #  Options object is created successfully
    def test_options_object_created_successfully(self):
        options = Options()
        assert isinstance(options, Options)


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


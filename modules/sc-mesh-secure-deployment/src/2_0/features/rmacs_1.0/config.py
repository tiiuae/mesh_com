import yaml
import os
import logging

# Define default configuration  
default_config = {
    "MULTICAST_CONFIG": {
        'wlp1s0': {'group': 'ff02::1', 'port': 12345},
        'halow1': {'group': 'ff02::1', 'port': 12346},
        'osf0': {'group': 'ff13::39', 'port': 12345},
    },
    "RMACS_Config": {
        "orchestra_node": False,
        "primary_radio" : "wlp1s0",
        "osf_interface": "osf0",
        "nw_interface": "wlp1s0",
        "halow_interface": "halow",
        "phy_interface": "phy1",
        "driver": "ath10k",
        "bridge_interface": "br-lan",
        "addr_prefix": "fdd8",
        "radio_interfaces": ["wlp1s0", "halow1", "osf0"],
        "radio_interface": [
            ["ath9k", "wlp3s0"],
            ["ath10k", "wlp2s0"],
            ["halow", "halow1"]
        ],
        "traffic_threshold": 2500,
        "phy_error_limit": 1500,
        "tx_timeout_limit": 1,
        "monitoring_sleep_time": 30,
        "max_error_check": 4,
        "cooldown_period": 30,
        "freq_quality_report": {
            5180: {"nodes": {}, "Average_quality": 1},
            5200: {"nodes": {}, "Average_quality": 1},
            5220: {"nodes": {}, "Average_quality": 1},
            5240: {"nodes": {}, "Average_quality": 1},
        },
        "freq_list": [5180, 5200, 5220, 5240],
        "seq_limit": 3,
        "hop_interval": 5,
        "stability_threshold": 2,
        "starting_frequency": "5180",
        "report_expiry_threshold": 30,
        "channel_bandwidth": 20,
        "beacon_count": 10,
        "client_beacon_count": 1,
        "buffer_period": 2,
        "channel_quality_index_threshold": 1,
        "bcqi_threshold_time": 30,
        "debug": False,
        "update_flag": True,
        "periodic_recovery_switch": 3.0,
        "periodic_operating_freq_broadcast": 15.0,
        "log_file": "/var/log/rmacs.log",
        "bin_file": "/root/sample.bin",
    }
}
def merge_dicts(default, user):
    """Recursively merge user config into default config."""
    for key, value in user.items():
        if isinstance(value, dict) and key in default:
            default[key] = merge_dicts(default[key], value)
        else:
            default[key] = value
    return default

def load_config(config_file_path):
    """Load configuration by merging default and user-provided configs."""
    if os.path.exists(config_file_path):
        with open(config_file_path, 'r') as file:
            user_config = yaml.safe_load(file)
            config = merge_dicts(default_config.copy(), user_config)
    else:
        config = default_config

    return config

def create_default_config(config_file_path):
    """Create a default configuration file if it does not exist."""
    if not os.path.exists(config_file_path):
        with open(config_file_path, 'w') as file:
            yaml.dump(default_config, file, sort_keys=False)
        logging.info(f"RMACS Default configuration file created at {config_file_path}")
    else:
        logging.info(f"RMACS Configuration file already exists at {config_file_path}")
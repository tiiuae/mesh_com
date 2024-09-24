import yaml
import os
import logging

# Define default configuration
default_config = {
    'customer_config': {
        'name': 'XYZ',
        'country': 'UAE',
    },
    'radio_config': {
        'always_on_radios': ['halow1'],
        'on_demand_radios': ['wlp1s0'],
    },
    'traffic_monitor': {
        'monitor_interval': 10,
        'threshold_high': 400,
        'threshold_low': 100,
        'traffic_error_threshold': 10,
        'dwell_time': 60,
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
            yaml.dump(default_config, file)
        logging.info(f"Default configuration file created at {config_file_path}")
    else:
        logging.info(f"Configuration file already exists at {config_file_path}")

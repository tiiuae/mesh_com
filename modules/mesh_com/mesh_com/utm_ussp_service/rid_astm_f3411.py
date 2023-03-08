import json
import unittest

class rid_astm_f3411:

    def __init__(self, timestamp, operational_status, position_lat, position_lng, position_alt, position_accuracy_h, position_accuracy_v, position_extrapolated, height_distance, height_reference, track, speed, timestamp_accuracy, speed_accuracy, vertical_speed):
        self.timestamp = timestamp
        self.operational_status = operational_status
        self.position_lat = position_lat
        self.position_lng = position_lng
        self.position_alt = position_alt
        self.position_accuracy_h = position_accuracy_h
        self.position_accuracy_v = position_accuracy_v
        self.position_extrapolated = position_extrapolated
        self.height_distance = height_distance
        self.height_reference = height_reference
        self.track = track
        self.speed = speed
        self.timestamp_accuracy = timestamp_accuracy
        self.speed_accuracy = speed_accuracy
        self.vertical_speed = vertical_speed

    def encode_data_fields(self):
        data = {
            "timestamp": self.timestamp,
            "operational_status": self.operational_status,
            "position": {
                "lat": self.position_lat,
                "lng": self.position_lng,
                "alt": self.position_alt,
                "accuracy_h": self.position_accuracy_h,
                "accuracy_v": self.position_accuracy_v,
                "extrapolated": self.position_extrapolated
            },
            "height": {
                "distance": self.height_distance,
                "reference": self.height_reference
            },
            "track": self.track,
            "speed": self.speed,
            "timestamp_accuracy": self.timestamp_accuracy,
            "speed_accuracy": self.speed_accuracy,
            "vertical_speed": self.vertical_speed
        }
        return json.dumps(data)


class TestEncoder(unittest.TestCase):

    def setUp(self):
        self.timestamp = "2022-07-18T14:22:52.541652+00:00"
        self.operational_status = "Airborne"
        self.position_lat = 46.9754225199202
        self.position_lng = 7.475076017275803
        self.position_alt = 620.0
        self.position_accuracy_h = "HAUnkown"
        self.position_accuracy_v = "VAUnknown"
        self.position_extrapolated = False
        self.height_distance = 50.0
        self.height_reference = "TakeoffLocation"
        self.track = 181.6975641099569
        self.speed = 4.91
        self.timestamp_accuracy = 0.0
        self.speed_accuracy = "SA3mps"
        self.vertical_speed = 0.0

    def test_encode_data_fields(self):
        encoder = rid_astm_f3411(
            self.timestamp,
            self.operational_status,
            self.position_lat,
            self.position_lng,
            self.position_alt,
            self.position_accuracy_h,
            self.position_accuracy_v,
            self.position_extrapolated,
            self.height_distance,
            self.height_reference,
            self.track,
            self.speed,
            self.timestamp_accuracy,
            self.speed_accuracy,
            self.vertical_speed
        )
        expected_output = '{"timestamp": "2022-07-18T14:22:52.541652+00:00", "operational_status": "Airborne", "position": {"lat": 46.9754225199202, "lng": 7.475076017275803, "alt": 620.0, "accuracy_h": "HAUnkown", "accuracy_v": "VAUnknown", "extrapolated": false}, "height": {"distance": 


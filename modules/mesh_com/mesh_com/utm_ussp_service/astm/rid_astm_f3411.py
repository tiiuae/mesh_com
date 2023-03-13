import json
import unittest

class rid_astm_f3411:

    def __init__(self):
        self.timestamp = "Unkown"
        self.operational_status = "Unkown"
        self.position_lat = "Unkown"
        self.position_lng = "Unkown"
        self.position_alt = "Unkown"
        self.position_accuracy_h = "HAUnkown"
        self.position_accuracy_v = "VAUnkown"
        self.position_extrapolated = "Unkown"
        self.height_distance = "Unkown"
        self.height_reference = "Unkown"
        self.track = "Unkown"
        self.speed = "Unkown"
        self.timestamp_accuracy = "Unkown"
        self.speed_accuracy = "Unkown"
        self.vertical_speed = "Unkown"

    def init_data_fields(self, timestamp, operational_status, position_lat, position_lng, position_alt, position_accuracy_h, position_accuracy_v, position_extrapolated, height_distance, height_reference, track, speed, timestamp_accuracy, speed_accuracy, vertical_speed):
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


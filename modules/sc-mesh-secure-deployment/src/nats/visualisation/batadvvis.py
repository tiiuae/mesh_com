"""
batadv-vis helper for python
"""
import subprocess
import json
import time


class BatAdvVis:
    """
    batadv-vis helper class
    """
    command = 'batadv-vis'

    def __init__(self):
        self.latest_topology = "{}"
        self.thread_running = True

    @staticmethod
    def remove_interfaces(visual_lines):
        """
        remove unwanted TT lines

        :return: str
        """
        lines = visual_lines.split("\n")
        new_visual = ""
        for line in lines:
            if line and "TT" not in line:
                new_visual += line + "\n"

        while '  ' in new_visual:
            new_visual = new_visual.replace('  ', ' ')

        return new_visual

    def get(self, format_type="jsondoc"):
        """
        get topology
        :return: str
        """
        if format_type in ("dot", "jsondoc", "json"):
            try:
                # returns byte string
                raw_data = subprocess.check_output([self.command,
                                                    '-f',
                                                    format_type],
                                                    stderr=subprocess.DEVNULL)
                if raw_data == 255:
                    raw_data = b'{}'
            except (FileNotFoundError, subprocess.CalledProcessError):
                raw_data = b'{}'

            if raw_data != b'{}':
                if format_type == "jsondoc":
                    json_data = json.loads(raw_data.decode('utf8'))

                    for i in range(raw_data.count(b"clients")):
                        json_data["vis"][i].pop("clients", None)

                    raw_data = json.dumps(json_data).encode()
                elif format_type == "dot":
                    raw_data = self.remove_interfaces(raw_data.decode('UTF-8')).encode()

        else:
            raw_data = b'{}'
        # return string
        return raw_data.decode('utf-8')

    def run(self):
        """
        Run method for task

        :return: None
        """
        while self.thread_running:
            self.latest_topology = self.get()
            time.sleep(1.0)


# Real user is mesh_executor
if __name__ == "__main__":
    # usage tip
    batadvvis = BatAdvVis()

    # get topology and example pretty print
    obj = json.loads(batadvvis.get())
    print(json.dumps(obj, indent=3))

import threading
import logging
from . import batadvvis
from . import batstat


class MeshTelemetry:
    """
    Mesh network telemetry collector
    """

    def __init__(self, loop_interval: int = 1000, logger: logging.Logger = None):
        self.thread_visual = None
        self.thread_stats = None
        # milliseconds to seconds
        self.interval = float(loop_interval / 1000.0)
        self.logger = logger
        self.batman_visual = batadvvis.BatAdvVis(self.interval * 0.2)
        self.batman = batstat.Batman(self.interval * 0.2)
        self.visualisation_enabled = False

    def mesh_visual(self):
        """
        Get mesh visualisation

        :return: mesh visualisation
        """
        return (
            f"[{self.batman_visual.latest_topology},"
            f"{self.batman.latest_stat}]".replace(": ", ":").replace(", ", ",")
        )

    def run(self):
        """
        Run method to start collecting visualisation telemetry

        :return: -
        """
        self.thread_visual = threading.Thread(
            target=self.batman_visual.run
        )  # create thread
        self.thread_visual.start()  # start thread
        self.thread_stats = threading.Thread(target=self.batman.run)  # create thread
        self.thread_stats.start()  # start thread
        self.visualisation_enabled = True  # publisher enabled

    def stop(self):
        """
        Stop method for collecting telemetry

        :return: -
        """
        self.visualisation_enabled = False  # publisher disabled
        if self.batman_visual.thread_running:
            self.batman_visual.thread_running = False  # thread loop disabled
            self.thread_visual.join()  # wait for thread to finish
        if self.batman.thread_running:
            self.batman.thread_running = False  # thread loop disabled
            self.thread_stats.join()  # wait for thread to finish

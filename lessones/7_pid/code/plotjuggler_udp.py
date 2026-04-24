import csv
import json
import os
import socket
import time
from typing import Optional, TextIO

PLOTJUGGLER_HOST = "127.0.0.1"
PLOTJUGGLER_PORT = 9870
CSV_PREFIX = "plotjuggler_data"


class PlotJugglerUdpClient:
    def __init__(
        self,
        host=PLOTJUGGLER_HOST,
        port=PLOTJUGGLER_PORT,
        save=False,
        dump_duration=5.0,
        output_dir=".",
    ):
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.save = save
        self.dump_duration = dump_duration
        self.output_dir = output_dir
        self.start_time = time.time()
        self.csv_file: Optional[TextIO] = None
        self.csv_writer: Optional[csv.DictWriter] = None
        self.csv_closed = False

        if self.save:
            self._open_csv()

    def send(self, target, current):
        timestamp = time.time()
        data = {
            "time": timestamp,
            "position/setpoint": target,
            "position/feedback": current,
        }
        self.socket.sendto(json.dumps(data).encode("utf-8"), self.address)

        if self.save and not self.csv_closed:
            if self.csv_file is None or self.csv_writer is None:
                return

            self.csv_writer.writerow(data)
            self.csv_file.flush()

            if timestamp - self.start_time >= self.dump_duration:
                self.close_csv()

    def _open_csv(self):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, f"{CSV_PREFIX}_{timestamp}.csv")

        fieldnames = ["time", "position/setpoint", "position/feedback"]
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writeheader()

    def close_csv(self):
        if self.csv_file is None or self.csv_closed:
            return

        self.csv_file.close()
        self.csv_closed = True

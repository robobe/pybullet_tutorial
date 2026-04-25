import csv
import json
import os
import socket
import time
from enum import Enum
from typing import Optional, TextIO

PLOTJUGGLER_HOST = "127.0.0.1"
PLOTJUGGLER_PORT = 9870
CSV_PREFIX = "plotjuggler_data"


class TimeBase(Enum):
    WALL = "wall"
    RELATIVE = "relative"


class PlotJugglerUdpClient:
    def __init__(
        self,
        host=PLOTJUGGLER_HOST,
        port=PLOTJUGGLER_PORT,
        save=False,
        dump_duration=5.0,
        output_dir=".",
        time_base=TimeBase.WALL,
        extra_fieldnames=None,
    ):
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.save = save
        self.dump_duration = dump_duration
        self.output_dir = output_dir
        self.start_time = time.time()
        self.time_base = time_base
        self.extra_fieldnames = extra_fieldnames or []
        self.csv_file: Optional[TextIO] = None
        self.csv_writer: Optional[csv.DictWriter] = None
        self.csv_closed = False

        if self.save:
            self._open_csv()

    def send(self, target, current, extra_data=None):
        now = time.time()
        timestamp = self._get_message_time(now)
        data = {
            "time": timestamp,
            "position/setpoint": target,
            "position/feedback": current,
        }
        if extra_data:
            data.update(extra_data)

        self.socket.sendto(json.dumps(data).encode("utf-8"), self.address)

        if self.save and not self.csv_closed:
            if self.csv_file is None or self.csv_writer is None:
                return

            self.csv_writer.writerow(data)
            self.csv_file.flush()

            if now - self.start_time >= self.dump_duration:
                self.close_csv()

    def _get_message_time(self, now):
        if self.time_base == TimeBase.RELATIVE:
            return now - self.start_time

        return now

    def _open_csv(self):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, f"{CSV_PREFIX}_{timestamp}.csv")

        fieldnames = [
            "time",
            "position/setpoint",
            "position/feedback",
            *self.extra_fieldnames,
        ]
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writeheader()

    def close_csv(self):
        if self.csv_file is None or self.csv_closed:
            return

        self.csv_file.close()
        self.csv_closed = True

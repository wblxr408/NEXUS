"""Align a flight-controller boot clock to the ROS Unix clock."""


class BootTimeAligner:
    def __init__(self, reset_threshold_ns=1_000_000_000, future_tolerance_ns=50_000_000):
        self.reset_threshold_ns = int(reset_threshold_ns)
        self.future_tolerance_ns = int(future_tolerance_ns)
        self.offset_ns = None
        self.last_boot_ns = None
        self.last_ros_ns = None
        self.reset_count = 0

    def align_us(self, sample_us, arrival_unix_ns):
        return self.align_ns(int(sample_us) * 1000, arrival_unix_ns)

    def align_ns(self, sample_ns, arrival_unix_ns):
        boot_ns = int(sample_ns)
        arrival_ns = int(arrival_unix_ns)
        if boot_ns <= 0 or arrival_ns <= 0:
            raise ValueError("sample and arrival timestamps must be positive")
        reset = self.last_boot_ns is not None and boot_ns + self.reset_threshold_ns < self.last_boot_ns
        if self.offset_ns is None or reset:
            self.offset_ns = arrival_ns - boot_ns
            self.last_ros_ns = None
            if reset:
                self.reset_count += 1
        else:
            # The smallest observed arrival-minus-sample offset is the least
            # delayed estimate of the epoch relationship.
            self.offset_ns = min(self.offset_ns, arrival_ns - boot_ns)
        aligned = self.offset_ns + boot_ns
        if aligned > arrival_ns + self.future_tolerance_ns:
            self.offset_ns = arrival_ns - boot_ns
            aligned = arrival_ns
            self.reset_count += 1
        if self.last_ros_ns is not None:
            aligned = max(aligned, self.last_ros_ns + 1)
        self.last_boot_ns, self.last_ros_ns = boot_ns, aligned
        return aligned

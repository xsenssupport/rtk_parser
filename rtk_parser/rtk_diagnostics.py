#!/usr/bin/env python3

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

class RTKDiagnostics:
    @staticmethod
    def create_diagnostic_status(name: str, level: int, message: str, values: dict) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        status.level = level
        status.message = message
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        return status

    @staticmethod
    def check_satellite_count(count: int) -> tuple:
        if count >= 8:
            return (DiagnosticStatus.OK, "Good satellite count")
        elif count >= 5:
            return (DiagnosticStatus.WARN, "Low satellite count")
        else:
            return (DiagnosticStatus.ERROR, "Insufficient satellites")

    @staticmethod
    def check_snr(snr: float) -> tuple:
        if snr >= 35:
            return (DiagnosticStatus.OK, "Good signal strength")
        elif snr >= 25:
            return (DiagnosticStatus.WARN, "Marginal signal strength")
        else:
            return (DiagnosticStatus.ERROR, "Poor signal strength")

    @staticmethod
    def check_baseline(distance: float) -> tuple:
        if distance <= 10000:  # 10 km
            return (DiagnosticStatus.OK, "Baseline within range")
        elif distance <= 20000:  # 20 km
            return (DiagnosticStatus.WARN, "Baseline approaching limit")
        else:
            return (DiagnosticStatus.ERROR, "Baseline too long")
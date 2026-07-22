import sys
import types
import unittest


fake_can = types.ModuleType("can")
fake_can.BusABC = object
fake_can.CanError = Exception


class Message:
    def __init__(self, arbitration_id, data, is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = is_extended_id


fake_can.Message = Message
fake_can.interface = types.SimpleNamespace(Bus=None)
sys.modules.setdefault("can", fake_can)

from can_motor_controller_mit.can_interface import CANInterface


class FakeBus:
    def __init__(self, responses):
        self.responses = list(responses)
        self.sent = []

    def send(self, message):
        self.sent.append(message)

    def recv(self, timeout):
        return self.responses.pop(0) if self.responses else None


class MITCANInterfaceTests(unittest.TestCase):
    def test_id2_mit_request_uses_0x502_and_accepts_f1_status(self):
        status = Message(0x002, bytes.fromhex("F1800080080001"))
        bus = FakeBus([status])
        interface = CANInterface(timeout=0.01)
        interface._bus = bus
        payload = bytes.fromhex("8000800000000800")

        response = interface.send_mit_and_receive(2, payload, timeout=0.01)

        self.assertEqual(bus.sent[0].arbitration_id, 0x502)
        self.assertEqual(bus.sent[0].data, payload)
        self.assertIs(response, status)


if __name__ == "__main__":
    unittest.main()

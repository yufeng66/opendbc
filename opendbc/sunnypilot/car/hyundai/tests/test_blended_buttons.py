import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.hyundai.carcontroller import BLENDED_RES_COPIES
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags

CLU11 = 0x4f1
FRAME_NANOS = 10_000_000  # 100 Hz control loop


class TestBlendedCLU11Buttons(unittest.TestCase):
  """CLU11 button injection on CAN-FD blended cars (2023 Palisade) under stock long: phase-locked to the cluster's
  own CLU11 so our frames don't collide with it on bus 1."""

  def setUp(self):
    CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_PALISADE_2023)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HYUNDAI_PALISADE_2023)
    assert CP.flags & HyundaiFlags.CAN_CANFD_BLENDED and not CP.openpilotLongitudinalControl
    self.CI = CarInterface(CP, CP_SP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    self.bus = CanBus(CP).ECAN
    self.t = 0
    self._step()  # the first parser update doesn't report the message yet

  def _step(self, cluster_frame=True, cancel=False, resume=False, brake=False):
    self.t += FRAME_NANOS
    msgs = [self.packer.make_can_msg("CLU11", self.bus, {"CF_Clu_CruiseSwState": 0})] if cluster_frame else []
    self.CI.update([(self.t, msgs)])
    self.CI.CS.out.brakePressed = brake
    CC = structs.CarControl()
    CC.cruiseControl.cancel = cancel
    CC.cruiseControl.resume = resume
    _, can_sends = self.CI.apply(CC.as_reader(), structs.CarControlSP(), self.t)
    return [s for s in can_sends if s[0] == CLU11]

  def test_cancel_only_right_after_a_cluster_frame(self):
    assert len(self._step(cancel=True)) == 1
    assert self._step(cluster_frame=False, cancel=True) == []
    assert len(self._step(cancel=True)) == 1

  def test_cancel_held_while_braking(self):
    for _ in range(10):
      assert self._step(cancel=True, brake=True) == []
    # still requested after the brake is released (stock cruise didn't cancel itself): it goes out then
    assert len(self._step(cancel=True)) == 1

  def test_resume_bursts_are_short_and_phase_locked(self):
    sent = []
    for i in range(100):  # 1 s, cluster CLU11 at 50 Hz
      fresh = i % 2 == 0
      msgs = self._step(cluster_frame=fresh, resume=True)
      if msgs:
        assert fresh
        sent.append(len(msgs))
    assert sent, "resume was never sent"
    assert all(n == BLENDED_RES_COPIES for n in sent)
    # ~10 Hz windows of up to 3 bursts: well under the 25 copies x ~5 cycles per window sent before
    assert sum(sent) <= 1.0 / 0.15 * 3 * BLENDED_RES_COPIES


if __name__ == "__main__":
  unittest.main()

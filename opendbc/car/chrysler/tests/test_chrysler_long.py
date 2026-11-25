import unittest

from opendbc.can.parser import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, CarControllerParams, ChryslerSafetyFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState


def _params(candidate, alpha_long):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
  return CP, CP_SP


class TestChryslerLongitudinal(unittest.TestCase):
  def test_off_by_default(self):
    for candidate in (CAR.RAM_1500_5TH_GEN, CAR.RAM_HD_5TH_GEN):
      with self.subTest(candidate=candidate):
        CP, _ = _params(candidate, alpha_long=False)
        self.assertTrue(CP.alphaLongitudinalAvailable)
        self.assertFalse(CP.openpilotLongitudinalControl)
        self.assertTrue(CP.pcmCruise)
        self.assertFalse(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.LONG)

  def test_toggle_enables_ram_only(self):
    for candidate in (CAR.RAM_1500_5TH_GEN, CAR.RAM_HD_5TH_GEN):
      with self.subTest(candidate=candidate):
        CP, _ = _params(candidate, alpha_long=True)
        self.assertTrue(CP.openpilotLongitudinalControl)
        self.assertFalse(CP.pcmCruise)
        self.assertEqual(CP.longitudinalActuatorDelay, 0.5)
        self.assertTrue(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.LONG)
        self.assertEqual(CP.minEnableSpeed, -1.0)

    for candidate in (CAR.CHRYSLER_PACIFICA_2018, CAR.JEEP_GRAND_CHEROKEE):
      with self.subTest(candidate=candidate):
        CP, _ = _params(candidate, alpha_long=True)
        self.assertFalse(CP.alphaLongitudinalAvailable)
        self.assertFalse(CP.openpilotLongitudinalControl)
        self.assertFalse(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.LONG)

  def test_ram_hd_safety_flag_includes_platform(self):
    CP, _ = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.RAM_HD)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.LONG)

  def _das3(self, CI, CC):
    # The first frame's counter does not match the parser's expected counter, so keep two.
    das3 = []
    for _ in range(4):
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), 0)
      das3.extend(msg for msg in sends if msg[0] == 0x1F4)
      if len(das3) >= 2:
        break
    self.assertGreaterEqual(len(das3), 2)
    parser = CANParser("chrysler_ram_hd_generated", [("DAS_3", 50)], 0)
    parser.update([(0, das3[-2:])])
    return parser.vl["DAS_3"]

  def test_controller_torque_and_brake(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])

    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    CC.actuators.accel = 1.0
    CC.actuators.longControlState = LongCtrlState.pid
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.TORQUE_FROM_ACCEL)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST_MAX"], 1)
    self.assertAlmostEqual(sig["ACC_DECEL"], CarControllerParams.INACTIVE_DECEL, places=2)
    self.assertEqual(sig["ACC_DECEL_REQ"], 0)
    self.assertLessEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.MAX_TORQUE_CMD)

    CC.actuators.accel = 5.0
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.ACCEL_MAX * CarControllerParams.TORQUE_FROM_ACCEL)
    self.assertLess(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.MAX_TORQUE)

    CC.actuators.accel = -2.0
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], 0)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST_MAX"], 0)
    self.assertAlmostEqual(sig["ACC_DECEL"], -2.0, places=2)
    self.assertEqual(sig["ACC_DECEL_REQ"], 1)

    # Between 0 and the brake threshold the command coasts. Untuned threshold from RamBetaLong.
    CC.actuators.accel = -0.1
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], 0)
    self.assertAlmostEqual(sig["ACC_DECEL"], CarControllerParams.INACTIVE_DECEL, places=2)

  def test_pedal_forces_inactive_frame(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    CI.CS.out.gasPressed = True

    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    CC.actuators.accel = 1.5
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ACC_ACTIVE"], 0)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], 0)
    self.assertAlmostEqual(sig["ACC_DECEL"], CarControllerParams.INACTIVE_DECEL, places=2)

  def test_stock_long_does_not_send_das3(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=False)
    CI = CarInterface(CP, CP_SP)
    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    CC.actuators.accel = 1.0
    _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), 0)
    self.assertFalse(any(msg[0] == 0x1F4 for msg in sends))

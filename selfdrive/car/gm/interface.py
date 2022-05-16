#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.gm.values import CAR, CruiseButtons, \
                                    AccState, CarControllerParams
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

GearShifter = car.CarState.GearShifter
ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    params = CarControllerParams()
    return params.ACCEL_MIN, params.ACCEL_MAX

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)
    ret.carName = "gm"
    ret.safetyModel = car.CarParams.SafetyModel.gm
    ret.pcmCruise = False  # stock cruise control is kept off

    # Presence of a camera on the object bus is ok.
    # Have to go to read_only if ASCM is online (ACC-enabled cars),
    # or camera is on powertrain bus (LKA cars without ACC).

    ret.enableGasInterceptor = 0x201 in fingerprint[0]
    ret.openpilotLongitudinalControl = ret.enableGasInterceptor

    tire_stiffness_factor = 0.5

    ret.minSteerSpeed = 10 * CV.KPH_TO_MS
    ret.steerRateCost = 0.35  #0.3625  default : 2.0
    ret.steerActuatorDelay = 0.2  #0.1925  default: 0.2 Default delay, not measured yet

    ret.minEnableSpeed = -1
    ret.mass = 1625. + STD_CARGO_KG
    ret.safetyModel = car.CarParams.SafetyModel.gm
    ret.wheelbase = 2.60096
    ret.steerRatio = 16.8
    ret.steerRatioRear = 0.
    ret.centerToFront = ret.wheelbase * 0.49 # wild guess
    ret.lateralTuning.init('lqr')

    ret.lateralTuning.lqr.scale = 1975.0
    ret.lateralTuning.lqr.ki = 0.032
    ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
    ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
    ret.lateralTuning.lqr.c = [1., 0.]
    ret.lateralTuning.lqr.k = [-110., 451.]
    ret.lateralTuning.lqr.l = [0.33, 0.318]
    ret.lateralTuning.lqr.dcGain = 0.00225




    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.longitudinalTuning.kpBP = [0.0, 35.0]
    ret.longitudinalTuning.kpV = [0.5, 0.4]
    ret.longitudinalTuning.kiBP = [0., 35.]
    ret.longitudinalTuning.kiV = [0.18, 0.2]

    if ret.enableGasInterceptor:
      ret.gasMaxBP = [0.0, 5.0, 9.0, 35.0]
      ret.gasMaxV =  [0.4, 0.5, 0.7, 0.7]

    ret.stoppingControl = False
    ret.startAccel = 0.4

    ret.steerLimitTimer = 2.5
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)

    ret.cruiseState.enabled = self.CS.main_on or self.CS.adaptive_Cruise
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    buttonEvents = []

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != CruiseButtons.UNPRESS:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == CruiseButtons.DECEL_SET:
        be.type = ButtonType.decelCruise
      elif but == CruiseButtons.CANCEL:
        be.type = ButtonType.cancel
      elif but == CruiseButtons.MAIN:
        be.type = ButtonType.altButton3
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents

    events = self.create_common_events(ret)

    if ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.belowEngageSpeed)
    if self.CS.park_brake:
      events.add(EventName.parkBrake)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(car.CarEvent.EventName.belowSteerSpeed)
# if self.CP.enableGasInterceptor:
    if self.CS.adaptive_Cruise and ret.brakePressed:
      events.add(EventName.pedalPressed)
      self.CS.adaptive_Cruise = False
      self.CS.enable_lkas = True

    # handle button presses
    if not self.CS.main_on and self.CP.enableGasInterceptor:
      for b in ret.buttonEvents:
        # if (b.type == ButtonType.decelCruise and not b.pressed) and not self.CS.adaptive_Cruise:
        #   self.CS.adaptive_Cruise = True
        #   self.CS.enable_lkas = True
        #   events.add(EventName.buttonEnable)
        # if (b.type == ButtonType.accelCruise and not b.pressed) and not self.CS.adaptive_Cruise:
        #   self.CS.adaptive_Cruise = False
        #   self.CS.enable_lkas = False
        #   events.add(EventName.buttonEnable)
        if (b.type == ButtonType.cancel and b.pressed) and self.CS.adaptive_Cruise:
          self.CS.adaptive_Cruise = False
          self.CS.enable_lkas = True
          events.add(EventName.buttonCancel)
    elif self.CS.main_on:
      self.CS.adaptive_Cruise = False
      self.CS.enable_lkas = True


    #Added by jc01rho inspired by JangPoo
    if self.CS.main_on  and ret.cruiseState.enabled and ret.gearShifter == GearShifter.drive and ret.vEgo > 2 and not ret.brakePressed :
      if ret.cruiseState.available and not ret.seatbeltUnlatched and not ret.espDisabled and self.flag_pcmEnable_able :

        if self.flag_pcmEnable_initialSet == False :
          self.initial_pcmEnable_counter = self.initial_pcmEnable_counter + 1
          if self.initial_pcmEnable_counter > 750 :
            events.add(EventName.pcmEnable)
            self.flag_pcmEnable_initialSet = True
            self.flag_pcmEnable_able = False
            self.initial_pcmEnable_counter = 0
        else :
          events.add(EventName.pcmEnable)
          self.flag_pcmEnable_able = False
          # self.flag_pcmEnable_initialSet = True
          # self.initial_pcmEnable_counter = 0
    else  :
      self.flag_pcmEnable_able = True
    ret.events = events.to_msg()


    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out

  def apply(self, c):
    hud_v_cruise = c.hudControl.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    # For Openpilot, "enabled" includes pre-enable.
    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators,
                               hud_v_cruise, c.hudControl.lanesVisible,
                               c.hudControl.leadVisible, c.hudControl.visualAlert)

    self.frame += 1
    return can_sends


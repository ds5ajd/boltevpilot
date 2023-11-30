#!/usr/bin/env python3
from typing import List
from cereal import car
from common.numpy_fast import interp
from math import fabs
from common.conversions import Conversions as CV
from selfdrive.car.gm.values import CAR, CruiseButtons, AccState, CarControllerParams
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params
GearShifter = car.CarState.GearShifter
ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

def get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED):
  x = ANGLE * (desired_angle + ANGLE_OFFSET)
  sigmoid = x / (1 + fabs(x))
  return (SIGMOID_SPEED * sigmoid * v_ego) + (SIGMOID * sigmoid) + (SPEED * v_ego)



class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.keep_Lat_When_Brake = Params().get_bool('KeepLatWhenBrake')


  def _update(self, c: car.CarControl) -> car.CarState:
    pass


  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    params = CarControllerParams(CP)
    v_current_kph = current_speed * CV.MS_TO_KPH
    # return params.ACCEL_MIN, params.ACCEL_MAX
    accel_max_bp = [10., 20., 30., 50., 70., 80.] #=> [10., 20., 50.]
    accel_max_v =[0.9, 1.0, 0.85, 0.82, 0.9, 0.95] #=> [0.9, 1.0, 0.95]

    return params.ACCEL_MIN, interp(v_current_kph, accel_max_bp, accel_max_v)

  # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_bolt_euv(desired_angle, v_ego):
    ANGLE = 0.0758345580739845
    ANGLE_OFFSET = 0.31396926577596984
    SIGMOID_SPEED = 0.04367532050459129
    SIGMOID = 0.43144116109994846
    SPEED = -0.002654134623368279
    return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED)

  @staticmethod
  def get_steer_feedforward_bolt(desired_angle, v_ego):
    ANGLE = 0.06370624896135679
    ANGLE_OFFSET = 0.#32536345911579184
    SIGMOID_SPEED = 0.06479105208670367
    SIGMOID = 0.34485246691603205
    SPEED = -0.0010645479469461995
    return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED)


  def get_steer_feedforward_function(self):
      return self.get_steer_feedforward_bolt
    #else:
      #return CarInterfaceBase.get_steer_feedforward_default

    
  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, disable_radar=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "gm"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
    ret.pcmCruise = False  # stock cruise control is kept off

    # GM port is a community feature
    # TODO: make a port that uses a car harness and it only intercepts the camera
    # ret.communityFeature = True

    #for neokii integration
    ret.maxSteeringAngleDeg = 1000.
    # for neokii integration end.

    # Presence of a camera on the object bus is ok.
    # Have to go to read_only if ASCM is online (ACC-enabled cars),
    # or camera is on powertrain bus (LKA cars without ACC).
    # ret.enableGasInterceptor = 0x201 in fingerprint[0]
    ret.enableGasInterceptor = Params().get_bool('CommaPedal')
    ret.restartForceAccel = Params().get_bool('RestartForceAccel')
    ret.openpilotLongitudinalControl = ret.enableGasInterceptor



    tire_stiffness_factor = 0.5

    ret.minSteerSpeed = 11 * CV.KPH_TO_MS
    ret.steerRateCost = 0.35 # def : 2.0
    ret.steerActuatorDelay = 0.2  # def: 0.2 Default delay, not measured yet

    ret.minEnableSpeed = -1
    ret.mass = 1625. + STD_CARGO_KG
    ret.wheelbase = 2.60096
    ret.steerRatio = 16.85
    ret.steerRatioRear = 0.
    ret.centerToFront = ret.wheelbase * 0.49 # wild guess
    ret.disableLateralLiveTuning = True

    lateral_control = Params().get("LateralControl", encoding='utf-8')
    if lateral_control == 'INDI':
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGainBP = [0.]
      ret.lateralTuning.indi.innerLoopGainV = [3.3]
      ret.lateralTuning.indi.outerLoopGainBP = [0.]
      ret.lateralTuning.indi.outerLoopGainV = [2.8]
      ret.lateralTuning.indi.timeConstantBP = [0.]
      ret.lateralTuning.indi.timeConstantV = [1.4]
      ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
      ret.lateralTuning.indi.actuatorEffectivenessV = [1.8]
    elif lateral_control == 'LQR':
      ret.lateralTuning.init('lqr')

      ret.lateralTuning.lqr.scale = 1950.0
      ret.lateralTuning.lqr.ki = 0.032
      ret.lateralTuning.lqr.dcGain = 0.002237852961363602

      ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
      ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
      ret.lateralTuning.lqr.c = [1., 0.]
      ret.lateralTuning.lqr.k = [-110.73572306, 451.22718255]
      ret.lateralTuning.lqr.l = [0.3233671, 0.3185757]
      
      
    elif lateral_control == 'PID':
      ret.lateralTuning.init('pid')
      ret.minEnableSpeed = -1
      # ret.minSteerSpeed = 5 * CV.MPH_TO_MS
      ret.mass = 1616. + STD_CARGO_KG
      ret.wheelbase = 2.60096
      ret.steerRatio = 16.8
      ret.steerRatioRear = 0.
      ret.centerToFront = 2.0828 #ret.wheelbase * 0.4 # wild guess
      tire_stiffness_factor = 1.0
      # still working on improving lateral
      ret.steerRateCost = 0.5
      ret.steerActuatorDelay = 0.
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[10., 41.0], [10., 41.0]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.14, 0.24], [0.01, 0.021]]
      ret.lateralTuning.pid.kdBP = [0.]
      ret.lateralTuning.pid.kdV = [0.5]
      ret.lateralTuning.pid.kf = 1. # for get_steer_feedforward_bolt()
      
      
      
      
    else:
      ret.lateralTuning.init('torque')
      ret.lateralTuning.torque.useSteeringAngle = True
      max_lat_accel = 2.6531724862969748 #2.35
      ret.lateralTuning.torque.kp = 1.0 / max_lat_accel
      ret.lateralTuning.torque.kf = 0.1919764879840985 #1.0 / max_lat_accel
      ret.lateralTuning.torque.ki = 0.009054123646805178 #0.1 / max_lat_accel
      ret.lateralTuning.torque.friction = 0.175

      ret.lateralTuning.torque.kd = 1.0
      ret.lateralTuning.torque.deadzone = 0.03


    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    # longitudinal
    ret.longitudinalTuning.kpBP = [0., 18.0 * CV.KPH_TO_MS, 80.0 * CV.KPH_TO_MS] #[0., 18.0 * CV.KPH_TO_MS, 36.0 * CV.KPH_TO_MS, 80. * CV.KPH_TO_MS]
    ret.longitudinalTuning.kpV = [1.10, 1.0, 0.76] #[1.20, 1.0, 0.76], [1.16, 1.1, 0.76]=>good ==>[1.18, 1.17, 1.0, 0.76]
    ret.longitudinalTuning.kiBP = [0., 18.0 * CV.KPH_TO_MS, 80.0 * CV.KPH_TO_MS] #[0., 18.0 * CV.KPH_TO_MS, 36.0 * CV.KPH_TO_MS, 80. * CV.KPH_TO_MS]
    ret.longitudinalTuning.kiV =  [0.140, 0.125, 0.115] #[0.140,0.125,0.115], [0.135, 0.130, 0.115]=> good ==>[0.140, 0.135, 0.130, 0.115]
     
    ret.longitudinalTuning.deadzoneBP = [0., 30.*CV.KPH_TO_MS]
    ret.longitudinalTuning.deadzoneV = [0., 0.10]
    ret.longitudinalActuatorDelayLowerBound = 0.12 #0.12
    ret.longitudinalActuatorDelayUpperBound = 0.35 #0.35
    
    ret.stopAccel = 0.0 #-2.2
    ret.stoppingDecelRate = 4.0 #4.0
    ret.vEgoStopping = 0.3
    ret.vEgoStarting = 0.23 #0.23(good)
    ret.stoppingControl = True
    
    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

    return ret

  # returns a car.CarState
  def update(self, c: car.CarControl, can_strings: List[bytes]) -> car.CarState:
    self.cp.update_strings(can_strings)
    self.cp_loopback.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_loopback)

    ret.cruiseState.enabled = self.CS.main_on or self.CS.adaptive_Cruise
    ret.canValid = self.cp.can_valid and self.cp_loopback.can_valid
    ret.canTimeout = any(cp.bus_timeout for cp in self.can_parsers if cp is not None)
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
    # belowsteerspeed alertevent는 내지 않도록 한다. 텍스트로 표시만 따로 하여 debug ui 출력을 확보한다.
    # if ret.vEgo < self.CP.minSteerSpeed:
    #   events.add(car.CarEvent.EventName.belowSteerSpeed)
    if self.CP.enableGasInterceptor:
      if ret.cruiseState.enabled and ret.brakePressed:
        if self.keep_Lat_When_Brake and not self.CS.adaptive_Cruise and self.CS.enable_lkas : # KeepLatWhenBrake things.
          pass
        else :
          events.add(EventName.pedalPressed)
        self.CS.adaptive_Cruise = False
        self.CS.enable_lkas = True

    # handle button presses
    if self.CP.enableGasInterceptor:
      if not self.CS.main_on : #lat dis-engage
        for b in ret.buttonEvents:
          if (b.type == ButtonType.decelCruise and not b.pressed) and not ret.cruiseState.enabled:
            self.CS.adaptive_Cruise = True
            self.CS.enable_lkas = True
            events.add(EventName.buttonEnable)
            break
          if (b.type == ButtonType.accelCruise and not b.pressed) and not ret.cruiseState.enabled:
            self.CS.adaptive_Cruise = True
            self.CS.enable_lkas = False
            events.add(EventName.buttonEnable)
            break
          if (b.type == ButtonType.cancel and b.pressed) and self.CS.adaptive_Cruise:
            self.CS.adaptive_Cruise = False
            self.CS.enable_lkas = False
            events.add(EventName.buttonCancel)
            break
          if (b.type == ButtonType.altButton3 and b.pressed) and not ret.cruiseState.enabled :
            self.CS.adaptive_Cruise = False
            self.CS.enable_lkas = True
            # events.add(EventName.buttonEnable)
            break
      else :#lat engage
        # self.CS.adaptive_Cruise = False
        # self.CS.enable_lkas = True
        #
        for b in ret.buttonEvents:
          if not self.CS.adaptive_Cruise and (b.type == ButtonType.altButton3 and b.pressed) : #and self.CS.adaptive_Cruise
            self.CS.adaptive_Cruise = False
            self.CS.enable_lkas = False
            break

    else :
      if self.CS.main_on: #wihtout pedal case
        self.CS.adaptive_Cruise = False
        self.CS.enable_lkas = True
        if ret.brakePressed and not self.keep_Lat_When_Brake: # KeepLatWhenBrake things.
          self.CS.enable_lkas = False
          events.add(EventName.pedalPressed)

      else:
        self.CS.adaptive_Cruise = False
        self.CS.enable_lkas = False

    #Added by jc01rho inspired by JangPoo
    #some other logics in interfaces.py
    if self.CS.main_on  and self.CS.enable_lkas and not self.CS.adaptive_Cruise and ret.cruiseState.enabled and ret.gearShifter == GearShifter.drive and ret.vEgo > 2.1 and not ret.brakePressed :
      if ret.cruiseState.available and not ret.seatbeltUnlatched and not ret.espDisabled and self.flag_pcmEnable_able :

        if self.flag_pcmEnable_initialSet == False :
          self.initial_pcmEnable_counter = self.initial_pcmEnable_counter + 1
          if self.initial_pcmEnable_counter > 750 :
            # events.add(EventName.pcmEnable)
            # self.flag_pcmEnable_initialSet = True
            self.flag_pcmEnable_able = False
            self.initial_pcmEnable_counter = 0
        else :
          self.flag_pcmEnable_able = False
          events.add(EventName.pcmEnable)
          # self.flag_pcmEnable_initialSet = True
          # self.initial_pcmEnable_counter = 0
    else  :
      self.flag_pcmEnable_able = True
    ###
    if self.CC.scc_smoother is not None:
      self.CC.scc_smoother.inject_events(events)
    ret.events = events.to_msg()

    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out

  def apply(self, c, controls):
    # return self.CC.update(c, self.CS, controls)
    hud_v_cruise = c.hudControl.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    # For Openpilot, "enabled" includes pre-enable.
    new_actuators, can_sends = self.CC.update(c, c.enabled, self.CS, controls ,
                               c.actuators,
                               hud_v_cruise, c.hudControl.lanesVisible,
                               c.hudControl.leadVisible, c.hudControl.visualAlert)


    return new_actuators, can_sends

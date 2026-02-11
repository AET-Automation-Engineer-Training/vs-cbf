## Simple MPC-CBF controller for Nav2 plugin

```
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    MPCPlanner:
      plugin: "nav2_mpc_cbf_controller::MPCCBFController"
      vx_max: 0.5
      wz_max: 1.0
      gamma_cbf: 1.0
      horizon_T: 2.0
      dt: 0.1
      prediction_horizon: 20
      Q_position: 5.0
      Q_theta: 2.0
      R_vx: 0.5
      R_wz: 0.5
```
# Registered X500 slow approach and PX4 AUTO LAND

Run `a0d1287d4971954615a614f91c190076` completed all 22 Gazebo rehearsal checks on the laptop. The controller tracked a fixed 45° local NED heading with 0.203° RMS heading error and 0.117 m RMS position error during the route. The aircraft reached 5.007 m/s and stayed within 0.25 m/s of 5 m/s for 4.20 continuous seconds.

| Landing measurement | Result |
| --- | ---: |
| Mean downward speed during the approach | 0.0980 m/s |
| Approach duration | 12.80 s |
| Approach descent | 1.255 m |
| Peak logged PX4 AUTO LAND downward setpoint | 0.6000 m/s |
| Peak measured downward speed during PX4 AUTO LAND | 0.6560 m/s |
| Horizontal distance from target at first ON_GROUND sample | 0.0665 m |
| Continuous ground, disarm, and zero actuator confirmation | 2.00 s |

[Repeat run checks and evidence digests](repeat-landing-result.json)

The request specified 200 m. The planner reduced the route to 189.0625 m to stay within the declared time and energy budget. That budget and the full 200 m distance remain separate from this landing result. The checks apply to the registered simulated X500 in the flat validation world.

The initial diagnostic run showed that PX4 blends the LAND speed into its normal downward speed setting at altitude. The repeat configuration set both speed parameters to 0.6 m/s and audited the PX4 trajectory setpoint log. [Initial diagnostic measurements](initial-landing-result.json)

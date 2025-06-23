

#if 0
1. High SoC (>90-95%) → Limit or disable regen
Regen is tapered down linearly above 90% SoC
At 100%, regen is fully disabled
If cell voltage is near 4.20V, regen stops even before 100% SoC

2. Cold Battery (<15°C) → Heavily derated regen
Below 0°C: regen can be entirely blocked
Above 10-15°C: regen returns gradually
Battery preconditioning will actively heat the pack before fast driving or Supercharging to restore regen

3. Variable Regen Tuning Based on Drive Mode
In Sport or Track Mode:
    Regen limit is less conservative (until thermal constraints hit)
In Normal Mode:
    Regen is smoother and tapers early


Torque request is ramped to avoid sudden drop-off
Dashboard shows regen limit bar or faded regen icon
Battery heater used to enable regen in cold

#endif

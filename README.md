# piezo_valve_unit
## Data-driven pressure control based on Gaussian processes

Welcome to this project that demonstrates a data-driven pressure controller for Festo’s VEAE piezoelectric valves.

**Disclaimer**: This code is not plug-and-play due to variability in hardware and control system setups. However, it serves as a comprehensive guideline for building a high-accuracy pressure controller, following the outlined implementation steps.

---

### CAD  
The block and the cap are created in FreeCAD V1.0
_work in progress_:
- [x] Valve block (main part), SLA printed
- [x] Cap, FDM printed
- [ ] Assembly and parts list
- [ ] Construction details


### Electric circuit
The distributor board is created in KiCAD V8.0
_work in progress_:
- [x] Overview
- [x] Layout
- [x] Circuit, KiCad 9.0
- [ ] BOM

### m-code  
This MATLAB code provides a reference implementation of a pressure control framework for soft pneumatic actuators (SPAs), using Festo's VEAE piezo valves. Two valves are assembled in an SLA-printed housing to form a compact 3/3 valve unit. Due to its small size, the valve unit is designed to be mounted directly on or near the actuator. This proximity, combined with volume-adaptive control, enables high-precision pressure regulation.

The code was developed and tested on a MATLAB real-time target machine but is structured to be easily ported to microcontrollers, as it relies on LUT (lookup table) based implementations of the Gaussian processes.

The only external dependency is Peter Corke’s Robotics Toolbox for MATLAB, which is used for motion generation (see <https://petercorke.com/toolboxes/robotics-toolbox/>).

**Procedure:**  
Start with the `pressure_control` script. It will guide you through the complete implementation process.

---

### License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

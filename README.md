# virtual_lablinx
This is Hudson LabLinx simulation to allow for software development before/without getting the real device. Lablinx etc. are all trademarks of Hudson. 


Implemented vs stubbed commands

Using the new registry, the simulator now recognises 177 commands. Of those, 16 are implemented with real behaviour (GetStopSensors, Dispense, Return, ListCommands, AcknowledgeSend, GetIgnoreStop, GetIgnoreStops, HasPlate, IgnoreAllStops, IgnoreStop, IgnoreStopRange, MovePlate, ReceivePlate, SendPlate, ShiftPlates, GetNumTracks). The remaining 161 commands return the 9999 error indicating no mock implementation at this time.
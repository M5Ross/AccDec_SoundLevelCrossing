# AccDec_SoundLevelCrossing

Dcc Accessory Decoder for a Rail Crossing.

Originally derived from an example of library NmraDcc.

Library required:

- [NmraDcc] (public)
- [DFRobotDFPlayerMini] (public)
- [ConfCVlib] (manual installation)
- [DccSerialCom] (manual installation)

Features:

- Up to 4 bars controlled compleately independenly with movements fully customizable.
- 2 outputs for red lights, fixed or alternating configurable.
- control via:
  - 8 sensor inputs for complete authonomus control, up to 4 traks monitorable.
  - DCC control like a simple trak switch.
  - manual control via pushbuttons/switch.
- sound system based on DFPlayerMini breakout board, for a complete sound customization.
- Fully configurable, adaptable to any rail layout.
- Configuration via DCC CV or my custom PC tool [DecoderConfigurator].

[NmraDcc]: https://github.com/mrrwa/NmraDcc
[ConfCVlib]: https://github.com/M5Ross/ConfCVlib
[DccSerialCom]: https://github.com/M5Ross/DccSerialCom
[DecoderConfigurator]: https://github.com/M5Ross/DecoderConfigurator
[DFRobotDFPlayerMini]: https://github.com/DFRobot/DFRobotDFPlayerMini
# HEX Motor Example

This is a demo project to show how to use the HEX-XXXX series of motors.

## Before You Start

Please read all contents of the exact motor you purchased at [docs site](https://docs.hexfellow.com/hex-motor/) before you start using this repo.

## Start controling multiple motors

As stated in the docs, to control multiple motors in single can frame, you need to first configure each motor's RPDO id. We will use two motors with compressed MIT, default mappings. As for TPDO, we will use the factory default. We will also enable HEARTBEAT Monitoring for both motors.

We assume the motor ids are 0x01 and 0x02. The NMT master will be 0x10.

As stated in docs, the SDOs we need to configure are:
> If a step has a (Saveable) at the end, it means you can skip this if you already saved the parameter via 0x1010. Refer to docs for more details.

- Read 0x1018[1] and 0x1018[3] to make sure it's our motor, and the firmware version is expected.
- Enable Short-Circuit Breaking by writing `1u8` to 0x2040[0] (Optional, Saveable)
- Limit maximum torque by writing Permille number (80% in this case) `800u16` to 0x6072[0] (Optional, Saveable)
- Write each motor's PDO. Things to write will be different for each motor. (Saveable)
- Write `(0x10u32 << 16) | 250u32` to 0x1016[1] to enable HEARTBEAT Monitoring. (Saveable)
- Select MIT mode by writing `5u8` to 0x6060[0].
- Enable compressed MIT by writing `1u8` to 0x2004[1].
- Write initial MIT control values to 0x2004[2] and 0x2004[3].
- Write control word 0x6040[0] step by step, first write `6u16`, then `7u16`, last `0x0Fu16`.
- NMT set to operational.
- Start sending master's heartbeat to enable heartbeat monitoring.

Remember you can choose to save what you want using 0x1010 and skip a lot of initializations, or choose to initialize everything step by step. Weather to configure the PDOs via by hand(e.g. using canopenlinux) or using program is also up to you. CANOpen is this flexible.


Here for sake of simplicity, we assume you have saved everything we need, by hand or by program.

## Rust

For rust, you need to find a nice CANOpen lib yourself. For application level, the docs site has an example to decode the can frames from motor.

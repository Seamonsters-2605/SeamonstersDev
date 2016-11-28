# Seamonsters Code Development

This repository is meant for robot code that is not tied to a specifiec year.

SeamonstersDev is forked from SeamonstersTemplate. SeamonstersTemplate is an upstream source. To merge any changes from SeamonstersTemplate, do:

```
git fetch upstream
git merge upstream/master
```

## Directory Structure
Not including anything from SeamonstersTemplate.

- `CommandsAndSubsystems.md`: Our notes on pyfrc Commands and Subsystems.
- `stingray.py`: Project Stingray robot code.
- `stingray/`: Modules for the Project Stingray bot.
- `testBot.py`: Robot that is modified for testing various things.
- `holoBot.py`: Simple bot used for testing holonomic drives.
- `bTest.py`: Very simple bot that spins a single motor - used for testing
    Blender simulation.
- `robotSimTest.py`: Used for testing pyfrc robot simulation. See `sim/README` for details.

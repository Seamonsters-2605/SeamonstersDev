# Seamonsters Code Development

This repository is meant for robot code that is not tied to a specifiec year.

SeamonstersDev is forked from SeamonstersTemplate. SeamonstersTemplate should be added as an upstream source - to do this, do `git remote add upstream https://github.com/Seamonsters-2605/SeamonstersTemplate`.

Then, to merge any changes from SeamonstersTemplate, do:

```
git fetch upstream
git merge upstream/master
```

## Directory Structure
Not including anything from SeamonstersTemplate.

- `CommandsAndSubsystems.md`: Our notes on pyfrc Commands and Subsystems.
- `stingray.py`: Project Stingray robot code.
- `stingray/`: Modules for the Project Stingray bot.
- `ahrsDemo.py`: NavX demo.d 
- `holoBot.py`: Holonomic drive for the demo bot.
- `robotSimTest.py`: Used for testing pyfrc robot simulation. See `sim/README` for details.

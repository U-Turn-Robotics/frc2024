import os
import typing

import wpilib as wp


def findAutos():
    autosPath = os.path.join(wp.getDeployDirectory(), "pathplanner", "autos")
    return [f.rstrip(".auto") for f in os.listdir(autosPath) if f.endswith(".auto")]


class AutoSelector:
    def __init__(self):
        self.autoSelector = wp.SendableChooser()
        autos = findAutos()

        defaultAuto = autos[0]
        self.autoSelector.setDefaultOption(defaultAuto, defaultAuto)
        for auto in autos:
            if auto != defaultAuto:
                self.autoSelector.addOption(auto, auto)

        wp.SmartDashboard.putData("Auto Selector", self.autoSelector)

        self.lastAutoSelected = None

    def checkNewAutoSelected(self, auto_loader: typing.Callable[[str], None]):
        autoSelected: str = self.autoSelector.getSelected()
        if autoSelected != self.lastAutoSelected:
            auto_loader(autoSelected)

            self.lastAutoSelected = autoSelected

import typing

from photonlibpy.photonTrackedTarget import PhotonTrackedTarget


def getNearestTarget(targets: typing.List[PhotonTrackedTarget]) -> PhotonTrackedTarget:
    nearestTarget = None
    for target in targets:
        if nearestTarget is None:
            nearestTarget = target
            continue
        if target.getArea() > nearestTarget.getArea():
            nearestTarget = target
    return nearestTarget

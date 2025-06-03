module Svc {
    # Include the Vector3 type
    include "Svc/MagneticDetumble/MagneticDetumbleTypes.fpp"

    @ Port for passing angular velocity vector
    port AngVel(ref val: Svc.Vector3)

    @ Port for passing magnetic field vector
    port MagField(ref val: Svc.Vector3)

    @ Port for commanding magnetic dipole
    port MagDipoleCmd(ref val: Svc.Vector3)
}
